#include "lock_control.h"
#include "button.h"
#include "ws2812b_led.h"
#include "freertos/queue.h"
#include "ble_module.h"
#include "esp_mac.h"

#define LOCK_CONTROL_TAG "LOCK_CONTROL"

typedef enum
{
    OPEN_MODE_ONCE,
    OPEN_MODE_ALWAYS,
    OPEN_MODE_ONCE_BLE,
} open_mode_t;

static lock_status_t current_lock_state = STATE_NORAML_DEFAULT;
static TimerHandle_t temp_open_timer = NULL;      // 用来恢复临时开门状态的定时器
static TimerHandle_t ble_temp_open_timer = NULL;  // 用来恢复蓝牙临时开门状态的定时器
static TimerHandle_t lock_timer = NULL;           // 用来恢复锁定状态的定时器
static TimerHandle_t long_press_ble_timer = NULL; // 蓝牙长按计时器，用来判断是否进入蓝牙配对模式

// 状态切换函数声明
void transition_to_state(lock_status_t new_state);
void transition_to_STATE_NORAML_DEFAULT(); // 转换到默认刷卡开门状态，灯效可以被配置，默认是彩虹呼吸灯
void transition_to_STATE_TEMP_OPEN_END();
void transition_to_STATE_BLE_TEMP_OPEN_END();

// 状态处理函数声明
void handle_power_on_black();

/**
 * @brief 恢复到正常状态，关闭D0线对地短接的MOSFET，LOCK线恢复到开漏状态，使宿舍门锁模块进入正常状态，使用校园卡开门
 *
 */
void lock_set_normal();

/**
 * @brief 让门锁进入开门状态，通过拉低LOCK线，使宿舍门锁模块进入开门状态
 * @attention 注意！因为不能确定是单次开门调用的还是常开模式调用的，所以这里不改变状态，在调用的地方通过参数open_mode来判断要去的状态
 *
 */
void lock_set_open(open_mode_t open_mode);

/**
 * @brief 让门锁进入锁定状态，通过拉高或拉低D0线，使数据无法被传输，因为D0和ESP32电平不匹配，这里是用了一个MOSFET来对地短接，所以GPIO高电平使MOSFET打开，对地短接，导致数据无法传输。
 *
 *
 */
void lock_set_lock();

/**
 * @brief 进入临时开门状态后启用的定时器，TIME_RECOVER_TEMP_OPEN秒后恢复到正常状态
 *
 */
void start_timer_temp_open();

/**
 * @brief 开始蓝牙长按计时器，6s之后进入蓝牙配对模式，同时蓝牙进入配对灯效在这里开始播放
 *
 */
void start_ble_long_press_timer();

/**
 * @brief 停止蓝牙长按计时器，不用进入蓝牙配对模式
 *
 */
void stop_ble_long_press_timer();

/**
 * @brief 发送一个蓝牙配对信号量，让ble_module进入配对模式
 *
 */
void ble_start_pairing(void);

/**
 * @brief 恢复出厂设置，忘记蓝牙、WI-FI
 *
 * TODO: 具体的恢复出厂设置操作，蓝牙和WI-FI的操作
 */
void factory_reset_start(void);

/* 工具函数声明和定义 */
static const char *get_lock_state_name(lock_status_t state)
{
    switch (state)
    {
    case STATE_POWER_ON_BLACK:
        return "STATE_POWER_ON_BLACK";
    case STATE_NORAML_DEFAULT:
        return "STATE_NORAML_DEFAULT";
    case STATE_TEMP_OPEN:
        return "STATE_TEMP_OPEN";
    case STATE_ALWAYS_OPEN:
        return "STATE_ALWAYS_OPEN";
    case STATE_LOCKED:
        return "STATE_LOCKED";
    case STATE_TEMP_OPEN_END:
        return "STATE_TEMP_OPEN_END";
    case STATE_LOCK_END:
        return "STATE_LOCK_END";
    case STATE_BLE_TEMP_OPEN:
        return "STATE_BLE_TEMP_OPEN";
    case STATE_BLE_TEMP_OPEN_END:
        return "STATE_BLE_TEMP_OPEN_END";
    case STATE_BLE_PAIRING_PREPARE:
        return "STATE_BLE_PAIRING_PREPARE";
    case STATE_BLE_PAIRING_IN_PROGRESS:
        return "STATE_BLE_PAIRING_IN_PROGRESS";
    case STATE_BLE_PAIRING_TIME_OUT:
        return "STATE_BLE_PAIRING_TIME_OUT";
    case STATE_RESTORY_FACTORY_SETTINGS_PREPARE:
        return "STATE_RESTORY_FACTORY_SETTINGS_PREPARE";
    case STATE_RESTORY_FACTORY_SETTINGS:
        return "STATE_RESTORY_FACTORY_SETTINGS";

    default:
        return "Unknown Lock State";
    }
}

static ws2812b_state_effect_t default_led_effect = LED_EFFECT_DEFAULT_STATE;

void start_timer_temp_open()
{
    if (temp_open_timer == NULL)
    {
        temp_open_timer = xTimerCreate("TempOpenTimer", pdMS_TO_TICKS(TIME_RECOVER_TEMP_OPEN), pdFALSE, NULL, (TimerCallbackFunction_t)transition_to_STATE_TEMP_OPEN_END);
    }
    xTimerStart(temp_open_timer, 0);
}

void start_timer_ble_temp_open()
{
    if (ble_temp_open_timer == NULL)
    {
        ble_temp_open_timer = xTimerCreate("BleTempOpenTimer", pdMS_TO_TICKS(TIME_BLE_RECOVER_TEMP_OPEN), pdFALSE, NULL, (TimerCallbackFunction_t)transition_to_STATE_BLE_TEMP_OPEN_END);
    }
    xTimerStart(ble_temp_open_timer, 0);
}

void start_timer_lock()
{
    if (lock_timer == NULL)
    {
        lock_timer = xTimerCreate("LockTimer", pdMS_TO_TICKS(TIME_RECOVER_LOCK), pdFALSE, NULL, (TimerCallbackFunction_t)lock_set_normal);
    }
    xTimerStart(lock_timer, 0);
}

void reset_timer(TimerHandle_t *timer)
{
    if (timer != NULL && *timer != NULL)
    {
        xTimerStop(*timer, 0);
        xTimerDelete(*timer, 0);
        *timer = NULL; // 防止二次释放
    }
}

// 初始化函数
void lock_control_init(void)
{

    // 初始化 GPIO
    uint64_t gpio_output_sel = (1ULL << OUTPUT_LED_D4) | (1ULL << OUTPUT_LED_D5) | (1ULL << CTL_D0);
    gpio_config_t output_io_conf = {
        .pin_bit_mask = gpio_output_sel,
        .mode = GPIO_MODE_OUTPUT,              // 输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,     // 不需要上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 不需要下拉
        .intr_type = GPIO_INTR_DISABLE         // 不需要中断
    };
    gpio_config(&output_io_conf);

    // 单独配置锁控制引脚，因为不希望外表LOCK信号被单片机主动拉高，这样会导致开不了门
    uint64_t lock_ctl_output_sel = (1ULL << CTL_LOCK);
    gpio_config_t lock_ctl_io_conf = {
        .pin_bit_mask = lock_ctl_output_sel,
        .mode = GPIO_MODE_OUTPUT_OD,           // 输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,     // 不需要上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 不需要下拉
        .intr_type = GPIO_INTR_DISABLE         // 不需要中断
    };
    gpio_config(&lock_ctl_io_conf);

    gpio_set_level(CTL_LOCK, 1); // 默认恢复LOCK线到开漏状态，不对门锁模块产生影响

    // 初始化状态为上电之后的黑屏状态，但是具体的上电灯效是在ws2812component里做的
    transition_to_state(STATE_POWER_ON_BLACK);

    // 初始化按键事件队列
    button_event_queue = xQueueCreate(10, sizeof(button_event_t));
    if (button_event_queue == NULL)
    {
        ESP_LOGE(LOCK_CONTROL_TAG, "Failed to create button event queue");
        return;
    }

    // 创建任务
    xTaskCreate(lock_control_task, "lock_control_task", 2048, NULL, 10, NULL);
}

/**
 * @brief 主状态机，具体图维护在sax的mermaid账号中
 * // [MermaidChart: dbe783ad-e5ab-4b68-8150-56c056ea9093]
 *
 * 如何移植灯效控制到状态机中？
 *
 * @param pvParameters
 */
void lock_control_task(void *pvParameters)
{
    button_event_t event;

    while (1)
    {
        if (xQueueReceive(button_event_queue, &event, portMAX_DELAY))
        {
            switch (current_lock_state)
            {
            case STATE_POWER_ON_BLACK:
                if (event == BUTTON_EVENT_LONG_PRESS_HOLD_3S)
                {
                    handle_power_on_black();
                }

                break;
            case STATE_NORAML_DEFAULT:
                if (event == BUTTON_EVENT_SINGLE_CLICK)
                {
                    lock_set_open(OPEN_MODE_ONCE); // 单击打开门
                }
                else if (event == BUTTON_EVENT_DOUBLE_CLICK)
                {
                    lock_set_open(OPEN_MODE_ALWAYS); // 双击进入常开模式
                }
                else if (event == BUTTON_EVENT_DECUPLE_CLICK)
                {
                    lock_set_lock();
                }
                else if (event == BLE_BUTTON_EVENT_SINGLE_CLICK)
                {
                    lock_set_open(OPEN_MODE_ONCE_BLE);
                }
                else if (event == BUTTON_EVENT_LONG_PRESS_START)
                {
                    start_ble_long_press_timer();
                    transition_to_state(STATE_BLE_PAIRING_PREPARE);
                }

                break;
            case STATE_TEMP_OPEN:
                if (event == BUTTON_EVENT_SINGLE_CLICK)
                {
                    transition_to_STATE_TEMP_OPEN_END();
                }
                else if (event == BUTTON_EVENT_DOUBLE_CLICK)
                {
                    lock_set_open(OPEN_MODE_ALWAYS); // 双击进入常开模式
                }
                break;

            case STATE_TEMP_OPEN_END:
                ws2812b_switch_effect(LED_EFFECT_OPEN_MODE_END); // 开门结束的闪烁
                vTaskDelay(pdMS_TO_TICKS(600));
                lock_set_normal();
                break;

            case STATE_ALWAYS_OPEN:
                if (event == BUTTON_EVENT_SINGLE_CLICK || event == BUTTON_EVENT_DOUBLE_CLICK) // 单击或双击都可以关闭常开模式🚪
                {
                    transition_to_STATE_TEMP_OPEN_END();
                }
                break;

            case STATE_LOCKED: // 发布产品里面没有焊接硬件电阻，所以这个状态不会有锁门的现象
                if (event == BUTTON_EVENT_SINGLE_CLICK || event == BUTTON_EVENT_DOUBLE_CLICK) // 单击或双击都可以关闭锁定模式🔒
                {
                    reset_timer(&lock_timer); // 关闭锁定设置的定时器
                    lock_set_normal();
                }
                else if (event == BUTTON_EVENT_LONG_PRESS_START) // 长按进入恢复出厂设置状态
                {

                    reset_timer(&lock_timer); // 关闭锁定设置的定时器
                    lock_set_normal();
                    ws2812b_switch_effect(LED_EFFECT_CONFIRM_FACTORY_RESET);
                    transition_to_state(STATE_RESTORY_FACTORY_SETTINGS_PREPARE);
                }else if (event == BUTTON_EVENT_DECUPLE_CLICK)
                {
                    if(default_led_effect == LED_EFFECT_DEFAULT_STATE){
                        default_led_effect = LED_EFFECT_POWER_ON_ANIMATION;

                    }else if (default_led_effect == LED_EFFECT_POWER_ON_ANIMATION){
                        default_led_effect = LED_EFFECT_DEFAULT_STATE;
                    }
                }
                
                break;

            case STATE_BLE_TEMP_OPEN:
                if (event == BUTTON_EVENT_SINGLE_CLICK)
                {
                    transition_to_STATE_BLE_TEMP_OPEN_END();
                }
                else if (event == BUTTON_EVENT_DOUBLE_CLICK)
                {
                    lock_set_open(OPEN_MODE_ALWAYS); // 双击进入常开模式
                }
                break;

            case STATE_BLE_TEMP_OPEN_END:
                ws2812b_switch_effect(LED_EFFECT_OPEN_BLUETOOTH_FINISHED); // 开门结束的闪烁
                vTaskDelay(pdMS_TO_TICKS(600));
                lock_set_normal();

                break;

            case STATE_BLE_PAIRING_PREPARE:
                if (event == BUTTON_EVENT_LONG_PRESS_HOLD_6S) // 灯效播放完了（6s），正好能够进入配对模式
                {
                    transition_to_state(STATE_BLE_PAIRING_IN_PROGRESS);
                    ws2812b_switch_effect(LED_EFFECT_BLE_PAIRING_MODE);
                }
                else if (event == BUTTON_EVENT_LONG_PRESS_END)
                {
                    ESP_LOGI(LOCK_CONTROL_TAG, "Long press end detected, stop pairing");
                    stop_ble_long_press_timer();
                    transition_to_state(STATE_NORAML_DEFAULT);
                    ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
                }

                break;

            case STATE_BLE_PAIRING_IN_PROGRESS:
                // if (event == BUTTON_EVENT_LONG_PRESS_HOLD_6S)
                // {
                //     transition_to_state(STATE_BLE_PAIRING_TIME_OUT);
                //     ws2812b_switch_effect(LED_EFFECT_BLE_PAIRING_TIMEOUT);
                // }
                break;
            case STATE_BLE_PAIRING_TIME_OUT:
                // if (event == BUTTON_EVENT_SINGLE_CLICK)
                // {
                //     transition_to_state(STATE_NORAML_DEFAULT);
                //     ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
                // }
                break;
            case STATE_RESTORY_FACTORY_SETTINGS_PREPARE:
                if (event == BUTTON_EVENT_LONG_PRESS_HOLD_6S)
                {
                    transition_to_state(STATE_RESTORY_FACTORY_SETTINGS);
                    ws2812b_switch_effect(LED_EFFECT_FACTORY_RESETTING);
                    vTaskDelay(pdMS_TO_TICKS(4000));
                }
                else if (event == BUTTON_EVENT_LONG_PRESS_END)
                {
                    // stop_factory_reset_long_press_timer();
                    transition_to_state(STATE_NORAML_DEFAULT);
                    ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
                }
                break;
            case STATE_RESTORY_FACTORY_SETTINGS:
                ESP_LOGI(LOCK_CONTROL_TAG, "Factory reset start");
                ws2812b_switch_effect(LED_EFFECT_FINISH_FACTORY_RESET);
                vTaskDelay(pdMS_TO_TICKS(1000));
                factory_reset_start(); // 这个地方直接重启，不用再切换状态了

                break;

            default:
                break;
            }
        }
    }
}

// 状态切换
void transition_to_state(lock_status_t new_state)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Transitioning from state %s to state %s", get_lock_state_name(current_lock_state), get_lock_state_name(new_state));
    current_lock_state = new_state;
}


void transition_to_STATE_NORAML_DEFAULT(){
    
    if (default_led_effect == LED_EFFECT_DEFAULT_STATE){ // 默认是彩虹呼吸灯，配置黑灯后按10下再按10下开回来
        transition_to_state(STATE_NORAML_DEFAULT);
        ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
    }else if (default_led_effect == LED_EFFECT_POWER_ON_ANIMATION)  // 按10下后，再按10下，关闭默认彩虹呼吸灯
    {
        transition_to_state(STATE_NORAML_DEFAULT);
        ws2812b_switch_effect(LED_EFFECT_POWER_ON_ANIMATION);
    }
    
}

void transition_to_STATE_TEMP_OPEN_END()
{
    reset_timer(&temp_open_timer); // 关闭单次开门设置的定时器
    transition_to_state(STATE_TEMP_OPEN_END);
    send_button_event(BUTTON_EVENT_NONE_UPDATE_LOCK_CONTROL);
}

void transition_to_STATE_BLE_TEMP_OPEN_END()
{
    reset_timer(&ble_temp_open_timer); // 关闭单次开门设置的定时器
    transition_to_state(STATE_BLE_TEMP_OPEN_END);
    send_button_event(BUTTON_EVENT_NONE_UPDATE_LOCK_CONTROL);
}

void handle_power_on_black()
{
    vTaskDelay(pdMS_TO_TICKS(666));
    ws2812b_switch_effect(LED_EFFECT_FIRST_POWER_ON_ACTIVATE);
    transition_to_state(STATE_NORAML_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(6000));
    ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
}

void lock_set_lock(void)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Locking door, lockstate:  %s", get_lock_state_name(current_lock_state));
    gpio_set_level(CTL_D0, 1); // 通过开启MOSFET，拉低D0线，使数据无法传输
    // 打开两个板载LED
    gpio_set_level(OUTPUT_LED_D5, 1);
    gpio_set_level(OUTPUT_LED_D4, 1);

    start_timer_lock(); // 开启定时器，TIME_RECOVER_LOCK秒后恢复到正常状态

    // 灯效和状态机切换到锁定状态
    ws2812b_switch_effect(LED_EFFECT_LOCK_DOOR);
    transition_to_state(STATE_LOCKED);
}

void lock_set_open(open_mode_t open_mode)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Opening door, lockstate:  %s", get_lock_state_name(current_lock_state));
    gpio_set_level(CTL_LOCK, 0); // 通过拉低LOCK线，使宿舍门锁模块进入开门状态
    // 打开一个板载LED
    gpio_set_level(OUTPUT_LED_D4, 0);
    gpio_set_level(OUTPUT_LED_D5, 1);

    // 灯效和状态机切换到对应开门状态
    if (open_mode == OPEN_MODE_ONCE)
    {
        start_timer_temp_open();                            // 开启定时器，TIME_RECOVER_TEMP_OPEN秒后恢复到正常状态
        ws2812b_switch_effect(LED_EFFECT_SINGLE_OPEN_DOOR); // 同时开启LED效果
        transition_to_state(STATE_TEMP_OPEN);               // 然后进入临时开门状态
    }
    else if (open_mode == OPEN_MODE_ALWAYS)
    {
        ws2812b_switch_effect(LED_EFFECT_ALWAYS_OPEN_MODE);
        transition_to_state(STATE_ALWAYS_OPEN);
    }
    else if (open_mode == OPEN_MODE_ONCE_BLE)
    {
        start_timer_ble_temp_open(); // 开启定时器，TIME_RECOVER_TEMP_OPEN秒后恢复到正常状态
        ws2812b_switch_effect(LED_EFFECT_OPEN_BLUETOOTH_NEARBY);
        transition_to_state(STATE_BLE_TEMP_OPEN);
    }
}

void lock_set_normal(void)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Setting door to normal, from lockstate:  %s", get_lock_state_name(current_lock_state));
    gpio_set_level(CTL_LOCK, 1); // 恢复LOCK线到开漏状态
    gpio_set_level(CTL_D0, 0);   // 通过关闭MOSFET，恢复D0线到正常状态
    // 关闭两个板载LED
    gpio_set_level(OUTPUT_LED_D4, 0);
    gpio_set_level(OUTPUT_LED_D5, 0);

    // 灯效和状态机切换到正常状态
    ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
    transition_to_state(STATE_NORAML_DEFAULT);
}

lock_status_t get_current_lock_state()
{
    return current_lock_state;
}

void start_ble_long_press_timer()
{
    long_press_ble_timer = xTimerCreate("long_press_ble_timer", 6000 / portTICK_PERIOD_MS, pdFALSE, NULL, (TimerCallbackFunction_t)ble_start_pairing);
    if (long_press_ble_timer != NULL)
    {
        xTimerStart(long_press_ble_timer, 0);
    }
    else
    {
        ESP_LOGE(LOCK_CONTROL_TAG, "Failed to create long press timer");
    }

    ws2812b_switch_effect(LED_EFFECT_BLE_TRY_PAIRING); // 播放蓝牙配对灯效
}

void stop_ble_long_press_timer()
{
    if (long_press_ble_timer != NULL)
    {
        xTimerStop(long_press_ble_timer, 0);
        xTimerDelete(long_press_ble_timer, 0);
        long_press_ble_timer = NULL;
    }
}

void ble_start_pairing(void)
{
    xTimerStop(long_press_ble_timer, 0);
    xTimerDelete(long_press_ble_timer, 0);

    xSemaphoreGive(pairing_semaphore);
    ws2812b_switch_effect(LED_EFFECT_BLE_PAIRING_MODE);
}

void factory_reset_start(void)
{

    // esp_err_t err = esp_wifi_restore();
    // if (err == ESP_OK)
    // {
    //     ESP_LOGI(LOCK_CONTROL_TAG, "Wi-Fi restored to factory settings");
    // }
    // else
    // {
    //     ESP_LOGE(LOCK_CONTROL_TAG, "Failed to restore Wi-Fi to factory settings");
    // }

    // // 恢复蓝牙
    // err = esp_ble_gap_remove_bond_device_all();
    // if (err == ESP_OK)
    // {
    //     ESP_LOGI(LOCK_CONTROL_TAG, "Bluetooth restored to factory settings");
    // }
    // else
    // {
    //     ESP_LOGE(LOCK_CONTROL_TAG, "Failed to restore Bluetooth to factory settings");
    // }

    // 重启设备
    esp_restart();
}