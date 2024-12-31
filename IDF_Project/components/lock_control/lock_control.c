#include "lock_control.h"
#include "button.h"
#include "ws2812b_led.h"
#include "freertos/queue.h"

#define LOCK_CONTROL_TAG "LOCK_CONTROL"
#define TIME_RECOVER_TEMP_OPEN 30000 // 定义超时时间 (ms)

static lock_status_t current_lock_state = STATE_NORAML_DEFAULT;
static TimerHandle_t temp_open_timer = NULL;
static TimerHandle_t lock_timer = NULL;

// 状态切换函数声明
void transition_to_state(lock_status_t new_state);

// 状态处理函数声明
void handle_power_on_black();
void lock_set_normal();
void lock_set_open();
void lock_set_lock();
void start_timer_temp_open();

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
    default:
        return "Unknown Lock State";
    }
}

// 初始化函数
void lock_control_init(void)
{
    // 初始化状态机
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

// 状态机主任务
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
                handle_power_on_black();
                break;
            case STATE_NORAML_DEFAULT:
                if (event == BUTTON_EVENT_SINGLE_CLICK)
                {
                    lock_set_normal();
                    start_timer_temp_open();
                    ws2812b_switch_effect(LED_EFFECT_SINGLE_OPEN_DOOR);
                    transition_to_state(STATE_TEMP_OPEN);
                    ESP_LOGI(LOCK_CONTROL_TAG, "lock set to open, lockstate: %s", get_lock_state_name(current_lock_state));
                }
                else if (event == BUTTON_EVENT_DOUBLE_CLICK)
                {
                    lock_set_open();
                    // set_light_effect("breathing");
                    transition_to_state(STATE_ALWAYS_OPEN);
                }
                else if (event == BUTTON_EVENT_MULTI_CLICK)
                {
                    lock_set_lock();
                    // set_light_effect("amusement");
                    transition_to_state(STATE_LOCKED);
                }
                break;
            case STATE_TEMP_OPEN:
                if (event == BUTTON_EVENT_SINGLE_CLICK)
                {
                    lock_set_normal();
                    // set_light_effect("default");
                    transition_to_state(STATE_NORAML_DEFAULT);
                    ESP_LOGI(LOCK_CONTROL_TAG, "lock set to normal, lockstate: %s", get_lock_state_name(current_lock_state));
                }
                else if (event == BUTTON_EVENT_DOUBLE_CLICK)
                {
                    lock_set_open();
                    // set_light_effect("breathing");
                    transition_to_state(STATE_ALWAYS_OPEN);
                }
                break;
            case STATE_ALWAYS_OPEN:
                if (event == BUTTON_EVENT_SINGLE_CLICK)
                {
                    lock_set_normal();
                    // set_light_effect("default");
                    transition_to_state(STATE_NORAML_DEFAULT);
                }
                break;
            case STATE_LOCKED:
                if (event == BUTTON_EVENT_SINGLE_CLICK)
                {
                    lock_set_normal();
                    // set_light_effect("default");
                    transition_to_state(STATE_NORAML_DEFAULT);
                }
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

void handle_power_on_black()
{
    ws2812b_switch_effect(LED_EFFECT_FIRST_POWER_ON_ACTIVATE);
    transition_to_state(STATE_NORAML_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(6000));
    ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
}

// 定时器逻辑
void start_timer_temp_open()
{
    if (temp_open_timer == NULL)
    {
        temp_open_timer = xTimerCreate("TempOpenTimer", pdMS_TO_TICKS(TIME_RECOVER_TEMP_OPEN), pdFALSE, NULL, (TimerCallbackFunction_t)lock_set_lock);
    }
    xTimerStart(temp_open_timer, 0);
}

void reset_timer(TimerHandle_t timer)
{
    if (timer != NULL)
    {
        xTimerStop(timer, 0);
        xTimerDelete(timer, 0);
    }
}

void lock_set_lock(void)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Locking door, lockstate: %d", current_lock_state);
    gpio_set_level(CTL_D0, 1);
    gpio_set_level(OUTPUT_LED_D5, 1);
    gpio_set_level(OUTPUT_LED_D4, 1);
    current_lock_state = STATE_TEMP_OPEN;
}

void lock_set_open(void)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Opening door, lockstate: %d", STATE_LOCKED);
    gpio_set_level(CTL_LOCK, 1);
    gpio_set_level(OUTPUT_LED_D4, 0);
    gpio_set_level(OUTPUT_LED_D5, 1);
    current_lock_state = STATE_ALWAYS_OPEN;
}

void lock_set_normal(void)
{
    // ESP_LOGI(LOCK_CONTROL_TAG, "Setting door to normal, lockstate: %d", STATE_LOCKED);
    gpio_set_level(CTL_LOCK, 0);
    gpio_set_level(CTL_D0, 0);
    gpio_set_level(OUTPUT_LED_D4, 0);
    gpio_set_level(OUTPUT_LED_D5, 0);
    current_lock_state = STATE_LOCKED;
}