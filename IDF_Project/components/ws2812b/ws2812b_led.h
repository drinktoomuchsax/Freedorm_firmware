#ifndef WS2812B_LED_H
#define WS2812B_LED_H

// 默认效果（可选）
#define DEFAULT_EFFECT LED_EFFECT_POWER_ON_ANIMATION
#define WS2812B_NUMBER_OF_EFFECTS LED_EFFECT_COUNT + 1
typedef enum
{
    LED_EFFECT_DEBUG = 0,
    // 以下为交互状态
    LED_EFFECT_DEFAULT_STATE,             // 默认状态
    LED_EFFECT_SINGLE_OPEN_DOOR,          // 单次开门
    LED_EFFECT_ALWAYS_OPEN_MODE,          // 常开模式
    LED_EFFECT_OPEN_MODE_END,             // 开门结束闪烁
    LED_EFFECT_CONFIRM_FACTORY_RESET,     // 确定恢复出厂设置
    LED_EFFECT_FACTORY_RESETTING,         // 恢复出厂设置中（发包给后端通知微信解绑，清蓝牙，清CAS
    LED_EFFECT_FINISH_FACTORY_RESET,      // 设备初始化完毕
    LED_EFFECT_BLE_TRY_PAIRING,           // ble_尝试进入蓝牙配对
    LED_EFFECT_BLE_PAIRING_MODE,          // ble_进入蓝牙配对状态
    LED_EFFECT_BLE_CONNECTED_FIRST_TIME,  // ble_蓝牙连接（初次连接建立成功
    LED_EFFECT_VISITOR_CODE_OPEN_DOOR,    // 访客码限时开门
    LED_EFFECT_VISITOR_CODE_TIME_EXPIRED, // 访客码限时开门时间结束
    LED_EFFECT_OPEN_BLUETOOTH_NEARBY,     // open_蓝牙靠近开门
    LED_EFFECT_OPEN_BLUETOOTH_FINISHED,   // open_蓝牙靠近开门结束
    LED_EFFECT_LOCK_DOOR,                 // open_锁门
    LED_EFFECT_POWER_ON_ANIMATION,        // 默认上电之后的灯效, LED不发光，黑屏
    LED_EFFECT_FIRST_POWER_ON_ACTIVATE,   // 第一次上电激活
    /*!没有效果，一定要放在最后，用来判断效果数量!*/
    LED_EFFECT_COUNT,
} ws2812b_state_effect_t;
typedef enum
{
    LED_MODE_LOOP = 0,
    LED_LOOP_MODE_PINGPONG,
    LED_LOOP_MODE_SINGLE,
} ws2812b_loop_mode_t;

typedef enum
{
    LED_DIRECTION_TOP_DOWN = 0,
    LED_DIRECTION_BOTTOM_UP,
} ws2812b_direction_t;

typedef struct
{
    uint32_t hue;
    uint32_t saturation;
    uint32_t value;
} ws2812b_color_hsv_t;

typedef struct
{
    uint32_t red;
    uint32_t green;
    uint32_t blue;
} ws2812b_color_rgb_t;

typedef struct
{
    ws2812b_color_rgb_t color_rgb;
    ws2812b_direction_t direction;
    ws2812b_loop_mode_t loop_mode;
} ws2812b_effect_args_t;

// 用来传递参数的结构体
typedef struct
{
    ws2812b_effect_args_t effect_args;
    ws2812b_state_effect_t current_effect;
} ws2812b_queue_data_t;

#define FREEDORM_BLUE {0, 0, 255}
#define WHITE_RGB {255, 255, 255}
#define RED_RGB {0, 0, 255}
#define GREEN_RGB {0, 255, 0}
#define BLUE_RGB {255, 0, 0}
#define YELLOW_RGB {0, 255, 255}
#define CYAN_RGB {255, 255, 0}
#define PURPLE_RGB {255, 0, 255}

// 效果队列句柄
extern QueueHandle_t effect_queue;
extern TaskHandle_t xLedTaskHandle; // 声明任务句柄

void ws2812b_led_init(void);
void ws2812b_switch_effect(ws2812b_state_effect_t effect);
void loop_ws2812b_effect();

#endif // WS2812B_LED_H