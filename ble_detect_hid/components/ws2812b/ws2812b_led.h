#ifndef WS2812B_LED_H
#define WS2812B_LED_H

// 默认效果（可选）
#define DEFAULT_EFFECT LED_EFFECT_RANDOM_COLOR
#define WS2812B_NUMBER_OF_EFFECTS LED_EFFECT_METEOR + 1
typedef enum
{
    LED_EFFECT_RAINBOW_WAVE = 0, // 必须是0，不然不能知道EFFECT的数量
    LED_EFFECT_RAINBOW_ALL,
    LED_EFFECT_BREATHING_WAVE,
    LED_EFFECT_BREATHING_ALL,
    LED_EFFECT_RAINBOW_BREATHING_ALL,
    LED_EFFECT_RAINBOW_BREATHING_WAVE,
    LED_EFFECT_RANDOM_COLOR,
    /*!流星效果，一定要放在最后，用来判断效果数量!*/
    LED_EFFECT_METEOR,
} ws2812b_effect_t;
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

#define FREEDORM_BLUE {0, 0, 255}
#define WHITE_RGB {255, 255, 255}
#define RED_RGB {255, 0, 0}
#define GREEN_RGB {0, 255, 0}
#define BLUE_RGB {0, 0, 255}
#define YELLOW_RGB {255, 255, 0}
#define CYAN_RGB {0, 255, 255}
#define PURPLE_RGB {255, 0, 255}

// 效果队列句柄
extern QueueHandle_t effect_queue;
// 用来传递参数的结构体
typedef struct
{
    ws2812b_effect_args_t effect_args;
    ws2812b_effect_t current_effect;
} ws2812b_queue_data_t;

void ws2812b_led_init(void);

#endif // WS2812B_LED_H