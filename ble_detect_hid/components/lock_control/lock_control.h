#ifndef LOCK_CONTROL_H
#define LOCK_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

typedef enum
{
    DOOR_LOCKED = 0,
    DOOR_UNLOCKED = 1,
    DOOR_NORMAL = 2,
    DOOR_UNKNOWN = 69
} door_status_t;

void flip_gpio(int gpio_num);
void lock_the_door(void);
void unlock_the_door(void);
door_status_t get_door_status(void);

#endif // LOCK_CONTROL_H