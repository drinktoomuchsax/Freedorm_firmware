#include "lock_control.h"
#include "driver/gpio.h"
#include "button.h"

void lock_the_door(void)
{
    gpio_set_level(CTL_D0, 1);
}

void unlock_the_door(void)
{
    gpio_set_level(CTL_LOCK, 1);
}

door_status_t get_door_status(void)
{
    if (gpio_get_level(CTL_LOCK) == 1)
    {
        return DOOR_UNLOCKED;
    }
    else if (gpio_get_level(CTL_D0) == 1)
    {
        return DOOR_LOCKED;
    }
    else if ((gpio_get_level(CTL_D0) == 0) && (gpio_get_level(CTL_LOCK) == 0))
    {
        return DOOR_NORMAL;
    }
    else
    {
        return DOOR_UNKNOWN;
    }
}