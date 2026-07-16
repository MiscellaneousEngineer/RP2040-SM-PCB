//Power sequences for devices on the watch
//Drivers handle low level commands

typedef enum {
    PWR_ON,
    PWR_IDLE,
    PWR_SUSPEND,
    PWR_OFF
} pw_level_t;

int st7735_set_power(pw_level_t 1);
