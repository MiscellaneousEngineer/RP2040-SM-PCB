#include "pwr.h"
#include "hw.c"

typedef struct
{
    hw_id_t id;
    int (*set_power)(pw_level_t);
    uint8_t suspend_order;
} pw_dev_t;

static const pw_dev_t pw_devs[] = {
    {HW_BACKLIGHT, bl_set_power, 0},
    {HW_GPS, gps_set_power, 2}

}