typedef enum {
    HW_DISPLAY,
    HW_GPS,
    HW_RTC,
    HW_SPOX,
    HW_MAGNMTR,
    HW_IMU,
    HW_BLE,
    HW_BUTTONS,
    HW_BACKLIGHT

} hw_id_t;

int hw_init(void);
bool hw_present(hw_id_t id);
int hw_suspend(hw_id_t id);
int hw_resume(hw_id_t id);