//Kernel Events

typedef enum {
    EVT_NONE = 0,
    EVT_TICK_1S,
    EVT_BUTTON,
    EVT_GPS_FIX,
    EVT_GPS_TIME,
    EVT_BATT_UPDATE,
    EVT_APP_SWITCH,
    EVT_RESTART,
    EVT_REDRAW,
    EVT_LOW_BATT,
    EVT_COUNT

}   evt_type_t;

typedef struct {
    uint8_t type;
    uint8_t a;
    uint16_t b;
    uint32_t data;
} event_t;
