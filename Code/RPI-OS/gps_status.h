/*
 * gps_status.h - High-level GPS telemetry + control for the watch OS.
 *
 * Sits on top of ubx.c. Everything here is non-blocking except where noted,
 * so it can be called from the main render loop without stalling the UI.
 *
 * Usage:
 *     gps_init();                       // once, after uart_init()
 *     ...
 *     while (1) {
 *         gps_service();                // every loop iteration
 *         gps_status_t st;
 *         gps_get(&st);                 // cheap snapshot, always valid
 *         draw_my_window(&st);
 *     }
 *
 * When the diagnostic sub-window is open, call gps_diag_active(true) so the
 * expensive polls (NAV-SAT, MON-RF) run. Call gps_diag_active(false) when it
 * closes: at 9600 baud a full NAV-SAT is ~380 ms of airtime and will starve
 * your NMEA/PVT stream if you poll it continuously.
 */

#ifndef GPS_STATUS_H
#define GPS_STATUS_H

#include <stdint.h>
#include <stdbool.h>
#include "ubx.h"

/* ─────────────── enums for the UI ─────────────── */

typedef enum {
    GPS_SEARCH_OFFLINE = 0,   /* no bytes from the module at all */
    GPS_SEARCH_IDLE,          /* module alive, GNSS engine stopped or asleep */
    GPS_SEARCH_COLD,          /* running, zero satellites in view */
    GPS_SEARCH_SEARCHING,     /* signals being searched, none locked yet */
    GPS_SEARCH_ACQUIRING,     /* satellites locked, not enough for a fix */
    GPS_SEARCH_FIX_2D,
    GPS_SEARCH_FIX_3D,
} gps_search_t;

typedef enum {
    GPS_PWR_FULL = 0,         /* continuous tracking */
    GPS_PWR_PSMOO,            /* power save, on/off */
    GPS_PWR_PSMCT,            /* power save, cyclic tracking */
    GPS_PWR_ENGINE_STOPPED,   /* CFG-RST 0x08: link alive, GNSS off */
    GPS_PWR_STANDBY,          /* RXM-PMREQ: software standby, ~46 uA */
    GPS_PWR_HW_OFF,           /* host cut VCC/V_IO */
} gps_power_t;

typedef enum {
    GPS_ANT_UNKNOWN = 0,      /* supervisor not configured (the normal case) */
    GPS_ANT_OK,
    GPS_ANT_SHORT,
    GPS_ANT_OPEN,
    GPS_ANT_INIT,
} gps_ant_t;

typedef enum {
    GPS_RF_UNKNOWN = 0,
    GPS_RF_OK,
    GPS_RF_WARNING,
    GPS_RF_CRITICAL,
} gps_jam_t;

/* Heuristic front-end health from AGC + C/N0, usable without the antenna
 * supervisor. See gps_rf_calibrate() before trusting this. */
typedef enum {
    GPS_RF_HEALTH_UNKNOWN = 0,
    GPS_RF_HEALTH_GOOD,       /* AGC mid-range, satellites with usable C/N0 */
    GPS_RF_HEALTH_WEAK,       /* AGC pegged high, no signal reaching the front end */
    GPS_RF_HEALTH_SATURATED,  /* AGC pegged low, strong interference or overdrive */
} gps_rf_health_t;

/* ─────────────── the snapshot ─────────────── */

typedef struct {
    /* link */
    bool            link_up;          /* frames received in the last 3 s */
    uint32_t        link_age_ms;      /* since last frame from the module */

    /* search / fix */
    gps_search_t    search;
    uint8_t         fix_type;         /* raw UBX: 0 none .. 3 3D, 5 time only */
    bool            fix_ok;
    uint8_t         sv_in_view;       /* satellites reported by NAV-SAT */
    uint8_t         sv_used;          /* used in the solution */
    uint8_t         sv_searching;     /* qualityInd == 1 */
    uint8_t         sv_locked;        /* qualityInd >= 4 */
    uint8_t         cno_best;         /* dBHz, strongest satellite */
    uint8_t         cno_avg;          /* dBHz, mean over tracked satellites */
    uint32_t        ttff_ms;          /* 0 until first fix */
    uint32_t        uptime_ms;        /* module ms-since-startup */
    uint16_t        pdop_1e2;
    uint32_t        hacc_mm;

    /* time / position, only meaningful with fix_ok */
    bool            time_valid;
    uint16_t        year;
    uint8_t         month, day, hour, min, sec;
    int32_t         lat_1e7, lon_1e7;
    int32_t         alt_msl_mm;

    /* antenna / RF front end */
    gps_ant_t       ant_status;       /* from the supervisor; UNKNOWN if not wired */
    bool            ant_supervised;   /* true only if you enabled the supervisor */
    bool            ant_powered;      /* MON-RF antPower, or the host GPIO state */
    bool            ant_bias_enabled; /* what WE last commanded */
    uint8_t         lna_mode;         /* 0 normal, 1 low gain, 2 bypass */
    uint16_t        agc;              /* 0..8191 */
    uint16_t        noise;
    uint8_t         jam_ind;          /* 0..255 */
    gps_jam_t       jamming;
    gps_rf_health_t rf_health;

    /* power */
    gps_power_t     power;
    bool            gnss_running;

    /* freshness */
    uint32_t        pvt_age_ms;
    uint32_t        rf_age_ms;
    uint32_t        sat_age_ms;
} gps_status_t;

/* ─────────────── lifecycle ─────────────── */

/* Call once after uart_init(). Configures the module: UBX + NMEA on UART1,
 * NAV-PVT and NAV-STATUS streaming at 1 Hz, unused NMEA sentences off.
 * Blocking, takes ~1.5 s (waits for the module to boot, then ACKs).
 * Returns false if the module never acknowledged anything. */
bool gps_init(void);

/* Optional: keep receiving NMEA lines (for your existing minmea code).
 * Must be set BEFORE gps_init(), because gps_init() decides whether to leave
 * the RMC sentence enabled. Pass NULL (or don't call it) to run UBX-only. */
void gps_set_nmea_callback(ubx_nmea_cb_t cb);

/* Pump RX and run the poll scheduler. Non-blocking. Call every loop. */
void gps_service(void);

/* Copy out the current snapshot. Never blocks, never fails. */
void gps_get(gps_status_t *out);

/* Tell the driver whether the diagnostic window is on screen.
 * true  = poll NAV-SAT and MON-RF every 2 s (costs bandwidth)
 * false = don't (default) */
void gps_diag_active(bool active);

/* Force an immediate refresh of the expensive telemetry. Non-blocking:
 * the answers land in the snapshot a few hundred ms later. */
void gps_refresh(void);

/* ─────────────── control ─────────────── */

/* Antenna / LNA bias.
 *
 * On the MAX-M10S the internal LNA and SAW are always in the RF path and are
 * NOT switchable. What this controls is LNA_EN (pin 13), which is intended to
 * bias an EXTERNAL active antenna or LNA. If your antenna is passive, this is
 * a no-op electrically and gps_status_t.ant_* will stay UNKNOWN. That is
 * expected, not a bug.
 *
 * If you gate the antenna bias with a host GPIO instead (recommended, see
 * gps_hw_antenna_hook below), this also drives that. */
bool gps_antenna_enable(bool on);

/* Internal LNA gain: 0 = normal, 1 = low gain (module default), 2 = bypass.
 * Bypass only makes sense with a high-gain external LNA. Blocking, ~50 ms. */
bool gps_lna_mode(uint8_t mode);

/* Optional host-side antenna/LNA power switch. If you wire the antenna bias
 * behind a GPIO + load switch, register the setter here and gps_antenna_enable()
 * will drive it alongside the UBX config. Pass NULL to disable.
 * Register this AFTER gps_init(), which clears internal state. */
typedef void (*gps_hw_antenna_hook_t)(bool on);
void gps_set_hw_antenna_hook(gps_hw_antenna_hook_t hook);

/* Enable the antenna supervisor. ONLY call this if you have built the external
 * current-sense / open-detect hardware and wired it to module PIOs. Without it
 * the module will report bogus SHORT/OPEN states. Off by default. */
bool gps_antenna_supervisor_enable(uint8_t switch_pio, uint8_t short_pio, uint8_t open_pio);

/* GNSS engine on/off. The UART link stays alive, config is retained, the
 * receiver answers polls. This is the cheap "pause GPS" for the UI.
 * Drops from ~9.5 mA to roughly the standby figure while staying responsive. */
bool gps_engine_stop(void);
bool gps_engine_start(void);

/* Software standby (RXM-PMREQ). ~46 uA at 3.3 V V_IO, 120 nA on VCC.
 * The link goes dead. duration_ms = 0 means indefinite.
 * Pair with gps_wake(). Use this when the watch itself goes dormant. */
void gps_standby(uint32_t duration_ms);
void gps_wake(void);   /* blocking, ~250 ms while the module reboots from BBR */

/* Power save modes. PSMCT (cyclic) roughly halves tracking current.
 * Note: BeiDou B1C is not supported in power save mode. */
bool gps_power_mode(gps_power_t mode);   /* FULL / PSMOO / PSMCT only */

/* Restarts. Cold start throws away almanac and ephemeris: TTFF goes back to
 * ~27 s. Only expose this behind a "reset GPS" confirmation in the UI. */
void gps_hot_start(void);
void gps_warm_start(void);
void gps_cold_start(void);

/* Constellations. Fewer = less current, worse TTFF and urban accuracy.
 * Constraint from the datasheet: BDS B1I cannot coexist with GLONASS.
 * SBAS and QZSS require GPS. Blocking, ~100 ms, forces an internal restart. */
bool gps_constellations(bool gps, bool galileo, bool beidou, bool glonass,
                        bool sbas, bool qzss);

/* Navigation rate. meas_ms 1000 = 1 Hz. Above 1 Hz you must raise the baud. */
bool gps_nav_rate(uint16_t meas_ms);

/* Baud rate. Reconfigures the module then the Pico UART. Blocking, ~200 ms. */
bool gps_baudrate(uint32_t baud);

/* ─────────────── calibration ─────────────── */

/* Record the current AGC as the "healthy, antenna connected, sky visible"
 * baseline. Call once on the bench with a good sky view and store the result
 * in your config. Without this, rf_health is a coarse guess. */
void gps_rf_calibrate(void);
uint16_t gps_rf_baseline(void);
void gps_rf_set_baseline(uint16_t agc);

/* ─────────────── display helpers ─────────────── */

const char *gps_search_str(gps_search_t s);   /* "SEARCHING", "3D FIX", ... */
const char *gps_power_str(gps_power_t p);     /* "FULL", "STANDBY", ... */
const char *gps_ant_str(gps_ant_t a);         /* "OK", "SHORT", "N/A", ... */
const char *gps_jam_str_ui(gps_jam_t j);
const char *gps_rf_health_str(gps_rf_health_t h);

/* 0..100 signal-quality bar for the UI, derived from C/N0 and satellite count. */
uint8_t gps_signal_bar(const gps_status_t *st);

#endif /* GPS_STATUS_H */
