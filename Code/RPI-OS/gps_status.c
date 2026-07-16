/*
 * gps_status.c - High-level GPS telemetry + control for the watch OS.
 */

#include "gps_status.h"
#include <string.h>
#include "pico/stdlib.h"

/* ─────────────── internal state ─────────────── */

static struct {
    gps_power_t  power;
    bool         gnss_running;
    bool         ant_bias_enabled;
    bool         ant_supervised;
    uint8_t      lna_mode;

    bool         diag_active;
    absolute_time_t next_rf;
    absolute_time_t next_sat;

    uint32_t     rf_stamp_ms;
    uint32_t     sat_stamp_ms;
    uint32_t     pvt_stamp_ms;
    uint32_t     last_itow;
    uint32_t     last_msss;

    uint16_t     agc_baseline;   /* 0 = uncalibrated */

    gps_hw_antenna_hook_t hw_ant;
} G;

#define RF_POLL_MS   2000
#define SAT_POLL_MS  2000
#define LINK_DEAD_MS 3000

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

/* ─────────────── NMEA passthrough ─────────────── */
/* We stream NAV-PVT, so NMEA is not needed for the status window. Kept as a
 * hook so your existing minmea code can still run if you want it. */
static ubx_nmea_cb_t user_nmea_cb = NULL;
void gps_set_nmea_callback(ubx_nmea_cb_t cb) { user_nmea_cb = cb; }
static void nmea_trampoline(const char *line) { if (user_nmea_cb) user_nmea_cb(line); }

/* ─────────────── init ─────────────── */

bool gps_init(void)
{
    memset(&G, 0, sizeof(G));
    G.power            = GPS_PWR_FULL;
    G.gnss_running     = true;
    G.ant_bias_enabled = false;   /* module boots with LNA_EN high, but we have
                                     not commanded anything, so report false
                                     until gps_antenna_enable() is called */
    G.lna_mode         = 1;       /* datasheet default: low gain */
    G.next_rf          = get_absolute_time();
    G.next_sat         = get_absolute_time();

    ubx_init(nmea_trampoline);

    /* The module needs about a second after power-on before it parses UBX. */
    sleep_ms(1200);

    /* Flush whatever it sent during boot. */
    ubx_service();

    if (!ubx_enable_ubx_output()) return false;

    ubx_cfg_begin(UBX_LAYER_PERSIST);
    /* Stream the two cheap ones. */
    ubx_cfg_add_u1(CFG_MSGOUT_UBX_NAV_PVT_UART1,    1);
    ubx_cfg_add_u1(CFG_MSGOUT_UBX_NAV_STATUS_UART1, 1);
    /* NAV-SAT and MON-RF are polled on demand, not streamed. */
    ubx_cfg_add_u1(CFG_MSGOUT_UBX_NAV_SAT_UART1, 0);
    ubx_cfg_add_u1(CFG_MSGOUT_UBX_MON_RF_UART1,  0);
    /* Kill the NMEA we do not parse. Keeps 9600 baud breathable. */
    ubx_cfg_add_u1(CFG_MSGOUT_NMEA_GSV_UART1, 0);
    ubx_cfg_add_u1(CFG_MSGOUT_NMEA_GLL_UART1, 0);
    ubx_cfg_add_u1(CFG_MSGOUT_NMEA_VTG_UART1, 0);
    ubx_cfg_add_u1(CFG_MSGOUT_NMEA_GSA_UART1, 0);
    ubx_cfg_add_u1(CFG_MSGOUT_NMEA_GGA_UART1, 0);
    ubx_cfg_add_u1(CFG_MSGOUT_NMEA_RMC_UART1, user_nmea_cb ? 1 : 0);
    if (!ubx_cfg_commit(1000)) return false;

    ubx_set_nav_rate(1000, 1);
    return true;
}

/* ─────────────── service ─────────────── */

void gps_diag_active(bool active)
{
    G.diag_active = active;
    if (active) {
        /* Poll immediately so the window is populated by the time it renders. */
        G.next_rf  = get_absolute_time();
        G.next_sat = delayed_by_ms(get_absolute_time(), 400);
    }
}

void gps_refresh(void)
{
    G.next_rf  = get_absolute_time();
    G.next_sat = delayed_by_ms(get_absolute_time(), 400);
}

void gps_service(void)
{
    ubx_service();

    /* Note freshness of the streamed messages. */
    if (ubx_pvt.valid && ubx_pvt.iTOW != G.last_itow) {
        G.last_itow   = ubx_pvt.iTOW;
        G.pvt_stamp_ms = now_ms();
    }
    if (ubx_status.valid && ubx_status.msss_ms != G.last_msss) {
        G.last_msss = ubx_status.msss_ms;
    }
    if (ubx_rf.valid && G.rf_stamp_ms == 0) G.rf_stamp_ms = now_ms();

    if (!G.diag_active) return;
    if (G.power == GPS_PWR_STANDBY || G.power == GPS_PWR_HW_OFF) return;

    /* Stagger the two expensive polls so they never collide on the wire. */
    if (time_reached(G.next_rf)) {
        ubx_poll_rf();
        G.rf_stamp_ms = now_ms();
        G.next_rf = make_timeout_time_ms(RF_POLL_MS);
    }
    if (time_reached(G.next_sat)) {
        ubx_poll_sat();
        G.sat_stamp_ms = now_ms();
        G.next_sat = make_timeout_time_ms(SAT_POLL_MS);
    }
}

/* ─────────────── derivation ─────────────── */

static gps_ant_t map_ant(uint8_t raw)
{
    switch (raw) {
        case 0:  return GPS_ANT_INIT;
        case 2:  return GPS_ANT_OK;
        case 3:  return GPS_ANT_SHORT;
        case 4:  return GPS_ANT_OPEN;
        default: return GPS_ANT_UNKNOWN;   /* 1 = DONTKNOW */
    }
}

static gps_rf_health_t derive_rf_health(uint16_t agc, uint8_t cno_best, uint8_t sv_in_view)
{
    if (!ubx_rf.valid) return GPS_RF_HEALTH_UNKNOWN;

    /* AGC runs 0..8191. Pegged near the top means the front end is cranking
     * gain and finding nothing: antenna disconnected, or shielded. Pegged near
     * the bottom means it is backing off hard: strong in-band interference.
     *
     * These thresholds are deliberately conservative. Run gps_rf_calibrate()
     * on the bench with a good sky view and store the baseline; then the
     * comparison is against your actual hardware, not a guess. */
    const uint16_t HIGH = G.agc_baseline ? (uint16_t)(G.agc_baseline + 1500) : 7200;
    const uint16_t LOW  = G.agc_baseline ? (uint16_t)(G.agc_baseline > 1500
                                                      ? G.agc_baseline - 1500 : 200)
                                         : 800;

    if (agc >= HIGH && cno_best < 20 && sv_in_view == 0) return GPS_RF_HEALTH_WEAK;
    if (agc <= LOW) return GPS_RF_HEALTH_SATURATED;
    if (cno_best >= 25) return GPS_RF_HEALTH_GOOD;
    if (agc >= HIGH) return GPS_RF_HEALTH_WEAK;
    return GPS_RF_HEALTH_UNKNOWN;
}

void gps_get(gps_status_t *st)
{
    memset(st, 0, sizeof(*st));

    uint32_t t = now_ms();
    uint32_t last = ubx_last_rx_ms();

    st->link_age_ms = last ? (t - last) : 0xFFFFFFFFu;
    st->link_up     = (last != 0) && (st->link_age_ms < LINK_DEAD_MS);

    /* ── power ── */
    st->power        = G.power;
    st->gnss_running = G.gnss_running;

    /* ── satellites ── */
    uint32_t cno_sum = 0;
    uint8_t  cno_n   = 0;
    if (ubx_sats.valid) {
        st->sv_in_view = ubx_sats.numSvs;
        for (uint8_t i = 0; i < ubx_sats.numSvs; i++) {
            const ubx_sat_t *s = &ubx_sats.sat[i];
            if (s->used) st->sv_used++;
            if (s->qualityInd == 1) st->sv_searching++;
            if (s->qualityInd >= 4) st->sv_locked++;
            if (s->cno > 0) {
                cno_sum += s->cno;
                cno_n++;
                if (s->cno > st->cno_best) st->cno_best = s->cno;
            }
        }
        if (cno_n) st->cno_avg = (uint8_t)(cno_sum / cno_n);
    }

    /* ── fix / PVT ── */
    if (ubx_pvt.valid) {
        st->fix_type   = ubx_pvt.fixType;
        st->fix_ok     = ubx_pvt.gnssFixOK;
        st->pdop_1e2   = ubx_pvt.pDOP_1e2;
        st->hacc_mm    = ubx_pvt.hAcc_mm;
        st->time_valid = (ubx_pvt.timeValidFlags & 0x04) != 0;  /* fullyResolved */
        st->year  = ubx_pvt.year;
        st->month = ubx_pvt.month;
        st->day   = ubx_pvt.day;
        st->hour  = ubx_pvt.hour;
        st->min   = ubx_pvt.min;
        st->sec   = ubx_pvt.sec;
        st->lat_1e7    = ubx_pvt.lat_1e7;
        st->lon_1e7    = ubx_pvt.lon_1e7;
        st->alt_msl_mm = ubx_pvt.hMSL_mm;
        /* NAV-PVT numSV is authoritative for "used"; NAV-SAT may be stale. */
        if (ubx_pvt.numSV) st->sv_used = ubx_pvt.numSV;
    }

    if (ubx_status.valid) {
        st->ttff_ms   = ubx_status.ttff_ms;
        st->uptime_ms = ubx_status.msss_ms;
    }

    /* ── search state ── */
    if (!st->link_up) {
        st->search = GPS_SEARCH_OFFLINE;
    } else if (!G.gnss_running ||
               G.power == GPS_PWR_ENGINE_STOPPED ||
               G.power == GPS_PWR_STANDBY) {
        st->search = GPS_SEARCH_IDLE;
    } else if (st->fix_ok && st->fix_type == 3) {
        st->search = GPS_SEARCH_FIX_3D;
    } else if (st->fix_ok && st->fix_type == 2) {
        st->search = GPS_SEARCH_FIX_2D;
    } else if (st->sv_locked > 0) {
        st->search = GPS_SEARCH_ACQUIRING;
    } else if (st->sv_in_view > 0 || st->sv_searching > 0) {
        st->search = GPS_SEARCH_SEARCHING;
    } else {
        st->search = GPS_SEARCH_COLD;
    }

    /* ── antenna / RF ── */
    st->ant_supervised   = G.ant_supervised;
    st->ant_bias_enabled = G.ant_bias_enabled;
    st->lna_mode         = G.lna_mode;

    if (ubx_rf.valid) {
        st->agc     = ubx_rf.agcCnt;
        st->noise   = ubx_rf.noisePerMS;
        st->jam_ind = ubx_rf.jamInd;
        switch (ubx_rf.jammingState) {
            case 1:  st->jamming = GPS_RF_OK;       break;
            case 2:  st->jamming = GPS_RF_WARNING;  break;
            case 3:  st->jamming = GPS_RF_CRITICAL; break;
            default: st->jamming = GPS_RF_UNKNOWN;  break;
        }
        /* Only trust antStatus if we actually enabled the supervisor. */
        st->ant_status  = G.ant_supervised ? map_ant(ubx_rf.antStatus) : GPS_ANT_UNKNOWN;
        st->ant_powered = G.ant_supervised ? (ubx_rf.antPower == 1) : G.ant_bias_enabled;
    } else {
        st->ant_status = GPS_ANT_UNKNOWN;
        st->jamming    = GPS_RF_UNKNOWN;
    }

    st->rf_health = derive_rf_health(st->agc, st->cno_best, st->sv_in_view);

    /* ── freshness ── */
    st->pvt_age_ms = G.pvt_stamp_ms ? (t - G.pvt_stamp_ms) : 0xFFFFFFFFu;
    st->rf_age_ms  = G.rf_stamp_ms  ? (t - G.rf_stamp_ms)  : 0xFFFFFFFFu;
    st->sat_age_ms = G.sat_stamp_ms ? (t - G.sat_stamp_ms) : 0xFFFFFFFFu;
}

/* ─────────────── control ─────────────── */

void gps_set_hw_antenna_hook(gps_hw_antenna_hook_t hook) { G.hw_ant = hook; }

bool gps_antenna_enable(bool on)
{
    if (G.hw_ant) G.hw_ant(on);
    bool ok = ubx_cfg_set_l(CFG_HW_ANT_CFG_VOLTCTRL, on, UBX_LAYER_PERSIST);
    if (ok || G.hw_ant) G.ant_bias_enabled = on;
    return ok;
}

bool gps_lna_mode(uint8_t mode)
{
    if (mode > 2) return false;
    bool ok = ubx_lna_mode(mode);
    if (ok) G.lna_mode = mode;
    return ok;
}

bool gps_antenna_supervisor_enable(uint8_t switch_pio, uint8_t short_pio, uint8_t open_pio)
{
    ubx_cfg_begin(UBX_LAYER_PERSIST);
    ubx_cfg_add_l (CFG_HW_ANT_CFG_VOLTCTRL,   true);
    ubx_cfg_add_l (CFG_HW_ANT_CFG_SHORTDET,   true);
    ubx_cfg_add_l (CFG_HW_ANT_CFG_OPENDET,    true);
    ubx_cfg_add_l (CFG_HW_ANT_CFG_PWRDOWN,    true);
    ubx_cfg_add_l (CFG_HW_ANT_CFG_RECOVER,    true);
    ubx_cfg_add_u1(CFG_HW_ANT_SUP_SWITCH_PIN, switch_pio);
    ubx_cfg_add_u1(CFG_HW_ANT_SUP_SHORT_PIN,  short_pio);
    ubx_cfg_add_u1(CFG_HW_ANT_SUP_OPEN_PIN,   open_pio);
    bool ok = ubx_cfg_commit(1000);
    if (ok) { G.ant_supervised = true; G.ant_bias_enabled = true; }
    return ok;
}

bool gps_engine_stop(void)
{
    ubx_gnss_stop();
    G.gnss_running = false;
    G.power = GPS_PWR_ENGINE_STOPPED;
    return true;
}

bool gps_engine_start(void)
{
    ubx_gnss_start();
    G.gnss_running = true;
    G.power = GPS_PWR_FULL;
    /* Fix data is now stale. */
    ubx_pvt.valid = false;
    ubx_sats.valid = false;
    return true;
}

void gps_standby(uint32_t duration_ms)
{
    ubx_software_standby(duration_ms, true, false);   /* wake on UART RX */
    G.power = GPS_PWR_STANDBY;
    G.gnss_running = false;
    ubx_pvt.valid = false;
    ubx_sats.valid = false;
    ubx_rf.valid = false;
}

void gps_wake(void)
{
    ubx_wake();                 /* blocking ~250 ms */
    G.power = GPS_PWR_FULL;
    G.gnss_running = true;
    G.rf_stamp_ms = G.sat_stamp_ms = G.pvt_stamp_ms = 0;
    gps_refresh();
}

bool gps_power_mode(gps_power_t mode)
{
    uint8_t v;
    switch (mode) {
        case GPS_PWR_FULL:  v = 0; break;
        case GPS_PWR_PSMOO: v = 1; break;
        case GPS_PWR_PSMCT: v = 2; break;
        default: return false;      /* the other states are not CFG-PM modes */
    }
    bool ok = ubx_cfg_set_u1(CFG_PM_OPERATEMODE, v, UBX_LAYER_PERSIST);
    if (ok) { G.power = mode; G.gnss_running = true; }
    return ok;
}

void gps_hot_start(void)  { ubx_hot_start();  G.gnss_running = true; G.power = GPS_PWR_FULL; }
void gps_warm_start(void) { ubx_warm_start(); G.gnss_running = true; G.power = GPS_PWR_FULL; }
void gps_cold_start(void)
{
    ubx_cold_start();
    G.gnss_running = true;
    G.power = GPS_PWR_FULL;
    ubx_pvt.valid = false;
    ubx_sats.valid = false;
    ubx_status.valid = false;
}

bool gps_constellations(bool gps, bool gal, bool bds, bool glo, bool sbas, bool qzss)
{
    /* Datasheet constraints, enforced here so the UI cannot produce a NAK. */
    if (bds && glo) return false;          /* BDS B1I and GLONASS L1OF are exclusive */
    if ((sbas || qzss) && !gps) return false;

    ubx_cfg_begin(UBX_LAYER_PERSIST);
    ubx_cfg_add_l(CFG_SIGNAL_GPS_ENA,  gps);
    ubx_cfg_add_l(CFG_SIGNAL_GAL_ENA,  gal);
    ubx_cfg_add_l(CFG_SIGNAL_BDS_ENA,  bds);
    ubx_cfg_add_l(CFG_SIGNAL_GLO_ENA,  glo);
    ubx_cfg_add_l(CFG_SIGNAL_SBAS_ENA, sbas);
    ubx_cfg_add_l(CFG_SIGNAL_QZSS_ENA, qzss);
    return ubx_cfg_commit(2000);           /* forces an internal restart, be patient */
}

bool gps_nav_rate(uint16_t meas_ms)  { return ubx_set_nav_rate(meas_ms, 1); }
bool gps_baudrate(uint32_t baud)     { return ubx_set_baudrate(baud); }

/* ─────────────── calibration ─────────────── */

void gps_rf_calibrate(void)     { if (ubx_rf.valid) G.agc_baseline = ubx_rf.agcCnt; }
uint16_t gps_rf_baseline(void)  { return G.agc_baseline; }
void gps_rf_set_baseline(uint16_t agc) { G.agc_baseline = agc; }

/* ─────────────── display helpers ─────────────── */

const char *gps_search_str(gps_search_t s)
{
    switch (s) {
        case GPS_SEARCH_OFFLINE:   return "OFFLINE";
        case GPS_SEARCH_IDLE:      return "IDLE";
        case GPS_SEARCH_COLD:      return "COLD";
        case GPS_SEARCH_SEARCHING: return "SEARCHING";
        case GPS_SEARCH_ACQUIRING: return "ACQUIRING";
        case GPS_SEARCH_FIX_2D:    return "2D FIX";
        case GPS_SEARCH_FIX_3D:    return "3D FIX";
        default: return "?";
    }
}

const char *gps_power_str(gps_power_t p)
{
    switch (p) {
        case GPS_PWR_FULL:            return "FULL";
        case GPS_PWR_PSMOO:           return "SAVE";
        case GPS_PWR_PSMCT:           return "CYCLIC";
        case GPS_PWR_ENGINE_STOPPED:  return "STOPPED";
        case GPS_PWR_STANDBY:         return "STANDBY";
        case GPS_PWR_HW_OFF:          return "OFF";
        default: return "?";
    }
}

const char *gps_ant_str(gps_ant_t a)
{
    switch (a) {
        case GPS_ANT_OK:    return "OK";
        case GPS_ANT_SHORT: return "SHORT";
        case GPS_ANT_OPEN:  return "OPEN";
        case GPS_ANT_INIT:  return "INIT";
        default: return "N/A";     /* supervisor not wired: say so, do not lie */
    }
}

const char *gps_jam_str_ui(gps_jam_t j)
{
    switch (j) {
        case GPS_RF_OK:       return "CLEAR";
        case GPS_RF_WARNING:  return "WARN";
        case GPS_RF_CRITICAL: return "JAMMED";
        default: return "N/A";
    }
}

const char *gps_rf_health_str(gps_rf_health_t h)
{
    switch (h) {
        case GPS_RF_HEALTH_GOOD:      return "GOOD";
        case GPS_RF_HEALTH_WEAK:      return "NO SIGNAL";
        case GPS_RF_HEALTH_SATURATED: return "OVERLOAD";
        default: return "UNKNOWN";
    }
}

uint8_t gps_signal_bar(const gps_status_t *st)
{
    if (!st->link_up || st->sv_in_view == 0) return 0;

    /* C/N0 of 20 dBHz is barely trackable, 45 is excellent. */
    uint8_t cno = st->cno_best;
    uint32_t q  = (cno <= 20) ? 0 : (cno >= 45 ? 100 : (uint32_t)(cno - 20) * 4);

    /* Weight by how many satellites are actually usable. */
    uint32_t n = st->sv_used >= 8 ? 100 : (uint32_t)st->sv_used * 12;

    return (uint8_t)((q * 2 + n) / 3);
}
