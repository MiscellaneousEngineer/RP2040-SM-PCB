/*
 * ubx.c - UBX protocol driver for u-blox MAX-M10S on RP2040 (Pico SDK)
 */

#include "ubx.h"
#include <string.h>
#include "pico/stdlib.h"

ubx_pvt_t     ubx_pvt;
ubx_status_t  ubx_status;
ubx_rf_t      ubx_rf;
ubx_satlist_t ubx_sats;

/* ─────────────── little-endian readers ─────────────── */
static inline uint8_t  rdu1(const uint8_t *p, int o) { return p[o]; }
static inline int8_t   rdi1(const uint8_t *p, int o) { return (int8_t)p[o]; }
static inline uint16_t rdu2(const uint8_t *p, int o) { return (uint16_t)(p[o] | (p[o+1] << 8)); }
static inline int16_t  rdi2(const uint8_t *p, int o) { return (int16_t)rdu2(p, o); }
static inline uint32_t rdu4(const uint8_t *p, int o) {
    return (uint32_t)p[o] | ((uint32_t)p[o+1] << 8) |
           ((uint32_t)p[o+2] << 16) | ((uint32_t)p[o+3] << 24);
}
static inline int32_t  rdi4(const uint8_t *p, int o) { return (int32_t)rdu4(p, o); }

/* ─────────────── RX state machine ─────────────── */
enum { S_IDLE, S_SYNC2, S_CLS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CKA, S_CKB, S_NMEA };

static struct {
    int      st;
    uint8_t  cls, id;
    uint16_t len, idx;
    uint8_t  payload[UBX_MAX_PAYLOAD];
    uint8_t  cka, ckb;
    bool     overflow;

    char     nmea[UBX_NMEA_LINE_MAX];
    size_t   nmea_pos;

    ubx_nmea_cb_t nmea_cb;

    /* last-seen bookkeeping for the blocking waiters */
    volatile bool    ack_seen;
    volatile uint8_t ack_cls, ack_id;
    volatile bool    ack_ok;
    volatile uint8_t last_cls, last_id;
    volatile bool    msg_seen;
} R;

void ubx_init(ubx_nmea_cb_t nmea_cb)
{
    memset(&R, 0, sizeof(R));
    R.st = S_IDLE;
    R.nmea_cb = nmea_cb;
    memset(&ubx_pvt, 0, sizeof(ubx_pvt));
    memset(&ubx_status, 0, sizeof(ubx_status));
    memset(&ubx_rf, 0, sizeof(ubx_rf));
    memset(&ubx_sats, 0, sizeof(ubx_sats));
}

/* ─────────────── TX ─────────────── */
static void ck_acc(uint8_t b, uint8_t *a, uint8_t *c) { *a = (uint8_t)(*a + b); *c = (uint8_t)(*c + *a); }

void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t a = 0, c = 0;
    uint8_t hdr[6] = { 0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };

    ck_acc(cls, &a, &c);
    ck_acc(id, &a, &c);
    ck_acc(hdr[4], &a, &c);
    ck_acc(hdr[5], &a, &c);
    for (uint16_t i = 0; i < len; i++) ck_acc(payload[i], &a, &c);

    uart_write_blocking(UBX_UART, hdr, 6);
    if (len) uart_write_blocking(UBX_UART, payload, len);
    uint8_t ck[2] = { a, c };
    uart_write_blocking(UBX_UART, ck, 2);
}

void ubx_poll(uint8_t cls, uint8_t id) { ubx_send(cls, id, NULL, 0); }

/* ─────────────── decoders ─────────────── */
static void decode_nav_pvt(const uint8_t *p, uint16_t len)
{
    if (len < 92) return;
    ubx_pvt.valid          = true;
    ubx_pvt.iTOW           = rdu4(p, 0);
    ubx_pvt.year           = rdu2(p, 4);
    ubx_pvt.month          = rdu1(p, 6);
    ubx_pvt.day            = rdu1(p, 7);
    ubx_pvt.hour           = rdu1(p, 8);
    ubx_pvt.min            = rdu1(p, 9);
    ubx_pvt.sec            = rdu1(p, 10);
    ubx_pvt.timeValidFlags = rdu1(p, 11);
    ubx_pvt.fixType        = rdu1(p, 20);
    ubx_pvt.gnssFixOK      = (rdu1(p, 21) & 0x01) != 0;
    ubx_pvt.numSV          = rdu1(p, 23);
    ubx_pvt.lon_1e7        = rdi4(p, 24);
    ubx_pvt.lat_1e7        = rdi4(p, 28);
    ubx_pvt.height_mm      = rdi4(p, 32);
    ubx_pvt.hMSL_mm        = rdi4(p, 36);
    ubx_pvt.hAcc_mm        = rdu4(p, 40);
    ubx_pvt.vAcc_mm        = rdu4(p, 44);
    ubx_pvt.gSpeed_mms     = rdi4(p, 60);
    ubx_pvt.pDOP_1e2       = rdu2(p, 76);
}

static void decode_nav_status(const uint8_t *p, uint16_t len)
{
    if (len < 16) return;
    uint8_t flags  = rdu1(p, 5);
    uint8_t flags2 = rdu1(p, 7);
    ubx_status.valid         = true;
    ubx_status.gpsFix        = rdu1(p, 4);
    ubx_status.gpsFixOk      = (flags & 0x01) != 0;
    ubx_status.diffSoln      = (flags & 0x02) != 0;
    ubx_status.spoofDetState = (uint8_t)((flags2 >> 3) & 0x03);
    ubx_status.ttff_ms       = rdu4(p, 8);
    ubx_status.msss_ms       = rdu4(p, 12);
}

static void decode_mon_rf(const uint8_t *p, uint16_t len)
{
    if (len < 4) return;
    uint8_t nBlocks = rdu1(p, 1);
    if (nBlocks < 1 || len < (uint16_t)(4 + 24)) return;
    const uint8_t *b = p + 4;               /* block 0 = the L1 RF path */
    ubx_rf.valid        = true;
    ubx_rf.jammingState = (uint8_t)(rdu1(b, 1) & 0x03);
    ubx_rf.antStatus    = rdu1(b, 2);
    ubx_rf.antPower     = rdu1(b, 3);
    ubx_rf.postStatus   = rdu4(b, 4);
    ubx_rf.noisePerMS   = rdu2(b, 12);
    ubx_rf.agcCnt       = rdu2(b, 14);
    ubx_rf.jamInd       = rdu1(b, 16);
}

static void decode_nav_sat(const uint8_t *p, uint16_t len)
{
    if (len < 8) return;
    uint8_t n = rdu1(p, 5);
    if (n > 32) n = 32;
    if (len < (uint16_t)(8 + 12 * n)) n = (uint8_t)((len - 8) / 12);
    ubx_sats.valid  = true;
    ubx_sats.numSvs = n;
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t *s = p + 8 + 12 * i;
        uint32_t f = rdu4(s, 8);
        ubx_sats.sat[i].gnssId     = rdu1(s, 0);
        ubx_sats.sat[i].svId       = rdu1(s, 1);
        ubx_sats.sat[i].cno        = rdu1(s, 2);
        ubx_sats.sat[i].elev       = rdi1(s, 3);
        ubx_sats.sat[i].azim       = rdi2(s, 4);
        ubx_sats.sat[i].qualityInd = (uint8_t)(f & 0x07);
        ubx_sats.sat[i].used       = (f & 0x08) != 0;
    }
}

static volatile uint32_t g_last_rx_ms = 0;
uint32_t ubx_last_rx_ms(void) { return g_last_rx_ms; }

static void dispatch(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len)
{
    R.last_cls = cls;
    R.last_id  = id;
    R.msg_seen = true;
    g_last_rx_ms = to_ms_since_boot(get_absolute_time());

    if (cls == UBX_CLASS_ACK) {
        R.ack_seen = true;
        R.ack_ok   = (id == UBX_ACK_ACK);
        R.ack_cls  = len >= 2 ? p[0] : 0;
        R.ack_id   = len >= 2 ? p[1] : 0;
        return;
    }
    if (cls == UBX_CLASS_NAV) {
        switch (id) {
            case UBX_NAV_PVT:    decode_nav_pvt(p, len);    break;
            case UBX_NAV_STATUS: decode_nav_status(p, len); break;
            case UBX_NAV_SAT:    decode_nav_sat(p, len);    break;
            default: break;
        }
        return;
    }
    if (cls == UBX_CLASS_MON && id == UBX_MON_RF) decode_mon_rf(p, len);
}

/* ─────────────── RX pump ─────────────── */
static void feed(uint8_t c)
{
    switch (R.st) {
    case S_IDLE:
        if (c == 0xB5) { R.st = S_SYNC2; }
        else if (c == '$') { R.st = S_NMEA; R.nmea_pos = 0; R.nmea[R.nmea_pos++] = '$'; }
        break;

    case S_SYNC2:
        R.st = (c == 0x62) ? S_CLS : S_IDLE;
        break;

    case S_CLS:
        R.cls = c; R.cka = c; R.ckb = c; R.st = S_ID;
        break;

    case S_ID:
        R.id = c; ck_acc(c, &R.cka, &R.ckb); R.st = S_LEN1;
        break;

    case S_LEN1:
        R.len = c; ck_acc(c, &R.cka, &R.ckb); R.st = S_LEN2;
        break;

    case S_LEN2:
        R.len |= (uint16_t)c << 8;
        ck_acc(c, &R.cka, &R.ckb);
        R.idx = 0;
        R.overflow = (R.len > UBX_MAX_PAYLOAD);
        R.st = R.len ? S_PAYLOAD : S_CKA;
        break;

    case S_PAYLOAD:
        ck_acc(c, &R.cka, &R.ckb);
        if (!R.overflow) R.payload[R.idx] = c;
        if (++R.idx >= R.len) R.st = S_CKA;
        break;

    case S_CKA:
        R.st = (c == R.cka) ? S_CKB : S_IDLE;
        break;

    case S_CKB:
        if (c == R.ckb && !R.overflow) dispatch(R.cls, R.id, R.payload, R.len);
        R.st = S_IDLE;
        break;

    case S_NMEA:
        if (c == '\n') {
            R.nmea[R.nmea_pos] = '\0';
            g_last_rx_ms = to_ms_since_boot(get_absolute_time());
            if (R.nmea_cb) R.nmea_cb(R.nmea);
            R.st = S_IDLE;
        } else if (c == '\r') {
            /* skip */
        } else if (R.nmea_pos < sizeof(R.nmea) - 1) {
            R.nmea[R.nmea_pos++] = (char)c;
        } else {
            R.st = S_IDLE;   /* runaway line, drop it */
        }
        break;
    }
}

void ubx_service(void)
{
    while (uart_is_readable(UBX_UART)) feed(uart_getc(UBX_UART));
}

/* ─────────────── blocking waiters ─────────────── */
bool ubx_wait_ack(uint8_t cls, uint8_t id, uint32_t timeout_ms)
{
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    R.ack_seen = false;
    while (!time_reached(deadline)) {
        ubx_service();
        if (R.ack_seen) {
            if (R.ack_cls == cls && R.ack_id == id) return R.ack_ok;
            R.ack_seen = false;   /* ACK for something else, keep waiting */
        }
        tight_loop_contents();
    }
    return false;
}

bool ubx_wait_msg(uint8_t cls, uint8_t id, uint32_t timeout_ms)
{
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    R.msg_seen = false;
    while (!time_reached(deadline)) {
        ubx_service();
        if (R.msg_seen) {
            if (R.last_cls == cls && R.last_id == id) return true;
            R.msg_seen = false;
        }
        tight_loop_contents();
    }
    return false;
}

/* ─────────────── CFG-VALSET ─────────────── */
static uint8_t  vs_buf[UBX_MAX_PAYLOAD];
static uint16_t vs_len;

void ubx_cfg_begin(uint8_t layers)
{
    vs_buf[0] = 0x00;      /* version 0 (no transactions) */
    vs_buf[1] = layers;
    vs_buf[2] = 0x00;
    vs_buf[3] = 0x00;
    vs_len = 4;
}

static void vs_key(uint32_t key)
{
    vs_buf[vs_len++] = (uint8_t)(key      );
    vs_buf[vs_len++] = (uint8_t)(key >>  8);
    vs_buf[vs_len++] = (uint8_t)(key >> 16);
    vs_buf[vs_len++] = (uint8_t)(key >> 24);
}

void ubx_cfg_add_u1(uint32_t key, uint8_t val)
{
    if (vs_len + 5 > UBX_MAX_PAYLOAD) return;
    vs_key(key); vs_buf[vs_len++] = val;
}
void ubx_cfg_add_l(uint32_t key, bool val) { ubx_cfg_add_u1(key, val ? 1 : 0); }

void ubx_cfg_add_u2(uint32_t key, uint16_t val)
{
    if (vs_len + 6 > UBX_MAX_PAYLOAD) return;
    vs_key(key);
    vs_buf[vs_len++] = (uint8_t)(val);
    vs_buf[vs_len++] = (uint8_t)(val >> 8);
}

void ubx_cfg_add_u4(uint32_t key, uint32_t val)
{
    if (vs_len + 8 > UBX_MAX_PAYLOAD) return;
    vs_key(key);
    vs_buf[vs_len++] = (uint8_t)(val      );
    vs_buf[vs_len++] = (uint8_t)(val >>  8);
    vs_buf[vs_len++] = (uint8_t)(val >> 16);
    vs_buf[vs_len++] = (uint8_t)(val >> 24);
}

bool ubx_cfg_commit(uint32_t timeout_ms)
{
    ubx_send(UBX_CLASS_CFG, UBX_CFG_VALSET, vs_buf, vs_len);
    return ubx_wait_ack(UBX_CLASS_CFG, UBX_CFG_VALSET, timeout_ms);
}

bool ubx_cfg_set_u1(uint32_t k, uint8_t v, uint8_t l)  { ubx_cfg_begin(l); ubx_cfg_add_u1(k, v); return ubx_cfg_commit(1000); }
bool ubx_cfg_set_u2(uint32_t k, uint16_t v, uint8_t l) { ubx_cfg_begin(l); ubx_cfg_add_u2(k, v); return ubx_cfg_commit(1000); }
bool ubx_cfg_set_u4(uint32_t k, uint32_t v, uint8_t l) { ubx_cfg_begin(l); ubx_cfg_add_u4(k, v); return ubx_cfg_commit(1000); }
bool ubx_cfg_set_l (uint32_t k, bool v, uint8_t l)     { ubx_cfg_begin(l); ubx_cfg_add_l (k, v); return ubx_cfg_commit(1000); }

void ubx_cfg_get(uint32_t key, uint8_t layer)
{
    uint8_t p[8] = {
        0x00, layer, 0x00, 0x00,
        (uint8_t)(key), (uint8_t)(key >> 8), (uint8_t)(key >> 16), (uint8_t)(key >> 24)
    };
    ubx_send(UBX_CLASS_CFG, UBX_CFG_VALGET, p, sizeof(p));
}

/* ─────────────── power / lifecycle ─────────────── */
void ubx_reset(uint16_t bbrMask, uint8_t resetMode)
{
    uint8_t p[4] = { (uint8_t)(bbrMask), (uint8_t)(bbrMask >> 8), resetMode, 0x00 };
    ubx_send(UBX_CLASS_CFG, UBX_CFG_RST, p, sizeof(p));
    /* CFG-RST is never acknowledged. Do not wait for an ACK. */
}

bool ubx_gnss_stop(void)
{
    ubx_reset(UBX_BBR_HOT, UBX_RST_GNSS_STOP);
    sleep_ms(50);
    return true;
}

bool ubx_gnss_start(void)
{
    ubx_reset(UBX_BBR_HOT, UBX_RST_GNSS_START);
    sleep_ms(50);
    return true;
}

void ubx_hot_start(void)  { ubx_reset(UBX_BBR_HOT,  UBX_RST_SW); }
void ubx_warm_start(void) { ubx_reset(UBX_BBR_WARM, UBX_RST_SW); }
void ubx_cold_start(void) { ubx_reset(UBX_BBR_COLD, UBX_RST_SW); }

void ubx_software_standby(uint32_t duration_ms, bool wake_uart, bool wake_extint0)
{
    uint32_t flags = (1u << 1);       /* backup  */
    flags |= (1u << 2);               /* force: enter regardless of USB link */
    uint32_t wakeup = 0;
    if (wake_uart)    wakeup |= (1u << 3);   /* UARTRX */
    if (wake_extint0) wakeup |= (1u << 5);   /* EXTINT0 */

    uint8_t p[16];
    memset(p, 0, sizeof(p));
    p[0] = 0x00;                       /* version 0 */
    p[4] = (uint8_t)(duration_ms      );
    p[5] = (uint8_t)(duration_ms >>  8);
    p[6] = (uint8_t)(duration_ms >> 16);
    p[7] = (uint8_t)(duration_ms >> 24);
    p[8]  = (uint8_t)(flags      );
    p[9]  = (uint8_t)(flags >>  8);
    p[10] = (uint8_t)(flags >> 16);
    p[11] = (uint8_t)(flags >> 24);
    p[12] = (uint8_t)(wakeup      );
    p[13] = (uint8_t)(wakeup >>  8);
    p[14] = (uint8_t)(wakeup >> 16);
    p[15] = (uint8_t)(wakeup >> 24);

    ubx_send(UBX_CLASS_RXM, UBX_RXM_PMREQ, p, sizeof(p));
    /* PMREQ is not acknowledged. */
}

void ubx_wake(void)
{
    uint8_t junk = 0xFF;                /* any edge on RXD wakes it */
    uart_write_blocking(UBX_UART, &junk, 1);
    sleep_ms(20);
    uart_write_blocking(UBX_UART, &junk, 1);
    sleep_ms(200);                       /* receiver reboots from BBR */
}

/* ─────────────── antenna / LNA ─────────────── */
bool ubx_antenna_supervisor(bool voltctrl, bool shortdet, bool opendet,
                            bool pwrdown_on_short, bool auto_recover)
{
    ubx_cfg_begin(UBX_LAYER_PERSIST);
    ubx_cfg_add_l(CFG_HW_ANT_CFG_VOLTCTRL, voltctrl);
    ubx_cfg_add_l(CFG_HW_ANT_CFG_SHORTDET, shortdet);
    ubx_cfg_add_l(CFG_HW_ANT_CFG_OPENDET,  opendet);
    ubx_cfg_add_l(CFG_HW_ANT_CFG_PWRDOWN,  pwrdown_on_short);
    ubx_cfg_add_l(CFG_HW_ANT_CFG_RECOVER,  auto_recover);
    return ubx_cfg_commit(1000);
}

bool ubx_antenna_power(bool on)
{
    /* With VOLTCTRL enabled the receiver drives LNA_EN high; disabling it
     * releases the pin and kills the active-antenna bias. */
    return ubx_cfg_set_l(CFG_HW_ANT_CFG_VOLTCTRL, on, UBX_LAYER_PERSIST);
}

bool ubx_lna_mode(uint8_t mode)
{
    return ubx_cfg_set_u1(CFG_HW_RF_LNA_MODE, mode, UBX_LAYER_PERSIST);
}

/* ─────────────── polls ─────────────── */
void ubx_poll_pvt(void)    { ubx_poll(UBX_CLASS_NAV, UBX_NAV_PVT); }
void ubx_poll_status(void) { ubx_poll(UBX_CLASS_NAV, UBX_NAV_STATUS); }
void ubx_poll_rf(void)     { ubx_poll(UBX_CLASS_MON, UBX_MON_RF); }
void ubx_poll_sat(void)    { ubx_poll(UBX_CLASS_NAV, UBX_NAV_SAT); }
void ubx_poll_ver(void)    { ubx_poll(UBX_CLASS_MON, UBX_MON_VER); }

/* ─────────────── convenience ─────────────── */
bool ubx_enable_ubx_output(void)
{
    ubx_cfg_begin(UBX_LAYER_PERSIST);
    ubx_cfg_add_l(CFG_UART1INPROT_UBX,  true);
    ubx_cfg_add_l(CFG_UART1OUTPROT_UBX, true);
    ubx_cfg_add_l(CFG_UART1INPROT_NMEA, true);
    ubx_cfg_add_l(CFG_UART1OUTPROT_NMEA, true);
    return ubx_cfg_commit(1000);
}

bool ubx_set_nav_rate(uint16_t meas_ms, uint16_t nav_ratio)
{
    ubx_cfg_begin(UBX_LAYER_PERSIST);
    ubx_cfg_add_u2(CFG_RATE_MEAS, meas_ms);
    ubx_cfg_add_u2(CFG_RATE_NAV,  nav_ratio);
    return ubx_cfg_commit(1000);
}

bool ubx_set_baudrate(uint32_t baud)
{
    /* The ACK comes back at the OLD baud rate, then the port switches. */
    bool ok = ubx_cfg_set_u4(CFG_UART1_BAUDRATE, baud, UBX_LAYER_PERSIST);
    sleep_ms(100);
    uart_set_baudrate(UBX_UART, baud);
    sleep_ms(100);
    return ok;
}

const char *ubx_fix_str(uint8_t f)
{
    switch (f) {
        case 0: return "no fix";
        case 1: return "dead reckoning";
        case 2: return "2D";
        case 3: return "3D";
        case 4: return "GNSS+DR";
        case 5: return "time only";
        default: return "?";
    }
}

const char *ubx_ant_status_str(uint8_t s)
{
    switch (s) {
        case 0: return "INIT";
        case 1: return "UNKNOWN";
        case 2: return "OK";
        case 3: return "SHORT";
        case 4: return "OPEN";
        default: return "?";
    }
}

const char *ubx_jam_str(uint8_t s)
{
    switch (s) {
        case 0: return "unknown/disabled";
        case 1: return "ok";
        case 2: return "warning";
        case 3: return "critical";
        default: return "?";
    }
}
