/*
 * ubx.h - UBX protocol driver for u-blox MAX-M10S on RP2040 (Pico SDK)
 *
 * Coexists with NMEA: feed the RX byte stream into ubx_service(), which
 * demultiplexes UBX frames (0xB5 0x62 ...) from NMEA lines ($.....\r\n)
 * and hands the NMEA lines to your existing minmea code.
 *
 * NOTE ON KEY IDs: the 32-bit configuration keys below are taken from the
 * u-blox M10 SPG 5.10 Interface description (UBX-21035062). Cross-check any
 * key you rely on against that document before shipping; a wrong key just
 * produces a UBX-ACK-NAK, which ubx_wait_ack() will report.
 */

#ifndef UBX_H
#define UBX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hardware/uart.h"

/* ─────────────── build-time wiring ─────────────── */
#ifndef UBX_UART
#define UBX_UART        uart0
#endif
#ifndef UBX_MAX_PAYLOAD
#define UBX_MAX_PAYLOAD 256      /* NAV-SAT with many SVs can exceed this; raise if needed */
#endif
#ifndef UBX_NMEA_LINE_MAX
#define UBX_NMEA_LINE_MAX 128    /* NMEA 4.11 sentences fit in 82 bytes incl. CRLF */
#endif

/* ─────────────── message classes / IDs ─────────────── */
#define UBX_CLASS_NAV   0x01
#define UBX_CLASS_RXM   0x02
#define UBX_CLASS_INF   0x04
#define UBX_CLASS_ACK   0x05
#define UBX_CLASS_CFG   0x06
#define UBX_CLASS_MON   0x0A
#define UBX_CLASS_TIM   0x0D

#define UBX_ACK_NAK     0x00
#define UBX_ACK_ACK     0x01

#define UBX_CFG_RST     0x04
#define UBX_CFG_CFG     0x09
#define UBX_CFG_VALSET  0x8A
#define UBX_CFG_VALGET  0x8B
#define UBX_CFG_VALDEL  0x8C

#define UBX_RXM_PMREQ   0x41

#define UBX_NAV_PVT     0x07
#define UBX_NAV_STATUS  0x03
#define UBX_NAV_DOP     0x04
#define UBX_NAV_SAT     0x35
#define UBX_NAV_SIG     0x43
#define UBX_NAV_TIMEUTC 0x21
#define UBX_NAV_TIMEGPS 0x20
#define UBX_NAV_CLOCK   0x22

#define UBX_MON_VER     0x04
#define UBX_MON_HW      0x09   /* deprecated on M10, use MON-RF */
#define UBX_MON_GNSS    0x28
#define UBX_MON_COMMS   0x36
#define UBX_MON_RF      0x38
#define UBX_MON_SPAN    0x31

/* ─────────────── VALSET/VALGET layers ─────────────── */
#define UBX_LAYER_RAM   0x01
#define UBX_LAYER_BBR   0x02
#define UBX_LAYER_FLASH 0x04   /* MAX-M10S has no flash: will NAK */
#define UBX_LAYER_DEFAULT_GET 0x07  /* VALGET only: read factory default */

/* Persist across power cycles only if V_BCKP is held up. */
#define UBX_LAYER_PERSIST (UBX_LAYER_RAM | UBX_LAYER_BBR)

/* ─────────────── configuration keys ─────────────── *
 * Encoding of the key tells you the storage size:
 *   0x1... = L  (1 byte bool)   0x2... = U1/E1/X1
 *   0x3... = U2/X2              0x4... = U4/X4/I4
 */

/* UART1 port */
#define CFG_UART1_BAUDRATE        0x40520001u  /* U4 */
#define CFG_UART1_STOPBITS        0x20520002u  /* E1 */
#define CFG_UART1_DATABITS        0x20520003u  /* E1 */
#define CFG_UART1_PARITY          0x20520004u  /* E1 */
#define CFG_UART1_ENABLED         0x10520005u  /* L  */
#define CFG_UART1INPROT_UBX       0x10730001u  /* L  */
#define CFG_UART1INPROT_NMEA      0x10730002u  /* L  */
#define CFG_UART1INPROT_RTCM3X    0x10730004u  /* L  */
#define CFG_UART1OUTPROT_UBX      0x10740001u  /* L  */
#define CFG_UART1OUTPROT_NMEA     0x10740002u  /* L  */

/* Navigation rate */
#define CFG_RATE_MEAS             0x30210001u  /* U2, ms between measurements */
#define CFG_RATE_NAV              0x30210002u  /* U2, measurements per solution */
#define CFG_RATE_TIMEREF          0x20210003u  /* E1, 0=UTC 1=GPS 2=GLO 3=BDS 4=GAL */

/* NMEA message output rate on UART1 (0 = off, N = every Nth nav solution) */
#define CFG_MSGOUT_NMEA_DTM_UART1 0x209100a7u
#define CFG_MSGOUT_NMEA_GBS_UART1 0x209100deu
#define CFG_MSGOUT_NMEA_GGA_UART1 0x209100bbu
#define CFG_MSGOUT_NMEA_GLL_UART1 0x209100cau
#define CFG_MSGOUT_NMEA_GNS_UART1 0x209100b6u
#define CFG_MSGOUT_NMEA_GRS_UART1 0x209100cfu
#define CFG_MSGOUT_NMEA_GSA_UART1 0x209100c0u
#define CFG_MSGOUT_NMEA_GST_UART1 0x209100d4u
#define CFG_MSGOUT_NMEA_GSV_UART1 0x209100c5u
#define CFG_MSGOUT_NMEA_RMC_UART1 0x209100acu
#define CFG_MSGOUT_NMEA_VLW_UART1 0x209100e8u
#define CFG_MSGOUT_NMEA_VTG_UART1 0x209100b1u
#define CFG_MSGOUT_NMEA_ZDA_UART1 0x209100d9u

/* UBX message output rate on UART1 */
#define CFG_MSGOUT_UBX_NAV_PVT_UART1     0x20910007u
#define CFG_MSGOUT_UBX_NAV_STATUS_UART1  0x2091001bu
#define CFG_MSGOUT_UBX_NAV_DOP_UART1     0x20910039u
#define CFG_MSGOUT_UBX_NAV_SAT_UART1     0x20910016u
#define CFG_MSGOUT_UBX_NAV_SIG_UART1     0x20910346u
#define CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1 0x2091005cu
#define CFG_MSGOUT_UBX_MON_RF_UART1      0x2091035au
#define CFG_MSGOUT_UBX_MON_COMMS_UART1   0x2091034fu

/* Navigation engine */
#define CFG_NAVSPG_DYNMODEL       0x20110021u  /* E1: 0 portable, 2 stationary, 3 pedestrian,
                                                  4 automotive, 5 sea, 6..8 airborne, 9 wrist */
#define CFG_NAVSPG_FIXMODE        0x20110011u  /* E1: 1=2D only, 2=3D only, 3=auto */
#define CFG_NAVSPG_INFIL_MINELEV  0x201100a4u  /* I1, degrees */
#define CFG_NAVSPG_INFIL_MINCNO   0x201100a6u  /* U1, dBHz */

/* Constellations (a change here forces a receiver restart internally) */
#define CFG_SIGNAL_GPS_ENA        0x1031001fu
#define CFG_SIGNAL_GPS_L1CA_ENA   0x10310001u
#define CFG_SIGNAL_SBAS_ENA       0x10310020u
#define CFG_SIGNAL_GAL_ENA        0x10310021u
#define CFG_SIGNAL_BDS_ENA        0x10310022u
#define CFG_SIGNAL_QZSS_ENA       0x10310024u
#define CFG_SIGNAL_GLO_ENA        0x10310025u

/* Power management */
#define CFG_PM_OPERATEMODE        0x20d00001u  /* E1: 0=full, 1=PSMOO, 2=PSMCT */
#define CFG_PM_POSUPDATEPERIOD    0x40d00002u  /* U4, s (PSMOO) */
#define CFG_PM_ACQPERIOD          0x40d00003u  /* U4, s */
#define CFG_PM_GRIDOFFSET         0x40d00004u  /* U4, s */
#define CFG_PM_ONTIME             0x30d00005u  /* U2, s */
#define CFG_PM_MINACQTIME         0x20d00006u  /* U1, s */
#define CFG_PM_MAXACQTIME         0x20d00010u  /* U1, s */
#define CFG_PM_EXTINTWAKE         0x10d0000cu  /* L  */
#define CFG_PM_EXTINTBACKUP       0x10d0000du  /* L  */
#define CFG_PM_EXTINTINACTIVE     0x10d0000eu  /* L  */
#define CFG_PM_LIMITPEAKCURR      0x10d0000fu  /* L  */
#define CFG_PM_WAITTIMEFIX        0x10d00009u  /* L  */
#define CFG_PM_UPDATEEPH          0x10d0000au  /* L  */
#define CFG_PM_DONOTENTEROFF      0x10d00008u  /* L  */

/* Hardware / antenna supervisor + LNA */
#define CFG_HW_ANT_CFG_VOLTCTRL   0x10a3002eu  /* L: enable active antenna voltage control */
#define CFG_HW_ANT_CFG_SHORTDET   0x10a3002fu  /* L */
#define CFG_HW_ANT_CFG_SHORTDET_POL 0x10a30030u
#define CFG_HW_ANT_CFG_OPENDET    0x10a30031u  /* L */
#define CFG_HW_ANT_CFG_OPENDET_POL 0x10a30032u
#define CFG_HW_ANT_CFG_PWRDOWN    0x10a30033u  /* L: power down antenna on short */
#define CFG_HW_ANT_CFG_PWRDOWN_POL 0x10a30034u
#define CFG_HW_ANT_CFG_RECOVER    0x10a30035u  /* L: auto recovery from short */
#define CFG_HW_ANT_SUP_SWITCH_PIN 0x20a30036u  /* U1: PIO used as LNA_EN */
#define CFG_HW_ANT_SUP_SHORT_PIN  0x20a30037u  /* U1 */
#define CFG_HW_ANT_SUP_OPEN_PIN   0x20a30038u  /* U1 */
#define CFG_HW_RF_LNA_MODE        0x20a30057u  /* E1: 0=normal, 1=low gain, 2=bypass */

/* Time pulse */
#define CFG_TP_PULSE_DEF          0x20050023u  /* E1: 0=period, 1=freq */
#define CFG_TP_TP1_ENA            0x10050007u  /* L */
#define CFG_TP_PERIOD_TP1         0x40050002u  /* U4, us */
#define CFG_TP_LEN_TP1            0x40050004u  /* U4, us */
#define CFG_TP_PERIOD_LOCK_TP1    0x40050003u  /* U4, us, used once locked */
#define CFG_TP_LEN_LOCK_TP1       0x40050005u  /* U4, us */
#define CFG_TP_USE_LOCKED_TP1     0x1005000au  /* L */

/* ─────────────── CFG-RST reset modes ─────────────── */
#define UBX_RST_HW_IMMEDIATE      0x00
#define UBX_RST_SW                0x01
#define UBX_RST_SW_GNSS_ONLY      0x02
#define UBX_RST_HW_AFTER_SHUTDOWN 0x04
#define UBX_RST_GNSS_STOP         0x08   /* stop the GNSS engine, keep host link alive */
#define UBX_RST_GNSS_START        0x09   /* restart it */

#define UBX_BBR_HOT               0x0000
#define UBX_BBR_WARM              0x0001
#define UBX_BBR_COLD              0xFFFF

/* ─────────────── decoded telemetry ─────────────── */
typedef struct {
    bool     valid;
    uint32_t iTOW;
    uint8_t  fixType;      /* 0 none, 1 dead-reck, 2 2D, 3 3D, 4 GNSS+DR, 5 time only */
    bool     gnssFixOK;
    uint8_t  numSV;
    int32_t  lon_1e7, lat_1e7;
    int32_t  height_mm, hMSL_mm;
    uint32_t hAcc_mm, vAcc_mm;
    int32_t  gSpeed_mms;
    uint16_t year; uint8_t month, day, hour, min, sec;
    uint8_t  timeValidFlags;   /* bit0 validDate, bit1 validTime, bit2 fullyResolved */
    uint16_t pDOP_1e2;
} ubx_pvt_t;

typedef struct {
    bool     valid;
    uint8_t  gpsFix;
    bool     gpsFixOk;
    bool     diffSoln;
    uint8_t  spoofDetState;
    uint32_t ttff_ms;    /* time to first fix */
    uint32_t msss_ms;    /* ms since startup */
} ubx_status_t;

typedef struct {
    bool     valid;
    uint8_t  antStatus;    /* 0 INIT, 1 DONTKNOW, 2 OK, 3 SHORT, 4 OPEN */
    uint8_t  antPower;     /* 0 OFF, 1 ON, 2 DONTKNOW */
    uint8_t  jammingState; /* 0 unknown, 1 ok, 2 warning, 3 critical */
    uint8_t  jamInd;       /* 0..255 CW jamming indicator */
    uint16_t noisePerMS;
    uint16_t agcCnt;       /* 0..8191 */
    uint32_t postStatus;
} ubx_rf_t;

typedef struct {
    uint8_t gnssId, svId, cno;
    int8_t  elev;
    int16_t azim;
    bool    used;
    uint8_t qualityInd;   /* 0 no signal .. 7 code+carrier locked */
} ubx_sat_t;

typedef struct {
    bool      valid;
    uint8_t   numSvs;
    ubx_sat_t sat[32];
} ubx_satlist_t;

/* Latest decoded telemetry, updated by ubx_service(). */
extern ubx_pvt_t     ubx_pvt;
extern ubx_status_t  ubx_status;
extern ubx_rf_t      ubx_rf;
extern ubx_satlist_t ubx_sats;

/* ─────────────── API ─────────────── */

/* Called for each complete NMEA line (no CR/LF, NUL-terminated). */
typedef void (*ubx_nmea_cb_t)(const char *line);
void ubx_init(ubx_nmea_cb_t nmea_cb);

/* Pump the UART. Call every loop iteration. Non-blocking. */
void ubx_service(void);

/* ms-since-boot of the last valid frame received (0 = nothing ever received). */
uint32_t ubx_last_rx_ms(void);

/* Raw frame TX. */
void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
void ubx_poll(uint8_t cls, uint8_t id);          /* zero-length request */

/* Blocking wait for ACK-ACK/ACK-NAK matching cls/id. Keeps servicing RX. */
bool ubx_wait_ack(uint8_t cls, uint8_t id, uint32_t timeout_ms);

/* Blocking wait for a specific message to arrive (e.g. MON-RF after a poll). */
bool ubx_wait_msg(uint8_t cls, uint8_t id, uint32_t timeout_ms);

/* CFG-VALSET helpers. layers = UBX_LAYER_* bitmask. Return true on ACK. */
bool ubx_cfg_set_u1(uint32_t key, uint8_t  val, uint8_t layers);
bool ubx_cfg_set_u2(uint32_t key, uint16_t val, uint8_t layers);
bool ubx_cfg_set_u4(uint32_t key, uint32_t val, uint8_t layers);
bool ubx_cfg_set_l (uint32_t key, bool     val, uint8_t layers);

/* Batch VALSET: build then commit. Fewer round-trips, atomic apply. */
void ubx_cfg_begin(uint8_t layers);
void ubx_cfg_add_u1(uint32_t key, uint8_t  val);
void ubx_cfg_add_u2(uint32_t key, uint16_t val);
void ubx_cfg_add_u4(uint32_t key, uint32_t val);
void ubx_cfg_add_l (uint32_t key, bool     val);
bool ubx_cfg_commit(uint32_t timeout_ms);

/* CFG-VALGET: request one key; result arrives as a CFG-VALGET response. */
void ubx_cfg_get(uint32_t key, uint8_t layer);

/* ── power / lifecycle ── */
bool ubx_gnss_stop(void);    /* CFG-RST 0x08: engine off, UART still answers */
bool ubx_gnss_start(void);   /* CFG-RST 0x09 */
void ubx_reset(uint16_t bbrMask, uint8_t resetMode); /* no ACK is sent for CFG-RST */
void ubx_hot_start(void);
void ubx_warm_start(void);
void ubx_cold_start(void);

/* RXM-PMREQ: software standby. duration_ms 0 = indefinite.
 * wake_uart: wake on any activity on RXD. wake_extint0: wake on EXTINT rising. */
void ubx_software_standby(uint32_t duration_ms, bool wake_uart, bool wake_extint0);
void ubx_wake(void);         /* send a dummy byte to tickle RXD */

/* ── antenna / LNA ── */
bool ubx_antenna_supervisor(bool enable_voltctrl, bool shortdet, bool opendet,
                            bool pwrdown_on_short, bool auto_recover);
bool ubx_antenna_power(bool on);   /* drives LNA_EN via the supervisor */
bool ubx_lna_mode(uint8_t mode);   /* 0 normal, 1 low gain, 2 bypass */

/* ── telemetry polls ── */
void ubx_poll_pvt(void);
void ubx_poll_status(void);
void ubx_poll_rf(void);
void ubx_poll_sat(void);
void ubx_poll_ver(void);

/* ── convenience ── */
bool ubx_set_baudrate(uint32_t baud);     /* reconfigures GPS then the Pico UART */
bool ubx_set_nav_rate(uint16_t meas_ms, uint16_t nav_ratio);
bool ubx_enable_ubx_output(void);         /* turn on UBX in/out on UART1 */

const char *ubx_fix_str(uint8_t fixType);
const char *ubx_ant_status_str(uint8_t s);
const char *ubx_jam_str(uint8_t s);

#endif /* UBX_H */
