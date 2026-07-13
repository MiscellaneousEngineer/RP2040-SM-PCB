#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// ---- Pins (drive L298N IN1 / IN2) ----
#define PIN_A 2u   // GP2 -> IN1 -> OUT1 -> base of Q1
#define PIN_B 3u   // GP3 -> IN2 -> OUT2 -> base of Q2

// ---- Editable timing (adjust during bring-up) ----
static const uint32_t DRIVE_HZ    = 60;    // raise a few Hz if the core shows the saturation hook
static const uint32_t DEADTIME_US = 300;   // both-off gap; covers BJT storage time

// ---- Derived per-cycle timing ----
#define PERIOD_US (1000000UL / DRIVE_HZ)        // 16666 us at 60 Hz
#define HALF_US   (PERIOD_US / 2)               // 8333 us
#define ON_US     (HALF_US - DEADTIME_US)       // 8033 us drive per phase

// ---- Burst cadence ----
static const uint32_t BURST_ON_MS  = 6000;
static const uint32_t BURST_OFF_MS = 6000;

// ---- Waveform state ----
typedef enum { PH_A, PH_DEAD1, PH_B, PH_DEAD2 } Phase;
static volatile Phase      phase;
static volatile bool       waveActive     = false;
static volatile bool       stopAtBoundary = false;
static alarm_id_t          waveAlarm      = 0;

static inline void bothLow(void) {
    gpio_put(PIN_A, 0);
    gpio_put(PIN_B, 0);
}

// Fires at the END of the current phase. Applies the next phase and
// returns its duration in microseconds. A negative return tells the SDK
// to reschedule relative to the last scheduled time, which is drift-free.
static int64_t wave_cb(alarm_id_t id, void *user) {
    (void)id; (void)user;
    switch (phase) {
        case PH_A:                       // A done -> dead
            bothLow();
            phase = PH_DEAD1;
            return -(int64_t)DEADTIME_US;

        case PH_DEAD1:                    // dead done -> B on
            gpio_put(PIN_A, 0);
            gpio_put(PIN_B, 1);
            phase = PH_B;
            return -(int64_t)ON_US;

        case PH_B:                        // B done -> dead
            bothLow();
            phase = PH_DEAD2;
            return -(int64_t)DEADTIME_US;

        case PH_DEAD2:                    // full cycle complete: stop here if asked
            if (stopAtBoundary) {
                bothLow();
                waveActive = false;
                waveAlarm  = 0;
                return 0;                 // both off, cycle balanced, end alarm
            }
            gpio_put(PIN_B, 0);
            gpio_put(PIN_A, 1);
            phase = PH_A;
            return -(int64_t)ON_US;
    }
    return 0;
}

void startWaveform(void) {
    if (waveActive) return;
    stopAtBoundary = false;
    waveActive = true;
    bothLow();
    gpio_put(PIN_A, 1);                   // begin on phase A
    phase = PH_A;
    // fire_if_past = true so a missed deadline still runs promptly
    waveAlarm = add_alarm_in_us(ON_US, wave_cb, NULL, true);
}

// Clean stop: honored only at the next both-off boundary, so the
// transformer always ends a burst at zero net flux.
void requestStop(void) {
    stopAtBoundary = true;
}

// Hard off: forces both transistors off immediately, regardless of state.
void hardOff(void) {
    if (waveAlarm) { cancel_alarm(waveAlarm); waveAlarm = 0; }
    waveActive = false;
    bothLow();
}

static void gpio_setup(void) {
    gpio_init(PIN_A); gpio_set_dir(PIN_A, GPIO_OUT); gpio_put(PIN_A, 0);
    gpio_init(PIN_B); gpio_set_dir(PIN_B, GPIO_OUT); gpio_put(PIN_B, 0);
}

int main(void) {
    stdio_init_all();
    gpio_setup();
    sleep_ms(100);                        // let external base pulldowns hold off

    bool          burstOn    = true;
    absolute_time_t burstMark = get_absolute_time();
    startWaveform();

    while (true) {
        if (burstOn) {
            if (absolute_time_diff_us(burstMark, get_absolute_time())
                    >= (int64_t)BURST_ON_MS * 1000) {
                requestStop();
                burstOn   = false;
                burstMark = get_absolute_time();
            }
        } else {
            if (absolute_time_diff_us(burstMark, get_absolute_time())
                    >= (int64_t)BURST_OFF_MS * 1000) {
                startWaveform();
                burstOn   = true;
                burstMark = get_absolute_time();
            }
        }
        tight_loop_contents();
    }
}