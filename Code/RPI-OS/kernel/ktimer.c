//This is the kernel timer - a non blocking timer system that stores any timer event requested and flags when the timer is complete ( thus triggering the relevant event / allowing a loop to execute)

#include "ktimer.h"
#include "evetq.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "pico/time.h"

typedef struct {
    bool     in_use;
    bool     repeat;
    uint64_t deadline_us;    /* absolute, on the 64-bit us timer */
    uint32_t period_us;
    uint8_t  evt;
    uint8_t  arg;
} ktimer_t;

static ktimer_t timers[KTIMER_MAX];
static int      alarm_num = -1;

/* All of the following run with the alarm IRQ masked (either in the ISR
   itself, or inside a save_and_disable_interrupts region). */

static uint64_t earliest_deadline(void)
{
    uint64_t best = UINT64_MAX;
    for (int i = 0; i < KTIMER_MAX; i++)
        if (timers[i].in_use && timers[i].deadline_us < best)
            best = timers[i].deadline_us;
    return best;
}

static void fire_due(uint64_t now) //check due alarms and fire events. let apps handle them once created
{
    for (int i = 0; i < KTIMER_MAX; i++) {
        if (!timers[i].in_use)          continue;
        if (timers[i].deadline_us > now) continue;

        event_t e = { .type = timers[i].evt, .a = timers[i].arg, .b = 0, .data = 0 };
        evtq_push(&e);

        if (timers[i].repeat) {
            /* Drift-free: advance from the OLD deadline, not from 'now'.
               If we fell behind by more than one period, skip whole periods
               so the timer stays phase-aligned instead of firing a burst. */
            do {
                timers[i].deadline_us += timers[i].period_us;
            } while (timers[i].deadline_us <= now);
        } else {
            timers[i].in_use = false;
        }
    }
}

static void reprogram(void)
{
    for (;;) {
        uint64_t next = earliest_deadline();
        if (next == UINT64_MAX) {                 /* nothing armed */
            hardware_alarm_cancel(alarm_num);
            return;
        }
        /* Returns true if 'next' is already in the past (alarm NOT set). */
        if (!hardware_alarm_set_target(alarm_num, from_us_since_boot(next)))
            return;                               /* armed for the future */
        /* Target passed while we were computing it: service and retry. */
        fire_due(time_us_64());
    }
}

static void ktimer_isr(uint alarm)
{
    (void)alarm;
    fire_due(time_us_64());
    reprogram();
}

void ktimer_init(void)
{
    for (int i = 0; i < KTIMER_MAX; i++) timers[i].in_use = false;
    alarm_num = hardware_alarm_claim_unused(true);
    hardware_alarm_set_callback(alarm_num, ktimer_isr);
}

int ktimer_start(uint32_t period_ms, bool repeat, evt_type_t evt, uint8_t arg)
{
    uint32_t save = save_and_disable_interrupts();   /* guard vs the alarm ISR */

    int id = -1;
    for (int i = 0; i < KTIMER_MAX; i++)
        if (!timers[i].in_use) { id = i; break; }

    if (id >= 0) {
        uint64_t now = time_us_64();
        timers[id].in_use      = true;
        timers[id].repeat      = repeat;
        timers[id].period_us   = period_ms * 1000u;
        timers[id].deadline_us = now + (uint64_t)period_ms * 1000u;
        timers[id].evt         = (uint8_t)evt;
        timers[id].arg         = arg;
        reprogram();
    }

    restore_interrupts(save);
    return id;
}

void ktimer_stop(int id)
{
    if (id < 0 || id >= KTIMER_MAX) return;
    uint32_t save = save_and_disable_interrupts();
    timers[id].in_use = false;
    reprogram();
    restore_interrupts(save);
}