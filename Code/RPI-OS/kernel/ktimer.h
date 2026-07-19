#ifndef KTIMER_H
#define KTIMER_H
#include <stdint.h>
#include <stdbool.h>
#include "event.h"

#define KTIMER_MAX 16

void ktimer_init(void);

/* Fire event {evt, arg} after period_ms, then every period_ms if repeat.
   Returns a timer id >= 0, or -1 if every slot is in use. */
int  ktimer_start(uint32_t period_ms, bool repeat, evt_type_t evt, uint8_t arg);

/* Cancel a timer. Safe on an already-stopped or invalid id. */
void ktimer_stop(int id);
#endif