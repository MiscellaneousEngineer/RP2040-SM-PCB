//These are the event queuqe definitions

#ifndef EVETQ_H
#define EVETQ_H

#include <stdbool.h>
#include "event.h"

void evtq_init(void);
bool evtq_push(const event_t *e);   /* safe from ISR and either core; false if full  */
bool evtq_pop(event_t *e);          /* core 0 kernel loop only;       false if empty */
bool evtq_empty(void);
#endif