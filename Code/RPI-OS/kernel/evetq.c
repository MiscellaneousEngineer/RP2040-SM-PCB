//This is the event queque handler - it stores events that happen ("flags" that get popped) and makes sure they get to the correct function

#include "evetq.h"
#include "pico/sync.h"       /* spin locks */

#define EVTQ_CAPACITY 64u                 /* must be a power of two to avoid uneccesary divisions*/
#define EVTQ_MASK     (EVTQ_CAPACITY - 1u)

static event_t          buf[EVTQ_CAPACITY];
static volatile uint32_t head;            /* next write slot (free-running) */
static volatile uint32_t tail;            /* next read slot  (free-running) */
static spin_lock_t      *lock;

void evtq_init(void)
{
    head = tail = 0;
    lock = spin_lock_instance(spin_lock_claim_unused(true));
}

bool evtq_push(const event_t *e)
{
    bool ok = false;
    uint32_t save = spin_lock_blocking(lock);   /* disables IRQs here, takes the lock */
    if ((head - tail) < EVTQ_CAPACITY) {        /* not full */
        buf[head & EVTQ_MASK] = *e;
        head++;
        ok = true;
    }
    spin_unlock(lock, save);                    /* releases lock, restores IRQs */
    return ok;
}

bool evtq_pop(event_t *e)
{
    bool ok = false;
    uint32_t save = spin_lock_blocking(lock);
    if (head != tail) {                         /* not empty */
        *e = buf[tail & EVTQ_MASK];
        tail++;
        ok = true;
    }
    spin_unlock(lock, save);
    return ok;
}

bool evtq_empty(void)
{
    return head == tail;                        /* unlocked read is fine, see below */
}