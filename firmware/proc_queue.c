#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "proc_queue.h"
#include "utils.h"

/*
 * Theory of operation:
 * The user will provide a function, arguments for the function, and a priority
 * for the function.
 * The arguments should be within scope at any time, e.g. global or on heap.
 *
 * The user is expected to add items to the queue when an interrupt fires.
 * e.g. DMA transfer finishes and now needs to be processed.
 *
 * When the code is done being executed, the user should call proc_free to
 * return memory.
 *
 * Right now this is only set up for single-core operation.
 */

/*
 * Predefine a number of processing queues and let this code handle the head
 * and tail.
 * Storing the tail isn't strictly necessary, but it saves time when enqueueing
 */
proc_queue queues[NUM_PRIO];

proc_queue *queue_p0 = &queues[0];
proc_queue *queue_p1 = &queues[1];

void proc_init(void)
{
    uint8_t i;
    proc_queue *q;

    for (i = 0; i < ARRAY_SIZE(queues); ++i) {
        q = &queues[i];

        q->head = 0;
        q->tail = 0;
    }
}

static proc_info *proc_next_open(proc_queue *q)
{
    return &q->pi[q->tail];
}

static proc_info *proc_next_dequeue(proc_queue *q)
{
    return &q->pi[q->head];
}

static void proc_inc_and_rollover(uint8_t *val, uint8_t max)
{
    if (*val == max) {
        *val = 0;
    } else {
        ++*val;
    }
}

static bool proc_queue_empty(const proc_queue *q)
{
    return (q->head == q->tail);
}

bool proc_enqueue(pfn_proc_t proc_fn, void *pfn_args, uint8_t prio)
{
    DBG_ASSERT(prio < ARRAY_SIZE(queues));

    proc_queue *q = &queues[prio];
    proc_info *node = proc_next_open(q);

    node->proc_fn = proc_fn;
    node->pfn_args = pfn_args;

    proc_inc_and_rollover(&q->tail, ARRAY_SIZE(q->pi) - 1);

    return true;
}

proc_info *proc_dequeue(proc_queue *q)
{
    if (proc_queue_empty(q)) {
        return NULL;
    }

    proc_info *node = proc_next_dequeue(q);

    proc_inc_and_rollover(&q->head, ARRAY_SIZE(q->pi) - 1);

    return node;
}

/*
 * Finds the next thing to execute from the queues.
 * Execution order is from lowest to highest priority.
 *
 * A NULL return value is valid, it means there's nothing to execute.
 */
proc_info *proc_next(void)
{
    uint8_t i;
    proc_queue *q;
    proc_info *proc;

    for (i = 0; i < ARRAY_SIZE(queues); ++i) {
        q = &queues[i];
        proc = proc_dequeue(q);
        if (proc) {
            return proc;
        }
    }

    return NULL;
}
