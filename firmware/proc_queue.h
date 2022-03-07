#ifndef PRIO_QUEUE_H
#define PRIO_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

typedef void *(*pfn_proc_t)(void *);
typedef uint8_t prio_t;
typedef uint32_t opcode_t;

/* To notify the user what functions are to be used in the queue */
#define __prio_queue

#define PRIORITY_LEVEL_HIGHEST 0
#define PRIORITY_LEVEL_LOWEST  1

#define NUM_PRIO 2
#define MAX_QUEUE 32

typedef struct {
    pfn_proc_t proc_fn;
    void *pfn_args;
} proc_info;

typedef struct {
    proc_info pi[MAX_QUEUE];
    uint8_t head;
    uint8_t tail;
} proc_queue;

void proc_init(void);
bool proc_enqueue(pfn_proc_t proc_fn, void *pfn_args, uint8_t prio);
proc_info *proc_dequeue(proc_queue *q);
proc_info *proc_next(void);

#endif
