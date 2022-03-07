#ifndef UTILS_H
#define UTILS_H

#include <assert.h>

#include "hardware/gpio.h"
#include "arm_utils.h"

#define ERR_DEBUG 0

/* To tell the user what's an IRQ handler */
#define __irq_handler

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*x))


#define DB_PRINT_L(level, ...) do { \
        if (ERR_DEBUG > (level)) { \
                    printf(": %s: ", __func__); \
                    printf(__VA_ARGS__); \
                } \
} while (0)

#define DB_PRINT(level, ...) do { \
        if (ERR_DEBUG > (level)) { \
                    printf(__VA_ARGS__); \
                } \
} while (0)

/*
 * Use instead of normal assertions.
 * we should catch all asserts before a user sees them, but we don't want the
 * off-chance that the user runs into an assert and it interrupts the user
 * experience.
 */
#define DBG_ASSERT(x) do { \
    if (ERR_DEBUG > 1) {   \
        assert(x);         \
    }                      \
} while (0)


/*
 * The NOP here is arbritrary, it's just hopefully enough for the GPIO to go
 * high before we set it back to low
 */
static void GPIO_TOGGLE(volatile uint8_t gpio_num) {
    gpio_put(gpio_num, 1);
    __NOP;
    __NOP;
    __NOP;
    __NOP;
    gpio_put(gpio_num, 0);
}

static inline void memset32(void *buf, uint32_t val, size_t words)
{
    /* Ensure alignment */
    assert(!(((uint32_t)buf) & 3));

    uint32_t *p = (uint32_t *)buf;
    while (words) {
        *p = val;
        ++p;
        --words;
    }
}

#endif
