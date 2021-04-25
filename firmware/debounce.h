#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "board_io.h"

#define DEBOUNCE_SLIDE_TIME_MS 15
/*
 * This shit bounces like crazy.
 * The microswitch is the same as slides, but the button has its own bounce.
 * This is bad enough where it may interfere with normal play.
 */
#define DEBOUNCE_PRESS_TIME_MS 50

/*
 * This handles what we should do right away to stop bouncing.
 * For now it disables the correspoinding GPIO interrupt
 */
static inline void debounce_start(uint8_t gpio)
{
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
}

int64_t debounce_timer_elapsed(alarm_id_t id, void *arg);

#endif
