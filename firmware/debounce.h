#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "board_io.h"

/* Measured using a scope with some extra buffer room added. */
#define DEBOUNCE_SLIDE_PRESS_TIME_MS 15
#define DEBOUNCE_SLIDE_RELEASE_TIME_MS 5
#define DEBOUNCE_PRESS_PRESS_TIME_MS 20
#define DEBOUNCE_PRESS_RELEASE_TIME_MS 50

#endif
