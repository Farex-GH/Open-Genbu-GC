#ifndef WS2812_H
#define WS2812_H

#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "pinmap.h"

typedef void (*pattern)(uint len, uint t);

static inline void put_pixel(uint32_t pixel_grb, uint8_t pio_sm) {
    pio_sm_put_blocking(pio0, pio_sm, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t) (r) << 8) | ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

void ring_led_init(void);

#endif
