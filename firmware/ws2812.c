/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * With modifications by Farex.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

#include "board_io.h"
#include "arm_utils.h"
#include "pinmap.h"
#include "ws2812.h"
#include "utils.h"
#include "proc_queue.h"

typedef struct {
    uint32_t leds[NUM_WS2812 / 4];
} led_quadrant;

typedef struct {
    uint8_t pio_sm;
    uint8_t quad_start;
    union {
        uint32_t leds[NUM_WS2812];
        led_quadrant quadrants[4];
        struct {
            uint32_t up[NUM_WS2812 / 4];
            uint32_t right[NUM_WS2812 / 4];
            uint32_t down[NUM_WS2812 / 4];
            uint32_t left[NUM_WS2812 / 4];
        } __attribute__((packed)) dirs;
    } leds;
} ws2812_dirs;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint32_t duration_ms;
    uint32_t poll_ms;
} strobe_color;

typedef struct {
    uint8_t r;
    int16_t r_inc;
    uint8_t g;
    int16_t g_inc;
    uint8_t b;
    int16_t b_inc;
    
    const strobe_color *colors;
    size_t num_colors;
    size_t curr;

    /* Iterations on this color */
    uint32_t count;
    uint32_t max_count;

    uint32_t num_leds;
    uint8_t pio_sm;
} ws2812_strobe;

void ws2812_set_leds(const io_map_container *ioc, ws2812_dirs *lring,
                     ws2812_dirs *rring, ws2812_strobe *lstrobe,
                     ws2812_strobe *rstrobe);
    
ws2812_dirs rring = {.pio_sm = 0, .quad_start = 0};
ws2812_dirs lring = {.pio_sm = 1, .quad_start = 2};
ws2812_strobe rstrobe = {.pio_sm = 0, .num_leds = NUM_WS2812};
ws2812_strobe lstrobe = {.pio_sm = 1, .num_leds = NUM_WS2812};

alarm_pool_t *led_pool;

strobe_color colors[] = {
    {
        .r = 0x80,
        .g = 0x00,
        .b = 0x40,
        .duration_ms = 1024,
        .poll_ms = 16,
    },
    {
        .r = 0x40,
        .g = 0x40,
        .b = 0x80,
        .duration_ms = 1024,
        .poll_ms = 16,
    },
    {
        .r = 0x00,
        .g = 0x40,
        .b = 0x80,
        .duration_ms = 1024,
        .poll_ms = 16,
    },
    {
        .r = 0x00,
        .g = 0x80,
        .b = 0x80,
        .duration_ms = 1024,
        .poll_ms = 16,
    },
    {
        .r = 0x00,
        .g = 0x80,
        .b = 0x40,
        .duration_ms = 1024,
        .poll_ms = 16,
    },
    {
        .r = 0x40,
        .g = 0x00,
        .b = 0x80,
        .duration_ms = 1024,
        .poll_ms = 16,
    },
};

void put_leds(const uint32_t *leds, size_t len, uint8_t pio_sm)
{
    size_t i;
    for (i = 0; i < len; ++i) {
        put_pixel(leds[i], pio_sm);
    }
}

void ws2812_color_init(ws2812_strobe *s)
{
    const strobe_color *curr = &s->colors[s->curr];
    const strobe_color *next;

    if (s->curr + 1 >= s->num_colors) {
        s->curr = 0;
    } else {
        ++s->curr;
    }
    next = &s->colors[s->curr];

    /*
     * TODO: Support fractional increases. Used fixed point since we have no
     * FPU.
     */
    s->count = 0;
    s->max_count = next->duration_ms / next->poll_ms;
    s->r_inc = (next->r - curr->r) / s->max_count;
    s->g_inc = (next->g - curr->g) / s->max_count;
    s->b_inc = (next->b - curr->b) / s->max_count;
}

void ws2812_strobe_update(ws2812_strobe *s)
{
    if (s->count + 1 >= s->max_count) {
        ws2812_color_init(s);
    }

    ++s->count;
    s->r += s->r_inc;
    s->g += s->g_inc;
    s->b += s->b_inc;
}

void ws2812_strobe_init(ws2812_strobe *s, const strobe_color *colors, size_t num,
                        uint32_t num_leds, uint8_t pio_sm)
{
    const strobe_color *first;

    memset(s, 0, sizeof(*s));
    s->colors = colors;
    s->num_colors = num;
    s->num_leds = num_leds;
    s->pio_sm = pio_sm;

    first = &colors[0];
    s->count = 0;
    s->max_count = first->duration_ms / first->poll_ms;
    s->r_inc = first->r / s->max_count;
    s->g_inc = first->g / s->max_count;
    s->b_inc = first->b / s->max_count;
}

void format_leds(const board_io *io, size_t io_sz, ws2812_dirs *ring,
                 ws2812_strobe *strobe)
{
    size_t i;
    size_t quad_index = ring->quad_start;

    for (i = 0; i < io_sz; ++i) {
        /* Only change ring LED state on IO pins that control it */
        if (!io[i].has_ring_led) {
            continue;
        }
        /*
         * Set any button presses to the inverse of the strobe, otherwise just
         * set the strobe color.
         */
        if (io[i].state == STATE_BUTTON_PRESSED) {
            memset32(ring->leds.quadrants[quad_index].leds,
                     urgb_u32(~strobe->r - 0x80, ~strobe->g - 0x80,
                              ~strobe->b - 0x80),
                     ARRAY_SIZE(ring->leds.quadrants->leds));
        } else {
            memset32(ring->leds.quadrants[quad_index].leds,
                     urgb_u32(strobe->r, strobe->g, strobe->b),
                     ARRAY_SIZE(ring->leds.quadrants->leds));
        }
        ++quad_index;
        if (quad_index >= ARRAY_SIZE(ring->leds.quadrants->leds)) {
            quad_index = 0;
        }
    }

    ws2812_strobe_update(strobe);
}

__prio_queue void *ws2812_update_leds(void *args)
{
    ws2812_set_leds(&io_container, &lring, &rring, &lstrobe, &rstrobe);
    return NULL;
}

static int64_t ring_led_update_cb(alarm_id_t id, void *args)
{
    proc_enqueue(ws2812_update_leds, NULL, PRIORITY_LEVEL_LOWEST);
    return 0;
}

void ws2812_set_leds(const io_map_container *ioc, ws2812_dirs *lring,
                     ws2812_dirs *rring, ws2812_strobe *lstrobe,
                     ws2812_strobe *rstrobe)
{
    format_leds(ioc->btn_map, ioc->btn_size, rring, rstrobe);
    format_leds(ioc->dpad_map, ioc->dpad_size, lring, lstrobe);
    /*
     * TODO: A better method would be to:
     * - Wait for any previous WS2812 PIO DMA interrupts to be finished
     * - DMA L, then DMA R
     */
    put_leds(rring->leds.leds, ARRAY_SIZE(rring->leds.leds), rring->pio_sm);
    put_leds(lring->leds.leds, ARRAY_SIZE(lring->leds.leds), lring->pio_sm);

    alarm_pool_add_alarm_in_ms(led_pool, lstrobe->colors[lstrobe->curr].poll_ms,
                               ring_led_update_cb, NULL, true);
}

void ring_led_init(void)
{
    PIO pio = pio0;
    uint32_t prog1;
    uint32_t prog2;

    prog1 = pio_add_program(pio, &ws2812_program);
    prog2 = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, 0, prog1, BOOSTER_R_WS2812_DIN, 800000, false);
    ws2812_program_init(pio, 1, prog2, BOOSTER_L_WS2812_DIN, 800000, false);
    
    ws2812_strobe_init(&rstrobe, colors, ARRAY_SIZE(colors), NUM_WS2812, 0);
    ws2812_strobe_init(&lstrobe, colors, ARRAY_SIZE(colors), NUM_WS2812, 1); 

    /*
     * Pico SDK mallocs this, we never free.
     * This pool handles ring led updating.
     *
     * We need 2 spots, since we update both rings at the same time, and the
     * alarm pool doesn't clear up our spot until we finish processing it.
     * TODO: Having a separate timer for L and R might be better and more
     * flexible.
     */
    led_pool = alarm_pool_create(1, 2);

    /* Start the ring LEDs */
    proc_enqueue(ws2812_update_leds, NULL, PRIORITY_LEVEL_LOWEST);
}
