/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include "pico/multicore.h"

#include "board_io.h"
#include "arm_utils.h"
#include "pinmap.h"
#include "ws2812.h"
#include "utils.h"

typedef struct {
    uint8_t pio_sm;
    union {
        uint32_t leds[NUM_WS2812];
        struct {
            uint32_t up[NUM_WS2812 / 4];
            uint32_t right[NUM_WS2812 / 4];
            uint32_t down[NUM_WS2812 / 4];
            uint32_t left[NUM_WS2812 / 4];
        } dirs;
    } leds;
} ws2812_dirs;

void put_leds(const uint32_t *leds, size_t len, uint8_t pio_sm)
{
    size_t i;
    for (i = 0; i < len; ++i) {
        put_pixel(leds[i], pio_sm);
    }
}

void format_leds(const io_map_container *ioc, ws2812_dirs *lbooster,
                 ws2812_dirs *rbooster, uint32_t color)
{
    memset(rbooster->leds.leds, 0, sizeof(rbooster->leds.leds));
    memset(lbooster->leds.leds, 0, sizeof(lbooster->leds.leds));
    /* We assume the ordering in IOC is URDL and comes first */
    /* TODO: Create a test to ensure this ordering is maintained */

    /* Find out which directional LEDs are set */
    if (ioc->btn_map[0].state == STATE_BUTTON_PRESSED) {
        memset32(rbooster->leds.dirs.up, color,
                 ARRAY_SIZE(rbooster->leds.dirs.up));
    }
    if (ioc->btn_map[1].state == STATE_BUTTON_PRESSED) {
        memset32(rbooster->leds.dirs.right, color,
                 ARRAY_SIZE(rbooster->leds.dirs.right));
    }
    if (ioc->btn_map[2].state == STATE_BUTTON_PRESSED) {
        memset32(rbooster->leds.dirs.down, color,
                 ARRAY_SIZE(rbooster->leds.dirs.down));
    }
    if (ioc->btn_map[3].state == STATE_BUTTON_PRESSED) {
        memset32(rbooster->leds.dirs.left, color,
                 ARRAY_SIZE(rbooster->leds.dirs.left));
    }
    if (ioc->dpad_map[0].state == STATE_BUTTON_PRESSED) {
        memset32(lbooster->leds.dirs.up, color,
                 ARRAY_SIZE(lbooster->leds.dirs.up));
    }
    if (ioc->dpad_map[1].state == STATE_BUTTON_PRESSED) {
        memset32(lbooster->leds.dirs.right, color,
                 ARRAY_SIZE(lbooster->leds.dirs.right));
    }
    if (ioc->dpad_map[2].state == STATE_BUTTON_PRESSED) {
        memset32(lbooster->leds.dirs.down, color,
                 ARRAY_SIZE(lbooster->leds.dirs.down));
    }
    if (ioc->dpad_map[3].state == STATE_BUTTON_PRESSED) {
        memset32(lbooster->leds.dirs.left, color,
                 ARRAY_SIZE(lbooster->leds.dirs.left));
    }
}

void ws2812_set_leds(const io_map_container *ioc, ws2812_dirs *lbooster,
                     ws2812_dirs *rbooster)
{
    format_leds(ioc, lbooster, rbooster, 0x0000ffff);
    /*
     * TODO: A better method would be to:
     * - Wait for any previous WS2812 PIO DMA interrupts to be finished
     * - DMA L, then DMA R
     * - Going sequentially is probably fine and won't be noticeable by the
     *   user, but DMAing things is better.
     */
    put_leds(lbooster->leds.leds, ARRAY_SIZE(lbooster->leds.leds),
             lbooster->pio_sm);
    put_leds(rbooster->leds.leds, ARRAY_SIZE(rbooster->leds.leds),
             rbooster->pio_sm);
}

void cpu1_control_loop(ws2812_dirs *lbooster, ws2812_dirs *rbooster)
{
    io_map_container *ioc;
    uint32_t fifo_val;
    while (1) {
        if (multicore_fifo_pop_timeout_us(1000, &fifo_val)) {
            DB_PRINT_L(5, "FIFO Popped!\n");
            multicore_fifo_clear_irq();
            /*
             * We expect this to be the address of the global IO map container.
             * To be safe, we'll cast and verify that nothing else messed with
             * the FIFO.
             */
            ioc = ioc_safe_cast((void *)fifo_val);
            if (ioc) {
                ws2812_set_leds(ioc, lbooster, rbooster);
            }
        } else {
            DB_PRINT_L(5, "CPU1 Woke up but could not pop FIFO\n");
            multicore_fifo_clear_irq();
            __WFE;
        }
    }
}

void handle_ring_led(void) {
    PIO pio = pio0;
    uint32_t prog1;
    uint32_t prog2;
    ws2812_dirs lbooster = {.pio_sm = 1};
    ws2812_dirs rbooster = {.pio_sm = 0};

    prog1 = pio_add_program(pio, &ws2812_program);
    prog2 = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, 0, prog1, BOOSTER_R_WS2812_DIN, 800000, false);
    ws2812_program_init(pio, 1, prog2, BOOSTER_L_WS2812_DIN, 800000, false);

    cpu1_control_loop(&lbooster, &rbooster);
}
