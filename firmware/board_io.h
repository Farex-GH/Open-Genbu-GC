#ifndef BOARD_IO_H
#define BOARD_IO_H

#include <stdbool.h>
#include <stdint.h>
#include "proc_queue.h"

#define STATE_BUTTON_PRESSED 0
#define STATE_BUTTON_RELEASED 1

#define BTN_POLL_RATE_MS 1

#define TYPE_IOMC 0xf33df00d

typedef struct {
    uint8_t gpio;
    uint8_t debounce_press_ms;
    uint8_t debounce_release_ms;
    bool state;
    bool bouncing;
    bool has_ring_led;
} board_io;

typedef struct {
    /*
     * TODO: If we're clever about this, we can simplify some GPIO iterating
     * code by combining the button and d-pad maps and being able to iterate
     * over that combined map, and access the separate ones when needed.
     */
    board_io *btn_map;
    size_t btn_size;
    board_io *dpad_map;
    size_t dpad_size;
} io_map_container;

void *board_io_usb_prewrite(void *args);
void board_io_init(void);
const void update_leds(io_map_container *ioc);
void update_press_led(const io_map_container *ioc);
void update_slide_led(const io_map_container *ioc);
__prio_queue void *poll_buttons(void *arg);

extern io_map_container io_container;

#endif
