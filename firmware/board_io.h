#ifndef BOARD_IO_H
#define BOARD_IO_H

#include <stdbool.h>
#include <stdint.h>

#define STATE_BUTTON_PRESSED 0
#define STATE_BUTTON_RELEASED 1

/*
 * Looked like 5-6ms on a scope, with what looked like some decent capacitance
 * on the line.
 * Round up to 10 to be safe.
 * This number won't have any impact on how long it takes to send a packet,
 * only how long until we decide to start reading from a certain pin again.
 * The user is unlikely to hit a button 100 times a second, so this is fine.
 */
#define DEBOUNCE_MS 10

#define TYPE_IOMC 0xf33df00d

typedef struct {
    uint8_t gpio;
    bool state;
    bool latched;
} board_io;

typedef struct {
    /* For IPC, we want to safely cast to this type */
    uint32_t type;
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

/*
 * If the IO container is being passed around, instead of blindly casting to it,
 * we'll try a cast and check if it's actually an IO map container.
 */
static inline io_map_container *ioc_safe_cast(void *arg)
{
    io_map_container *ioc = (io_map_container *)arg;
    return ioc->type == TYPE_IOMC ? ioc : NULL;
}

#endif
