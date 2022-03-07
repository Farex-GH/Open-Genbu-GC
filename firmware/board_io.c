#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"

#include "proc_queue.h"
#include "pinmap.h"
#include "utils.h"
#include "arm_utils.h"
#include "io.h"
#include "usb_gamepad.h"
#include "board_io.h"
#include "debounce.h"

board_io btn_map[NUM_BTN] = {
    {
        .gpio = BOOSTER_R_U,
        .debounce_press_ms = DEBOUNCE_SLIDE_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_SLIDE_RELEASE_TIME_MS,
        .has_ring_led = true,
    },
    {
        .gpio = BOOSTER_R_R,
        .debounce_press_ms = DEBOUNCE_SLIDE_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_SLIDE_RELEASE_TIME_MS,
        .has_ring_led = true,
    },

    {
        .gpio = BOOSTER_R_D,
        .debounce_press_ms = DEBOUNCE_SLIDE_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_SLIDE_RELEASE_TIME_MS,
        .has_ring_led = true,
    },
    {
        .gpio = BOOSTER_R_L,
        .debounce_press_ms = DEBOUNCE_SLIDE_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_SLIDE_RELEASE_TIME_MS,
        .has_ring_led = true,
    },
    {
        .gpio = BOOSTER_R_PRESS,
        .debounce_press_ms = DEBOUNCE_PRESS_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_PRESS_RELEASE_TIME_MS,
    },
    {
        .gpio = BOOSTER_L_PRESS,
        .debounce_press_ms = DEBOUNCE_PRESS_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_PRESS_RELEASE_TIME_MS,
    },
    {
        .gpio = BTN_START,
        .debounce_press_ms = DEBOUNCE_PRESS_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_PRESS_RELEASE_TIME_MS,
    }
};

board_io dpad_map[NUM_DPAD] = {
    {
        .gpio = BOOSTER_L_U,
        .debounce_press_ms = DEBOUNCE_SLIDE_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_SLIDE_RELEASE_TIME_MS,
        .has_ring_led = true,
    },
    {
        .gpio = BOOSTER_L_R,
        .debounce_press_ms = DEBOUNCE_SLIDE_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_SLIDE_RELEASE_TIME_MS,
        .has_ring_led = true,
    },
    {
        .gpio = BOOSTER_L_D,
        .debounce_press_ms = DEBOUNCE_SLIDE_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_SLIDE_RELEASE_TIME_MS,
        .has_ring_led = true,
    },
    {
        .gpio = BOOSTER_L_L,
        .debounce_press_ms = DEBOUNCE_SLIDE_PRESS_TIME_MS,
        .debounce_release_ms = DEBOUNCE_SLIDE_RELEASE_TIME_MS,
        .has_ring_led = true,
    }
};

alarm_pool_t *io_pool;

/* When passing maps around, we need to be aware of their size as well */
io_map_container io_container = {
    .type = TYPE_IOMC,
    .btn_map = btn_map,
    .btn_size = ARRAY_SIZE(btn_map),
    .dpad_map = dpad_map,
    .dpad_size = ARRAY_SIZE(dpad_map)
};


static inline void set_led_from_state(const board_io *btn, uint8_t led)
{
    gpio_put(led, btn->state);
}

static void update_led(board_io *pin)
{
    switch(pin->gpio) {
    case BOOSTER_R_PRESS:
        set_led_from_state(pin, BOOSTER_R_LED);
        break;
    case BOOSTER_L_PRESS:
        set_led_from_state(pin, BOOSTER_L_LED);
        break;
    case BTN_START:
        set_led_from_state(pin, BTN_START_LED);
        break;
    case BOOSTER_R_U:
    case BOOSTER_R_R:
    case BOOSTER_R_D:
    case BOOSTER_R_L:
    case BOOSTER_L_U:
    case BOOSTER_L_R:
    case BOOSTER_L_D:
    case BOOSTER_L_L:
        /*
         * Ignore slide leds, the WS2812 handler will periodically update them,
         * including with slide press state.
         */
    default:
        DBG_ASSERT(0);
    }
}

static int64_t debounce_cb(alarm_id_t id, void *args)
{
    board_io *pin = (board_io *)args;
    pin->bouncing = false;

    return 0;
}

static void gpio_start_debounce(board_io *pin)
{
    uint32_t debounce_ms;

    /* If we're high, we were released. */
    if (pin->state) {
        debounce_ms = pin->debounce_press_ms;
    } else {
        debounce_ms = pin->debounce_release_ms;
    }
    alarm_pool_add_alarm_in_ms(io_pool, debounce_ms, debounce_cb, pin, true);
}

/* Button presses (as opposed to slides) have different LED handling */
static void handle_state_change(board_io *pin, bool new_state)
{
    pin->bouncing = true;
    pin->state = new_state;

    update_led(pin);
    gpio_start_debounce(pin);
}

static bool get_button_state(board_io *map, size_t sz)
{
    size_t i;
    bool new_state;
    bool state_changed = false;

    for (i = 0; i < sz; ++i) {
        /* Skip over anything currently bouncing */
        if (map[i].bouncing) {
            continue;
        }

        new_state = gpio_get(map[i].gpio);
        if (map[i].state != new_state) {
            handle_state_change(&map[i], new_state);

            state_changed = true;
        }
    }

    return state_changed;
}

/*
 * Instead of having the timer queue directly call the function that polls the
 * buttons, we enqueue that process in our processing queue.
 *
 * This is because if we directly call the button handling from the timer queue,
 * it might be possible that the timer queue fires more frequently than we send
 * USB packets.  This is because the USB polling rate is 1ms while the button
 * polling rate is also 1ms. However, USB transfers have more overhead than
 * reading GPIOs, so they should take longer in theory.
 * What could potentially happen then, is the timer firing in the middle of a
 * USB transfer, causing potential problems.
 *
 * With enqueueing the poll_buttons process, we ensure that if poll_buttons
 * causes a USB transfer and the poll_buttons timer fires during the transfer,
 * the next poll_buttons call happens after the USB transfer.
 *
 * A mutex could also be used to prevent this from happening, but feeding
 * everything through the processing queue is cleaner and more straightforward.
 */
static int64_t proc_poll_buttons(alarm_id_t id, void *arg)
{
    proc_enqueue(poll_buttons, &io_container, PRIORITY_LEVEL_HIGHEST);
    return 0;
}

__prio_queue void *poll_buttons(void *arg)
{
    io_map_container *ioc = &io_container;
    bool state_changed;

    state_changed = get_button_state(ioc->dpad_map, ioc->dpad_size);
    state_changed = get_button_state(ioc->btn_map, ioc->btn_size) == true ?
                     true : state_changed;
    if (state_changed) {
        proc_enqueue(usb_gamepad_format_and_send, &io_container,
                     PRIORITY_LEVEL_HIGHEST);
    }

    alarm_pool_add_alarm_in_ms(io_pool, BTN_POLL_RATE_MS, proc_poll_buttons,
                               NULL, true);
    return NULL;
}

static void poll_button_init(void)
{
    proc_enqueue(poll_buttons, &io_container, PRIORITY_LEVEL_HIGHEST);
}

static void io_map_init(void)
{
    size_t i;

    /* Buttons */
    for (i = 0; i < ARRAY_SIZE(btn_map); ++i) {
        gpio_set_input_enabled(btn_map[i].gpio, true);
        gpio_set_pulls(btn_map[i].gpio, true, false);
        /* Give the pins time to rise */
        while(!gpio_get(btn_map[i].gpio));
        btn_map[i].state = gpio_get(btn_map[i].gpio);
    }

    for (i = 0; i < ARRAY_SIZE(dpad_map); ++i) {
        gpio_set_input_enabled(dpad_map[i].gpio, true);
        gpio_set_pulls(dpad_map[i].gpio, true, false);
        /* Give the pins time to rise */
        while(!gpio_get(dpad_map[i].gpio));
        dpad_map[i].state = gpio_get(dpad_map[i].gpio);
    }

    /* LEDs */
    gpio_init(BOOSTER_R_LED);
    gpio_init(BOOSTER_L_LED);
    gpio_init(BTN_START_LED);
    gpio_set_dir(BOOSTER_R_LED, GPIO_OUT);
    gpio_set_dir(BOOSTER_L_LED, GPIO_OUT);
    gpio_set_dir(BTN_START_LED, GPIO_OUT);
    gpio_put(BOOSTER_R_LED, 1);
    gpio_put(BOOSTER_L_LED, 1);
    gpio_put(BTN_START_LED, 1);
    gpio_set_dir(BOOSTER_R_WS2812_DIN, true);
    gpio_set_dir(BOOSTER_L_WS2812_DIN, true);
}

static void debounce_init(void)
{
    /*
     * Pico SDK mallocs this, we never free.
     * This pool handles debounce and button polling.
     *
     * We make the size arbitrarily large, but at least as large as the
     * number of buttons we have to support, since they could all be bouncing
     * at the same time.
     */
    io_pool = alarm_pool_create(0, 10 + ((io_container.btn_size +
                                         io_container.dpad_size) * 2));
}

void board_io_init(void)
{
    io_map_init();
    debounce_init();
    poll_button_init();
}
