#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/structs/iobank0.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
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
    { .gpio = BOOSTER_R_U },
    { .gpio = BOOSTER_R_R },
    { .gpio = BOOSTER_R_D },
    { .gpio = BOOSTER_R_L },
    { .gpio = BOOSTER_R_PRESS },
    { .gpio = BOOSTER_L_PRESS },
    { .gpio = BTN_START }
};

board_io dpad_map[NUM_DPAD] = {
    { .gpio = BOOSTER_L_U },
    { .gpio = BOOSTER_L_R },
    { .gpio = BOOSTER_L_D },
    { .gpio = BOOSTER_L_L }
};

alarm_pool_t *debounce_pool;

/* When passing maps around, we need to be aware of their size as well */
io_map_container io_container = {
    .type = TYPE_IOMC,
    .btn_map = btn_map,
    .btn_size = ARRAY_SIZE(btn_map),
    .dpad_map = dpad_map,
    .dpad_size = ARRAY_SIZE(dpad_map)
};

/* Anything to be done before we send the data should be handled here */
__prio_queue void *board_io_usb_prewrite(void *args)
{
    DB_PRINT_L(3, "\n");

    io_map_container *ioc = (io_map_container *)args;
    update_leds(ioc);
    proc_enqueue(usb_gamepad_format_and_send, ioc,
                 PRIORITY_LEVEL_HIGHEST);
}

static inline void set_led_from_state(const board_io *btn, uint8_t led)
{
    gpio_put(led, btn->state);
}

void update_press_led(const io_map_container *ioc)
{
    size_t i;
    const board_io *map = ioc->btn_map;

    DBG_ASSERT(map[4].gpio == BOOSTER_R_PRESS);
    DBG_ASSERT(map[5].gpio == BOOSTER_L_PRESS);
    DBG_ASSERT(map[6].gpio == BTN_START);

    set_led_from_state(&map[4], BOOSTER_R_LED);
    set_led_from_state(&map[5], BOOSTER_L_LED);
    set_led_from_state(&map[6], BTN_START_LED);
}

/*
 * Slides (or whatever they're called in GC), have LEDs controlled by WS2812s.
 * These could be doing anything, ranging from breathing animations, to
 * rainbow strobing, to only turning on based on what slides are triggered.
 *
 * Because of this, we don't do anything except hand it off to CPU1, which
 * controls the WS2812s.
 */
void update_slide_led(const io_map_container *ioc)
{
    /*
     * Hardware provides FIFOs for IPC, but they only take in uint32_t.
     * Pass in the address of this and everything should be fine.
     */
    if (!multicore_fifo_push_timeout_us((uint32_t)ioc, 1000)) {
        DB_PRINT_L(1, "Ring LED FIFO push missed!\n");
    }
    /*
     * No need to SEV, the FIFO push does it for us.
     * I don't agree with that design choice, but it is what it is.
     */
}

const void update_leds(io_map_container *ioc)
{
    update_press_led(ioc);
    update_slide_led(ioc);
}

static bool is_gpio_irq_set(uint8_t gpio, uint32_t events)
{
    volatile uint32_t reg = iobank0_hw->intr[gpio / 8];
    /* GPIO ISR register has 4 banks of 8 GPIOs, each with 4 ISRs */
    return reg & (events << (4 * (gpio % 8)));
}

static int64_t debounce_cb(alarm_id_t id, void *args)
{
    board_io *pin = (board_io *)args;
    gpio_set_irq_enabled(pin->gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                         true);

    GPIO_TOGGLE(PIN_DBG_1);

    /*
     * Update state from anything that might have happened when this pin's
     * IRQ was disabled.
     */
    pin->state = gpio_get(pin->gpio);
    /* TODO: Would be better to do this on a pin-by-pin basis */
    update_press_led(&io_container);

    DB_PRINT(2, "Debounce for %d done!\n", pin->gpio);
    return 0;
}

static void gpio_start_debounce(board_io *pin)
{
    uint32_t debounce_ms;
    gpio_set_irq_enabled(pin->gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                         false);
    DB_PRINT(2, "Starting debounce for %d.\n", pin->gpio);
    if (pin->gpio == BOOSTER_R_PRESS || pin->gpio == BOOSTER_L_PRESS) {
        debounce_ms = DEBOUNCE_PRESS_TIME_MS;
    } else {
        debounce_ms = DEBOUNCE_SLIDE_TIME_MS;
    }
    if (alarm_pool_add_alarm_in_ms(debounce_pool, debounce_ms, debounce_cb, pin,
                                   true) <= 0) {
        GPIO_TOGGLE(PIN_DBG_0);
    }
}

static bool board_io_iter_map(board_io *map, size_t sz)
{
    size_t i;
    bool new_state;
    bool state_changed = false;

    for (i = 0; i < sz; ++i) {

        DB_PRINT(4, "gpio: %d, state: %d\n", map[i].gpio, map[i].state);

        new_state = gpio_get(map[i].gpio);
        if (map[i].state != new_state) {
            /*
             * Disable interrupts on any GPIOs that triggered the IRQ ASAP.
             * The button presses have a consistent bounce that happens roughly
             * 1us after the initial press.
             * This means we most likely don't have time to disable the GPIO IRQ
             * outside of the GPIO ISR.
             *
             * To re-enable the IRQ, we'll insert a timer into a timer pool
             * whose callback enables the IRQ of any GPIOs whose interrupts
             * were set.
             */
            map[i].state = new_state;
            state_changed = true;
            gpio_start_debounce(&map[i]);
            DB_PRINT(4, "gpio: %d, new state: %d\n", map[i].gpio, map[i].state);
        }
        gpio_acknowledge_irq(map[i].gpio,
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    }

    return state_changed;
}

__irq_handler void board_io_irq_handler(void)
{
    bool state_changed;
    bool dpad_changed;
    bool btn_changed;

    btn_changed = board_io_iter_map(btn_map, ARRAY_SIZE(btn_map));
    dpad_changed = board_io_iter_map(dpad_map, ARRAY_SIZE(dpad_map));
    state_changed = btn_changed || dpad_changed;

    DB_PRINT_L(3, "state_changed=%d\n", state_changed);

    if (state_changed) {
        proc_enqueue(board_io_usb_prewrite, &io_container,
                     PRIORITY_LEVEL_HIGHEST);
    }
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

static void isr_init(void)
{
    uint32_t ien;
    size_t i;
    uint32_t events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

    io_irq_ctrl_hw_t *irq_base = get_core_num() ?
                                 &iobank0_hw->proc1_irq_ctrl :
                                 &iobank0_hw->proc0_irq_ctrl;

    for (i = 0; i < io_container.btn_size; ++i) {
        gpio_set_irq_enabled(io_container.btn_map[i].gpio, events, true);
    }

    for (i = 0; i < io_container.dpad_size; ++i) {
        gpio_set_irq_enabled(io_container.dpad_map[i].gpio, events, true);
    }
    irq_set_exclusive_handler(IO_IRQ_BANK0, board_io_irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

static void debounce_init(void)
{
    /* Pico SDK mallocs this, we never free */
    debounce_pool = alarm_pool_create(0, (io_container.btn_size +
                                      io_container.dpad_size) * 2);
}

void board_io_init(void)
{
    io_map_init();
    isr_init();
    debounce_init();
}
