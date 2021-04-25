#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "usb_common.h"

#include "utils.h"
#include "arm_utils.h"
#include "proc_queue.h"
#include "board_io.h"
#include "pinmap.h"
#include "descriptors.h"
#include "usb_gamepad.h"
#include "usb_driver.h"

bool ep1_in_busy = false;

/* Global needed so EP1 CB knows if it needs to re-send packets */
bool gamepad_held;

/* TODO: Write a test that ensures this map matches the io_map mapping */
/* TODO: These are all messed up because testing was done with the butons in
 * the wrong orientation
 */
/* Values stolen from Genbu */
const gamepad_btn io_btn_map[NUM_BTN] = {
    { .indeces = {0, 14}, .bits = {0x08, 0xff}, .num = 2 }, /* RU */
    { .indeces = {0, 13}, .bits = {0x04, 0xff}, .num = 2 }, /* RR */
    { .indeces = {0, 12}, .bits = {0x02, 0xff}, .num = 2 }, /* RD */
    { .indeces = {0, 11}, .bits = {0x01, 0xff}, .num = 2 }, /* RL */
    { .indeces = {0, 11}, .bits = {0x20, 0xff}, .num = 2 }, /* RPRESS */
    { .indeces = {0, 15}, .bits = {0x10, 0xff}, .num = 2 }, /* LPRESS */
    { .indeces = {1},     .bits = {0x01}, .num = 1 }        /* START */
};

const gamepad_btn io_dpad_map[NUM_DPAD] = {
    { .indeces = {2, 8},     .bits = {0x06, 0xff}, .num = 2 }, /* LU */
    { .indeces = {2, 9},     .bits = {0x08, 0xff}, .num = 2 }, /* LR */
    { .indeces = {2, 7},     .bits = {0x02, 0xff}, .num = 2 }, /* LD */
    { .indeces = {2, 10},    .bits = {0x04, 0xff}, .num = 2 } /* LL */
};

const uint8_t gamepad_template[] = {
                           0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x02, 0x80, 0x01, 0x00,
                           0x02, 0x00, 0x02
                           };

__prio_queue void *send_gamepad(void *buf)
{
    uint8_t *usb_buf = (uint8_t *)buf;

    DB_PRINT_L(3, "Starting transfer\n");
    for (size_t i = 0; i < ARRAY_SIZE(gamepad_template); ++i) {
        DB_PRINT(4, "%.2x%c", usb_buf[i], (!(i % 0x08) && i > 0) ? '\n' : ' ');
    }
    DB_PRINT(4, "\n");
    /*
     * XXX: Need a way to pass in size of buf, rather than assuming
     * (correctly) that it is a gamepad buffer
     */
    usb_start_transfer(usb_get_endpoint_configuration(EP1_IN_ADDR), usb_buf,
                       ARRAY_SIZE(gamepad_template));

    return NULL;
}

void ep1_in_cb(uint8_t *buf, uint16_t len)
{
    DB_PRINT_L(3, "gamepad_held=%d\n", gamepad_held);

    ep1_in_busy = false;
    if (gamepad_held) {
        /*
         * This looks scary, but should be okay.
         * buf points to USB memory, which has a global instance.
         * This memory should not change between when this function happens and
         * when the function added to the queue is executed, since that would
         * mean another USB transfer would have to have happened on EP1.
         *
         * TODO: A more proper thing to do would to be contain buf and len in
         * a struct that has a longer lifetime.
         * It would have to be global, but that's ugly.
         *
         * XXX: Commented for now because the host (at least on the switch)
         * sees the button as held until a "release" packet is sent.
         */
//        proc_enqueue(send_gamepad, buf, PRIORITY_LEVEL_HIGHEST);
    }
}

void ep2_out_cb(uint8_t *buf, uint16_t len)
{
    DB_PRINT_L(3, "EP2 RX:\n");
    for (size_t i = 0; i < len; ++i) {
        DB_PRINT(3, "%.2x%c", buf[i], (!(i % 0x08) && i > 0) ? '\n' : ' ');
    }
    /* Ignore the host on this EP */
}

static void usb_btn_map_to_buf(const gamepad_btn *map, size_t index,
                               uint8_t *buf)
{
    size_t i;
    const gamepad_btn *btn = &map[index];

    for (i = 0; i < btn->num; ++i) {
        buf[btn->indeces[i]] ^= btn->bits[i];
    }
}

static bool usb_dpad_map_to_buf(const gamepad_btn *usb_map,
                                const io_map_container *ioc, uint8_t *usb_buf)
{
    size_t i, j;
    const board_io *io_map = ioc->dpad_map;
    uint8_t press_cnt = 0;
    uint8_t dpad_val = 0;
    const gamepad_btn *btn;

    /*
     * D-pad input needs special handling on byte 2.
     * U D L R each have an even number assigned  to them, and that number is
     * added, averaged, and put in the buffer.
     *
     * XXX: In the case where the d-pad value is 8, we set the buffer to 0
     * (0x08 ^ 0x08), unless there's another value, then it's averaged like any
     * other value, but it's an 8 instead of 0.
     * We do this in a clunky way, and there should be an obvious better way.
     *
     * If there are multiple inputs, we add the inputs and then divide by the
     * number of inputs.
     * This might break on combinations such as LR, UD, and 3-4 button combos,
     * but this can't happen in normal situations.
     *
     * Along with this, there's an FF byte set at a set location in the USB
     * buffer.
     */
    for (i = 0; i < ioc->dpad_size; ++i) {
        if (io_map[i].state == STATE_BUTTON_PRESSED) {
            ++press_cnt;
            btn = &usb_map[i];

            /*
             * HACK: UR needs to be 1, because U acts as 0 and R is 2.
             * However, for UL, which is 7, U acts as 8 and and L is 6
             */
            if (press_cnt > 1 && (dpad_val == 0x08 && btn->bits[0] == 0x02)) {
                dpad_val = 0;
            }
            dpad_val += btn->bits[0];

            /* Now do the FF byte(s) */
            for (j = 1; j < btn->num; ++j) {
                usb_buf[btn->indeces[j]] ^= btn->bits[j];
            }
        }
    }

    /* Now add the d-pad input */
    if (press_cnt == 1) {
        usb_buf[btn->indeces[0]] = dpad_val == 0x08 ? 0 : dpad_val;
    } else if (press_cnt > 1) {
        dpad_val /= press_cnt;
        usb_buf[btn->indeces[0]] = dpad_val;
    }

    return !!press_cnt;
}

__prio_queue void *usb_gamepad_format_and_send(void *arg)
{
    uint8_t usb_buf[ARRAY_SIZE(gamepad_template)];
    size_t i;
    io_map_container *ioc = (io_map_container *)arg;
    board_io *io = ioc->btn_map;
    bool all_high = true;
    bool dpad_all_high;
    DB_PRINT_L(3, "Formatting buffer\n");

    memcpy(usb_buf, gamepad_template, ARRAY_SIZE(usb_buf));
    for (i = 0; i < ioc->btn_size; ++i) {
        if (io[i].state == STATE_BUTTON_PRESSED) {
            usb_btn_map_to_buf(io_btn_map, i, usb_buf);
            all_high = false;
        }
    }
    dpad_all_high = !usb_dpad_map_to_buf(io_dpad_map, ioc, usb_buf);

    if (!dpad_all_high || !all_high) {
        all_high = false;
    }

    /*
     * Make sure messages don't get gobbled, spin if needed.
     * In practice, I don't think I've seen this get triggered.
     */
    while(ep1_in_busy);
    send_gamepad(usb_buf);
    ep1_in_busy = true;

    /*
     * XXX: Maybe ugly?  I would need to revisit this.
     * If not all of the buttons are high, we need to add another send function
     * to the priority queue.  This is because we need to keep sending presses
     * to tell the host that we are holding the button down.
     *
     * This will be handled by the EP1 callback.  An alternative would be to
     * add a timer to the timer queue equal to the polling rate, and when that
     * elapses, the callback for that is usb_gamepad_format_and_send.
     * In order to communicate with ep1_cb(), we use a global telling it if we
     * have buttons held and need to keep sending packets.
     * Another alternative would be something that is evoked when the
     * USB transfer complete interrupt is set.
     *
     * When the user releases all buttons the GPIO IRQ will update the map
     * accordingly, meaning all buttons will be high and we stop re-sending
     * packets.
     */
    if (all_high) {
        gamepad_held = false;
    } else {
        gamepad_held = true;
    }

    return NULL;
}
