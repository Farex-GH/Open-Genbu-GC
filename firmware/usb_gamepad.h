#ifndef USB_GAMEPAD_H
#define USB_GAMEPAD_H

#include <stdint.h>

#include "proc_queue.h"

typedef struct {
    uint8_t indeces[4];
    uint8_t bits[4];
    size_t num;
} gamepad_btn;

typedef struct {
    uint8_t *buf;
    size_t size;
} buf_t;

void ep1_in_cb(uint8_t *buf, uint16_t len);
void ep2_out_cb(uint8_t *buf, uint16_t len);

__prio_queue void *send_gamepad(void *buf);
__prio_queue void *usb_gamepad_format_and_send(void *arg);

#endif
