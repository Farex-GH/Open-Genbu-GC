/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * With modifications by Farex.
 */

#ifndef USB_DRIVER_H
#define USB_DRIVER_H

#include <stdint.h>
#include <stdlib.h>

#include "usb_common.h"

#include "descriptors.h"

#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

/*
 * Using the driver:
 *
 * - Set up your descriptors to whatever you need.
 *   Descriptors are in the usb descriptors files.
 *   Certain things aren't supported, but HID devices shouldn't have any
 *   problems.
 *   I have not tested CDC, VUD, composite devices, or any other device class.
 *   The driver right now is just enough to get HID working on most things.
 *
 * - Initialize any EP callbacks using usb_ep_add_callback.
 *   These will be called after the EP transfer finishes.
 *   Don't modify EP0 callbacks unless you know what you're doing.
 *
 * - Initialize and enumerate USB by using usb_device_init.
 *   If you want, you could also check if you're configured by using
 *   usb_is_configured
 *
 * - To send data, use usb_start_transfer.
 *   This will DMA data using the available USB DMAs.
 *   If transferring a packet larger than the EP size, you cannot simply
 *   while loop over usb_start_transfer to keep transferring as fast as
 *   possible.
 *   Instead, use a callback so you know when your transfer is done. That
 *   callback will then tell you that you're ready to send more data.
 */

typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);

/* Struct in which we keep the endpoint configuration */
struct usb_endpoint_configuration {
    const struct usb_endpoint_descriptor *descriptor;
    usb_ep_handler handler;

    /*
     * Pointers to endpoint + buffer control registers
     * in the USB controller DPSRAM
     */
    volatile uint32_t *endpoint_control;
    volatile uint32_t *buffer_control;
    volatile uint8_t *data_buffer;

    /* Toggle after each packet (unless replying to a SETUP) */
    uint8_t next_pid;
};

/* Struct in which we keep the device configuration */
struct usb_device_configuration {
    const struct usb_device_descriptor *device_descriptor;
    const struct usb_interface_descriptor *interface_descriptor;
    const struct usb_configuration_descriptor *config_descriptor;
    const struct usb_hid_descriptor *hid_descriptor;
    const unsigned char *lang_descriptor;
    const unsigned char **descriptor_strings;
    struct usb_endpoint_configuration endpoints[USB_NUM_ENDPOINTS];
};

/*
 * TODO: Integrate this state machine with other transfers, not just HID
 * report.
 * The Raspberry Pi example assumes all of their transfers complete in 64 bytes.
 */
typedef enum {
    XFER_STATE_DONE = 0x00,
    XFER_STATE_DEVICE_DESCR = 0x01,
    XFER_STATE_INTERFACE_DESCR = 0x02,
    XFER_STATE_CONFIG_DESCR = 0x03,
    XFER_STATE_ENDPOINT_DESCR = 0x04,
    XFER_STATE_HID_REPORT = 0x05
} USBXferState;

void usb_device_init(void);
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf,
                        uint16_t len);
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr);
void usb_bus_reset(void);
void ep0_in_cb(uint8_t *buf, uint16_t len);
void ep0_out_cb(uint8_t *buf, uint16_t len);
bool usb_is_configured(void);
bool usb_ep_add_callback(uint8_t ep_addr, usb_ep_handler cb);

#endif
