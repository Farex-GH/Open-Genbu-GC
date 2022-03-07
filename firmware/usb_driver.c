/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * With modifications by Farex.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "usb_common.h"
#include "hardware/regs/usb.h"
#include "hardware/structs/usb.h"
#include "hardware/irq.h"
#include "hardware/resets.h"

#include "io.h"
#include "utils.h"
#include "arm_utils.h"
#include "proc_queue.h"
#include "board_io.h"
#include "pinmap.h"
#include "descriptors.h"
#include "usb_driver.h"

/* Callbacks for when the EP finishes transferring */
void ep0_in_cb(uint8_t *buf, uint16_t len);
void ep0_out_cb(uint8_t *buf, uint16_t len);

/* USB Globals */
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;
static uint8_t ep0_buf[64];

/*
 * For packets too large to fit in a single transfer, we need to keep track of
 * how much we have transferred
 */
size_t descr_sent;
size_t descr_expected;
USBXferState usb_xfer_state = XFER_STATE_DONE;

static struct usb_device_configuration dev_config = {
        .device_descriptor = &device_descriptor,
        .interface_descriptor = &interface_descriptor,
        .config_descriptor = &config_descriptor,
        .hid_descriptor = &hid_descriptor,
        .lang_descriptor = lang_descriptor,
        .descriptor_strings = descriptor_strings,
        .endpoints = {
                {
                        .descriptor = &ep0_out,
                        .handler = &ep0_out_cb,
                        .endpoint_control = NULL, /* NA for EP0 */
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
                        /* EP0 in and out share a data buffer */
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep0_in,
                        .handler = &ep0_in_cb,
                        .endpoint_control = NULL, /* NA for EP0 */
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
                        /* EP0 in and out share a data buffer */
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep1_in,
                        .endpoint_control = &usb_dpram->ep_ctrl[0].in,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[1].in,
                        /* First free EPx buffer */
                        .data_buffer = &usb_dpram->epx_data[0 * 64],
                },
                {
                        .descriptor = &ep2_out,
                        .endpoint_control = &usb_dpram->ep_ctrl[1].out,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[2].out,
                        /* Second free EPx buffer */
                        .data_buffer = &usb_dpram->epx_data[1 * 64],
                }
        }
};

/**
 * @brief Given an endpoint address, return the usb_endpoint_configuration of
 * that endpoint. Returns NULL if an endpoint of that address is not found.
 *
 * @param addr
 * @return struct usb_endpoint_configuration
 */
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr)
{
    struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor &&
            (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

/**
 * @brief Given a C string, fill the EP0 data buf with a USB string descriptor
 * for that string.
 *
 * @param C string you would like to send to the USB host
 * @return the length of the string descriptor in EP0 buf
 */
/* TODO: This can be done at compile time. */
static uint8_t usb_prepare_string_descriptor(const unsigned char *str)
{
    /*
     * (2 for bLength + bDescriptorType + strlen * 2) because host expects
     * UTF-16
     */
    uint8_t bLength = 2 + (strlen(str) * 2);
    static const uint8_t bDescriptorType = 0x03;

    volatile uint8_t *buf = &ep0_buf[0];
    *buf++ = bLength;
    *buf++ = bDescriptorType;

    uint8_t c;

    do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
    } while (c != '\0');

    return bLength;
}

/**
 * @brief Take a buffer pointer located in the USB RAM and return as an offset
 * of the RAM.
 *
 * @param buf
 * @return uint32_t
 */
static inline uint32_t usb_buffer_offset(volatile uint8_t *buf)
{
    return (uint32_t) buf ^ (uint32_t) usb_dpram;
}

/**
 * @brief Set up the endpoint control register for an endpoint
 * (if applicable. Not valid for EP0).
 *
 * @param ep
 */
static void usb_setup_endpoint(const struct usb_endpoint_configuration *ep)
{
    if (!ep->endpoint_control) {
        return;
    }

    /* Get the data buffer as an offset of the USB controller's DPRAM */
    uint32_t dpram_offset = usb_buffer_offset(ep->data_buffer);
    uint32_t reg = EP_CTRL_ENABLE_BITS
                   | EP_CTRL_INTERRUPT_PER_BUFFER
                   | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB)
                   | dpram_offset;
    *ep->endpoint_control = reg;
}

/**
 * @brief Set up the endpoint control register for each endpoint.
 */
static void usb_setup_endpoints(void)
{
    const struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && endpoints[i].handler) {
            usb_setup_endpoint(&endpoints[i]);
        }
    }
}

/**
 * @brief Set up the USB controller in device mode, clearing any previous state.
 */
void usb_device_init(void)
{
    /* Reset usb controller */
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    /* Clear any previous state in dpram just in case */
    memset(usb_dpram, 0, sizeof(*usb_dpram));

    /* Enable USB interrupt at processor */
    irq_set_enabled(USBCTRL_IRQ, true);

    /* Mux the controller to the onboard usb phy */
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    /* Force VBUS detect so the device thinks it is plugged into a host */
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS |
                  USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    /* Enable the USB controller in device mode. */
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    /* Enable an interrupt per EP0 transaction */
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;

    /*
     * Enable interrupts for when a buffer is done, when the bus is reset,
     * and when a setup packet is received
     */
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS |
                   USB_INTS_BUS_RESET_BITS |
                   USB_INTS_SETUP_REQ_BITS;

    /*
     * Set up endpoints (endpoint control registers)
     * described by device configuration
     */
    usb_setup_endpoints();

    /* Present full speed device by enabling pull up on DP */
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

/**
 * @brief Given an endpoint configuration, returns true if the endpoint
 * is transmitting data to the host (i.e. is an IN endpoint)
 *
 * @param ep, the endpoint configuration
 * @return true
 * @return false
 */
static inline bool ep_is_tx(struct usb_endpoint_configuration *ep)
{
    return ep->descriptor->bEndpointAddress & USB_DIR_IN;
}

/**
 * @brief Starts a transfer on a given endpoint.
 *
 * @param ep, the endpoint configuration.
 * @param buf, the data buffer to send. Only applicable if the endpoint is TX
 * @param len, the length of the data in buf (this example limits max len to
 *             one packet - 64 bytes)
 */
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf,
                        uint16_t len)
{
    /*
     * Transfers larger than the EP max size should be handled by the EP
     * callback recognizing the state and continuing the transfer.
     */
    assert(len <= 64);

    /* Prepare buffer control register value */
    uint32_t val = len | USB_BUF_CTRL_AVAIL;

    if (ep_is_tx(ep)) {
        /* Need to copy the data from the user buffer to the usb memory */
        /* Cast to remove compiler warning */
        memcpy((void *)ep->data_buffer, buf, len);
        /* Mark as full */
        val |= USB_BUF_CTRL_FULL;
    }

    /* Set pid and flip for next transfer.  This swaps between buffers. */
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;

    *ep->buffer_control = val;
}

/**
 * @brief Send device descriptor to host
 */
static void usb_handle_device_descriptor(volatile struct usb_setup_packet *pkt)
{
    size_t xfer_len;
    uint8_t *d = (uint8_t *) dev_config.device_descriptor;
    struct usb_endpoint_configuration *ep;
    ep = usb_get_endpoint_configuration(EP0_IN_ADDR);

    xfer_len = pkt->wLength <= sizeof(*dev_config.device_descriptor) ?
                               pkt->wLength :
                               sizeof(*dev_config.device_descriptor);

    ep->next_pid = 1;
    usb_start_transfer(ep, d, xfer_len);
}

static void usb_handle_device_descriptor_cont(void)
{
    uint8_t *d = (uint8_t *)dev_config.device_descriptor;
    size_t xfer_len;

    xfer_len = descr_expected - descr_sent;
    memcpy(ep0_buf, &hid_report_descriptor[descr_sent], xfer_len);
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR),
                       &d[descr_sent], xfer_len);
}

/**
 * @brief Send the configuration descriptor (and potentially the configuration
 * and endpoint descriptors) to the host.
 *
 * @param pkt, the setup packet received from the host.
 */
static void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt)
{
    uint8_t *buf = &ep0_buf[0];
    /* First request will want just the config descriptor */
    const struct usb_configuration_descriptor *d = dev_config.config_descriptor;
    const struct usb_endpoint_configuration *ep = dev_config.endpoints;

    memcpy((void *) buf, d, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    /* If we have more than just the config descriptor copy it all */
    if (pkt->wLength >= d->wTotalLength) {
        memcpy(buf, dev_config.interface_descriptor,
               sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);

        /* Send HID descriptor, if it exists */
        if (dev_config.hid_descriptor) {
            /* XXX: See FIXME in HID descriptor struct definition */
            assert(dev_config.hid_descriptor->bNumDescriptors == 1);

            memcpy(buf, dev_config.hid_descriptor,
                   sizeof(*dev_config.hid_descriptor));
            buf += sizeof(*dev_config.hid_descriptor);
        }

        /* Copy all the endpoint descriptors starting from EP1 */
        for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
            if (ep[i].descriptor) {
                memcpy(buf, ep[i].descriptor,
                       sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }
    }

    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0],
                       len);
}

/**
 * @brief Handle a BUS RESET from the host by setting the device address back
 * to 0.
 */
void usb_bus_reset(void)
{
    dev_addr = 0;
    should_set_address = false;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
}

/**
 * @brief Send the requested string descriptor to the host.
 *
 * @param pkt, the setup packet from the host.
 */
static void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt)
{
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], dev_config.lang_descriptor, len);
    } else {
        /* Prepare fills in ep0_buf */
        len =
            usb_prepare_string_descriptor(dev_config.descriptor_strings[i - 1]);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR),
                                                      &ep0_buf[0], len);
}

/**
 * @brief Handles a SET_ADDR request from the host. The actual setting of the
 *        device address in hardware is done in ep0_in_cb. This is because we
 *        have to acknowledge the request first as a device with address zero.
 *
 * @param pkt, the setup packet from the host.
 */
static void usb_set_device_address(volatile struct usb_setup_packet *pkt)
{
    /*
     * Set address is a bit of a strange case because we have to send a 0 length
     * status packet first with address 0
     */
    dev_addr = (pkt->wValue & 0xff);
    /* Will set address in the callback phase */
    should_set_address = true;
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

/**
 * @brief Handles a SET_CONFIGRUATION request from the host. Assumes one
 *        configuration so simply sends a zero length status packet back to the
 *        host.
 *
 * @param pkt, the setup packet from the host.
 */
static void usb_set_device_configuration(volatile struct usb_setup_packet *pkt)
{
    /* Only one configuration so just acknowledge the request */
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
    configured = true;
}

static void usb_handle_report_descriptor(volatile struct usb_setup_packet *pkt)
{
    uint16_t sent = 0;
    const uint8_t *p_desc = hid_report_descriptor;
    uint8_t xfer_len;

    xfer_len = 64 > pkt->wLength ? pkt->wLength : 64;
    memcpy(ep0_buf, hid_report_descriptor, xfer_len);
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR),
                       ep0_buf, xfer_len);

    /* Update global state so we know where we are */
    if (xfer_len == 64) {
        descr_sent = xfer_len;
        descr_expected = pkt->wLength > ARRAY_SIZE(hid_report_descriptor) ?
                         ARRAY_SIZE(hid_report_descriptor) : pkt->wLength;
        usb_xfer_state = XFER_STATE_HID_REPORT;
    } else {
        usb_xfer_state = XFER_STATE_DONE;
    }
}

static void usb_handle_report_descriptor_cont(void)
{
    uint8_t xfer_len;

    xfer_len = 64 > (descr_expected - descr_sent) ?
               (descr_expected - descr_sent) : 64;
    memcpy(ep0_buf, &hid_report_descriptor[descr_sent], xfer_len);
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR),
                       ep0_buf, xfer_len);

    /* update state */
    if (xfer_len == 64) {
        descr_sent += xfer_len;
    } else {
        usb_xfer_state = XFER_STATE_DONE;
    }
}

static void usb_set_idle(volatile struct usb_setup_packet *pkt)
{
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

static void usb_handle_get_status(volatile struct usb_setup_packet *pkt)
{
    uint8_t status[2] = {0};
    uint8_t attrs = dev_config.config_descriptor->bmAttributes;

    if (attrs & 0x40) {
        status[0] |= 0x01;
    }
    if (attrs & 0x01) {
        status[0] |= 0x02;
    }

    usb_start_transfer(usb_get_endpoint_configuration(pkt->wIndex),
                       status, 2);
}

static void usb_handle_clear_feature(volatile struct usb_setup_packet *pkt)
{
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

static void usb_handle_descriptor(volatile struct usb_setup_packet *pkt)
{
    uint16_t descriptor_type = pkt->wValue >> 8;

    switch (descriptor_type) {
    case USB_DT_DEVICE:
        usb_handle_device_descriptor(pkt);
        break;

    case USB_DT_CONFIG:
        usb_handle_config_descriptor(pkt);
        break;

    case USB_DT_STRING:
        usb_handle_string_descriptor(pkt);
        break;

    case USB_HID_DESCRIPTOR_REPORT:
        usb_handle_report_descriptor(pkt);
        break;

    default:
        DB_PRINT_L(1, "Unhandled GET_DESCRIPTOR type 0x%x\r\n",
                   descriptor_type);
    }
}

static void usb_handle_setup_dir_out_rec_dev(volatile struct usb_setup_packet *
                                             pkt)
{
    uint8_t req = pkt->bRequest;

    switch (req) {
    case USB_REQUEST_SET_ADDRESS:
        usb_set_device_address(pkt);
        break;
    case USB_REQUEST_SET_CONFIGURATION:
        usb_set_device_configuration(pkt);
        break;
    default:
        DB_PRINT_L(1, "Other OUT request (0x%x)\r\n", pkt->bRequest);
        break;
    }
}

static void usb_handle_setup_dir_out_rec_int(volatile struct usb_setup_packet *
                                             pkt)
{
    uint8_t req = pkt->bRequest;

    switch (req) {
    case USB_REQUEST_SET_IDLE:
        usb_set_idle(pkt);
        break;
    default:
        DB_PRINT_L(1, "Other OUT request (0x%x)\r\n", pkt->bRequest);
        break;
    }
}

static void usb_handle_setup_dir_out_rec_ep(volatile struct usb_setup_packet *
                                             pkt)
{
    uint8_t req = pkt->bRequest;

    switch (req) {
    case USB_REQUEST_GET_STATUS:
        usb_handle_get_status(pkt);
        break;
    case USB_REQUEST_CLEAR_FEATURE:
        usb_handle_clear_feature(pkt);
        break;
    default:
        DB_PRINT_L(1, "Other OUT request (0x%x)\r\n", pkt->bRequest);
        break;
    }
}

static void usb_handle_setup_dir_out(volatile struct usb_setup_packet *pkt)
{
    uint8_t req_recip = (pkt->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK);

    switch (req_recip) {
    case USB_REQ_TYPE_RECIPIENT_DEVICE:
        usb_handle_setup_dir_out_rec_dev(pkt);
        break;
    case USB_REQ_TYPE_RECIPIENT_INTERFACE:
        usb_handle_setup_dir_out_rec_int(pkt);
        break;
    case USB_REQ_TYPE_RECIPIENT_ENDPOINT:
        usb_handle_setup_dir_out_rec_ep(pkt);
        break;
    default:
        DB_PRINT_L(1, "Cannot handle setup OUT recipient %d\n", req_recip);
        break;
    }
}

static void usb_handle_setup_dir_in_rec_dev(volatile struct usb_setup_packet *
                                            pkt)
{
    uint8_t req = pkt->bRequest;

    switch (req) {
    case USB_REQUEST_GET_DESCRIPTOR:
        usb_handle_descriptor(pkt);
        break;
    default:
        DB_PRINT_L(1, "Other IN request (0x%x)\r\n", pkt->bRequest);
        break;
    }
}

static void usb_handle_setup_dir_in_rec_int(volatile struct usb_setup_packet *
                                            pkt)
{
    uint8_t req = pkt->bRequest;

    switch (req) {
    case USB_REQUEST_GET_DESCRIPTOR:
        usb_handle_descriptor(pkt);
        break;
    default:
        DB_PRINT_L(1, "Unhandled request 0x%x\r\n", req);
        break;
    }
}

static void usb_handle_setup_dir_in(volatile struct usb_setup_packet *pkt)
{
    uint8_t req_recip = (pkt->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK);

    switch(req_recip) {
    case USB_REQ_TYPE_RECIPIENT_DEVICE:
        usb_handle_setup_dir_in_rec_dev(pkt);
        break;
    case USB_REQ_TYPE_RECIPIENT_INTERFACE:
        usb_handle_setup_dir_in_rec_int(pkt);
        break;
    default:
        DB_PRINT_L(1, "Cannot handle setup IN recipient %d\n", req_recip);
        break;
    }
}

/**
 * @brief Respond to a setup packet from the host.
 */
static void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *)
                                            &usb_dpram->setup_packet;
    uint8_t req_direction = (pkt->bmRequestType & USB_REQ_TYPE_DIR_MASK);

    /* Reset PID to 1 for EP0 IN */
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;

    switch (req_direction) {
    case USB_DIR_OUT:
        usb_handle_setup_dir_out(pkt);
        break;
    case USB_DIR_IN:
        usb_handle_setup_dir_in(pkt);
        break;
    default:
        DBG_ASSERT(0);
    }
}

/**
 * @brief Notify an endpoint that a transfer has completed.
 *
 * @param ep, the endpoint to notify.
 */
static void usb_handle_ep_buf_done(struct usb_endpoint_configuration *ep)
{
    uint32_t buffer_control = *ep->buffer_control;
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK;

    ep->handler((uint8_t *) ep->data_buffer, len);
}

/**
 * @brief Find the endpoint configuration for a specified endpoint number and
 *        direction and notify it that a transfer has completed.
 *
 * @param ep_num
 * @param in
 */
static void usb_handle_buf_done(uint ep_num, bool in)
{
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);

    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint_configuration *ep = &dev_config.endpoints[i];
        if (ep->descriptor && ep->handler) {
            if (ep->descriptor->bEndpointAddress == ep_addr) {
                usb_handle_ep_buf_done(ep);
                return;
            }
        }
    }
}

/**
 * @brief Handle a "buffer status" irq. This means that one or more
 *        buffers have been sent / received. Notify each endpoint where this
 *        is the case.
 */
static void usb_handle_buf_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            usb_handle_buf_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

/**
 * @brief USB interrupt handler
 */
__irq_handler void isr_usbctrl(void) {
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    /* Setup packet received */
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }

    /* Buffer status, one or more buffers have completed */
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buf_status();
    }

    /* Bus is reset */
    if (status & USB_INTS_BUS_RESET_BITS) {
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }
}

static void usb_handle_xfer(USBXferState state)
{
    switch(state) {
    case XFER_STATE_HID_REPORT:
        usb_handle_report_descriptor_cont();
        break;
    }
}

void ep0_in_cb(uint8_t *buf, uint16_t len)
{
    if (should_set_address) {
        /* Set actual device address in hardware */
        usb_hw->dev_addr_ctrl = dev_addr;
        should_set_address = false;
    } else {
        if (usb_xfer_state != XFER_STATE_DONE) {
            usb_handle_xfer(usb_xfer_state);
        } else {
            /* Receive a zero length status packet from the host on EP0 OUT */
            usb_start_transfer(usb_get_endpoint_configuration(EP0_OUT_ADDR),
                               NULL, 0);
        }
    }
}

void ep0_out_cb(uint8_t *buf, uint16_t len)
{
}

bool usb_is_configured(void)
{
    return configured;
}

/*
 * Modify EP0 callbacks at your own risk!
 * ep_addr should have bit 7 set if they're IN EPs
 */
bool usb_ep_add_callback(uint8_t ep_addr, usb_ep_handler cb)
{
    size_t i;
    bool found = false;

    for (i = 0; i < ARRAY_SIZE(dev_config.endpoints); ++i) {
        if (dev_config.endpoints[i].descriptor->bEndpointAddress == ep_addr) {
            dev_config.endpoints[i].handler = cb;
            found = true;
            break;
        }
    }

    return found;
}
