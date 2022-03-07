/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_common.h"
#include "utils.h"
#include "proc_queue.h"
#include "descriptors.h"

/*
 * Shamelessly stolen from Genbu.
 * TODO: comment and make pretty.
 */
const uint8_t hid_report_descriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x15, 0x00, 0x25, 0x01,
    0x35, 0x00, 0x45, 0x01, 0x75, 0x01, 0x95, 0x0D, 0x05, 0x09,
    0x19, 0x01, 0x29, 0x0D, 0x81, 0x02, 0x95, 0x03, 0x81, 0x01,
    0x05, 0x01, 0x25, 0x07, 0x46, 0x3B, 0x01, 0x75, 0x04, 0x95,
    0x01, 0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x65, 0x00, 0x95,
    0x01, 0x81, 0x01, 0x26, 0xFF, 0x00, 0x46, 0xFF, 0x00, 0x09,
    0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35, 0x75, 0x08, 0x95,
    0x04, 0x81, 0x02, 0x06, 0x00, 0xFF, 0x09, 0x20, 0x09, 0x21,
    0x09, 0x22, 0x09, 0x23, 0x09, 0x24, 0x09, 0x25, 0x09, 0x26,
    0x09, 0x27, 0x09, 0x28, 0x09, 0x29, 0x09, 0x2A, 0x09, 0x2B,
    0x95, 0x0C, 0x81, 0x02, 0x0A, 0x21, 0x26, 0x95, 0x08, 0xB1,
    0x02, 0x0A, 0x21, 0x26, 0x91, 0x02, 0x26, 0xFF, 0x03, 0x46,
    0xFF, 0x03, 0x09, 0x2C, 0x09, 0x2D, 0x09, 0x2E, 0x09, 0x2F,
    0x75, 0x10, 0x95, 0x04, 0x81, 0x02, 0xC0
};

const struct usb_endpoint_descriptor ep0_out = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP0_OUT_ADDR,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
};

const struct usb_endpoint_descriptor ep0_in = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP0_IN_ADDR,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 64,
        .bInterval        = 0
};

const struct usb_device_descriptor device_descriptor = {
        .bLength         = sizeof(struct usb_device_descriptor),
        .bDescriptorType = USB_DT_DEVICE,
        .bcdUSB          = 0x0110, /* USB 1.1 device */
        .bDeviceClass    = 0,      /* Specified in interface descriptor */
        .bDeviceSubClass = 0,      /* No subclass */
        .bDeviceProtocol = 0,      /* No protocol */
        .bMaxPacketSize0 = 64,     /* Max packet size for ep0 */
        /*
         * Mooch off of Hori, the Switch is very picky with VID/PID combos,
         * and it seems like it prevents ones from working that behave
         * incorrectly.
         */
        .idVendor        = 0x0f0d, /* VID */
        .idProduct       = 0x00c1, /* PID */
        .bcdDevice       = 0x0,    /* Revision number */
        .iManufacturer   = 1,      /* Manufacturer string index */
        .iProduct        = 2,      /* Product string index */
        .iSerialNumber = 0,        /* No serial number */
        .bNumConfigurations = 1    /* One configuration */
};

const struct usb_interface_descriptor interface_descriptor = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bInterfaceNumber   = 0,
        .bAlternateSetting  = 0,
        .bNumEndpoints      = 2,
        .bInterfaceClass    = USB_INTERFACE_CLASS_HID,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = USB_INTERFACE_PROTOCOL_HID_NONE,
        .iInterface         = 0
};

const struct usb_hid_descriptor hid_descriptor = {
        .bLength            = sizeof(struct usb_hid_descriptor),
        .bHdrDescriptorType = USB_HID_DESCRIPTOR_HID,
        .bcdHID             = 0x0111,
        .bCountryCode       = 0,
        .bNumDescriptors    = 1,
        .bDescriptorType    = USB_HID_DESCRIPTOR_REPORT,
        .wDescriptorLength  = ARRAY_SIZE(hid_report_descriptor)
};

const struct usb_endpoint_descriptor ep1_in = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP1_IN_ADDR,
        .bmAttributes     = USB_TRANSFER_TYPE_INTERRUPT,
        .wMaxPacketSize   = 64,
        .bInterval        = 1
};

const struct usb_endpoint_descriptor ep2_out = {
        .bLength          = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = EP2_OUT_ADDR,
        .bmAttributes     = USB_TRANSFER_TYPE_INTERRUPT,
        .wMaxPacketSize   = 64,
        .bInterval        = 1
};

const struct usb_configuration_descriptor config_descriptor = {
        .bLength         = sizeof(struct usb_configuration_descriptor),
        .bDescriptorType = USB_DT_CONFIG,
        .wTotalLength    = (sizeof(config_descriptor) +
                            sizeof(interface_descriptor) +
                            sizeof(ep1_in) +
                            sizeof(ep2_out) +
                            sizeof(hid_descriptor)),
        .bNumInterfaces  = 1,
        .bConfigurationValue = 1, /* Configuration 1 */
        .iConfiguration = 0,      /* No string */
        .bmAttributes = 0xc0,     /* attributes: self powered, no remote wakeup */
        .bMaxPower = 0xfa         /* 500ma */
};

const unsigned char lang_descriptor[] = {
        4,         /* bLength */
        0x03,      /* bDescriptorType == String Descriptor */
        0x09, 0x04 /* language id = us english */
};

const unsigned char *descriptor_strings[] = {
        "Raspberry Pi",    /* Vendor */
        "Groove Coaster Controller" /* Product */
};
