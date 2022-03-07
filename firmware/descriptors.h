/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DESCRIPTORS_H
#define DESCRIPTORS_H

#include <stdint.h>

#include "usb_common.h"

#define EP0_IN_ADDR EPX_IN_ADDR(0)
#define EP0_OUT_ADDR EPX_OUT_ADDR(0)
#define EP1_IN_ADDR EPX_IN_ADDR(1)
#define EP2_OUT_ADDR EPX_OUT_ADDR(1)

struct usb_device_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} __packed;

struct usb_configuration_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} __packed;

/*
 * TODO: We only support sending one descriptor right now, because we only
 * care about sending the report descriptor so the device can function.
 *
 * To fix this, we would need to create an array that contains each descriptor
 * type, its size, and then functions to handle sending those descriptors when
 * requested.
 *
 * This would also need more effort when sending the HID descriptor.
 * Instead of doing a memcpy into the USB buffer, we would need to parse the
 * HID descriptor struct and the descriptors it points to, then copy those to
 * the USB buffer.
 */
struct usb_hid_descriptor {
    uint8_t bLength;
    uint8_t bHdrDescriptorType; /* USB uses the name bDescritptorType twice */
    uint16_t bcdHID;
    uint8_t bCountryCode;
    uint8_t bNumDescriptors;
    uint8_t bDescriptorType;
    uint16_t wDescriptorLength;
} __packed;

struct usb_interface_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} __packed;

struct usb_endpoint_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} __packed;

struct usb_endpoint_descriptor_long {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
    uint8_t bRefresh;
    uint8_t bSyncAddr;
} __attribute__((packed));

extern const uint8_t hid_report_descriptor[137];

extern const struct usb_endpoint_descriptor ep0_out;
extern const struct usb_endpoint_descriptor ep0_in;
extern const struct usb_device_descriptor device_descriptor;

extern const struct usb_interface_descriptor interface_descriptor;
extern const struct usb_hid_descriptor hid_descriptor;
extern const struct usb_endpoint_descriptor ep1_in;
extern const struct usb_endpoint_descriptor ep2_out;
extern const struct usb_configuration_descriptor config_descriptor;
extern const unsigned char lang_descriptor[];
extern const unsigned char *descriptor_strings[];

#endif
