/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2025 Samyar Sadat Akhavi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "hid_descriptors.h"


// The USB VID and PID used here are for testing purposes only!
#define USB_PID                 0x0001   // USB Product ID
#define USB_VID                 0x1FC9   // USB Vendor ID
#define USB_BCD                 0x0200   // USB Spec. Release Number
#define DEVICE_VER              0x0100   // Device Version
#define DEVICE_CURRENT          500      // Device Current (mA)
#define MANUFACTURER_NAME       "Samyar Projects"
#define PRODUCT_NAME            "The ROS Remote"
#define JOYSTICK_HID_ITF_NAME   "ROS Remote Joystick Interface"
#define LED_HID_ITF_NAME        "ROS Remote LED Interface"


//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = DEVICE_VER,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*) &desc_device;
}

//--------------------------------------------------------------------+
// HID Report Descriptor
//--------------------------------------------------------------------+

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const* tud_hid_descriptor_report_cb(uint8_t itf) {
    switch (itf) {
        case ITF_NUM_JOYSTICK_HID:
            return hid_joystick_report_desc;
        case ITF_NUM_LEDS_HID:
            return hid_leds_report_desc;
        default:
            return NULL;  // Unknown interface
    }
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_DESC_LEN)
#define EPNUM_HID_JOYSTICK  0x81   // Endpoint number (joystick)
#define EPNUM_HID_LEDS      0x82   // Endpoint number (LEDs)

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, DEVICE_CURRENT),
    TUD_HID_DESCRIPTOR(ITF_NUM_JOYSTICK_HID, 4, HID_ITF_PROTOCOL_NONE, sizeof(hid_joystick_report_desc), EPNUM_HID_JOYSTICK, CFG_TUD_HID_EP_BUFSIZE, 10),
    TUD_HID_DESCRIPTOR(ITF_NUM_LEDS_HID, 5, HID_ITF_PROTOCOL_NONE, sizeof(hid_leds_report_desc), EPNUM_HID_LEDS, CFG_TUD_HID_EP_BUFSIZE, 10)
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

extern char pico_unique_id[];

// array of pointer to string descriptors
char const* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },   // 0: Supported Language (English - 0x0409)
    MANUFACTURER_NAME,               // 1: Manufacturer Name
    PRODUCT_NAME,                    // 2: Product Name
    pico_unique_id,                  // 3: Unique ID/Serial Number
    JOYSTICK_HID_ITF_NAME,           // 4: Joystick HID Interface Name
    LED_HID_ITF_NAME                 // 5: LED HID Interface Name
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }

        const char* str = string_desc_arr[index];
        chr_count = strlen(str);
        
        if (chr_count > 31) {
            chr_count = 31;
        }

        for(uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}