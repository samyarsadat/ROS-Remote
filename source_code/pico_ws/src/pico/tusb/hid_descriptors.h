/*
    The ROS remote project - Raspberry Pi Pico firmware
    Copyright 2025 Samyar Sadat Akhavi.
    Written by Samyar Sadat Akhavi, 2025.
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
  
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once
#include "pico/stdlib.h"
#include <config/hw_defs.h>


#define AXES_INPUT_REPORT_ID              0x01
#define POT_INPUT_REPORT_ID               0x02
#define BUTTONS_INPUT_REPORT_ID           0x03
#define SWITCHES_INPUT_REPORT_ID          0x04
#define INPUT_POLL_OUTPUT_REPORT_ID       0x05
#define LED_OUTPUT_REPORT_ID              0x06
#define LED_STATES_FEATURE_REPORT_ID      0x07

// Interfaces
enum {
    ITF_NUM_JOYSTICK_HID,
    ITF_NUM_LEDS_HID,
    ITF_NUM_TOTAL
};

// Input (joystick axes) report (ID: 0x01, 6 bytes)
typedef struct {
    int16_t x;      // -512-512
    int16_t y;      // -512-512
    int16_t rz;     // -512-512
} __attribute__((packed)) hid_joy_axes_report_t;

// Input (potentiometer) report (ID: 0x02, 1 byte)
typedef struct {
    int8_t pot;   // -100-0
} __attribute__((packed)) hid_pot_report_t;

// Input (momentary buttons) report (ID: 0x03, 1 byte)
typedef struct {
    uint8_t buttons;   // Lower 5 bits used
} __attribute__((packed)) hid_buttons_report_t;

// Input (toggle switches) report (ID: 0x04, 1 byte)
typedef struct {
    uint8_t switches;   // Lower 5 bits used
} __attribute__((packed)) hid_switches_report_t;

// Output (LEDs) report (ID: 0x06, 4 bytes)
typedef struct {
    uint8_t index;      // 0-10
    uint8_t mode;       // 0-4
    uint16_t pwm_out;   // 0-65535
} __attribute__((packed)) hid_led_report_t;

// Input (LED states) report (ID: 0x07, 33 bytes)
typedef struct {
    uint8_t mode[NUMBER_OF_LEDS];       // 0-4 * 11
    uint16_t pwm_out[NUMBER_OF_LEDS];   // 0-65535 * 11
} __attribute__((packed)) hid_led_states_report_t;


static const uint8_t hid_joystick_report_desc[] = {
    0x05, 0x01,   // Usage Page: Generic Desktop
    0x09, 0x04,   // Usage: Joystick
    
    // Collection: Application
    0xA1, 0x01,            
        // ==== ANALOG AXES (X, Y, Rz, pot) ====
        0x85, AXES_INPUT_REPORT_ID,   // Report ID for axes input data
        0x05, 0x01,                   // Usage Page: Generic Desktop
        0x09, 0x30,                   // Usage: X axis
        0x09, 0x31,                   // Usage: Y axis
        0x09, 0x35,                   // Usage: Rz (yaw)
        0x16, 0x00, 0xFE,             // Logical minimum: -512 (two's complement)
        0x26, 0x00, 0x02,             // Logical maximum: +512
        0x75, 0x10,                   // Report size: 16 bits
        0x95, 0x03,                   // Report count: 3 (X, Y, and Rz)
        0x81, 0x02,                   // Input: Data, Variable, Absolute
        
        // === POTENTIOMETER (Slider) ===
        0x85, POT_INPUT_REPORT_ID,   // Report ID for potentiometer input data
        0x05, 0x01,                  // Usage Page: Generic Desktop
        0x09, 0x36,                  // Usage: Slider
        0x15, 0x9C,                  // Logical minimum: -100
        0x25, 0x00,                  // Logical maximum: 0
        0x75, 0x08,                  // Report size: 8 bits
        0x95, 0x01,                  // Report count: 1
        0x81, 0x02,                  // Input: Data, Variable, Absolute
        
        // ==== MOMENTARY BUTTONS (5) ====
        0x85, BUTTONS_INPUT_REPORT_ID,   // Report ID for button input data
        0x05, 0x09,                      // Usage Page: Button
        0x19, 0x01,                      // Usage minimum: Button 1
        0x29, 0x05,                      // Usage maximum: Button 5
        0x15, 0x00,                      // Logical minimum: 0
        0x25, 0x01,                      // Logical maximum: 1
        0x75, 0x01,                      // Report size: 1 bit
        0x95, 0x05,                      // Report count: 5 momentary buttons
        0x81, 0x42,                      // Input: Data, Variable, Absolute, Preferred (MC)

        0x95, 0x03,   // 3 bits of padding
        0x81, 0x01,   // Input: Constant (padding)
        
        // ==== TOGGLE SWITCHES (5) ====
        0x85, SWITCHES_INPUT_REPORT_ID,   // Report ID for switch input data
        0x05, 0x09,                       // Usage Page: Button
        0x19, 0x06,                       // Usage minimum: Button 6
        0x29, 0x0A,                       // Usage maximum: Button 10
        0x15, 0x00,                       // Logical minimum: 0
        0x25, 0x01,                       // Logical maximum: 1
        0x75, 0x01,                       // Report size: 1 bit
        0x95, 0x05,                       // Report count: 5 toggle switches
        0x81, 0x22,                       // Input: Data, Variable, Absolute, No Preferred (OOC)
        
        0x95, 0x03,   // 3 bits of padding
        0x81, 0x01,   // Input: Constant (padding)

        // ==== EMPTY OUTPUT REPORT FOR POLLING REQUEST ====
        0x85, INPUT_POLL_OUTPUT_REPORT_ID,   // Report ID for input poll request
        0x75, 0x08,                          // Report size: 8 bits
        0x95, 0x00,                          // Report count: 0 bytes
        0x91, 0x03,                          // Output: Constant, Variable, Absolute
    0xC0   // End Collection (Application)
};

static const uint8_t hid_leds_report_desc[] = {
    0x05, 0x59,   // Usage Page: Lighting & Illumination
    0x09, 0x01,   // Usage: Lamp Array
    
    // Collection: Application
    0xA1, 0x01,            
        // ==== LED CONTROL OUTPUT ====
        0x85, LED_OUTPUT_REPORT_ID,   // Report ID for output data
        0x05, 0x08,                   // Usage Page: LEDs
        0x09, 0x4B,                   // Usage: Generic Indicator
        
        // Collection: Logical
        0xA1, 0x02,
            0x06, 0x00, 0xFF,   // Usage Page: Vendor Defined

            // === LED INDEX (0-10 for 11 LEDs) ===
            0x09, 0x01,   // Usage: LED Index
            0x15, 0x00,   // Logical minimum: 0
            0x25, 0x0A,   // Logical maximum: 10 (11 LEDs)
            0x75, 0x08,   // Report size: 8 bits
            0x95, 0x01,   // Report count: 1
            0x91, 0x02,   // Output: Data, Variable, Absolute
            
            // === LED MODE (0-4) ===
            0x09, 0x02,   // Usage: LED Mode
            0x15, 0x00,   // Logical minimum: 0
            0x25, 0x04,   // Logical maximum: 4 (5 modes)
            0x75, 0x08,   // Report size: 8 bits
            0x95, 0x01,   // Report count: 1
            0x91, 0x02,   // Output: Data, Variable, Absolute
            
            // === LED BRIGHTNESS (0-65535) ===
            0x09, 0x03,         // Usage: LED Brightness
            0x15, 0x00,         // Logical minimum: 0
            0x26, 0xFF, 0xFF,   // Logical maximum: 65535
            0x75, 0x10,         // Report size: 16 bits
            0x95, 0x01,         // Report count: 1
            0x91, 0x02,         // Output: Data, Variable, Absolute
        0xC0,   // End Collection (Logical)

        // ==== LED STATES FEATURE REPORT ====
        0x85, LED_STATES_FEATURE_REPORT_ID,   // Report ID for LED states report
        0x05, 0x08,                           // Usage Page: LEDs
        0x09, 0x4B,                           // Usage: Generic Indicator

        // Collection: Logical
        0xA1, 0x02,
            0x06, 0x00, 0xFF,   // Usage Page: Vendor Defined

            // === LED MODE (0-4) ===
            0x09, 0x02,   // Usage: LED Mode
            0x15, 0x00,   // Logical minimum: 0
            0x25, 0x04,   // Logical maximum: 4 (5 modes)
            0x75, 0x08,   // Report size: 8 bits
            0x95, 0x0B,   // Report count: 11
            0xB1, 0x02,   // Feature: Data, Variable, Absolute
            
            // === LED BRIGHTNESS (0-65535) ===
            0x09, 0x03,         // Usage: LED Brightness
            0x15, 0x00,         // Logical minimum: 0
            0x26, 0xFF, 0xFF,   // Logical maximum: 65535
            0x75, 0x10,         // Report size: 16 bits
            0x95, 0x0B,         // Report count: 11
            0xB1, 0x02,         // Feature: Data, Variable, Absolute
        0xC0,   // End Collection (Logical)
    0xC0   // End Collection (Application)
};