/*
    The ROS remote project - Remote HID interface ROS package
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
#include <stdint.h>
#include <cstddef>


constexpr size_t MAX_REPORT_SIZE = (33 + 1);
constexpr size_t MAX_VERSION_STRING_LEN = 30;
constexpr size_t VERSION_STRING_INDEX = 6;

// Report IDs
constexpr uint8_t LED_OUTPUT_REPORT_ID = 0x06;
constexpr uint8_t LED_STATES_FEATURE_REPORT_ID = 0x07;

// Default VID and PID
constexpr uint16_t DEFAULT_VID = 0x1FC9;
constexpr uint16_t DEFAULT_PID = 0x0001;

// Interface number
constexpr size_t ITF_NUM_LEDS_HID = 1;

// ---- Report structures ----
constexpr size_t NUMBER_OF_LEDS = 11;
constexpr size_t NUMBER_OF_LED_MODES = 5;

// Output (LEDs) report (ID: 0x06, 5 bytes)
struct hid_led_report {
        uint8_t report_id = 0x06;
        uint8_t index;
        uint8_t mode;
        uint16_t pwm_out;
} __attribute__((packed));

// Feature (LED states) report (ID: 0x07, 34 bytes)
struct hid_led_states_report {
        uint8_t report_id;
        uint8_t mode[NUMBER_OF_LEDS];
        uint16_t pwm_out[NUMBER_OF_LEDS];
} __attribute__((packed));
