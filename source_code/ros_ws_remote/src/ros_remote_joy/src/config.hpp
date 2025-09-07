/*
    The ROS remote project - Remote joystick filtering ROS package
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


// Default parameter values
constexpr uint8_t DEFAULT_JOY_ACTIVE_LED_NUM = 0;
constexpr uint8_t DEFAULT_JOY_ACTIVE_BTN_NUM = 8;
constexpr uint8_t DEFAULT_JOY_OVERRIDE_BTN_NUM = 9;
constexpr uint8_t DEFAULT_REMOTE_LOCK_BTN_NUM = 5;
constexpr uint32_t DEFAULT_JOY_MUX_LOCK_PUB_RATE = 5;

// Joystick axes
enum : uint8_t { JOYSTICK_X_AXIS, JOYSTICK_Y_AXIS, JOYSTICK_RZ_AXIS, POTENTIOMETER_AXIS };

// LED modes
enum LED_MODE : uint8_t {
    LED_SOLID_PWM,
    LED_SLOW_FLASH,
    LED_SLOW_FADE,
    LED_FAST_FLASH,
    LED_FAST_FADE
};
