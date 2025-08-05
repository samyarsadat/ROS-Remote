/*
    The ROS remote project - Firmware hardware definitions
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


// LEDs
#define RIGHT_KD2_LED_PIN            0
#define RIGHT_GREEN_LED_PIN          1
#define RIGHT_BLUE_LED_PIN           2
#define LEFT_TOP_YELLOW_LED_PIN      8
#define LEFT_TOP_GREEN_LED_PIN       9
#define LEFT_RED_LED_PIN             10
#define LEFT_BOTTOM_YELLOW_LED_PIN   11
#define LEFT_BOTTOM_GREEN_1_LED_PIN  12
#define LEFT_BOTTOM_GREEN_2_LED_PIN  13
#define LEFT_RED_KD2_LED_PIN         14
#define LEFT_GREEN_KD2_LED_PIN       15
#define NUMBER_OF_LEDS               11   // Total number of LEDs

// Toggle switches
#define RIGHT_TOP_TOGGLE_SW_PIN  3
#define LEFT_KEY_SW_PIN          18
#define LEFT_TOP_TOGGLE_SW_PIN   19

// Buttons (toggle)
#define RIGHT_E_STOP_BTN_PIN  4
#define RIGHT_KD2_BTN_PIN     5

// Momentary buttons
#define LEFT_GREEN_RIGHT_BTN_PIN     6
#define LEFT_RED_BTN_PIN             7
#define LEFT_GREEN_KD2_BTN_PIN       20
#define LEFT_RED_KD2_BTN_PIN         21
#define LEFT_GREEN_LEFT_BTN_PIN      22
#define NUMBER_OF_MOMENTARY_BUTTONS  5     // Total number of momentary buttons

// Joystick
#define JOYSTICK_Y_AXIS_PIN  26
#define JOYSTICK_X_AXIS_PIN  27

// Potentiometer
#define POTENTIOMETER_PIN  28