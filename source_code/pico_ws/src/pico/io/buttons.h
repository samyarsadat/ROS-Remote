/*
    The ROS remote project - Buttons related IO helpers
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
#include "config/hw_defs.h"


const uint8_t momen_btn_pins_order[NUMBER_OF_MOMENTARY_BUTTONS] = {LEFT_GREEN_RIGHT_BTN_PIN, LEFT_RED_BTN_PIN, LEFT_GREEN_KD2_BTN_PIN,
                                                                   LEFT_RED_KD2_BTN_PIN, LEFT_GREEN_LEFT_BTN_PIN};
uint32_t momen_btn_lit[NUMBER_OF_MOMENTARY_BUTTONS] = {0};   // Last interrupt receive time (Last Interrupt Time)


// ---- Button de-bouncing function ----
// ---- Returns true if the button should be considered pressed, false if not. ----
// ---- NOTE: Only for the 5 momentary push buttons! ----
bool button_bounce_check(uint8_t pin);

// ---- Initialize momentary button pins ----
void init_momentary_buttons();