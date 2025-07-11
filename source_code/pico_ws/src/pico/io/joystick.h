/*
    The ROS remote project - Joystick and potentiometer helper
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
#include "config/sw_defs.h"
#include "FreeRTOS.h"
#include "timers.h"


float joystick_x_center_offset = DEFAULT_JOYSTICK_X_CENTER_OFFSET;
float joystick_y_center_offset = DEFAULT_JOYSTICK_Y_CENTER_OFFSET;
uint16_t joystick_x_deadzone = DEFAULT_JOYSTICK_X_DEADZONE;
uint16_t joystick_y_deadzone = DEFAULT_JOYSTICK_Y_DEADZONE;


// ---- Get joystick axis positions (readings) ----
// ---- These functions take into account the deadzone, offset, and inversion configs of the axis ----
// ---- They return values between -512 and +512, with 0 being center ----
int16_t get_joystick_x_val();
int16_t get_joystick_y_val();

// ---- Get potentiometer reading ----
// ---- This function takes into account the potentiometer's inversion config ----
// ---- It returns a value between 0 and 1024 ----
uint16_t get_potentiometer_val();