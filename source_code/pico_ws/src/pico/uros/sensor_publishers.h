/*
    The ROS remote project - Sensor Data MicroROS Publishers
    Copyright 2024-2025 Samyar Sadat Akhavi.
    Written by Samyar Sadat Akhavi, 2024-2025.
 
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
#include "FreeRTOS.h"
#include "task.h"


// ---- Timers ----
inline TaskHandle_t btn_state_publish_th, sw_state_publish_th, joystick_publish_th, potentiometer_publish_th;


// ---- Permanent switch states ----
void publish_sw_states(void *parameters);

// ---- Momentary button states ----
void publish_btn_states(void *parameters);

// ---- Joystick state ----
void publish_joystick_state(void *parameters);

// ---- Potentiometer state ----
void publish_potentiometer_state(void *parameters);


// ---- Timer control ----
void start_sensor_publishers(alarm_pool_t* alarm_pool);
void stop_sensor_publishers();