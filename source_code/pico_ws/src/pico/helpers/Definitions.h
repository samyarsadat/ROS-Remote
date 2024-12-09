/*
    The ROS remote project - Program Definitions
    Copyright 2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2024.
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
  
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https: www.gnu.org/licenses/>.
*/

#pragma once
#include "local_helpers_lib/Common_Definitions.h"
#include "Diag_Msgs.h"


// ------- Pin definitions -------

// ---- LEDs ----
#define right_kd2_led_pin            0
#define right_green_led_pin          1
#define right_blue_led_pin           2
#define left_top_yellow_led_pin      8
#define left_top_green_led_pin       9
#define left_red_led_pin             10
#define left_bottom_yellow_led_pin   11
#define left_bottom_green_1_led_pin  12
#define left_bottom_green_2_led_pin  13
#define left_red_kd2_led_pin         14
#define left_green_kd2_led_pin       15
#define number_of_leds               11

// ---- Toggle switches ----
#define right_top_toggle_sw_pin  3
#define left_key_sw_pin          18
#define left_top_toggle_sw_pin   19

// ---- Buttons ----
#define right_e_stop_btn_pin  4
#define right_kd2_btn_pin     5

// ---- Momentary Buttons ----
#define left_green_right_btn_pin     6
#define left_red_btn_pin             7
#define left_green_kd2_btn_pin       20
#define left_red_kd2_btn_pin         21
#define left_green_left_btn_pin      22
#define number_of_momentary_buttons  5
#define button_bounce_time_ms        200   // In milliseconds

// ---- Joystick ----
#define joystick_y_axis_pin  26
#define joystick_x_axis_pin  27

// ---- Potentiometer ----
#define potentiometer_pin  28

// ---- Pico ----
#define smps_power_save_pin  23


// ------- Other definitions -------

// ---- MicroROS node config ----
#define UROS_NODE_NAME                     "pico"
#define UROS_NODE_NAMESPACE                ""
#define UROS_DOMAIN_ID                     75
#define AGENT_WAITING_LED_TOGGLE_DELAY_MS  500   // In milliseconds
#define AGENT_AVAIL_LED_TOGGLE_DELAY_MS    250   // In milliseconds
#define EXECUTOR_EXEC_INTERVAL_MS          75    // In milliseconds
#define EXECUTOR_EXEC_TIME_LIMIT_MS        85    // In milliseconds
#define EXECUTOR_TIMEOUT_MS                50    // In milliseconds

// ---- Repeating timer intervals ----
#define sw_state_pub_rt_interval       100   // In milliseconds
#define joystick_pub_rt_interval       100   // In milliseconds
#define potentiometer_pub_rt_interval  100   // In milliseconds
#define led_slow_flash_interval        800   // In milliseconds
#define led_fast_flash_interval        400   // In milliseconds
#define led_fade_exec_interval         10    // In milliseconds

// ---- LEDs ----
#define led_fast_fading_time_ms  400   // In milliseconds
#define led_slow_fading_time_ms  800   // In milliseconds

// ---- Joystick ----
#define default_joystick_x_deadzone       100
#define default_joystick_y_deadzone       100
#define default_joystick_x_center_offset  0
#define default_joystick_y_center_offset  0
#define joystick_y_inverted               false
#define joystick_x_inverted               false

// ---- Potentiometer ----
#define potentiometer_inverted  true

// ---- Misc. ----
#define SETUP_TASK_STACK_DEPTH    1024       // In FreeRTOS words
#define TIMER_TASK_STACK_DEPTH    1024       // In FreeRTOS words
#define GENERIC_TASK_STACK_DEPTH  1024       // In FreeRTOS words
#define STARTUP_WAIT_TIME_S       3          // In seconds