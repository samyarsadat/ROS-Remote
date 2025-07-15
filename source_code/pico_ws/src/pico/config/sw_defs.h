/*
    The ROS remote project - Firmware software definitions
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
#include "pico_log_lib/logger.h"


// micro-ROS node config
#define UROS_NODE_NAME                     "pico"
#define UROS_NODE_NAMESPACE                ""
#define UROS_DOMAIN_ID                     75
#define AGENT_WAITING_LED_TOGGLE_DELAY_MS  500   // In milliseconds
#define AGENT_AVAIL_LED_TOGGLE_DELAY_MS    250   // In milliseconds
#define EXECUTOR_EXEC_INTERVAL_MS          50    // In milliseconds
#define EXECUTOR_EXEC_TIME_LIMIT_MS        60    // In milliseconds
#define EXECUTOR_TIMEOUT_MS                5     // In milliseconds

// Logger config
#define LOOGER_LOG_LEVEL   LOG_LVL_DEBUG
#define LOGGER_LOG_FORMAT  "[%TSTMP%] [%LVL%] [%FUNC%:%LINE%] [%TASK%]: %MSG%"

// Repeating timer intervals
#define SW_STATE_PUB_RT_INTERVAL       100   // In milliseconds
#define JOYSTICK_PUB_RT_INTERVAL       100   // In milliseconds
#define POTENTIOMETER_PUB_RT_INTERVAL  100   // In milliseconds
#define LED_SLOW_FLASH_INTERVAL        800   // In milliseconds
#define LED_FAST_FLASH_INTERVAL        400   // In milliseconds
#define LED_FADE_EXEC_INTERVAL         10    // In milliseconds

// LEDs
#define LED_FAST_FADING_TIME_MS  400   // In milliseconds
#define LED_SLOW_FADING_TIME_MS  800   // In milliseconds

// Joystick
#define DEFAULT_JOYSTICK_X_DEADZONE       100
#define DEFAULT_JOYSTICK_Y_DEADZONE       100
#define DEFAULT_JOYSTICK_X_CENTER_OFFSET  0
#define DEFAULT_JOYSTICK_Y_CENTER_OFFSET  0
#define JOYSTICK_Y_INVERTED               false
#define JOYSTICK_X_INVERTED               false

// Potentiometer
#define POTENTIOMETER_INVERTED  true

// FreeRTOS task stack sizes (all in FreeRTOS words)
#define SETUP_TASK_STACK_DEPTH   1024
#define RESET_TASK_STACK_DEPTH   1024
#define TIMER_TASK_STACK_DEPTH   512
#define LED_ST_TASK_STACK_DEPTH  128

// Misc.
#define TIMER_COMMAND_TIMEOUT_T    500    // In FreeRTOS ticks
#define WATCHDOG_RESET_TIMEOUT_MS  1000   // In milliseconds
#define PRE_RESET_WAIT_MS          5      // In milliseconds