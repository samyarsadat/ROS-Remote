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
#include "pico_log_lib/internal/common.h"


// Logger config
#define LOGGER_LOG_LEVEL   LOG_LVL_DEBUG
#define LOGGER_LOG_FORMAT  "[%TSTMP%] [%LVL%] [%FUNC%:%LINE%] [%TASK%]: %MSG%"

// Repeating timer intervals
#define SW_STATE_REPORT_INTERVAL    50    // In milliseconds
#define AXES_STATE_REPORT_INTERVAL  50    // In milliseconds
#define POT_STATE_REPORT_INTERVAL   50    // In milliseconds
#define LED_SLOW_FLASH_INTERVAL     800   // In milliseconds
#define LED_FAST_FLASH_INTERVAL     400   // In milliseconds
#define LED_FADE_EXEC_INTERVAL      10    // In milliseconds

// LEDs
#define LED_FAST_FADING_TIME_MS             400   // In milliseconds
#define LED_SLOW_FADING_TIME_MS             800   // In milliseconds
#define LED_TEST_DELAY_TICKS                100   // In FreeRTOS ticks
#define LED_STATE_REPORT_RETRY_COOLDOWN_MS  10    // In milliseconds

// Joystick
#define JOYSTICK_X_DEADZONE                100
#define JOYSTICK_Y_DEADZONE                100
#define JOYSTICK_X_CENTER_OFFSET           0
#define JOYSTICK_Y_CENTER_OFFSET           0
#define JOYSTICK_Y_INVERTED                false
#define JOYSTICK_X_INVERTED                false
#define JOYSTICK_AXIS_SWAP_BUTTON_ENABLED  true
#define JOYSTICK_AXIS_SWAP_BUTTON_NUM      1

// Potentiometer
#define POTENTIOMETER_INVERTED  true

// Momentary buttons
#define BUTTON_BOUNCE_TIME_MS    50   // De-bouncing bounce time, in milliseconds
#define BUTTON_POLL_INTERVAL_MS  10   // Polling task exec. interval, in milliseconds

// FreeRTOS task stack sizes (all in FreeRTOS words)
#define SETUP_TASK_STACK_DEPTH  1024
#define TIMER_TASK_STACK_DEPTH  512
#define TUSB_TASK_STACK_DEPTH   2048

// Status LED
#define STAT_LED_IDLE_CHECK_MS     1000   // In milliseconds
#define STAT_LED_SUSPENDED_MS      500    // In milliseconds
#define STAT_LED_UNMOUNTED_MS      250    // In milliseconds

// Task priorities
#define SETUP_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define TUSB_TASK_PRIORITY                (configMAX_PRIORITIES - 1)
#define AXES_REPORT_TASK_PRIORITY         (configMAX_PRIORITIES - 2)
#define BUTTON_REPORT_TASK_PRIORITY       (configMAX_PRIORITIES - 2)
#define BUTTON_POLL_TASK_PRIORITY         (configMAX_PRIORITIES - 2)
#define SW_REPORT_TASK_PRIORITY           (configMAX_PRIORITIES - 3)
#define POT_REPORT_TASK_PRIORITY          (configMAX_PRIORITIES - 3)
#define LED_STATES_REPORT_TASK_PRIORITY   (configMAX_PRIORITIES - 4)

// Misc.
#define TIMER_COMMAND_TIMEOUT_T    500    // In FreeRTOS ticks
#define TUSB_TASK_EXEC_RATE_MS     10     // In milliseconds
