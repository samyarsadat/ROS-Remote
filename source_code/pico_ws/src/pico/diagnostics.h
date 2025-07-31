/*
    The ROS remote project - Looger init
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
#include "pico_log_lib/logger.h"
#include "config/sw_defs.h"


// Logger configuration
inline logger_options_t logger_options = {
    .logging_level = LOOGER_LOG_LEVEL,
    .log_format = LOGGER_LOG_FORMAT,
    .ansi_styling = true,
    .process_style_tags = false  
};

inline Logger logger(&stdio_usb, &logger_options);
#define LOG(lvl, msg, ...) logger.log(__func__, "", __LINE__, lvl, msg, ##__VA_ARGS__);

// Reset task handle
inline TaskHandle_t reset_task_handle;

// Utility macros
#define REQ_SYSTEM_RESET()                     \
    (void) xTaskNotifyGive(reset_task_handle); \
    while (1);

#define REQ_SYSTEM_RESET_ISR()                          \
    vTaskNotifyGiveFromISR(reset_task_handle, nullptr); \
    while (1);
