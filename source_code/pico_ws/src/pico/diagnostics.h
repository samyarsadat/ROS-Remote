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
#include "uros_utils_lib/diag_util.h"
#include "uros/uros_init.h"


// Logger configuration
logger_options_t logger_options = {
    .logging_level = LOOGER_LOG_LEVEL,
    .log_format = LOGGER_LOG_FORMAT,
    .ansi_styling = true,
    .process_style_tags = false  
};

Logger logger(&stdio_usb, &logger_options);
#define LOG(lvl, msg, ...) logger.log(__func__, "", __LINE__, lvl, msg, ##__VA_ARGS__);

// Micro-ROS diagnostics
DiagPublisher diag_util(&diagnostics_pub);

// Reset task handle
TaskHandle_t reset_task_handle;

// Utility macros
const char* RETCODE_LOG_MSG = "Micro-ROS operation failed! Code: %d";
const char* RETCODE_CHECK_MSG = "%s failed! Error code: %d";

#define UROS_RETCODE_LOG(ret_code)                     \
    if (ret_code != RCL_RET_OK) {                      \
        LOG(LOG_LVL_FATAL, RETCODE_LOG_MSG, ret_code); \
    }

#define UROS_RETCODE_CHECK(ret_code)                   \
    if (ret_code != RCL_RET_OK) {                      \
        LOG(LOG_LVL_FATAL, RETCODE_LOG_MSG, ret_code); \
        return false;                                  \
    }

#define UROS_RETCODE_CHECK_MSG(ret_code, msg)                 \
    if (ret_code != RCL_RET_OK) {                             \
        LOG(LOG_LVL_FATAL, RETCODE_CHECK_MSG, msg, ret_code); \
        return false;                                         \
    }

#define REQ_SYSTEM_RESET(mem_mode)                                           \
    (void) xTaskNotify(reset_task_handle, mem_mode, eSetValueWithOverwrite); \
    while (1);

#define REQ_SYSTEM_RESET_ISR(mem_mode)                                             \
    xTaskNotifyFromISR(reset_task_handle, mem_mode, eSetValueWithOverwrite, NULL); \
    while (1);