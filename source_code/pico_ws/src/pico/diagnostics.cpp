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

#include "diagnostics.h"
#include "config/sw_defs.h"


// Logger configuration
logger_options_t logger_options = {
    .logging_level = LOGGER_LOG_LEVEL,
    .log_format = LOGGER_LOG_FORMAT,
    .ansi_styling = true,
    .process_style_tags = false  
};
Logger logger(&stdio_uart, &logger_options);

// Logger handle for C API
logger_handle_t logger_handle = static_cast<logger_handle_t>(&logger);