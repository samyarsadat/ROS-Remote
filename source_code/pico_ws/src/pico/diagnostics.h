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
#include "pico_log_lib/logger_c.h"


#ifdef __cplusplus
#include "pico_log_lib/logger.h"
extern Logger logger;
#define LOG(lvl, msg, ...) logger.log(__func__, "", __LINE__, lvl, msg, ##__VA_ARGS__);
#else
extern logger_handle_t logger_handle;
#define LOG(lvl, msg, ...) logger_log(logger_handle, __func__, "", __LINE__, lvl, msg, ##__VA_ARGS__);
#endif
