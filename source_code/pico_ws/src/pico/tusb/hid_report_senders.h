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


#ifdef __cplusplus
extern "C" 
{
#endif
    // ---- Task handles ----
    extern TaskHandle_t report_sw_states_th, report_button_states_th, 
                        report_axes_states_th, report_pot_state_th;
    extern TickType_t report_sw_states_lst, report_button_states_lst,   // Last Send Tick
                      report_pot_state_lst, report_axes_states_lst;

    // ---- Reporter tasks ----
    void create_hid_reporter_tasks();
    void delete_hid_reporter_tasks();

    // ---- Timer control ----
    void start_hid_reporters(alarm_pool_t* alarm_pool);
    void stop_hid_reporters();
#ifdef __cplusplus
}
#endif