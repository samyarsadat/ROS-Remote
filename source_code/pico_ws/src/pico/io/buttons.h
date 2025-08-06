/*
    The ROS remote project - Buttons related IO helpers
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
#include "FreeRTOS.h"
#include "task.h"


#ifdef __cplusplus
extern "C" 
{
#endif
    extern const uint8_t momen_btn_pins_order[];
    extern uint8_t momen_btn_ls_state, momen_btn_states;  // Lower 5 bits used.
    extern TaskHandle_t button_poll_task_th;

    // ---- Initialize momentary button pins ----
    void init_momentary_button_pins();

    // ---- Task creation & deletion ----
    void create_button_poll_task();
    void delete_button_poll_task();
#ifdef __cplusplus
}
#endif