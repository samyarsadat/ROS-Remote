/*
    The ROS remote project - State management functions
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
#include "FreeRTOS.h"
#include "timers.h"


#ifdef __cplusplus
extern "C"
{
#endif
    extern TimerHandle_t idle_hid_report_timer;

    // ---- Device state management ----
    void enter_state_mounted();
    void enter_state_unmounted();
    void enter_state_suspended();
    void enter_state_resumed();
#ifdef __cplusplus
}
#endif