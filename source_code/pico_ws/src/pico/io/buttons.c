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

#include "buttons.h"
#include "utils_lib/hardware.h"
#include "tusb/hid_report_senders.h"
#include "config/hw_defs.h"
#include "config/sw_defs.h"
#include "utils_lib/perf/profile.h"
#include "utils_lib/perf/exec_interval.h"
#include "diagnostics.h"


// Last button state change times (Last State Change)
uint32_t momen_btn_lsc[NUMBER_OF_MOMENTARY_BUTTONS] = {0};
uint8_t momen_btn_ls_state = 0, momen_btn_states = 0;  // LS State: Last Sent State - Lower 5 bits used.
TaskHandle_t button_poll_task_th = NULL;
const uint8_t momen_btn_pins_order[NUMBER_OF_MOMENTARY_BUTTONS] = {
    LEFT_GREEN_RIGHT_BTN_PIN, LEFT_RED_BTN_PIN, LEFT_GREEN_KD2_BTN_PIN,
    LEFT_RED_KD2_BTN_PIN, LEFT_GREEN_LEFT_BTN_PIN
};


// ---- Button polling task ----
void button_poll_task(void *parameters) {
    (void) parameters;
    TickType_t curr_time, wake = xTaskGetTickCount();
    uint32_t last_exec_time = 0;
    bool btn_state;

    while (true) {
        CHECK_EXEC_INTERVAL_DBG(&last_exec_time, (BUTTON_POLL_INTERVAL_MS + 2), "Button polling interval time limit exceeded!");
        curr_time = xTaskGetTickCount();

        if (momen_btn_states != momen_btn_ls_state) {
            xTaskNotifyGive(report_button_states_th);
            continue;   // Skip this cycle. We don't want to send two notifications this close to each other.
        }

        for (int i = 0; i < NUMBER_OF_MOMENTARY_BUTTONS; i++) {
            btn_state = !gpio_get(momen_btn_pins_order[i]);

            if (btn_state != ((momen_btn_states >> i) & 1) && (curr_time - momen_btn_lsc[i]) > pdMS_TO_TICKS(BUTTON_BOUNCE_TIME_MS)) {
                momen_btn_lsc[i] = curr_time;

                if (btn_state) {
                    momen_btn_states |= (1 << i);
                } else {
                    momen_btn_states &= ~(1 << i);
                }

                xTaskNotifyGive(report_button_states_th);
            }
        }

        xTaskDelayUntil(&wake, pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS));
    }
}

// ---- Initialize momentary button pins ----
void init_momentary_button_pins() {
    for (int i = 0; i < NUMBER_OF_MOMENTARY_BUTTONS; i++) {
        init_pin(momen_btn_pins_order[i], INPUT_PULLUP);
    }
}

// ---- Task creation & deletion ----
void create_button_poll_task() {
    assert(button_poll_task_th == NULL);
    (void) xTaskCreate(button_poll_task, "button_poll", TIMER_TASK_STACK_DEPTH, NULL, BUTTON_POLL_TASK_PRIORITY, &button_poll_task_th);
}

void delete_button_poll_task() {
    assert(button_poll_task_th != NULL);
    vTaskDelete(button_poll_task_th);
    button_poll_task_th = NULL;
}