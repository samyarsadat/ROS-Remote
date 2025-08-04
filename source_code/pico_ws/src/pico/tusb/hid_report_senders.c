/*
    The ROS remote project - HID report senders
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

#include "hid_report_senders.h"
#include "config/hw_defs.h"
#include "io/joystick.h"
#include "io/buttons.h"
#include "common/opassert.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hid_descriptors.h"
#include "utils_lib/perf/exec_interval.h"
#include "tusb.h"
#include "diagnostics.h"


// Timers & tasks
struct repeating_timer report_sw_states_rt, report_button_states_rt, report_axes_states_rt;
TaskHandle_t report_sw_states_th, report_button_states_th, report_axes_states_th;
const char* report_time_lim_msg = "HID report sending interval time limit exceeded!";


// ---- Toggle switch states ----
void report_sw_states_task(void *parameters) {
    (void) parameters;
    uint32_t last_pub_time = 0;
    uint32_t notification_value;
    bool retry_send = false;
    hid_switches_report_t report;

    while (true) {
        xTaskNotifyWait(0, 0xffffffff, &notification_value, portMAX_DELAY);
        CHECK_EXEC_INTERVAL(&last_pub_time, (SW_STATE_REPORT_INTERVAL + 10), report_time_lim_msg);

        // Explicit re-send has been requested.
        if (notification_value) {
            retry_send = true;
        }
        
        // TODO: for loop, please.
        uint8_t state = 0;
        state |= (!gpio_get(LEFT_KEY_SW_PIN)         << 0);
        state |= (!gpio_get(LEFT_TOP_TOGGLE_SW_PIN)  << 1);
        state |= (!gpio_get(RIGHT_E_STOP_BTN_PIN)    << 2);
        state |= (!gpio_get(RIGHT_KD2_BTN_PIN)       << 3);
        state |= (!gpio_get(RIGHT_TOP_TOGGLE_SW_PIN) << 4);

        if ((report.switches != state || retry_send) && tud_hid_ready()) {
            report.switches = state;
            retry_send = !tud_hid_report(SWITCHES_INPUT_REPORT_ID, &report, sizeof(report));
        }
    }
}

// ---- Momentary button states ----
void report_button_states_task(void *parameters) {
    (void) parameters;
    uint32_t last_pub_time = 0;
    uint32_t notification_value;
    bool last_report_successful = false;
    hid_buttons_report_t report;

    while (true) {
        xTaskNotifyWait(0, 0xffffffff, &notification_value, portMAX_DELAY);
        CHECK_EXEC_INTERVAL(&last_pub_time, (BTN_STATE_REPORT_INTERVAL + 10), report_time_lim_msg);
        
        if (!tud_hid_ready()) {
            continue;
        }

        if (report.buttons != momen_btn_states || !last_report_successful || notification_value == 1) {
            report.buttons = momen_btn_states;
            last_report_successful = tud_hid_report(BUTTONS_INPUT_REPORT_ID, &report, sizeof(report));
        }
    }
}

// ---- Joystick axes & potentiometer state ----
void report_axes_states_task(void *parameters) {
    (void) parameters;
    uint32_t last_pub_time = 0;
    uint32_t notification_value;
    bool last_report_successful = false;
    hid_joy_axes_report_t report;

    while (true) {
        xTaskNotifyWait(0, 0xffffffff, &notification_value, portMAX_DELAY);
        CHECK_EXEC_INTERVAL(&last_pub_time, (AXES_STATE_REPORT_INTERVAL + 10), report_time_lim_msg);

        if (!tud_hid_ready()) {
            continue;
        }

        hid_joy_axes_report_t new_report;
        new_report.y = get_joystick_y_val();
        new_report.pot = get_potentiometer_val();

        #if JOYSTICK_AXIS_SWAP_BUTTON_ENABLED
        if ((momen_btn_states >> JOYSTICK_AXIS_SWAP_BUTTON_NUM) & 1) {
            new_report.x = get_joystick_x_val();
            new_report.rz = 0;
        } else {
            new_report.x = 0;
            new_report.rz = get_joystick_x_val();
        }
        #else
        new_report.rz = get_joystick_x_val();
        new_report.x = 0;
        #endif

        bool report_changed = (new_report.x   != report.x) ||
                              (new_report.y   != report.y) ||
                              (new_report.rz  != report.rz) ||
                              (new_report.pot != report.pot);
        
        if (report_changed || !last_report_successful || notification_value == 1) {
            report = new_report;
            last_report_successful = tud_hid_report(AXES_INPUT_REPORT_ID, &report, sizeof(report));
        }
    }  
}


// ---- Timer callbacks for task notification ----
#define _TASK_NOTIFIER_TIMER_CB(name)                                           \
    bool name##_notify(struct repeating_timer *rt) {                            \
        (void) rt;                                                              \
        BaseType_t higher_prio_woken;                                           \
        (void) xTaskNotifyFromISR(name##_th, 0, eNoAction, &higher_prio_woken); \
        portYIELD_FROM_ISR(higher_prio_woken);                                  \
        return true;                                                            \
    }

_TASK_NOTIFIER_TIMER_CB(report_sw_states)
_TASK_NOTIFIER_TIMER_CB(report_button_states)
_TASK_NOTIFIER_TIMER_CB(report_axes_states)


// ---- Timer control ----
void start_hid_reporters(alarm_pool_t* alarm_pool) {
    // Staggered start to prevent potential report congestion.
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, SW_STATE_REPORT_INTERVAL, report_sw_states_notify, NULL, &report_sw_states_rt));
    vTaskDelay(pdMS_TO_TICKS(TUSB_TASK_EXEC_RATE_MS));
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, BTN_STATE_REPORT_INTERVAL, report_button_states_notify, NULL, &report_button_states_rt));
    vTaskDelay(pdMS_TO_TICKS(TUSB_TASK_EXEC_RATE_MS));
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, AXES_STATE_REPORT_INTERVAL, report_axes_states_notify, NULL, &report_axes_states_rt));
}

void stop_hid_reporters() {
    cancel_repeating_timer(&report_sw_states_rt);
    cancel_repeating_timer(&report_button_states_rt);
    cancel_repeating_timer(&report_axes_states_rt);
}