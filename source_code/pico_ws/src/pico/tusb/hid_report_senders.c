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
#include "io/leds.h"


#define REPORT_TIME_LIM_MSG  "HID report sending interval time limit exceeded!"

// Timers & tasks
struct repeating_timer report_sw_states_rt, report_axes_states_rt, report_pot_state_rt;
TaskHandle_t report_sw_states_th = NULL, report_button_states_th = NULL, 
             report_axes_states_th = NULL, report_pot_state_th = NULL,
             report_led_states_th = NULL;
TickType_t report_sw_states_lst = 0, report_button_states_lst = 0,   // Last Send Tick
           report_pot_state_lst = 0, report_axes_states_lst = 0;


// ******** EXTERNALLY TRIGGERED HID REPORT SENDERS ********
// ---- Momentary button states ----
void report_button_states_task(void *parameters) {
    (void) parameters;
    hid_buttons_report_t report;

    while (true) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        
        if (tud_hid_n_ready(ITF_NUM_JOYSTICK_HID)) {
            report.buttons = momen_btn_states;
            
            if (tud_hid_n_report(ITF_NUM_JOYSTICK_HID, BUTTONS_INPUT_REPORT_ID, &report, sizeof(report))) {
                momen_btn_ls_state = momen_btn_states;
            }

            report_button_states_lst = xTaskGetTickCount();
        }
    }
}

// ---- Send LED states ----
void report_led_states_task(void *parameters) {
    (void) parameters;
    hid_led_states_report_t report;

    while (true) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

        if (tud_hid_n_ready(ITF_NUM_LEDS_HID)) {
            for (int i = 0; i < NUMBER_OF_LEDS; i++) {
                led_state_t state = get_led_state(i);
                report.mode[i] = state.mode;
                report.pwm_out[i] = state.pwm_set_out;
            }

            if (tud_hid_n_report(ITF_NUM_LEDS_HID, LED_STATES_INPUT_REPORT_ID, &report, sizeof(report))) {
                continue;   
            }
        }

        (void) xTaskNotifyGive(report_led_states_th);

        // We want to avoid sending two reports in close succession.
        // Other reporters don't need delays, as their respective timers
        // act as a sufficient cooldown. This isn't ideal, but it's okay.
        // The task notification array size is 3, and I really doubt that
        // any host-side program is going to request LED states twice within 10ms.
        // Besides, the TUSB spin task is only executed every 10ms anyway.
        vTaskDelay(pdMS_TO_TICKS(LED_STATE_REPORT_RETRY_COOLDOWN_MS));
    }
}


// ******** TIMER-BASED HID REPORT SENDERS ********
// ---- Toggle switch states ----
void report_sw_states_task(void *parameters) {
    (void) parameters;
    uint32_t last_exec_time = 0;
    uint32_t notification_value;
    bool retry_send = false;
    hid_switches_report_t report;

    while (true) {
        xTaskNotifyWait(0, 0xffffffff, &notification_value, portMAX_DELAY);
        CHECK_EXEC_INTERVAL_DBG(&last_exec_time, (SW_STATE_REPORT_INTERVAL + 10), REPORT_TIME_LIM_MSG);

        // Explicit re-send has been requested.
        if (notification_value) {
            retry_send = true;
        }
        
        uint8_t state = 0;
        state |= (!gpio_get(LEFT_KEY_SW_PIN)         << 0);
        state |= (!gpio_get(LEFT_TOP_TOGGLE_SW_PIN)  << 1);
        state |= (!gpio_get(RIGHT_E_STOP_BTN_PIN)    << 2);
        state |= (!gpio_get(RIGHT_KD2_BTN_PIN)       << 3);
        state |= (!gpio_get(RIGHT_TOP_TOGGLE_SW_PIN) << 4);

        if ((report.switches != state || retry_send) && tud_hid_n_ready(ITF_NUM_JOYSTICK_HID)) {
            report.switches = state;
            retry_send = !tud_hid_n_report(ITF_NUM_JOYSTICK_HID, SWITCHES_INPUT_REPORT_ID, &report, sizeof(report));
            report_sw_states_lst = xTaskGetTickCount();
        }
    }
}

// ---- Joystick axes & potentiometer state ----
void report_axes_states_task(void *parameters) {
    (void) parameters;
    uint32_t last_exec_time = 0;
    uint32_t notification_value;
    bool retry_send = false;
    hid_joy_axes_report_t report;

    while (true) {
        xTaskNotifyWait(0, 0xffffffff, &notification_value, portMAX_DELAY);
        CHECK_EXEC_INTERVAL_DBG(&last_exec_time, (AXES_STATE_REPORT_INTERVAL + 10), REPORT_TIME_LIM_MSG);

        if (notification_value) {
            retry_send = true;
        }

        hid_joy_axes_report_t new_report;
        new_report.y = get_joystick_y_val();

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

        bool report_changed = (new_report.x  != report.x) ||
                              (new_report.y  != report.y) ||
                              (new_report.rz != report.rz);
        
        if ((report_changed || retry_send) && tud_hid_n_ready(ITF_NUM_JOYSTICK_HID)) {
            report = new_report;
            retry_send = !tud_hid_n_report(ITF_NUM_JOYSTICK_HID, AXES_INPUT_REPORT_ID, &report, sizeof(report));
            report_axes_states_lst = xTaskGetTickCount();
        }
    }  
}

// ---- Potentiometer state ----
void report_pot_state_task(void *parameters) {
    (void) parameters;
    uint32_t last_exec_time = 0;
    uint32_t notification_value;
    bool retry_send = false;
    hid_pot_report_t report;

    while (true) {
        xTaskNotifyWait(0, 0xffffffff, &notification_value, portMAX_DELAY);
        CHECK_EXEC_INTERVAL_DBG(&last_exec_time, (POT_STATE_REPORT_INTERVAL + 10), REPORT_TIME_LIM_MSG);

        if (notification_value) {
            retry_send = true;
        }

        uint8_t pot_val = get_potentiometer_val();

        if ((report.pot != pot_val || retry_send) && tud_hid_n_ready(ITF_NUM_JOYSTICK_HID)) {
            report.pot = pot_val;
            retry_send = !tud_hid_n_report(ITF_NUM_JOYSTICK_HID, POT_INPUT_REPORT_ID, &report, sizeof(report));
            report_pot_state_lst = xTaskGetTickCount();
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
_TASK_NOTIFIER_TIMER_CB(report_axes_states)
_TASK_NOTIFIER_TIMER_CB(report_pot_state)

// ---- Timer control ----
void start_hid_reporters(alarm_pool_t* alarm_pool) {
    // Staggered start to prevent potential report congestion.
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, SW_STATE_REPORT_INTERVAL, report_sw_states_notify, NULL, &report_sw_states_rt));
    vTaskDelay(pdMS_TO_TICKS(TUSB_TASK_EXEC_RATE_MS));
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, AXES_STATE_REPORT_INTERVAL, report_axes_states_notify, NULL, &report_axes_states_rt));
    vTaskDelay(pdMS_TO_TICKS(TUSB_TASK_EXEC_RATE_MS));
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, POT_STATE_REPORT_INTERVAL, report_pot_state_notify, NULL, &report_pot_state_rt));
}

void stop_hid_reporters() {
    cancel_repeating_timer(&report_sw_states_rt);
    cancel_repeating_timer(&report_axes_states_rt);
    cancel_repeating_timer(&report_pot_state_rt);
}


// ******** REPORTER TASK CREATION & DELETION ********
void create_hid_reporter_tasks() {
    assert(report_sw_states_th == NULL && report_button_states_th == NULL &&
           report_axes_states_th == NULL && report_pot_state_th == NULL &&
           report_led_states_th == NULL);
    
    (void) xTaskCreate(report_button_states_task, "button_report", TIMER_TASK_STACK_DEPTH, NULL, BUTTON_REPORT_TASK_PRIORITY, &report_button_states_th);
    (void) xTaskCreate(report_axes_states_task, "axes_report", TIMER_TASK_STACK_DEPTH, NULL, AXES_REPORT_TASK_PRIORITY, &report_axes_states_th);
    (void) xTaskCreate(report_sw_states_task, "switch_report", TIMER_TASK_STACK_DEPTH, NULL, SW_REPORT_TASK_PRIORITY, &report_sw_states_th);
    (void) xTaskCreate(report_pot_state_task, "pot_report", TIMER_TASK_STACK_DEPTH, NULL, POT_REPORT_TASK_PRIORITY, &report_pot_state_th);
    (void) xTaskCreate(report_led_states_task, "led_states_report", TIMER_TASK_STACK_DEPTH, NULL, LED_STATES_REPORT_TASK_PRIORITY, &report_led_states_th);
}

void delete_hid_reporter_tasks() {
    assert(report_sw_states_th != NULL && report_button_states_th != NULL && 
           report_axes_states_th != NULL && report_pot_state_th != NULL &&
           report_led_states_th != NULL);
    
    vTaskDelete(report_sw_states_th);
    report_sw_states_th = NULL;

    vTaskDelete(report_button_states_th);
    report_button_states_th = NULL;

    vTaskDelete(report_axes_states_th);
    report_axes_states_th = NULL;

    vTaskDelete(report_pot_state_th);
    report_pot_state_th = NULL;

    vTaskDelete(report_led_states_th);
    report_led_states_th = NULL;
}