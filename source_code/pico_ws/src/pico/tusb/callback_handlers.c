/*
    The ROS remote project - Raspberry Pi Pico firmware
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

#include "pico/stdlib.h"
#include "tusb.h"
#include "hid_descriptors.h"
#include "config/hw_defs.h"
#include "io/leds.h"
#include "diagnostics.h"
#include "hid_report_senders.h"
#include "state_management.h"
#include "config/sw_defs.h"
#include "common/opassert.h"


// ---- Device state callbacks ----
void tud_mount_cb() {
    enter_state_mounted();
}

void tud_umount_cb() {
    enter_state_unmounted();
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void) remote_wakeup_en;
    enter_state_suspended();
}

void tud_resume_cb() {
    if (tud_mounted()) {
        enter_state_resumed();
    } else {
        enter_state_unmounted();
    }
}


// Receive HID reports from the host
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    if (report_type == HID_REPORT_TYPE_OUTPUT) {
        switch (instance) {
            case ITF_NUM_LEDS_HID:
                if (report_id == LED_OUTPUT_REPORT_ID && bufsize == sizeof(hid_led_report_t)) {
                    hid_led_report_t* led_cmd = (hid_led_report_t*) buffer;
                    
                    if (led_cmd->index < NUMBER_OF_LEDS && led_cmd->mode <= LED_FAST_FADE) {
                        set_led_state(led_cmd->index, (LED_MODE_t) led_cmd->mode, led_cmd->pwm_out);
                        set_led_output_index(led_cmd->index);
                    }
                } else if (report_id == LED_GET_STATES_OUTPUT_REPORT_ID) {
                    assert(report_led_states_th != NULL);
                    (void) xTaskNotifyGive(report_led_states_th);
                }

                break;
            case ITF_NUM_JOYSTICK_HID:
                if (report_id == INPUT_POLL_OUTPUT_REPORT_ID) {
                    // Notify the tasks to re-send their reports.
                    LOG(LOG_LVL_DEBUG, "Received input poll report, notifying tasks to send reports.");
                    assert(report_axes_states_th != NULL && report_button_states_th != NULL && 
                           report_sw_states_th != NULL && report_pot_state_th != NULL);
                    
                    (void) xTaskNotify(report_axes_states_th, 1, eSetValueWithOverwrite);
                    (void) xTaskNotify(report_sw_states_th, 1, eSetValueWithOverwrite);
                    (void) xTaskNotify(report_pot_state_th, 1, eSetValueWithOverwrite);
                    (void) xTaskNotifyGive(report_button_states_th);
                }

                break;
        }
    }
}

// Nothing to get.
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
    return 0;
}

// In case of a transfer failure, try again.
void tud_hid_report_failed_cb(uint8_t instance, hid_report_type_t report_type, uint8_t const* report, uint16_t xferred_bytes) {
    (void) instance;
    (void) xferred_bytes;

    if (report_type == HID_REPORT_TYPE_INPUT) {
        uint8_t report_id = report[0];

        switch (instance) {
            case ITF_NUM_JOYSTICK_HID:
                LOG(LOG_LVL_ERROR, "HID input report transfer failed! Report ID: %d", report_id);

                // Notify the corresponding task to retry sending the report.
                // A notification value of 1 causes the report to be sent, even
                // if none of the report values have changed.
                switch (report_id) {
                    case BUTTONS_INPUT_REPORT_ID:
                        (void) xTaskNotifyGive(report_button_states_th);
                        break;
                    case AXES_INPUT_REPORT_ID:
                        (void) xTaskNotify(report_axes_states_th, 1, eSetValueWithOverwrite);
                        break;
                    case SWITCHES_INPUT_REPORT_ID:
                        (void) xTaskNotify(report_sw_states_th, 1, eSetValueWithOverwrite);
                        break;
                    case POT_INPUT_REPORT_ID:
                        (void) xTaskNotify(report_pot_state_th, 1, eSetValueWithOverwrite);
                        break;
                }

                break;
            case ITF_NUM_LEDS_HID:
                if (report_id == LED_STATES_INPUT_REPORT_ID) {
                    (void) xTaskNotifyGive(report_led_states_th);
                }
                break;
        }
    }
}

// Idle state
bool tud_hid_set_idle_cb(uint8_t instance, uint8_t idle_rate) {
    if (instance == ITF_NUM_JOYSTICK_HID) {
        const uint32_t idle_rate_ms = idle_rate * HID_REPORT_IDLE_RATE_UNIT_MS;
        assert(idle_hid_report_timer != NULL);

        if (idle_rate_ms > MIN_IDLE_REPORT_INTERVAL_MS) {
            LOG(LOG_LVL_DEBUG, "Joystick HID idle rate set to %d ms.", idle_rate_ms);
            return xTimerChangePeriod(idle_hid_report_timer, pdMS_TO_TICKS(idle_rate_ms), TIMER_COMMAND_TIMEOUT_T) == pdPASS &&
                   xTimerStart(idle_hid_report_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS;
        } else if (idle_rate_ms == 0) {
            //LOG(LOG_LVL_DEBUG, "Joystick HID idle reporting disabled.");
            //return xTimerStop(idle_hid_report_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS;
            LOG(LOG_LVL_DEBUG, "Ignoring idle_rate of 0 for joystick HID.");
            return true;
        } else {
            LOG(LOG_LVL_DEBUG, "Joystick HID idle rate set to %d ms, requested %d.", MIN_IDLE_REPORT_INTERVAL_MS, idle_rate_ms);
            return xTimerChangePeriod(idle_hid_report_timer, pdMS_TO_TICKS(MIN_IDLE_REPORT_INTERVAL_MS), TIMER_COMMAND_TIMEOUT_T) == pdPASS &&
                   xTimerStart(idle_hid_report_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS;
        }
    }

    return true;
}