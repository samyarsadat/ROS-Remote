/*
    The ROS remote project - Service server callbacks
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

#include "service_handlers.h"
#include "uros/uros_init.h"
#include "io/joystick.h"
#include "io/leds.h"
#include "diagnostics.h"
#include "config/hw_defs.h"


// ---- Get/set joystick configuration services ----
void get_joystick_config_callback(const void *req, void *res) {
    (void) req;
    remote_pico_coms__srv__GetJoystickConfig_Response *res_in = (remote_pico_coms__srv__GetJoystickConfig_Response *) res;

    LOG(LOG_LVL_INFO, "Get joystick configuration request received.");

    res_in->joystick_x_center_offset = joystick_x_center_offset;
    res_in->joystick_y_center_offset = joystick_y_center_offset;
    res_in->joystick_x_deadzone = joystick_x_deadzone;
    res_in->joystick_y_deadzone = joystick_y_deadzone;
}

void set_joystick_config_callback(const void *req, void *res) {
    remote_pico_coms__srv__SetJoystickConfig_Request *req_in = (remote_pico_coms__srv__SetJoystickConfig_Request *) req;
    (void) res;

    LOG(LOG_LVL_INFO, "Set joystick configuration request received. [xd: %d, yd: %d, xco: %.2f, yco: %.2f]",
        req_in->joystick_x_deadzone, req_in->joystick_y_deadzone, req_in->joystick_x_center_offset, req_in->joystick_y_center_offset);

    joystick_x_deadzone = req_in->joystick_x_deadzone;
    joystick_y_deadzone = req_in->joystick_y_deadzone;
    joystick_x_center_offset = req_in->joystick_x_center_offset;
    joystick_y_center_offset = req_in->joystick_y_center_offset;
}

// ---- Get/set LED states service ----
void get_led_states_callback(const void *req, void *res) {
    (void) req;
    remote_pico_coms__srv__GetLedStates_Response *res_in = (remote_pico_coms__srv__GetLedStates_Response *) res;

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        led_state_t state = get_led_state(i);
        res_in->led_modes[i] = state.mode;
        res_in->pwm_outputs[i] = state.pwm_set_out;
    }
}

void set_led_states_callback(const void *req, void *res) {
    remote_pico_coms__srv__SetLedStates_Request *req_in = (remote_pico_coms__srv__SetLedStates_Request *) req;
    (void) res;

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (req_in->set_state_mask[i]) {
            set_led_state(i, req_in->led_modes[i], req_in->pwm_outputs[i]);
        }
    }

    set_led_outputs();
}