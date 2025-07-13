/*
    The ROS remote project - MicroROS Init
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
    along with this program.  If not, see <https: www.gnu.org/licenses/>.
*/

#include "uros_init.h"
#include "diagnostics.h"
#include "uros_freertos_abstract_lib/uros_bridge.h"
#include "uros_utils_lib/misc.h"
#include "uros/service_handlers.h"
#include "selftest/leds.h"
#include "common/opassert.h"


// ---- Setup subscribers and publishers ----
bool uros_init_ent() {
    const rosidl_message_type_support_t *button_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(remote_pico_coms, msg, ButtonStates);
    const rosidl_message_type_support_t *switch_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(remote_pico_coms, msg, SwitchStates);
    const rosidl_message_type_support_t *joystick_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(remote_pico_coms, msg, JoystickState);
    const rosidl_message_type_support_t *potentiometer_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(remote_pico_coms, msg, PotentiometerState);
    const rosidl_message_type_support_t *diag_status_type = ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus);
    const rosidl_service_type_support_t *get_joystick_config_type = ROSIDL_GET_SRV_TYPE_SUPPORT(remote_pico_coms, srv, GetJoystickConfig);
    const rosidl_service_type_support_t *set_joystick_config_type = ROSIDL_GET_SRV_TYPE_SUPPORT(remote_pico_coms, srv, SetJoystickConfig);
    const rosidl_service_type_support_t *get_led_states_type = ROSIDL_GET_SRV_TYPE_SUPPORT(remote_pico_coms, srv, GetLedStates);
    const rosidl_service_type_support_t *set_led_states_type = ROSIDL_GET_SRV_TYPE_SUPPORT(remote_pico_coms, srv, SetLedStates);
    const rosidl_service_type_support_t *run_self_test_type = ROSIDL_GET_SRV_TYPE_SUPPORT(diagnostic_msgs, srv, SelfTest);

    // Publishers
    LOG(LOG_LVL_INFO, "Initializing publishers...");
    uRosBridgeAgent* bridge = uRosBridgeAgent::get_instance();
    UROS_RETCODE_CHECK(bridge->init_publisher(&button_state_pub, button_state_type, "inputs/buttons"));
    UROS_RETCODE_CHECK(bridge->init_publisher(&switch_state_pub, switch_state_type, "inputs/switches"));
    UROS_RETCODE_CHECK(bridge->init_publisher(&joystick_state_pub, joystick_state_type, "inputs/joystick"));
    UROS_RETCODE_CHECK(bridge->init_publisher(&potentiometer_state_pub, potentiometer_state_type, "inputs/potentiometer"));
    UROS_RETCODE_CHECK(bridge->init_publisher(&diagnostics_pub, diag_status_type, "diagnostics"));

    // Service servers
    LOG(LOG_LVL_INFO, "Initializing services...");
    UROS_RETCODE_CHECK(core1_executor.init_service(&get_joystick_config_srv, get_joystick_config_type, "inputs/joystick/get_config"));
    UROS_RETCODE_CHECK(core1_executor.init_service(&set_joystick_config_srv, set_joystick_config_type, "inputs/joystick/set_config"));
    UROS_RETCODE_CHECK(core0_executor.init_service(&get_led_states_srv, get_led_states_type, "outputs/leds/get_states"));
    UROS_RETCODE_CHECK(core0_executor.init_service(&set_led_states_srv, set_led_states_type, "outputs/leds/set_states"));
    UROS_RETCODE_CHECK(core0_executor.init_service(&led_selftest_srv, run_self_test_type, "self_test/leds"));

    // Add executors to the bridge
    opassert(bridge->uros_add_executor(&core0_executor));
    opassert(bridge->uros_add_executor(&core1_executor));

    return true;
}


// ---- Executor init ----
bool uros_exec_setup() {
    LOG(LOG_LVL_INFO, "Initializing micro-ROS executors...");

    UROS_RETCODE_CHECK(core1_executor.add_service(&get_joystick_config_srv, &get_joystick_config_req, &get_joystick_config_res, get_joystick_config_callback));
    UROS_RETCODE_CHECK(core1_executor.add_service(&set_joystick_config_srv, &set_joystick_config_req, &set_joystick_config_res, set_joystick_config_callback));
    UROS_RETCODE_CHECK(core0_executor.add_service(&get_led_states_srv, &get_led_states_req, &get_led_states_res, get_led_states_callback));
    UROS_RETCODE_CHECK(core0_executor.add_service(&set_led_states_srv, &set_led_states_req, &set_led_states_res, set_led_states_callback));
    UROS_RETCODE_CHECK(core0_executor.add_service(&led_selftest_srv, &led_selftest_req, &led_selftest_res, leds_selftest_callback));

    return true;
}