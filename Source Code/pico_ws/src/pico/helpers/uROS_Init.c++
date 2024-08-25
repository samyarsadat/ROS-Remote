/*
    The ROS remote project - MicroROS Init - Pico B
    Copyright 2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2024.
 
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



// ------- Libraries & Modules -------
#include "uROS_Init.h"
#include "Definitions.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "uros_freertos_helpers_lib/uROS_Bridge_Agent.h"



// ------- Variables -------

// ---- General ----
extern uRosBridgeAgent *bridge;   // From the main file.


// ---- Subscribers ----

// Misc.
rcl_subscription_t e_stop_sub;
std_msgs__msg__Empty e_stop_msg;


// ---- Publishers ----

// Button, joystick, potentiometer states
rcl_publisher_t button_state_pub, joystick_state_pub, potentiometer_state_pub;
remote_pico_coms__msg__ButtonStates button_state_msg;
remote_pico_coms__msg__JoystickState joystick_state_msg;
remote_pico_coms__msg__PotentiometerState potentiometer_state_msg;


// ---- Services ----

// Get/set joystick config
rcl_service_t get_joystick_config_srv, set_joystick_config_srv;
remote_pico_coms__srv__GetJoystickConfig_Request get_joystick_config_req;
remote_pico_coms__srv__GetJoystickConfig_Response get_joystick_config_res;
remote_pico_coms__srv__SetJoystickConfig_Request set_joystick_config_req;
remote_pico_coms__srv__SetJoystickConfig_Response set_joystick_config_res;

// Get/set LED states
rcl_service_t get_led_states_srv, set_led_states_srv;
remote_pico_coms__srv__GetLedStates_Request get_led_states_req;
remote_pico_coms__srv__GetLedStates_Response get_led_states_res;
remote_pico_coms__srv__SetLedStates_Request set_led_states_req;
remote_pico_coms__srv__SetLedStates_Response set_led_states_res;

// Initiate the self-test function
rcl_service_t run_self_test_srv;
diagnostic_msgs__srv__SelfTest_Request run_self_test_req;
diagnostic_msgs__srv__SelfTest_Response run_self_test_res;



// ------- Subscriber & service callback prototypes -------
extern void get_joystick_config_callback(const void *req, void *res);
extern void set_joystick_config_callback(const void *req, void *res);
extern void get_led_states_callback(const void *req, void *res);
extern void set_led_states_callback(const void *req, void *res);
extern void run_self_test_callback(const void *req, void *res);
extern void clean_shutdown();
extern void start_timers();
void clean_shutdown_callback(const void *msgin) { clean_shutdown(); }



// ------- Functions -------

// ---- Setup subscribers and publishers ----
void init_subs_pubs()
{
    write_log("Initializing publishers, subscribers, and services...", LOG_LVL_INFO, FUNCNAME_ONLY);

    const rosidl_message_type_support_t *empty_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty);
    const rosidl_message_type_support_t *button_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(remote_pico_coms, msg, ButtonStates);
    const rosidl_message_type_support_t *joystick_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(remote_pico_coms, msg, JoystickState);
    const rosidl_message_type_support_t *potentiometer_state_type = ROSIDL_GET_MSG_TYPE_SUPPORT(remote_pico_coms, msg, PotentiometerState);
    const rosidl_service_type_support_t *get_joystick_config_type = ROSIDL_GET_SRV_TYPE_SUPPORT(remote_pico_coms, srv, GetJoystickConfig);
    const rosidl_service_type_support_t *set_joystick_config_type = ROSIDL_GET_SRV_TYPE_SUPPORT(remote_pico_coms, srv, SetJoystickConfig);
    const rosidl_service_type_support_t *get_led_states_type = ROSIDL_GET_SRV_TYPE_SUPPORT(remote_pico_coms, srv, GetLedStates);
    const rosidl_service_type_support_t *set_led_states_type = ROSIDL_GET_SRV_TYPE_SUPPORT(remote_pico_coms, srv, SetLedStates);
    const rosidl_service_type_support_t *run_self_test_type = ROSIDL_GET_SRV_TYPE_SUPPORT(diagnostic_msgs, srv, SelfTest);

    // ---- Services ----
    write_log("Initializing services...", LOG_LVL_INFO, FUNCNAME_ONLY);
    bridge->init_service(&get_joystick_config_srv, get_joystick_config_type, "joystick/get_config");
    bridge->init_service(&set_joystick_config_srv, set_joystick_config_type, "joystick/set_config");
    bridge->init_service(&get_led_states_srv, get_led_states_type, "leds/get_states");
    bridge->init_service(&set_led_states_srv, set_led_states_type, "leds/set_states");
    bridge->init_service(&run_self_test_srv, run_self_test_type, "self_test/pico");


    // ---- Subscribers ----
    write_log("Initializing subscribers...", LOG_LVL_INFO, FUNCNAME_ONLY);

    // E-stop topic
    bridge->init_subscriber(&e_stop_sub, empty_type, "/e_stop");
    std_msgs__msg__Empty__init(&e_stop_msg);


    // ---- Publishers ----
    write_log("Initializing publishers...", LOG_LVL_INFO, FUNCNAME_ONLY);
    
    // Diagnostics
    diag_uros_init();

    // Sensor state topics
    bridge->init_publisher(&button_state_pub, button_state_type, "buttons/states");
    bridge->init_publisher(&joystick_state_pub, joystick_state_type, "joystick/state");
    bridge->init_publisher(&potentiometer_state_pub, potentiometer_state_type, "potentiometer/state");
    remote_pico_coms__msg__ButtonStates__init(&button_state_msg);
    remote_pico_coms__msg__JoystickState__init(&joystick_state_msg);
    remote_pico_coms__msg__PotentiometerState__init(&potentiometer_state_msg);


    write_log("Init. completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}


// ---- Executor init ----
void exec_init()
{
    write_log("Initializing MicroROS executor...", LOG_LVL_INFO, FUNCNAME_ONLY);

    bridge->uros_init_executor();
    bridge->add_service(&get_joystick_config_srv, &get_joystick_config_req, &get_joystick_config_res, get_joystick_config_callback);
    bridge->add_service(&set_joystick_config_srv, &set_joystick_config_req, &set_joystick_config_res, set_joystick_config_callback);
    bridge->add_service(&get_led_states_srv, &get_led_states_req, &get_led_states_res, get_led_states_callback);
    bridge->add_service(&set_led_states_srv, &set_led_states_req, &set_led_states_res, set_led_states_callback);
    bridge->add_service(&run_self_test_srv, &run_self_test_req, &run_self_test_res, run_self_test_callback);
    bridge->add_subscriber(&e_stop_sub, &e_stop_msg, clean_shutdown_callback);

    write_log("Init. completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}


// ---- Node init ----
bool uros_init()
{
    write_log("MicroROS init...", LOG_LVL_INFO, FUNCNAME_ONLY);

    bridge->uros_init_node(UROS_NODE_NAME, UROS_NODE_NAMESPACE, UROS_DOMAIN_ID);
    init_subs_pubs();
    exec_init();
    start_timers();
 
    write_log("Init. completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
    return true;   
}