/*
    The ROS remote project - MicroROS Init
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

#pragma once


// ------- Libraries & Modules -------
#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <sensor_msgs/msg/range.h>
#include <std_srvs/srv/set_bool.h>
#include <nav_msgs/msg/odometry.h>
#include <remote_pico_coms/msg/button_states.h>
#include <remote_pico_coms/msg/switch_states.h>
#include <remote_pico_coms/msg/joystick_state.h>
#include <remote_pico_coms/msg/potentiometer_state.h>
#include <remote_pico_coms/srv/get_joystick_config.h>
#include <remote_pico_coms/srv/get_led_states.h>
#include <remote_pico_coms/srv/set_joystick_config.h>
#include <remote_pico_coms/srv/set_led_states.h>
#include <diagnostic_msgs/srv/self_test.h>



// ------- Variables -------

// ---- Publishers ----

// Button, switch, joystick, potentiometer states
extern rcl_publisher_t button_state_pub, switch_state_pub, joystick_state_pub, potentiometer_state_pub;
extern remote_pico_coms__msg__ButtonStates button_state_msg;
extern remote_pico_coms__msg__SwitchStates switch_state_msg;
extern remote_pico_coms__msg__JoystickState joystick_state_msg;
extern remote_pico_coms__msg__PotentiometerState potentiometer_state_msg;


// ---- Services ----

// Get/set joystick config
extern rcl_service_t get_joystick_config_srv, set_joystick_config_srv;
extern remote_pico_coms__srv__GetJoystickConfig_Request get_joystick_config_req;
extern remote_pico_coms__srv__GetJoystickConfig_Response get_joystick_config_res;
extern remote_pico_coms__srv__SetJoystickConfig_Request set_joystick_config_req;
extern remote_pico_coms__srv__SetJoystickConfig_Response set_joystick_config_res;

// Get/set LED states
extern rcl_service_t get_led_states_srv, set_led_states_srv;
extern remote_pico_coms__srv__GetLedStates_Request get_led_states_req;
extern remote_pico_coms__srv__GetLedStates_Response get_led_states_res;
extern remote_pico_coms__srv__SetLedStates_Request set_led_states_req;
extern remote_pico_coms__srv__SetLedStates_Response set_led_states_res;

// Initiate the self-test function
extern rcl_service_t run_self_test_srv;
extern diagnostic_msgs__srv__SelfTest_Request run_self_test_req;
extern diagnostic_msgs__srv__SelfTest_Response run_self_test_res;



// ------- Functions -------

// ---- Setup subscribers and publishers ----
void init_subs_pubs();

// ---- Executor init ----
void exec_init();

// ---- Bridge init function ----
bool uros_init();