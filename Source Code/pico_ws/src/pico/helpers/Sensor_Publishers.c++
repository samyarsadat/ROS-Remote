/*
    The ROS remote project - Sensor Data MicroROS Publishers - Pico B
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
#include "Definitions.h"
#include "uROS_Init.h"
#include "helpers_lib/Helpers.h"
#include "IO_Helpers_General.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "uros_freertos_helpers_lib/uROS_Publishing_Handler.h"



// ------- Global variables ------- 

// ---- Publishing handler instance ----
extern uRosPublishingHandler *pub_handler;

// ---- Timer execution times storage (milliseconds) ----
extern uint32_t last_sw_state_publish_time, last_btn_state_publish_time;
extern uint32_t last_joystick_state_publish_time, last_potentiometer_state_publish_time;



// ------- Functions ------- 

// ---- Permanent switch states ----
void publish_sw_states(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_sw_state_publish_time, (sw_state_pub_rt_interval + 15), "Publish interval exceeded limits!", true);

        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

        switch_state_msg.time.sec = timestamp_sec;
        switch_state_msg.time.nanosec = timestamp_nanosec;

        switch_state_msg.left_key_sw = !gpio_get(left_key_sw_pin);
        switch_state_msg.left_top_toggle_sw = !gpio_get(left_top_toggle_sw_pin);
        switch_state_msg.right_e_stop_btn = !gpio_get(right_e_stop_btn_pin);
        switch_state_msg.right_kd2_btn = !gpio_get(right_kd2_btn_pin);
        switch_state_msg.right_top_toggle_sw = !gpio_get(right_top_toggle_sw_pin);

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &switch_state_pub;
        pub_item.message = &switch_state_msg;
        xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
    }
}


// ---- Momentary button states ----
void publish_btn_states(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_btn_state_publish_time, (btn_state_pub_rt_interval + 10), "Publish interval exceeded limits!", true);

        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

        button_state_msg.time.sec = timestamp_sec;
        button_state_msg.time.nanosec = timestamp_nanosec;

        button_state_msg.left_green_kd2_btn = !gpio_get(left_green_kd2_btn_pin);
        button_state_msg.left_green_left_btn = !gpio_get(left_green_left_btn_pin);
        button_state_msg.left_green_right_btn = !gpio_get(left_green_right_btn_pin);
        button_state_msg.left_red_btn = !gpio_get(left_red_btn_pin);
        button_state_msg.left_red_kd2_btn = !gpio_get(left_red_kd2_btn_pin);

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &button_state_pub;
        pub_item.message = &button_state_msg;
        xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
    }
}


// ---- Joystick state ----
void publish_joystick_state(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_joystick_state_publish_time, (joystick_pub_rt_interval + 15), "Publish interval exceeded limits!", true);

        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

        joystick_state_msg.time.sec = timestamp_sec;
        joystick_state_msg.time.nanosec = timestamp_nanosec;

        joystick_state_msg.joystick_x_axis_reading = get_joystick_x_val();
        joystick_state_msg.joystick_y_axis_reading = get_joystick_y_val();

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &joystick_state_pub;
        pub_item.message = &joystick_state_msg;
        xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
    }
}


// ---- Potentiometer state ----
void publish_potentiometer_state(void *parameters)
{
    while (true)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);   // Wait for notification indefinitely

        // Check execution time
        check_exec_interval(last_potentiometer_state_publish_time, (potentiometer_pub_rt_interval + 15), "Publish interval exceeded limits!", true);

        uint32_t timestamp_sec = to_ms_since_boot(get_absolute_time()) / 1000;
        uint32_t timestamp_nanosec = (to_ms_since_boot(get_absolute_time()) - (timestamp_sec * 1000)) * 1000000;

        potentiometer_state_msg.time.sec = timestamp_sec;
        potentiometer_state_msg.time.nanosec = timestamp_nanosec;

        potentiometer_state_msg.potentiometer_reading = get_potentiometer_val();

        uRosPublishingHandler::PublishItem_t pub_item;
        pub_item.publisher = &potentiometer_state_pub;
        pub_item.message = &potentiometer_state_msg;
        xQueueSendToBack(pub_handler->get_queue_handle(), (void *) &pub_item, 0);
    }
}