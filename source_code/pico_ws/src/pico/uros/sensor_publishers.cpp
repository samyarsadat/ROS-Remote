/*
    The ROS remote project - Sensor Data MicroROS Publishers
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

#include "sensor_publishers.h"
#include "config/hw_defs.h"
#include "uros_init.h"
#include "uros_utils_lib/general.h"
#include "io/joystick.h"
#include "config/diag_msg_defs.h"
#include "common/opassert.h"
#include "FreeRTOS.h"
#include "task.h"
#include "diagnostics.h"


// Timers
struct repeating_timer sw_state_publish_rt, joystick_publish_rt, potentiometer_publish_rt;


// ---- Permanent switch states ----
void publish_sw_states(void *parameters) {
    (void) parameters;
    remote_pico_coms__msg__SwitchStates msg;
    uint32_t last_pub_time = 0;

    while (true) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        check_exec_interval(last_pub_time, SW_STATE_PUB_RT_INTERVAL + 10,
                            PUB_TIMER_INTERVAL_EXCEEDED, "switches/publisher", true);

        msg.left_key_sw = !gpio_get(LEFT_KEY_SW_PIN);
        msg.left_top_toggle_sw = !gpio_get(LEFT_TOP_TOGGLE_SW_PIN);
        msg.right_e_stop_btn = !gpio_get(RIGHT_E_STOP_BTN_PIN);
        msg.right_kd2_btn = !gpio_get(RIGHT_KD2_BTN_PIN);
        msg.right_top_toggle_sw = !gpio_get(RIGHT_TOP_TOGGLE_SW_PIN);
        UROS_RETCODE_LOG(rcl_publish(&switch_state_pub, &msg, NULL));
    }
}

// ---- Momentary button states ----
void publish_btn_states(void *parameters) {
    (void) parameters;
    remote_pico_coms__msg__ButtonStates msg;
    uint32_t notification_value;

    while (true) {
        xTaskNotifyWait(0, 0xffffffff, &notification_value, portMAX_DELAY);

        msg.left_green_kd2_btn = false;
        msg.left_green_left_btn = false;
        msg.left_green_right_btn = false;
        msg.left_red_btn = false;
        msg.left_red_kd2_btn = false;

        switch (notification_value) {
            case LEFT_GREEN_RIGHT_BTN_PIN:
                msg.left_green_right_btn = true;
                break;
            case LEFT_RED_BTN_PIN:
                msg.left_red_btn = true;
                break;
            case LEFT_GREEN_KD2_BTN_PIN:
                msg.left_green_kd2_btn = true;
                break;
            case LEFT_RED_KD2_BTN_PIN:
                msg.left_red_kd2_btn = true;
                break;
            case LEFT_GREEN_LEFT_BTN_PIN:
                msg.left_green_left_btn = true;
                break;
        }

        UROS_RETCODE_LOG(rcl_publish(&button_state_pub, &msg, NULL));
    }
}

// ---- Joystick state ----
void publish_joystick_state(void *parameters) {
    (void) parameters;
    remote_pico_coms__msg__JoystickState msg;
    uint32_t last_pub_time = 0;

    while (true) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        check_exec_interval(last_pub_time, JOYSTICK_PUB_RT_INTERVAL + 10, 
                            PUB_TIMER_INTERVAL_EXCEEDED, "joystick/publisher", true);

        msg.joystick_x_axis_reading = get_joystick_x_val();
        msg.joystick_y_axis_reading = get_joystick_y_val();
        UROS_RETCODE_LOG(rcl_publish(&button_state_pub, &msg, NULL));
    }
}

// ---- Potentiometer state ----
void publish_potentiometer_state(void *parameters) {
    (void) parameters;
    remote_pico_coms__msg__PotentiometerState msg;
    uint32_t last_pub_time = 0;

    while (true) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        check_exec_interval(last_pub_time, POTENTIOMETER_PUB_RT_INTERVAL + 10, 
                            PUB_TIMER_INTERVAL_EXCEEDED, "potentiometer/publisher", true);

        msg.potentiometer_reading = get_potentiometer_val();
        UROS_RETCODE_LOG(rcl_publish(&potentiometer_state_pub, &msg, NULL));
    }
}


// ---- Timer callbacks for task notification ----
bool publish_sw_state_notify(struct repeating_timer *rt) {
    (void) rt;
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(sw_state_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_joystick_notify(struct repeating_timer *rt) {
    (void) rt;
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(joystick_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_potentiometer_notify(struct repeating_timer *rt) {
    (void) rt;
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(potentiometer_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}


// ---- Timer control ----
void start_sensor_publishers(alarm_pool_t* alarm_pool) {
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, SW_STATE_PUB_RT_INTERVAL, publish_sw_state_notify, NULL, &sw_state_publish_rt));
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, JOYSTICK_PUB_RT_INTERVAL, publish_joystick_notify, NULL, &joystick_publish_rt));
    opassert(alarm_pool_add_repeating_timer_ms(alarm_pool, POTENTIOMETER_PUB_RT_INTERVAL, publish_potentiometer_notify, NULL, &potentiometer_publish_rt));
}

void stop_sensor_publishers() {
    opassert(cancel_repeating_timer(&sw_state_publish_rt));
    opassert(cancel_repeating_timer(&joystick_publish_rt));
    opassert(cancel_repeating_timer(&potentiometer_publish_rt));
}