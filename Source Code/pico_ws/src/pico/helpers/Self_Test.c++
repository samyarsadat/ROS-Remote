/*
    The ROS remote project - Self Test Module - Pico B
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
#include "helpers_lib/Helpers.h"
#include "local_helpers_lib/Local_Helpers.h"
#include "Definitions.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/srv/self_test.h>
#include "FreeRTOS.h"
#include "IO_Helpers_General.h"
#include "semphr.h"



// ------- Global variables -------
std::vector<int> self_test_diag_data_slot_nums;
std::vector<diagnostic_msgs__msg__DiagnosticStatus> self_test_diag_status_reports;
SemaphoreHandle_t run_self_test_mutex = NULL;



// ------- Functions -------

// ---- Run self-test functions service callback ----
void run_self_test_callback(const void *req, void *res)
{
    // Tests the:
    // - LEDs

    diagnostic_msgs__srv__SelfTest_Request *req_in = (diagnostic_msgs__srv__SelfTest_Request *) req;
    diagnostic_msgs__srv__SelfTest_Response *res_in = (diagnostic_msgs__srv__SelfTest_Response *) res;

    write_log("Received self-test request.", LOG_LVL_INFO, FUNCNAME_ONLY);
    disable_diag_pub();
    

    // Perform LED self-test
    write_log("Performing LED self-test...", LOG_LVL_INFO, FUNCNAME_ONLY);
    set_in_self_test(true);

    for (int i = 0; i < number_of_leds; i++)
    {
        gpio_put_pwm(led_pins_order[i], 0);
    }

    for (int i = 0; i < number_of_leds; i++)
    {
        gpio_put_pwm(led_pins_order[i], 65535);
        vTaskDelay(200);
    }

    for (int i = 0; i < number_of_leds; i++)
    {
        gpio_put_pwm(led_pins_order[i], 0);
        vTaskDelay(200);
    }

    set_in_self_test(false);
            
    diag_publish_item_t pub_item = prepare_diag_publish_item(DIAG_LVL_OK, DIAG_NAME_SYSTEM, DIAG_ID_SYS_GENERAL, DIAG_OK_MSG_LED_TEST_PASS, NULL);
    self_test_diag_status_reports.push_back(*pub_item.diag_msg);
    self_test_diag_data_slot_nums.push_back(pub_item.allocated_slot);

    res_in->status.data = self_test_diag_status_reports.data();
    res_in->status.size = self_test_diag_status_reports.size();

    write_log("Self-test completed.", LOG_LVL_INFO, FUNCNAME_ONLY);
}