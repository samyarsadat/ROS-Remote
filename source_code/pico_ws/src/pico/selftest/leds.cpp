/*
    The ROS remote project - LED Self Test Module
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

#include "leds.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/srv/self_test.h>
#include "FreeRTOS.h"
#include "diagnostics.h"
#include "io/leds.h"
#include "uros_utils_lib/selftest_resp_helpers.h"
#include "uros_common/definitions.h"
#include "uros_utils_lib/diag_util.h"
#include "config/diag_msg_defs.h"


// Is an LED self-test in progress?
bool test_in_progress = false;


// ---- LEDs self-test FreeRTOS task ----
void leds_selftest_task(void* parameters) {
    (void) parameters;
    
    LOG(LOG_LVL_INFO, "Running LED self-test...");
    leds_test();
    LOG(LOG_LVL_INFO, "LED self-test completed.");

    test_in_progress = false;
    vTaskDelete(nullptr);
}

// ---- LEDs self-test service callback ----
void leds_selftest_callback(const void *req, void *res) {
    (void) req;
    diagnostic_msgs__srv__SelfTest_Response *res_in = (diagnostic_msgs__srv__SelfTest_Response *) res;

    LOG(LOG_LVL_INFO, "LED self-test requested.");

    if (!test_in_progress) {
        test_in_progress = true;
        (void) xTaskCreate(leds_selftest_task, "leds_selftest", LED_ST_TASK_STACK_DEPTH, nullptr, configMAX_PRIORITIES - 3, nullptr);
    } else {
        LOG(LOG_LVL_WARN, "LED self-test already in progress!")
    }

    selftest_resp_free(res_in);
    selftest_resp_init(res_in, DIAG_FIRMWARE_HARDWARE_ID, 1);
    selftest_stat_set(
        &res_in->status.data[0],
        DIAG_FIRMWARE_HARDWARE_ID,
        "leds",
        DIAG_OK_LED_TEST_PASS,
        DIAG_LVL_OK
    );

    res_in->passed = true;
}