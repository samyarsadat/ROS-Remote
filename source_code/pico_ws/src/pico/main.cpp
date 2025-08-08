/*
    The ROS remote project - Raspberry Pi Pico firmware
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

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "utils_lib/hardware.h"
#include "io/leds.h"
#include "io/buttons.h"
#include "utils_lib/adc/adc_lock.h"
#include "diagnostics.h"
#include "common/opassert.h"
#include "tusb/hid_report_senders.h"
#include "config/sw_defs.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "state_management.h"
#include "pico/unique_id.h"
#include "utils_lib/perf/exec_interval.h"


// ---- Global variables ----
char pico_unique_id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
alarm_pool_t *core_1_alarm_pool = nullptr;
TimerHandle_t status_led_timer = nullptr, idle_hid_report_timer = nullptr;
TaskHandle_t tusb_spin_task_th = nullptr;
bool mount_init = false, resume_init = false;


// ---- FreeRTOS task stack overflow hook ----
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    (void) xTask;
    panic("FreeRTOS stack overflow in task: %s", pcTaskName);
}

// ---- FreeRTOS malloc failure hook ----
void vApplicationMallocFailedHook() {
    panic("FreeRTOS malloc failed!");
}


// ---- LED flash state handler ----
void status_led_timer_call(TimerHandle_t timer) {
    TickType_t timer_period;
    bool led_always_on = false;

    if (mount_init && resume_init) {
        led_always_on = true;
        timer_period = pdMS_TO_TICKS(STAT_LED_IDLE_CHECK_MS);
    } else if (mount_init && !resume_init) {
        led_always_on = false;
        timer_period = pdMS_TO_TICKS(STAT_LED_SUSPENDED_MS);
    } else {
        led_always_on = false;
        timer_period = pdMS_TO_TICKS(STAT_LED_UNMOUNTED_MS);
    }

    if (xTimerGetPeriod(timer) != pdMS_TO_TICKS(timer_period)) {
        (void) xTimerChangePeriod(timer, pdMS_TO_TICKS(timer_period), TIMER_COMMAND_TIMEOUT_T);
    }

    gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN) || led_always_on);
}

// ---- Idle HID report timer ----
#define _IDLE_HID_NOTIFY_CHECK(name)                                                  \
    if (curr_time - name##_lst > curr_tmr_period - IDLE_NOTIFY_TIME_CHECK_MARGIN_T) { \
        (void) xTaskNotify(name##_th, 1, eSetValueWithOverwrite);                     \
    }

void idle_hid_report_timer_call(TimerHandle_t timer) {
    (void) timer;

    // In case the timer stop command fails for whatever reason.
    if (resume_init && mount_init) {
        assert(report_axes_states_th != NULL && report_button_states_th != NULL && 
               report_sw_states_th != NULL && report_pot_state_th != NULL);
        
        TickType_t curr_tmr_period = xTimerGetPeriod(timer);
        TickType_t curr_time = xTaskGetTickCount();
        
        // Note: This is only for the joystick HID interface.
        _IDLE_HID_NOTIFY_CHECK(report_axes_states)
        _IDLE_HID_NOTIFY_CHECK(report_sw_states)
        _IDLE_HID_NOTIFY_CHECK(report_pot_state)
        _IDLE_HID_NOTIFY_CHECK(report_button_states)
    }
}

// ---- TinyUSB spin task ----
void tusb_spin_task(void *parameters) {
    (void) parameters;
    LOG(LOG_LVL_INFO, "TinyUSB task started!");
    TickType_t wake = xTaskGetTickCount();
    uint32_t last_exec_time = 0;

    while (true) {
        CHECK_EXEC_INTERVAL_DBG(&last_exec_time, (TUSB_TASK_EXEC_RATE_MS + 2), "USB task execution time limit exceeded!");

        if (tud_task_event_ready()) {
            tud_task();
        }

        xTaskDelayUntil(&wake, pdMS_TO_TICKS(TUSB_TASK_EXEC_RATE_MS));
    }
}


// ---- Setup function (core 0) ----
void setup(void *parameters) {
    (void) parameters;
    LOG(LOG_LVL_INFO, "Core 0 setup task started!");
    LOG(LOG_LVL_INFO, "Hardware initialization.");
    
    // Force SMPS into PWM mode
    init_pin(PICO_SMPS_MODE_PIN, OUTPUT);
    gpio_put(PICO_SMPS_MODE_PIN, true);

    // Pin init
    init_pin(PICO_DEFAULT_LED_PIN, OUTPUT);
    init_pin(RIGHT_TOP_TOGGLE_SW_PIN, INPUT_PULLUP);
    init_pin(LEFT_KEY_SW_PIN, INPUT_PULLUP);
    init_pin(LEFT_TOP_TOGGLE_SW_PIN, INPUT_PULLUP);
    init_pin(RIGHT_E_STOP_BTN_PIN, INPUT_PULLUP);
    init_pin(RIGHT_KD2_BTN_PIN, INPUT_PULLUP);
    init_pin(JOYSTICK_Y_AXIS_PIN, INPUT_ADC);
    init_pin(JOYSTICK_X_AXIS_PIN, INPUT_ADC);
    init_pin(POTENTIOMETER_PIN, INPUT_ADC);
    init_momentary_button_pins();
    init_led_pins();

    // ADC init
    adc_init();
    opassert(adc_init_mutex());

    // Perform LED test
    LOG(LOG_LVL_INFO, "Performing LED test.");
    leds_test_blocking();

    // Create FreeRTOS timers
    LOG(LOG_LVL_INFO, "Creating FreeRTOS software timers.");
    status_led_timer = xTimerCreate("usb_status_led", pdMS_TO_TICKS(STAT_LED_UNMOUNTED_MS), pdTRUE, nullptr, status_led_timer_call);
    idle_hid_report_timer = xTimerCreate("idle_hid_report", pdMS_TO_TICKS(DEFAULT_IDLE_REPORT_INTERVAL_MS), pdTRUE, nullptr, idle_hid_report_timer_call);
    assert(status_led_timer != nullptr && idle_hid_report_timer != nullptr);
    led_timers_init();

    // Start the status LED blink timer
    (void) xTimerStart(status_led_timer, TIMER_COMMAND_TIMEOUT_T);

    // TinyUSB initialization
    LOG(LOG_LVL_INFO, "TinyUSB initialization.");
    tusb_init();
    (void) xTaskCreate(tusb_spin_task, "tusb_task", TUSB_TASK_STACK_DEPTH, nullptr, TUSB_TASK_PRIORITY, &tusb_spin_task_th);
    vTaskCoreAffinitySet(tusb_spin_task_th, 1 << 0);

    // Delete setup task
    vTaskDelete(nullptr);
}

// ---- Setup function (core 1) ----
void setup1(void *parameters) {
    (void) parameters;
    LOG(LOG_LVL_INFO, "Core 1 setup task started!");

    // Create alarm pool for core 1 timers
    LOG(LOG_LVL_INFO, "Creating core 1 alarm pool.");
    core_1_alarm_pool = alarm_pool_create(2, 3);

    // Delete setup task
    vTaskDelete(nullptr);
}


// ---- Device state management ----
void enter_state_resumed() {
    if(!resume_init && mount_init) {
        LOG(LOG_LVL_INFO, "Entering resumed state.");

        start_hid_reporters(core_1_alarm_pool);
        led_timers_start();
        leds_enable_override(false);
        set_led_outputs();
        vTaskResume(button_poll_task_th);
        opequal(xTimerStart(idle_hid_report_timer, TIMER_COMMAND_TIMEOUT_T), pdPASS);

        resume_init = true;
    }
}

void enter_state_suspended() {
    if (resume_init && mount_init) {
        LOG(LOG_LVL_INFO, "Entering suspended state.");

        stop_hid_reporters();
        led_timers_stop();
        leds_enable_override(true);
        vTaskSuspend(button_poll_task_th);
        opequal(xTimerStop(idle_hid_report_timer, TIMER_COMMAND_TIMEOUT_T), pdPASS);

        for (uint i = 0; i < NUMBER_OF_LEDS; i++) {
            gpio_put_pwm(led_pins_order[i], 0);
        }

        resume_init = false;
    }
}

void enter_state_mounted() {
    if (!mount_init) {
        LOG(LOG_LVL_INFO, "Entering mounted state, creating reporter tasks.");
        
        create_hid_reporter_tasks();
        create_button_poll_task();
        opequal(xTimerChangePeriod(idle_hid_report_timer, pdMS_TO_TICKS(DEFAULT_IDLE_REPORT_INTERVAL_MS), TIMER_COMMAND_TIMEOUT_T), pdPASS);
        mount_init = true;

        enter_state_resumed();
    }
}

void enter_state_unmounted() {
    if (mount_init) {
        LOG(LOG_LVL_INFO, "Entering unmounted state.");
        
        enter_state_suspended();
        delete_hid_reporter_tasks();
        delete_button_poll_task();

        for (uint i = 0; i < NUMBER_OF_LEDS; i++) {
            set_led_state(i, LED_SOLID_PWM, 0);
        }

        mount_init = false;
    }
}


// ****** END OF MAIN PROGRAM *******
// *********** ENTRYPOINT ***********
int main() {
    // Load the unique ID
    pico_get_unique_board_id_string(pico_unique_id, sizeof(pico_unique_id));

    // Logging over UART, USB HID used for data.
    stdio_uart_init();
    board_init();
    LOG(LOG_LVL_INFO, "Board init, program starting.");

    if (!logger.init_mutex()) {
        LOG(LOG_LVL_ERROR, "Logger mutex initialization failed!");
        return 0;
    }

    // Setup function tasks
    LOG(LOG_LVL_INFO, "Creating setup tasks.");
    xTaskCreateAffinitySet(setup, "setup_core0", SETUP_TASK_STACK_DEPTH, nullptr, SETUP_TASK_PRIORITY, (1 << 0), nullptr);
    xTaskCreateAffinitySet(setup1, "setup_core1", SETUP_TASK_STACK_DEPTH, nullptr, SETUP_TASK_PRIORITY, (1 << 1), nullptr);

    // Start FreeRTOS scheduler
    LOG(LOG_LVL_INFO, "Starting FreeRTOS scheduler...");
    vTaskStartScheduler();

    // We should never get to this point!
    LOG(LOG_LVL_FATAL, "Program exit. Scheduler start failed!");
    return 0;
}