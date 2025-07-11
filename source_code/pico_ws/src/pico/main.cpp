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
#include "hardware/watchdog.h"
#include "hardware/adc.h"
#include "uros/uros_init.h"
#include "uros/sensor_publishers.h"
#include "uros_freertos_abstract_lib/uros_bridge.h"
#include "uros_freertos_abstract_lib/uros_executor.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "utils_lib/hardware.h"
#include "io/leds.h"
#include "io/buttons.h"
#include "utils_lib/adc/adc_lock.h"
#include "diagnostics.h"
#include "uros/sensor_publishers.h"
#include "common/opassert.h"


// ---- Global variables ----
alarm_pool_t *core_1_alarm_pool;
TimerHandle_t waiting_for_agent_timer;
uRosBridgeAgent *bridge;


// ---- Graceful reset ----
void reset_task(void* parameters) {
    (void) parameters;
    uint32_t notification_value;
    xTaskNotifyWait(0, 0xffffffff, &notification_value, portMAX_DELAY);
    
    watchdog_disable();
    watchdog_enable(WATCHDOG_RESET_TIMEOUT_MS, true);
    LOG(LOG_LVL_FATAL, "A clean reset has been triggered.");

    // Stop all repeating timers
    stop_sensor_publishers(); watchdog_update();
    led_timers_stop(); watchdog_update();
    
    if (!notification_value) {
        // We don't really need to destroy the timers as we're resetting anyway, but it's a good practice.
        led_timers_destroy(); watchdog_update();
    }
    
    // IO cleanup
    all_leds_off();
    init_pin(PICO_DEFAULT_LED_PIN, OUTPUT);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    watchdog_update();

    // MicroROS cleanup
    if (!notification_value) {
        // This will stop the bridge agent as well as the executor agents.
        // It will also cancel their repeating timers, and finalize all micro-ROS resources.
        bridge->uros_fini(); watchdog_update();
        LOG(LOG_LVL_INFO, "Micro-ROS finalized. Suspending the scheduler...");
    } else {
        LOG(LOG_LVL_INFO, "Skipping micro-ROS cleanup due to memory allocation concerns.");
    }

    // Suspend the FreeRTOS scheduler
    // No FreeRTOS API calls beyond this point!
    taskENTER_CRITICAL();
    portDISABLE_INTERRUPTS();
    
    watchdog_reset();
    while (1);  // It should take around 1ms for reset.
}

// ---- FreeRTOS task stack overflow hook ----
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    (void) xTask;
    LOG(LOG_LVL_FATAL, "Stack overflow! Task: %s", pcTaskName);
    REQ_SYSTEM_RESET(0);
}

// ---- FreeRTOS malloc failure hook ----
void vApplicationMallocFailedHook() {
    LOG(LOG_LVL_FATAL, "Memory allocation failure!");
    REQ_SYSTEM_RESET(1);
}

// ---- GPIO IRQ callback ----
void gpio_irq_call(uint pin, uint32_t events) {
    (void) events;

    // Momentary button IRQs
    if (button_bounce_check(pin)) {
        BaseType_t higher_prio_woken;
        xTaskNotifyFromISR(btn_state_publish_th, static_cast<uint32_t>(pin), eSetValueWithOverwrite, &higher_prio_woken);
        portYIELD_FROM_ISR(higher_prio_woken);
    }
}

// ---- Waiting for agent LED flash timer callback ----
void waiting_for_agent_timer_call(TimerHandle_t timer) {
    if (bridge->get_agent_state() == uRosBridgeAgent::WAITING_FOR_AGENT) {
        gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get_out_level(PICO_DEFAULT_LED_PIN));
        return;
    }

    if (bridge->get_agent_state() == uRosBridgeAgent::AGENT_AVAILABLE) {
        if (xTimerGetPeriod(timer) != pdMS_TO_TICKS(AGENT_AVAIL_LED_TOGGLE_DELAY_MS)) {
            (void) xTimerChangePeriod(timer, pdMS_TO_TICKS(AGENT_AVAIL_LED_TOGGLE_DELAY_MS), 0);
        }

        gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get_out_level(PICO_DEFAULT_LED_PIN));
        return;
    }

    if (bridge->get_agent_state() == uRosBridgeAgent::AGENT_CONNECTED) {
        gpio_put(PICO_DEFAULT_LED_PIN, true);
    } else {
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    (void) xTimerDelete(timer, TIMER_COMMAND_TIMEOUT_T);
}

// ---- Setup function (core 0) ----
void setup(void *parameters) {
    (void) parameters;
    LOG(LOG_LVL_INFO, "Core 0 setup task started!");

    // Create timer tasks
    LOG(LOG_LVL_INFO, "Creating timer tasks...");
    (void) xTaskCreate(reset_task, "sys_reset", RESET_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, &reset_task_handle);
    (void) xTaskCreate(publish_joystick_state, "joystick_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 3, &joystick_publish_th);
    (void) xTaskCreate(publish_potentiometer_state, "potentiometer_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &potentiometer_publish_th);
    (void) xTaskCreate(publish_btn_states, "btn_states_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &btn_state_publish_th);
    (void) xTaskCreate(publish_sw_states, "sw_states_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &sw_state_publish_th);

    LOG(LOG_LVL_INFO, "Hardware initialization...");
    
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

    gpio_set_irq_callback(gpio_irq_call);
    init_momentary_buttons();
    init_leds();

    // ADC init
    adc_init();
    adc_set_temp_sensor_enabled(true);
    if (!adc_init_mutex()) {
        LOG(LOG_LVL_FATAL, "ADC mutex initialization failed!");
        REQ_SYSTEM_RESET(0);
        while (1);
    }

    // Create FreeRTOS timers
    LOG(LOG_LVL_INFO, "Creating FreeRTOS software timers...");
    waiting_for_agent_timer = xTimerCreate("agent_wait_led", pdMS_TO_TICKS(AGENT_WAITING_LED_TOGGLE_DELAY_MS), pdTRUE, NULL, waiting_for_agent_timer_call);
    assert(waiting_for_agent_timer != NULL);
    led_timers_init();

    // Start MicroROS bridge agent
    LOG(LOG_LVL_INFO, "Starting micro-ROS bridge...");
    (void) bridge->start(configMAX_PRIORITIES - 1);

    // Start the waiting for MicroROS agent LED blink timer
    (void) xTimerStart(waiting_for_agent_timer, TIMER_COMMAND_TIMEOUT_T);

    // Delete setup task
    vTaskDelete(NULL);
}

// ---- Setup function (core 1) ----
void setup1(void *parameters) {
    (void) parameters;
    LOG(LOG_LVL_INFO, "Core 1 setup task started!");

    // Create alarm pool for core 1 timers
    LOG(LOG_LVL_INFO, "Creating core 1 alarm pool...");
    core_1_alarm_pool = alarm_pool_create(2, 8);

    // Delete setup task
    vTaskDelete(NULL);
}

// ---- Micro-ROS init & fini functions ----
bool uros_init() {
    LOG(LOG_LVL_INFO, "Micro-ROS initializing...");
    
    UROS_RETCODE_CHECK_MSG(
        bridge->uros_init_node(UROS_NODE_NAME, UROS_NODE_NAMESPACE, UROS_DOMAIN_ID), 
        "Micro-ROS node initialization"
    );

    if (!uros_init_ent()) {
        LOG(LOG_LVL_ERROR, "Micro-ROS entity initialization failed!");
        return false;
    }

    UROS_RETCODE_CHECK_MSG(
        bridge->uros_init_executors(),
        "Micro-ROS executors initialization"
    );
    
    if (!uros_exec_setup()) {
        LOG(LOG_LVL_ERROR, "Micro-ROS executors setup failed!");
        return false;
    }

    (void) core0_executor.start(configMAX_PRIORITIES - 1, (1 << 0), true);
    (void) core1_executor.start(configMAX_PRIORITIES - 1, (1 << 1), true);

    LOG(LOG_LVL_INFO, "Micro-ROS initialized successfully.");
    return true;
}

void uros_fini() {
    LOG(LOG_LVL_INFO, "Requesting reset to finalize micro-ROS...");
    REQ_SYSTEM_RESET(0);
}


// ****** END OF MAIN PROGRAM *******
// *********** ENTRYPOINT ***********
int main() {
    // UART & USB STDIO outputs
    stdio_init_all();
    stdio_filter_driver(&stdio_uart);   // Filter the output of STDIO to UART.
    
    sleep_ms(10);
    LOG(LOG_LVL_INFO, "STDIO init, program starting.");

    if (!logger.init_mutex()) {
        LOG(LOG_LVL_ERROR, "Logger mutex initialization failed!");
        return 0;
    }

    // MicroROS pre-init
    LOG(LOG_LVL_INFO, "MicroROS pre-init...");
    bridge = uRosBridgeAgent::get_instance();
    bridge->configure(uros_init, uros_fini);

    // Setup function tasks
    LOG(LOG_LVL_INFO, "Creating setup tasks...");
    xTaskCreateAffinitySet(setup, "setup_core0", SETUP_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, (1 << 0), NULL);
    xTaskCreateAffinitySet(setup1, "setup_core1", SETUP_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, (1 << 1), NULL);

    // Start FreeRTOS scheduler
    LOG(LOG_LVL_INFO, "Starting FreeRTOS scheduler...");
    vTaskStartScheduler();

    // We should never get to this point!
    LOG(LOG_LVL_FATAL, "Program exit. Scheduler start failed!");
    return 0;
}