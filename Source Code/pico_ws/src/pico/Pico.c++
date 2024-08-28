/*
    The ROS remote project - Raspberry Pi Pico firmware
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
#include "hardware/adc.h"
#include "helpers/Self_Test.c++"
#include "helpers/Sensor_Publishers.c++"
#include "helpers/uROS_Init.h"
#include "timers.h"



// ------- Global variables -------

// ---- Misc. ----
alarm_pool_t *core_1_alarm_pool;

// ---- Timer execution times storage (milliseconds) ----
uint32_t last_sw_state_publish_time, last_btn_state_publish_time;
uint32_t last_joystick_state_publish_time, last_potentiometer_state_publish_time;

// ---- Timers ----
struct repeating_timer sw_state_publish_rt, btn_state_publish_rt, joystick_publish_rt, potentiometer_publish_rt;
TaskHandle_t btn_state_publish_th, sw_state_publish_th, joystick_publish_th, potentiometer_publish_th;
TimerHandle_t waiting_for_agent_timer, fast_led_flash_handler_timer;
TimerHandle_t slow_led_flash_handler_timer, led_fade_handler_timer;



// ------- Library object inits -------

// ---- MicroROS ----
uRosBridgeAgent *bridge;
uRosPublishingHandler *pub_handler;



// ------- Functions ------- 

// ---- Graceful shutdown ----
void clean_shutdown()
{
    write_log("A clean shutdown has been triggered. The program will now shut down.", LOG_LVL_FATAL, FUNCNAME_ONLY);

    // Stop all repeating timers
    cancel_repeating_timer(&sw_state_publish_rt);
    cancel_repeating_timer(&btn_state_publish_rt);
    cancel_repeating_timer(&joystick_publish_rt);
    cancel_repeating_timer(&potentiometer_publish_rt);
    xTimerStop(fast_led_flash_handler_timer, 0);
    xTimerStop(slow_led_flash_handler_timer, 0);
    xTimerStop(led_fade_handler_timer, 0);

    // IO cleanup
    all_leds_off();
    init_pin(onboard_led, OUTPUT);
    gpio_put(onboard_led, LOW);

    // MicroROS cleanup
    pub_handler->stop();
    bridge->uros_fini();

    write_log("MicroROS cleanup completed. Suspending the scheduler...", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Suspend the FreeRTOS scheduler
    // No FreeRTOS API calls beyond this point!
    vTaskSuspendAll();
}


// ---- FreeRTOS task stack overflow hook ----
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    write_log("Stack overflow! Task: " + std::string(pcTaskName, strlen(pcTaskName)), LOG_LVL_FATAL, FUNCNAME_ONLY);
    clean_shutdown();
}


// ---- FreeRTOS memory allocation failure hook ----
void vApplicationMallocFailedHook()
{
    // Not calling clean_shutdown() because it calls write_log() which uses malloc.
    // This shouldn't ever happen anyway, so it doesn't matter.
    panic("Memory allocation failed!");
}


// ---- Timer callbacks for task notification ----
bool publish_sw_state_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(sw_state_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_btn_state_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(btn_state_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_joystick_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(joystick_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}

bool publish_potentiometer_notify(struct repeating_timer *rt)
{
    BaseType_t higher_prio_woken;
    vTaskNotifyGiveFromISR(potentiometer_publish_th, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
    return true;
}


// ------- MicroROS subscriber & service callbacks ------- 

// ---- Get/set joystick configuration services ----
void get_joystick_config_callback(const void *req, void *res)
{
    remote_pico_coms__srv__GetJoystickConfig_Request *req_in = (remote_pico_coms__srv__GetJoystickConfig_Request *) req;
    remote_pico_coms__srv__GetJoystickConfig_Response *res_in = (remote_pico_coms__srv__GetJoystickConfig_Response *) res;

    write_log("Get joystick configuration request received!", LOG_LVL_INFO, FUNCNAME_ONLY);

    res_in->joystick_x_center_offset = joystick_x_center_offset;
    res_in->joystick_y_center_offset = joystick_y_center_offset;
    res_in->joystick_x_deadzone = joystick_x_deadzone;
    res_in->joystick_y_deadzone = joystick_y_deadzone;
}

void set_joystick_config_callback(const void *req, void *res)
{
    remote_pico_coms__srv__SetJoystickConfig_Request *req_in = (remote_pico_coms__srv__SetJoystickConfig_Request *) req;
    remote_pico_coms__srv__SetJoystickConfig_Response *res_in = (remote_pico_coms__srv__SetJoystickConfig_Response *) res;

    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Set joystick configuration request received! [xd: %d, yd: %d, xco: %.2f, yco: %.2f]", 
             req_in->joystick_x_deadzone, req_in->joystick_y_deadzone, req_in->joystick_x_center_offset, req_in->joystick_y_center_offset);
    write_log(buffer, LOG_LVL_INFO, FUNCNAME_ONLY);

    joystick_x_deadzone = req_in->joystick_x_deadzone;
    joystick_y_deadzone = req_in->joystick_y_deadzone;
    joystick_x_center_offset = req_in->joystick_x_center_offset;
    joystick_y_center_offset = req_in->joystick_y_center_offset;

    res_in->success = true;
}


// ---- Get/set LED states service ----
void get_led_states_callback(const void *req, void *res)
{
    remote_pico_coms__srv__GetLedStates_Request *req_in = (remote_pico_coms__srv__GetLedStates_Request *) req;
    remote_pico_coms__srv__GetLedStates_Response *res_in = (remote_pico_coms__srv__GetLedStates_Response *) res;

    // NOTE: This service is called very frequently, so there will be no logging!

    for (int i = 0; i < number_of_leds; i++)
    {
        led_state_t state = get_led_state(led_pins_order[i]);
        res_in->led_modes[i] = state.mode;
        res_in->pwm_outputs[i] = state.pwm_set_out;
    }
}

void set_led_states_callback(const void *req, void *res)
{
    remote_pico_coms__srv__SetLedStates_Request *req_in = (remote_pico_coms__srv__SetLedStates_Request *) req;
    remote_pico_coms__srv__SetLedStates_Response *res_in = (remote_pico_coms__srv__SetLedStates_Response *) res;

    // NOTE: This service is called very frequently, so there will be no logging!

    for (int i = 0; i < number_of_leds; i++)
    {
        if (req_in->set_state_mask[i])
        {
            set_led_state(led_pins_order[i], req_in->led_modes[i], req_in->pwm_outputs[i]);
        }
    }

    res_in->success = true;
}



// ------- Main program -------

// ---- MicroROS executor post-execution function ----
void uros_post_exec_call()
{
    // Cleanup for self-test diagnostics messages.
    if (!self_test_diag_data_slot_nums.empty())
    {
        for (auto slot_num : self_test_diag_data_slot_nums)
        {
            destroy_uros_diag_status_msg(slot_num);
            destroy_diag_kv_pair(slot_num);
            destroy_diag_kv_pair_refs(slot_num);
            destroy_diag_msg_object(slot_num);
            deallocate_slots(slot_num);
        }

        self_test_diag_status_reports.clear();
        self_test_diag_status_reports.shrink_to_fit();
        self_test_diag_data_slot_nums.clear();
        self_test_diag_data_slot_nums.shrink_to_fit();

        enable_diag_pub();
    }
}


// ---- Setup repeating timers ----
void start_timers()
{
    write_log("Starting hardware timers...", LOG_LVL_INFO, FUNCNAME_ONLY);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, sw_state_pub_rt_interval, publish_sw_state_notify, NULL, &sw_state_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, btn_state_pub_rt_interval, publish_btn_state_notify, NULL, &btn_state_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, joystick_pub_rt_interval, publish_joystick_notify, NULL, &joystick_publish_rt);
    alarm_pool_add_repeating_timer_ms(core_1_alarm_pool, potentiometer_pub_rt_interval, publish_potentiometer_notify, NULL, &potentiometer_publish_rt);

    write_log("Starting FreeRTOS timer...", LOG_LVL_INFO, FUNCNAME_ONLY);
    xTimerStart(fast_led_flash_handler_timer, 0);
    xTimerStart(slow_led_flash_handler_timer, 0);
    xTimerStart(led_fade_handler_timer, 0);
}


// ---- Waiting for agent LED flash timer callback ----
void waiting_for_agent_timer_call(TimerHandle_t timer)
{
    if (bridge->get_agent_state() == uRosBridgeAgent::WAITING_FOR_AGENT)
    {
        gpio_put(onboard_led, !gpio_get_out_level(onboard_led));
        return;
    }

    if (bridge->get_agent_state() == uRosBridgeAgent::AGENT_AVAILABLE)
    {
        if (xTimerGetPeriod(timer) != pdMS_TO_TICKS(AGENT_AVAIL_LED_TOGGLE_DELAY_MS))
        {
            xTimerChangePeriod(timer, AGENT_AVAIL_LED_TOGGLE_DELAY_MS, 0);
        }

        gpio_put(onboard_led, !gpio_get_out_level(onboard_led));
        return;
    }

    if (bridge->get_agent_state() == uRosBridgeAgent::AGENT_CONNECTED)
    {
        gpio_put(onboard_led, HIGH);
    }

    else
    {
        gpio_put(onboard_led, LOW);
    }

    xTimerDelete(timer, 0);
}


// ---- Setup function (Runs once on core 0) ----
void setup(void *parameters)
{
    init_print_uart_mutex();
    write_log("Core 0 setup task started!", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Initialize some mutexes
    diag_mutex_init();

    // Pin init
    init_leds();
    init_pin(onboard_led, OUTPUT);
    init_pin(right_top_toggle_sw_pin, INPUT_PULLUP);
    init_pin(left_key_sw_pin, INPUT_PULLUP);
    init_pin(left_top_toggle_sw_pin, INPUT_PULLUP);
    init_pin(right_e_stop_btn_pin, INPUT_PULLUP);
    init_pin(right_kd2_btn_pin, INPUT_PULLUP);
    init_pin(left_green_right_btn_pin, INPUT_PULLUP);
    init_pin(left_red_btn_pin, INPUT_PULLUP);
    init_pin(left_green_kd2_btn_pin, INPUT_PULLUP);
    init_pin(left_red_kd2_btn_pin, INPUT_PULLUP);
    init_pin(left_green_left_btn_pin, INPUT_PULLUP);
    init_pin(joystick_y_axis_pin, INPUT_ADC);
    init_pin(joystick_x_axis_pin, INPUT_ADC);
    init_pin(potentiometer_pin, INPUT_ADC);

    // Force SMPS into PWM mode
    init_pin(smps_power_save_pin, OUTPUT);
    gpio_put(smps_power_save_pin, HIGH);

    // Misc. init
    adc_init();
    adc_init_mutex();
    adc_set_temp_sensor_enabled(true);

    // Create timer tasks
    write_log("Creating timer tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    xTaskCreate(publish_joystick_state, "joystick_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 3, &joystick_publish_th);
    xTaskCreate(publish_potentiometer_state, "potentiometer_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &potentiometer_publish_th);
    xTaskCreate(publish_btn_states, "btn_states_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &btn_state_publish_th);
    xTaskCreate(publish_sw_states, "sw_states_publish", TIMER_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 4, &sw_state_publish_th);
    //vTaskCoreAffinitySet(joystick_publish_th, (1 << 1));      // Lock task to core 1
    vTaskCoreAffinitySet(potentiometer_publish_th, (1 << 1));   // Lock task to core 1
    vTaskCoreAffinitySet(btn_state_publish_th, (1 << 1));       // Lock task to core 1
    vTaskCoreAffinitySet(sw_state_publish_th, (1 << 1));        // Lock task to core 1

    // Create FreeRTOS timers
    write_log("Creating FreeRTOS software timers...", LOG_LVL_INFO, FUNCNAME_ONLY);
    waiting_for_agent_timer = xTimerCreate("waiting_for_agent_timer", pdMS_TO_TICKS(AGENT_WAITING_LED_TOGGLE_DELAY_MS), pdTRUE, NULL, waiting_for_agent_timer_call);
    slow_led_flash_handler_timer = xTimerCreate("slow_led_flash_timer", pdMS_TO_TICKS(led_slow_flash_interval), pdTRUE, NULL, led_slow_flashing_timer_call);
    fast_led_flash_handler_timer = xTimerCreate("fast_led_flash_timer", pdMS_TO_TICKS(led_fast_flash_interval), pdTRUE, NULL, led_fast_flashing_timer_call);
    led_fade_handler_timer = xTimerCreate("led_fade_handler_timer", pdMS_TO_TICKS(led_fade_exec_interval), pdTRUE, NULL, led_fading_timer_call);

    // Start MicroROS tasks
    write_log("Starting MicroROS tasks...", LOG_LVL_INFO, FUNCNAME_ONLY);
    check_bool(bridge->start(configMAX_PRIORITIES - 1, (1 << 0), true), RT_HARD_CHECK);
    check_bool(pub_handler->start(configMAX_PRIORITIES - 2, (1 << 0), true), RT_HARD_CHECK);

    // Start the waiting for MicroROS agent LED blink timer
    xTimerStart(waiting_for_agent_timer, 0);

    // Delete setup task
    vTaskDelete(NULL);
}


// ---- Setup function (Runs once on core 1) ----
void setup1(void *parameters)
{
    write_log("Core 1 setup task started!", LOG_LVL_INFO, FUNCNAME_ONLY);

    // Create alarm pool for core 1 timers
    write_log("Creating core 1 alarm pool...", LOG_LVL_INFO, FUNCNAME_ONLY);
    core_1_alarm_pool = alarm_pool_create(2, 8);

    // Delete setup task
    vTaskDelete(NULL);
}



// ******** END OF MAIN PROGRAM *********
// *********** STARTUP & INIT ***********

// ---- Entrypoint ----
int main() 
{
    // UART & USB STDIO outputs
    stdio_init_all();
    stdio_filter_driver(&stdio_uart);   // Filter the output of STDIO to UART.
    print_uart_usb_override();
    write_log("STDIO init, program starting...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);

    // Wait for 3 seconds
    init_pin(onboard_led, OUTPUT);

    for (int i = 0; i < STARTUP_WAIT_TIME_S; i++)
    {
        write_log("Startup wait " + std::to_string(i + 1) + "...", LOG_LVL_INFO, FUNCNAME_ONLY);
        gpio_put(onboard_led, HIGH);
        sleep_ms(500);
        gpio_put(onboard_led, LOW);
        sleep_ms(500);
    }

    // MicroROS pre-init
    write_log("MicroROS pre-init...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    bridge = uRosBridgeAgent::get_instance();
    pub_handler = uRosPublishingHandler::get_instance();
    bridge->pre_init(uros_init, clean_shutdown, uros_post_exec_call);
    pub_handler->pre_init(bridge);
    set_diag_pub_queue(pub_handler->get_queue_handle());

    // Stetup function tasks
    write_log("Creating setup tasks...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    xTaskCreateAffinitySet(setup, "setup_core_0", SETUP_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, (1 << 0), NULL);
    xTaskCreateAffinitySet(setup1, "setup_core_1", SETUP_TASK_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, (1 << 1), NULL);

    // Start FreeRTOS scheduler
    write_log("Starting FreeRTOS scheduler...", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    vTaskStartScheduler();

    // We should never get to this point!
    write_log("Program exit. Scheduler start failed!", LOG_LVL_INFO, FUNCNAME_LINE_ONLY);
    
    return 0;
}