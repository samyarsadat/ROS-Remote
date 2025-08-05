/*
    The ROS remote project - LED IO helper module
    Copyright 2025 Samyar Sadat Akhavi.
    Written by Samyar Sadat Akhavi, 2025.
 
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

#include "leds.h"
#include "config/hw_defs.h"
#include "config/sw_defs.h"
#include "utils_lib/hardware.h"
#include "semphr.h"
#include "common/opassert.h"


// ------- Global variables -------
TimerHandle_t fast_led_flash_handler_timer, slow_led_flash_handler_timer, led_fade_handler_timer;
led_state_t led_states[NUMBER_OF_LEDS];
bool led_control_override = false;
const uint8_t led_pins_order[NUMBER_OF_LEDS] = {
    RIGHT_KD2_LED_PIN, RIGHT_GREEN_LED_PIN, RIGHT_BLUE_LED_PIN, LEFT_TOP_YELLOW_LED_PIN, 
    LEFT_TOP_GREEN_LED_PIN, LEFT_RED_LED_PIN, LEFT_BOTTOM_YELLOW_LED_PIN, LEFT_BOTTOM_GREEN_1_LED_PIN, 
    LEFT_BOTTOM_GREEN_2_LED_PIN, LEFT_RED_KD2_LED_PIN, LEFT_GREEN_KD2_LED_PIN
};


// ---- Initialize LEDs and LED state structs ----
void init_leds() {
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        led_states[i].pin = led_pins_order[i];
        led_states[i].mode = 0;
        led_states[i].pwm_current_out = 0;
        led_states[i].pwm_fade_steps_per_cycle = 0;
        led_states[i].pwm_set_out = 0;
        led_states[i].led_fade_rising = true;

        init_pin(led_pins_order[i], OUTPUT_PWM);
        gpio_put_pwm(led_pins_order[i], 0);
    }
}

// ---- Enable LED output override ----
void leds_enable_override(bool enable) {
    led_control_override = enable;
}

// ---- Set a single LED's state ----
// ---- This does not affect the output directly, it just changes the state of the LED's data structure ----
// MODES:
// 0: Solid PWM output, 1: Flashing PWM output (slow), 2: Fading to and from PWM output (slow), 
// 3: Flashing PWM output (fast), 4: Fading to and from PWM output (fast)
void set_led_state(uint8_t index, LED_MODE_t mode, uint16_t pwm_output) {
    assert(index < NUMBER_OF_LEDS);
    led_states[index].mode = mode;
    led_states[index].pwm_set_out = pwm_output;
    led_states[index].pwm_current_out = 0;
    led_states[index].pwm_fade_steps_per_cycle = 0;
    led_states[index].led_fade_rising = true;
}

// ---- Get a single LED's state ----
led_state_t get_led_state(uint8_t index) {
    assert(index < NUMBER_OF_LEDS);
    return led_states[index];
}

// ---- Self-test ----
void leds_test_blocking() {
    led_control_override = true;

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        gpio_put_pwm(led_pins_order[i], 0);
    }

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        gpio_put_pwm(led_pins_order[i], 65535);
        vTaskDelay(LED_TEST_DELAY_TICKS);
    }

    vTaskDelay(LED_TEST_DELAY_TICKS * 2);

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        gpio_put_pwm(led_pins_order[i], 0);
        vTaskDelay(LED_TEST_DELAY_TICKS);
    }

    led_control_override = false;
    set_led_outputs();
}

// ---- INTERNAL: put_pwm function that takes the output mutex ----
inline void gpio_put_pwm_ovd(uint pin, uint16_t level) {
    if (!led_control_override) {
        gpio_put_pwm(pin, level);
    }
}

// ---- Set all LED outputs ----
// ---- This function only handles LEDs that are set to mode 0 (solid PWM) ----
// ---- Flashing and fading modes are handled by timer tasks ----
void set_led_outputs() {
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        set_led_output_index(i);
    }
}

// ---- Set the output for a single LED ----
// ---- This also only handles LEDs that are set to mode 0 (solid PWM) ----
void set_led_output_index(uint8_t index) {
    assert(index < NUMBER_OF_LEDS);
    if (led_states[index].mode == LED_SOLID_PWM) {
        gpio_put_pwm_ovd(led_states[index].pin, led_states[index].pwm_set_out);
        led_states[index].pwm_current_out = led_states[index].pwm_set_out;
    }
}


// ---- FreeRTOS timer callbacks ----
// ---- These are for the flashing and fading modes ----
void led_slow_flashing_timer_call(TimerHandle_t timer) {
    (void) timer;
    
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].mode == LED_SLOW_FLASH) {
            if (led_states[i].pwm_current_out == 0) {
                gpio_put_pwm_ovd(led_states[i].pin, led_states[i].pwm_set_out);
                led_states[i].pwm_current_out = led_states[i].pwm_set_out;
            } else {
                gpio_put_pwm_ovd(led_states[i].pin, 0);
                led_states[i].pwm_current_out = 0;
            }
        }
    }
}

void led_fast_flashing_timer_call(TimerHandle_t timer) {
    (void) timer;

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].mode == LED_FAST_FLASH) {
            if (led_states[i].pwm_current_out == 0) {
                gpio_put_pwm_ovd(led_states[i].pin, led_states[i].pwm_set_out);
                led_states[i].pwm_current_out = led_states[i].pwm_set_out;
            } else {
                gpio_put_pwm_ovd(led_states[i].pin, 0);
                led_states[i].pwm_current_out = 0;
            }
        }
    }
}

void led_fading_timer_call(TimerHandle_t timer) {
    (void) timer;

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].mode == LED_SLOW_FADE || led_states[i].mode == LED_FAST_FADE) {
            if (led_states[i].pwm_fade_steps_per_cycle == 0) {
                if (led_states[i].mode == LED_SLOW_FADE) {
                    led_states[i].pwm_fade_steps_per_cycle = led_states[i].pwm_set_out / (LED_SLOW_FADING_TIME_MS / LED_FADE_EXEC_INTERVAL);
                } else {   // Mode 4 (fast)
                    led_states[i].pwm_fade_steps_per_cycle = led_states[i].pwm_set_out / (LED_FAST_FADING_TIME_MS / LED_FADE_EXEC_INTERVAL);
                }
            }

            if (led_states[i].led_fade_rising) {
                led_states[i].pwm_current_out += led_states[i].pwm_fade_steps_per_cycle;

                if (led_states[i].pwm_current_out >= led_states[i].pwm_set_out) {
                    led_states[i].pwm_current_out = led_states[i].pwm_set_out;
                    led_states[i].led_fade_rising = false;
                }
            } else {
                led_states[i].pwm_current_out -= led_states[i].pwm_fade_steps_per_cycle;

                if (led_states[i].pwm_current_out <= 0) {
                    led_states[i].pwm_current_out = 0;
                    led_states[i].led_fade_rising = true;
                }
            }

            gpio_put_pwm_ovd(led_states[i].pin, led_states[i].pwm_current_out);
        }
    }
}


// ---- FreeRTOS timer control ----
void led_timers_init() {
    slow_led_flash_handler_timer = xTimerCreate("slow_led_flash_timer", pdMS_TO_TICKS(LED_SLOW_FLASH_INTERVAL), pdTRUE, NULL, led_slow_flashing_timer_call);
    fast_led_flash_handler_timer = xTimerCreate("fast_led_flash_timer", pdMS_TO_TICKS(LED_FAST_FLASH_INTERVAL), pdTRUE, NULL, led_fast_flashing_timer_call);
    led_fade_handler_timer = xTimerCreate("led_fade_handler_timer", pdMS_TO_TICKS(LED_FADE_EXEC_INTERVAL), pdTRUE, NULL, led_fading_timer_call);
    assert(slow_led_flash_handler_timer != NULL && fast_led_flash_handler_timer != NULL && led_fade_handler_timer != NULL);
}

bool led_timers_start() {
    return xTimerStart(slow_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS &&
           xTimerStart(fast_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS &&
           xTimerStart(led_fade_handler_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS;
}

void led_timers_stop() {
    (void) xTimerStop(slow_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T);
    (void) xTimerStop(fast_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T);
    (void) xTimerStop(led_fade_handler_timer, TIMER_COMMAND_TIMEOUT_T);
}