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


// ------- Global variables -------
TimerHandle_t fast_led_flash_handler_timer, slow_led_flash_handler_timer, led_fade_handler_timer;
led_state_t led_states[NUMBER_OF_LEDS];
bool in_self_test = false;


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

// ---- Set a single LED's state ----
// ---- This does not affect the output directly, it just changes the state of the LED's data structure ----
// MODES:
// 0: Solid PWM output, 1: Flashing PWM output (slow), 2: Fading to and from PWM output (slow), 
// 3: Flashing PWM output (fast), 4: Fading to and from PWM output (fast)
void set_led_state(uint8_t pin, uint8_t mode, uint16_t pwm_output) {
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].pin == pin) {
            led_states[i].mode = mode;
            led_states[i].pwm_set_out = pwm_output;
            led_states[i].pwm_current_out = 0;
            led_states[i].pwm_fade_steps_per_cycle = 0;
            led_states[i].led_fade_rising = true;
            return;
        }
    }
}

// ---- Get a single LED's state ----
led_state_t get_led_state(uint8_t pin) {
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].pin == pin) {
            return led_states[i];
        }
    }

    return led_state_t{0, 0, 0, 0, 0, false};
}

// ---- Self-test ----
void leds_test() {
    in_self_test = true;

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        gpio_put_pwm(led_pins_order[i], 0);
    }

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        gpio_put_pwm(led_pins_order[i], 65535);
        vTaskDelay(200);
    }

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        gpio_put_pwm(led_pins_order[i], 0);
        vTaskDelay(200);
    }

    in_self_test = false;
}

// ---- INTERNAL: put_pwm function that takes into account the self-test state ----
void gpio_put_pwm_wst(uint pin, uint16_t level) {
    if (!in_self_test) {
        gpio_put_pwm(pin, level);
    }
}

// ---- Set all LED outputs ----
// ---- This function only handles LEDs that are set to mode 0 (solid PWM) ----
// ---- Flashing and fading modes are handled by timer tasks ----
void set_led_outputs() {
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].mode == 0) {
            gpio_put_pwm_wst(led_states[i].pin, led_states[i].pwm_set_out);
            led_states[i].pwm_current_out = led_states[i].pwm_set_out;
        }
    }
}

// ---- Turn all LEDs off ----
void all_leds_off() {
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        set_led_state(led_pins_order[i], 0, 0);
    }

    set_led_outputs();
}

// ---- Turn all LEDs on ----
void all_leds_on() {
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        set_led_state(led_pins_order[i], 0, 65535);
    }

    set_led_outputs();
}

// ---- FreeRTOS timer callbacks ----
// ---- These are for the flashing and fading modes ----
void led_timers_init() {
    slow_led_flash_handler_timer = xTimerCreate("slow_led_flash_timer", pdMS_TO_TICKS(LED_SLOW_FLASH_INTERVAL), pdTRUE, nullptr, led_slow_flashing_timer_call);
    fast_led_flash_handler_timer = xTimerCreate("fast_led_flash_timer", pdMS_TO_TICKS(LED_FAST_FLASH_INTERVAL), pdTRUE, nullptr, led_fast_flashing_timer_call);
    led_fade_handler_timer = xTimerCreate("led_fade_handler_timer", pdMS_TO_TICKS(LED_FADE_EXEC_INTERVAL), pdTRUE, nullptr, led_fading_timer_call);
    assert(slow_led_flash_handler_timer != nullptr);
    assert(fast_led_flash_handler_timer != nullptr);
    assert(led_fade_handler_timer != nullptr);
}

void led_timers_destroy() {
    if (slow_led_flash_handler_timer != nullptr) {
        (void) xTimerDelete(slow_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T);
        slow_led_flash_handler_timer = nullptr;
    }

    if (fast_led_flash_handler_timer != nullptr) {
        (void) xTimerDelete(fast_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T);
        fast_led_flash_handler_timer = nullptr;
    }

    if (led_fade_handler_timer != nullptr) {
        (void) xTimerDelete(led_fade_handler_timer, TIMER_COMMAND_TIMEOUT_T);
        led_fade_handler_timer = nullptr;
    }
}

bool led_timers_start() {
    bool success[3] = {
        xTimerStart(slow_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS,
        xTimerStart(fast_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS,
        xTimerStart(led_fade_handler_timer, TIMER_COMMAND_TIMEOUT_T) == pdPASS
    };
    
    return success[0] && success[1] && success[2];
}

void led_timers_stop() {
    if (slow_led_flash_handler_timer != nullptr) {
        (void) xTimerStop(slow_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T);
    }

    if (fast_led_flash_handler_timer != nullptr) {
        (void) xTimerStop(fast_led_flash_handler_timer, TIMER_COMMAND_TIMEOUT_T);
    }

    if (led_fade_handler_timer != nullptr) {
        (void) xTimerStop(led_fade_handler_timer, TIMER_COMMAND_TIMEOUT_T);
    }
}

void led_slow_flashing_timer_call(TimerHandle_t timer) {
    (void) timer;
    
    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].mode == 1) {
            if (led_states[i].pwm_current_out == 0) {
                gpio_put_pwm_wst(led_states[i].pin, led_states[i].pwm_set_out);
                led_states[i].pwm_current_out = led_states[i].pwm_set_out;
            } else {
                gpio_put_pwm_wst(led_states[i].pin, 0);
                led_states[i].pwm_current_out = 0;
            }
        }
    }
}

void led_fast_flashing_timer_call(TimerHandle_t timer) {
    (void) timer;

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].mode == 3) {
            if (led_states[i].pwm_current_out == 0) {
                gpio_put_pwm_wst(led_states[i].pin, led_states[i].pwm_set_out);
                led_states[i].pwm_current_out = led_states[i].pwm_set_out;
            } else {
                gpio_put_pwm_wst(led_states[i].pin, 0);
                led_states[i].pwm_current_out = 0;
            }
        }
    }
}

void led_fading_timer_call(TimerHandle_t timer) {
    (void) timer;

    for (int i = 0; i < NUMBER_OF_LEDS; i++) {
        if (led_states[i].mode == 2 || led_states[i].mode == 4) {
            if (led_states[i].pwm_fade_steps_per_cycle == 0) {
                if (led_states[i].mode == 2) {
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

            gpio_put_pwm_wst(led_states[i].pin, led_states[i].pwm_current_out);
        }
    }
}