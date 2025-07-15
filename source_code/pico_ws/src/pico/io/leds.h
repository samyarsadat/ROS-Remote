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

#pragma once
#include "pico/stdlib.h"
#include "config/hw_defs.h"
#include "FreeRTOS.h"
#include "timers.h"


struct led_state
{
    uint8_t pin;
    uint8_t mode;
    uint16_t pwm_set_out;

    int32_t pwm_current_out;
    uint16_t pwm_fade_steps_per_cycle;
    bool led_fade_rising;   // true: rising, false: falling
};
typedef struct led_state led_state_t;

inline const uint8_t led_pins_order[NUMBER_OF_LEDS] = {RIGHT_KD2_LED_PIN, RIGHT_GREEN_LED_PIN, RIGHT_BLUE_LED_PIN, LEFT_TOP_YELLOW_LED_PIN, 
                                                       LEFT_TOP_GREEN_LED_PIN, LEFT_RED_LED_PIN, LEFT_BOTTOM_YELLOW_LED_PIN, LEFT_BOTTOM_GREEN_1_LED_PIN, 
                                                       LEFT_BOTTOM_GREEN_2_LED_PIN, LEFT_RED_KD2_LED_PIN, LEFT_GREEN_KD2_LED_PIN};


// ---- Initialize LEDs and LED state objects ----
void init_leds();

// ---- Set a single LED's state ----
// ---- This does not affect the output directly, it just changes the state of the LED's data structure ----
// MODES:
// 0: Solid PWM output, 1: Flashing PWM output (slow), 2: Fading to and from PWM output (slow), 
// 3: Flashing PWM output (fast), 4: Fading to and from PWM output (fast)
void set_led_state(uint8_t index, uint8_t mode, uint16_t pwm_output);

// ---- Get a single LED's state ----
led_state_t get_led_state(uint8_t index);

// ---- Self-test ----
void leds_test();

// ---- Set all LED outputs ----
// ---- This function only handles LEDs that are set to mode 0 (solid PWM) ----
// ---- Flashing and fading modes are handled by timer tasks ----
void set_led_outputs();

// ---- Turn all LEDs off ----
void all_leds_off();

// ---- Turn all LEDs on ----
void all_leds_on();

// ---- FreeRTOS timer callbacks ----
// ---- These are for the flashing and fading modes ----
void led_timers_init();
void led_timers_destroy();
bool led_timers_start();
void led_timers_stop();
void led_slow_flashing_timer_call(TimerHandle_t timer);
void led_fast_flashing_timer_call(TimerHandle_t timer);
void led_fading_timer_call(TimerHandle_t timer);