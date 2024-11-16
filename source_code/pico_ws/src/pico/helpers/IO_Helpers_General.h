/*
    The ROS remote project - IO Helper Module - General
    (LEDs, button states, joystick, etc.)
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

#pragma once


// ------- Libraries & Modules -------
#include "pico/stdlib.h"
#include "Definitions.h"
#include "FreeRTOS.h"
#include "timers.h"


// ------- Global variables -------

// ---- LED states ----
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

// ---- LEDs ----
extern const uint8_t led_pins_order[number_of_leds];

// ---- Momentary buttons ----
extern const uint8_t momen_btn_pins_order[number_of_momentary_buttons];

// ---- Joystick ----
extern float joystick_x_center_offset;
extern float joystick_y_center_offset;
extern uint16_t joystick_x_deadzone;
extern uint16_t joystick_y_deadzone;


// ------- Functions ------- 

// ---- Get joystick axis positions (readings) ----
// ---- These functions take into account the deadzone, offset, and inversion configs of the axis ----
// ---- They return values between -512 and +512, with 0 being center ----
int16_t get_joystick_x_val();
int16_t get_joystick_y_val();

// ---- Get potentiometer reading ----
// ---- This function takes into account the potentiometer's inversion config ----
// ---- It returns a value between 0 and 1024 ----
uint16_t get_potentiometer_val();

// ---- Button de-bouncing function ----
// ---- Returns true if the button should be considered pressed, false if not. ----
// ---- NOTE: Only for the 5 momentary push buttons! ----
bool button_bounce_check(uint8_t pin);

// ---- Initialize LEDs and LED state objects ----
void init_leds();

// ---- Set a single LED's state ----
// ---- This does not affect the output directly, it just changes the state of the LED's data structure ----
// MODES:
// 0: Solid PWM output, 1: Flashing PWM output (slow), 2: Fading to and from PWM output (slow), 
// 3: Flashing PWM output (fast), 4: Fading to and from PWM output (fast)
void set_led_state(uint8_t pin, uint8_t mode, uint16_t pwm_output);

// ---- Get a single LED's state ----
led_state_t get_led_state(uint8_t pin);

// ---- Set self-test state ----
void set_in_self_test(bool self_test_active);

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
void led_slow_flashing_timer_call(TimerHandle_t timer);
void led_fast_flashing_timer_call(TimerHandle_t timer);
void led_fading_timer_call(TimerHandle_t timer);