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


// ------- Libraries & Modules -------
#include "IO_Helpers_General.h"
#include "Definitions.h"
#include "helpers_lib/Helpers.h"
#include "hardware/adc.h"
#include "local_helpers_lib/Local_Helpers.h"



// ------- External functions -------
extern void clean_shutdown();



// ------- Global variables -------

// ---- LEDs ----
const uint8_t led_pins_order[number_of_leds] = {right_kd2_led_pin, right_green_led_pin, right_blue_led_pin, left_top_yellow_led_pin, 
                                                left_top_green_led_pin, left_red_led_pin, left_bottom_yellow_led_pin, left_bottom_green_1_led_pin, 
                                                left_bottom_green_2_led_pin, left_red_kd2_led_pin, left_green_kd2_led_pin};

// ---- LED states ----
led_state_t led_states[number_of_leds];

// ---- Momentary buttons ----
const uint8_t momen_btn_pins_order[number_of_momentary_buttons] = {left_green_right_btn_pin, left_red_btn_pin, left_green_kd2_btn_pin,
                                                                   left_red_kd2_btn_pin, left_green_left_btn_pin};
uint32_t momen_btn_lit[number_of_momentary_buttons];   // Last interrupt receive time (Last Interrupt Time)

// ---- Joystick ----
float joystick_x_center_offset = default_joystick_x_center_offset;
float joystick_y_center_offset = default_joystick_y_center_offset;
uint16_t joystick_x_deadzone = default_joystick_x_deadzone;
uint16_t joystick_y_deadzone = default_joystick_y_deadzone;

// ---- Self-test state ----
bool is_in_self_test = false;



// ------- Functions ------- 

// ---- INTERNAL: Calculate joystick output ---- 
// ---- Used in the two functions below ----
int16_t calc_joystick_reading(uint16_t adc_reading, bool inverted, float center_offset, uint16_t deadzone)
{
    // Inversion
    if (inverted)
    {
        adc_reading = 4095 - adc_reading;
    }

    // Center offset
    float reading = (adc_reading - (4095 / 2)) + center_offset;

    // Deadzone
    {
        if (reading > deadzone) 
        {
            reading = reading - deadzone;
        }

        else if (reading < (deadzone * -1)) 
        {
            reading = reading + deadzone;
        }

        else
        {
            reading = 0;
        }
    }

    float reading_max_val = (4095 / 2) + center_offset - deadzone;
    float reading_min_val = (-4095 / 2) + center_offset + deadzone;

    if (reading <= -1)
    {
        return (int16_t) map(reading, reading_min_val, -1, -512, -1);
    }

    else if (adc_reading >= 1)
    {
        return (int16_t) map(reading, 1, reading_max_val, 1, 512);
    }

    return 0;
}


// ---- Get joystick axis positions (readings) ----
// ---- These functions take into account the deadzone, offset, and inversion configs of the axis ----
// ---- They return values between -512 and +512, with 0 being center ----
int16_t get_joystick_x_val()
{
    if (check_bool(adc_take_mutex(), RT_SOFT_CHECK))
    {
        adc_select_input(get_gpio_adc_channel(joystick_x_axis_pin));
        sleep_us(10);
        uint16_t adc_reading = adc_read();
        adc_release_mutex();

        return calc_joystick_reading(adc_reading, joystick_x_inverted, joystick_x_center_offset, joystick_x_deadzone);
    }

    return 0;
}

int16_t get_joystick_y_val()
{
    if (check_bool(adc_take_mutex(), RT_SOFT_CHECK))
    {
        adc_select_input(get_gpio_adc_channel(joystick_y_axis_pin));
        sleep_us(10);
        uint16_t adc_reading = adc_read();
        adc_release_mutex();

        return calc_joystick_reading(adc_reading, joystick_y_inverted, joystick_y_center_offset, joystick_y_deadzone);
    }

    return 0;
}


// ---- Get potentiometer reading ----
// ---- This function takes into account the potentiometer's inversion config ----
// ---- It returns a value between 0 and 1024 ----
uint16_t get_potentiometer_val()
{
    if (check_bool(adc_take_mutex(), RT_SOFT_CHECK))
    {
        adc_select_input(get_gpio_adc_channel(potentiometer_pin));
        sleep_us(10);
        uint16_t adc_reading = adc_read();
        adc_release_mutex();

        if (!potentiometer_inverted)
        {
            return (uint16_t) map(adc_reading, 0, 4095, 0, 1024);
        }

        else
        {
            return (uint16_t) map(adc_reading, 0, 4095, 1024, 0);
        }
    }

    return 0;
}


// ---- Button de-bouncing function ----
// ---- Returns true if the button should be considered pressed, false if not. ----
// ---- NOTE: Only for the 5 momentary push buttons! ----
bool button_bounce_check(uint8_t pin)
{
    for (int i = 0; i < number_of_momentary_buttons; i++)
    {
        if (momen_btn_pins_order[i] == pin)
        {
            if (time_us_32() - momen_btn_lit[i] > (button_bounce_time_ms * 1000))
            {
                momen_btn_lit[i] = time_us_32();
                return true;
            }

            else
            {
                return false;
            }
        }
    }

    return false;
}


// ---- Initialize LEDs and LED state structs ----
void init_leds()
{
    for (int i = 0; i < number_of_leds; i++)
    {
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
void set_led_state(uint8_t pin, uint8_t mode, uint16_t pwm_output)
{
    for (int i = 0; i < number_of_leds; i++)
    {
        if (led_states[i].pin == pin)
        {
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
led_state_t get_led_state(uint8_t pin)
{
    for (int i = 0; i < number_of_leds; i++)
    {
        if (led_states[i].pin == pin)
        {
            return led_states[i];
        }
    }

    return led_state_t{0, 0, 0, 0, 0, false};
}


// ---- Set self-test state ----
void set_in_self_test(bool self_test_active)
{
    is_in_self_test = self_test_active;
}


// ---- INTERNAL: put_pwm function that takes into account the self-test state ----
void gpio_put_pwm_wst(uint pin, uint16_t level)
{
    if (!is_in_self_test)
    {
        gpio_put_pwm(pin, level);
    }
}


// ---- Set all LED outputs ----
// ---- This function only handles LEDs that are set to mode 0 (solid PWM) ----
// ---- Flashing and fading modes are handled by timer tasks ----
void set_led_outputs()
{
    for (int i = 0; i < number_of_leds; i++)
    {
        if (led_states[i].mode == 0)
        {
            gpio_put_pwm_wst(led_states[i].pin, led_states[i].pwm_set_out);
            led_states[i].pwm_current_out = led_states[i].pwm_set_out;
        }
    }
}


// ---- Turn all LEDs off ----
void all_leds_off()
{
    for (int i = 0; i < number_of_leds; i++)
    {
        set_led_state(led_pins_order[i], 0, 0);
    }

    set_led_outputs();
}


// ---- Turn all LEDs on ----
void all_leds_on()
{
    for (int i = 0; i < number_of_leds; i++)
    {
        set_led_state(led_pins_order[i], 0, 65535);
    }

    set_led_outputs();
}


// ---- FreeRTOS timer callbacks ----
// ---- These are for the flashing and fading modes ----
void led_slow_flashing_timer_call(TimerHandle_t timer)
{
    for (int i = 0; i < number_of_leds; i++)
    {
        if (led_states[i].mode == 1)
        {
            if (led_states[i].pwm_current_out == 0)
            {
                gpio_put_pwm_wst(led_states[i].pin, led_states[i].pwm_set_out);
                led_states[i].pwm_current_out = led_states[i].pwm_set_out;
            }

            else
            {
                gpio_put_pwm_wst(led_states[i].pin, 0);
                led_states[i].pwm_current_out = 0;
            }
        }
    }
}

void led_fast_flashing_timer_call(TimerHandle_t timer)
{
    for (int i = 0; i < number_of_leds; i++)
    {
        if (led_states[i].mode == 3)
        {
            if (led_states[i].pwm_current_out == 0)
            {
                gpio_put_pwm_wst(led_states[i].pin, led_states[i].pwm_set_out);
                led_states[i].pwm_current_out = led_states[i].pwm_set_out;
            }

            else
            {
                gpio_put_pwm_wst(led_states[i].pin, 0);
                led_states[i].pwm_current_out = 0;
            }
        }
    }
}

void led_fading_timer_call(TimerHandle_t timer)
{
    for (int i = 0; i < number_of_leds; i++)
    {
        if (led_states[i].mode == 2 || led_states[i].mode == 4)
        {
            if (led_states[i].pwm_fade_steps_per_cycle == 0)
            {
                if (led_states[i].mode == 2)
                {
                    led_states[i].pwm_fade_steps_per_cycle = led_states[i].pwm_set_out / (led_slow_fading_time_ms / led_fade_exec_interval);
                }

                else   // Mode 4 (fast)
                {
                    led_states[i].pwm_fade_steps_per_cycle = led_states[i].pwm_set_out / (led_fast_fading_time_ms / led_fade_exec_interval);
                }
            }

            if (led_states[i].led_fade_rising)
            {
                led_states[i].pwm_current_out += led_states[i].pwm_fade_steps_per_cycle;

                if (led_states[i].pwm_current_out >= led_states[i].pwm_set_out)
                {
                    led_states[i].pwm_current_out = led_states[i].pwm_set_out;
                    led_states[i].led_fade_rising = false;
                }
            }

            else
            {
                led_states[i].pwm_current_out -= led_states[i].pwm_fade_steps_per_cycle;

                if (led_states[i].pwm_current_out <= 0)
                {
                    led_states[i].pwm_current_out = 0;
                    led_states[i].led_fade_rising = true;
                }
            }

            gpio_put_pwm_wst(led_states[i].pin, led_states[i].pwm_current_out);
        }
    }
}