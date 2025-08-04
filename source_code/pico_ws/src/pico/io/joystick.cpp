/*
    The ROS remote project - Joystick and potentiometer helper
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

#include "joystick.h"
#include "config/hw_defs.h"
#include "config/sw_defs.h"
#include "hardware/adc.h"
#include "utils_lib/adc/adc_lock.h"
#include "utils_lib/math.h"
#include "utils_lib/hardware.h"
#include "diagnostics.h"


// ---- INTERNAL: Calculate joystick output ---- 
// ---- Used in the two functions below ----
inline int16_t calc_joystick_reading(uint16_t adc_reading, const bool inverted, 
                                     const float center_offset, const uint16_t deadzone) {
    // Inversion
    if (inverted) {
        adc_reading = 4095 - adc_reading;
    }

    // Center offset
    float reading = (adc_reading - (4095 / 2)) + center_offset;

    // Deadzone
    if (reading > deadzone) {
        reading = reading - deadzone;
    } else if (reading < (deadzone * -1)) {
        reading = reading + deadzone;
    } else {
        reading = 0;
    }

    float reading_max_val = (4095 / 2) + center_offset - deadzone;
    float reading_min_val = (-4095 / 2) + center_offset + deadzone;

    if (reading <= -1) {
        return map<int16_t>(reading, reading_min_val, -1, -512, -1);
    } else if (reading >= 1) {
        return map<int16_t>(reading, 1, reading_max_val, 1, 512);
    }

    return 0;
}

// ---- Get joystick axis positions (readings) ----
// ---- These functions take into account the deadzone, offset, and inversion configs of the axis ----
// ---- They return values between -512 and +512, with 0 being center ----
int16_t get_joystick_x_val() {
    if (adc_take_mutex()) {
        adc_select_input(get_gpio_adc_channel(JOYSTICK_X_AXIS_PIN));
        sleep_us(10);
        uint16_t adc_reading = adc_read();
        adc_release_mutex();

        return calc_joystick_reading(adc_reading, JOYSTICK_X_INVERTED, JOYSTICK_X_CENTER_OFFSET, JOYSTICK_X_DEADZONE);
    }

    LOG(LOG_LVL_ERROR, "Failed to acquire ADC mutex for joystick X axis reading.");
    return 0;
}

int16_t get_joystick_y_val() {
    if (adc_take_mutex()) {
        adc_select_input(get_gpio_adc_channel(JOYSTICK_Y_AXIS_PIN));
        sleep_us(10);
        uint16_t adc_reading = adc_read();
        adc_release_mutex();

        return calc_joystick_reading(adc_reading, JOYSTICK_Y_INVERTED, JOYSTICK_Y_CENTER_OFFSET, JOYSTICK_Y_DEADZONE);
    }

    LOG(LOG_LVL_ERROR, "Failed to acquire ADC mutex for joystick Y axis reading.");
    return 0;
}

// ---- Get potentiometer reading ----
// ---- This function takes into account the potentiometer's inversion config ----
// ---- It returns a value between 0 and 1024 ----
uint16_t get_potentiometer_val() {
    if (adc_take_mutex())  {
        adc_select_input(get_gpio_adc_channel(POTENTIOMETER_PIN));
        sleep_us(10);
        uint16_t adc_reading = adc_read();
        adc_release_mutex();

        #if !POTENTIOMETER_INVERTED
        return map<uint16_t>(adc_reading, 0, 4095, 0, 1024);
        #else
        return map<uint16_t>(adc_reading, 0, 4095, 1024, 0);
        #endif
    }

    LOG(LOG_LVL_ERROR, "Failed to acquire ADC mutex for potentiometer reading.");
    return 0;
}