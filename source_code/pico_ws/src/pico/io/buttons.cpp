/*
    The ROS remote project - Buttons related IO helpers
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

#include "buttons.h"
#include "utils_lib/hardware.h"


// Last interrupt receive time (Last Interrupt Time)
uint32_t momen_btn_lit[NUMBER_OF_MOMENTARY_BUTTONS] = {0};


// ---- Button de-bouncing function ----
// ---- Returns true if the button should be considered pressed, false if not. ----
// ---- NOTE: Only for the 5 momentary push buttons! ----
bool button_bounce_check(uint8_t pin) {
    for (int i = 0; i < NUMBER_OF_MOMENTARY_BUTTONS; i++) {
        if (momen_btn_pins_order[i] == pin) {
            if (time_us_32() - momen_btn_lit[i] > (BUTTON_BOUNCE_TIME_MS * 1000)) {
                momen_btn_lit[i] = time_us_32();
                return true;
            } else {
                return false;
            }
        }
    }

    return false;
}

// ---- Initialize momentary button pins ----
void init_momentary_buttons() {
    for (int i = 0; i < NUMBER_OF_MOMENTARY_BUTTONS; i++) {
        init_pin(momen_btn_pins_order[i], INPUT_PULLUP);
        gpio_set_irq_enabled(momen_btn_pins_order[i], GPIO_IRQ_EDGE_FALL, true);
    }
}