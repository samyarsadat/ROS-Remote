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

#include "helpers_lib/Helpers.h"
#include "hardware/adc.h"


int main() 
{
    stdio_init_all();
    adc_init();

    init_pin(0, OUTPUT_PWM);
    init_pin(1, OUTPUT_PWM);
    init_pin(2, OUTPUT_PWM);

    init_pin(3, INPUT_PULLUP);
    init_pin(4, INPUT_PULLUP);
    init_pin(5, INPUT_PULLUP);
    init_pin(6, INPUT_PULLUP);
    init_pin(7, INPUT_PULLUP);

    init_pin(8, OUTPUT_PWM);
    init_pin(9, OUTPUT_PWM);
    init_pin(10, OUTPUT_PWM);
    init_pin(11, OUTPUT_PWM);
    init_pin(12, OUTPUT_PWM);
    init_pin(13, OUTPUT_PWM);
    init_pin(14, OUTPUT_PWM);
    init_pin(15, OUTPUT_PWM);

    init_pin(18, INPUT_PULLUP);
    init_pin(19, INPUT_PULLUP);
    init_pin(20, INPUT_PULLUP);
    init_pin(21, INPUT_PULLUP);
    init_pin(22, INPUT_PULLUP);

    init_pin(26, INPUT_ADC);
    init_pin(27, INPUT_ADC);
    init_pin(28, INPUT_ADC);

    init_pin(23, OUTPUT);
    gpio_put(23, HIGH);

    /*while (true)
    {
        for (int i = 0; i < 16; i++)
        {
            if (i == 3)
            {
                i = 8;
            }

            gpio_put_pwm(i, 65535);
            sleep_ms(100);
        }

        for (int i = 0; i < 16; i++)
        {
            if (i == 3)
            {
                i = 8;
            }

            gpio_put_pwm(i, 0);
            sleep_ms(100);
        }
    }*/

    for (int i = 0; i < 16; i++)
    {
        if (i == 3)
        {
            i = 8;
        }

        gpio_put_pwm(i, 65535);
        sleep_ms(250);
    }

    while (true)
    {
        // ADC
        adc_select_input(0);
        sleep_ms(1);
        float adc0 = adc_read();
        
        adc_select_input(1);
        sleep_ms(1);
        float adc1 = adc_read();
        
        adc_select_input(2);
        sleep_ms(1);
        float adc2 = adc_read();
        
        printf("A0: %.2f, A1: %.2f, A2: %.2f\r\n", adc0, adc1, adc2);

        // DIGITAL
        //printf("D3 %i, D4 %i, D5 %i, D6 %i, D7 %i, D18 %i, D19 %i, D20 %i, D21 %i, D22 %i\r\n", gpio_get(3), gpio_get(4), gpio_get(5), 
        //                                                                                        gpio_get(6), gpio_get(7), gpio_get(18), 
        //                                                                                        gpio_get(19), gpio_get(20), gpio_get(21), 
        //                                                                                        gpio_get(22));
    }
    
    return 0;
}