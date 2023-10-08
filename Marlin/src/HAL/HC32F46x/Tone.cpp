/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * Copypaste of SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * Description: Tone function for HC32F46x
 * Derived from https://forum.arduino.cc/index.php?topic=136500.msg2903012#msg2903012
 */

#ifdef TARGET_HC32F46x

#include "../../inc/MarlinConfig.h"
#include "HAL.h"

#include <timera/timera_pwm.h>

static gpio_pin_t tone_pin;
static timera_config_t *unit;
en_timera_channel_t channel;
volatile static int64_t duration_cnt;

void Tone_Handler();

void tone(const gpio_pin_t _pin, const unsigned int frequency, const unsigned long duration/*=0*/) {
    tone_pin = _pin;

    int duty = 50;
    duration_cnt = (100 / duty) * 2 * frequency * duration / 1000;

    en_port_func_t port_function;
    pinMode(_pin, OUTPUT_PWM);
    timera_get_assignment(_pin, unit, channel, port_function);
    timera_pwm_set_period(unit, channel, FREQ_TO_PERIOD(frequency, TIMERA_PWM_UNIT_US));
    timera_pwm_set_duty(unit, channel, duty);
    attachInterrupt(_pin, &Tone_Handler, FALLING);
}

void noTone(const gpio_pin_t _pin) {
    timera_pwm_set_duty(unit, channel, 0);
    timera_pwm_set_period(unit, channel, 0);
    detachInterrupt(_pin);
}

void Tone_Handler() {
    if (duration_cnt) {
      duration_cnt--;
    }
    else noTone(tone_pin);                         // turn off interrupt
}

#endif // TARGET_HC32F46x
