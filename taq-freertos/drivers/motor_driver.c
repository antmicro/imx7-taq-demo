/*
 * Copyright (c) 2015-2016 Antmicro Ltd.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdlib.h>
#include "motor_driver.h"
#include "board.h"
#include "seg_timers.h"
#include "debug_console_imx.h"
#include "gpio_pins.h"

void Motors_Init(void) {
    PWM_Init();

    PWM2_SetWidth(0);
    PWM3_SetWidth(0);

    PWM2_Enable();
    PWM3_Enable();
}

void Motors_SetSpeed(short leftSpeed, short rightSpeed) {
    if (leftSpeed < 0) {
        GPIO_WritePinOutput(gpioMoto1A.base, gpioMoto1A.pin, false);
        GPIO_WritePinOutput(gpioMoto1B.base, gpioMoto1B.pin, true);
        if (leftSpeed < -1024) {
            leftSpeed = -1024;
        }
        PWM2_SetWidth(abs(leftSpeed));
    } else if (leftSpeed > 0) {
        GPIO_WritePinOutput(gpioMoto1A.base, gpioMoto1A.pin, true);
        GPIO_WritePinOutput(gpioMoto1B.base, gpioMoto1B.pin, false);
        if (leftSpeed > 1024) {
            leftSpeed = 1024;
        }
        PWM2_SetWidth(leftSpeed);
    } else {
        GPIO_WritePinOutput(gpioMoto1A.base, gpioMoto1A.pin, false);
        GPIO_WritePinOutput(gpioMoto1B.base, gpioMoto1B.pin, false);
        PWM2_SetWidth(0);
    }

    if (rightSpeed < 0) {
        GPIO_WritePinOutput(gpioMoto2A.base, gpioMoto2A.pin, true);
        GPIO_WritePinOutput(gpioMoto2B.base, gpioMoto2B.pin, false);
        if (rightSpeed < -1024) {
            rightSpeed = -1024;
        }
        PWM3_SetWidth(abs(rightSpeed));
    } else if (rightSpeed > 0) {
        GPIO_WritePinOutput(gpioMoto2A.base, gpioMoto2A.pin, false);
        GPIO_WritePinOutput(gpioMoto2B.base, gpioMoto2B.pin, true);
        if (rightSpeed > 1024) {
            rightSpeed = 1024;
        }
        PWM3_SetWidth(rightSpeed);
    } else {
        GPIO_WritePinOutput(gpioMoto2A.base, gpioMoto2A.pin, false);
        GPIO_WritePinOutput(gpioMoto2B.base, gpioMoto2B.pin, false);
        PWM3_SetWidth(0);
    }
}
