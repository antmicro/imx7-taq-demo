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

#include "seg_timers.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "debug_console_imx.h"
#include "MCIMX7D_M4.h"

void PWM_Init() {
    //            Repeat once         | Prescaler              | Clock hf            | Set at rollover    |
    PWM2->PWMCR = PWM_PWMCR_REPEAT(1) | PWM_PWMCR_PRESCALER(0) | PWM_PWMCR_CLKSRC(2) | PWM_PWMCR_POUTC(0);
    PWM2->PWMSAR = 1; //First sample

    //Check errors
    if (PWM2->PWMSR && PWM_PWMSR_FWE_MASK)
        PWM2->PWMSR = PWM_PWMSR_FWE_MASK;
    if (PWM2->PWMSR && PWM_PWMSR_ROV_MASK)
        PWM2->PWMSR = PWM_PWMSR_ROV_MASK;
    if (PWM2->PWMSR && PWM_PWMSR_CMP_MASK)
        PWM2->PWMSR = PWM_PWMSR_CMP_MASK;

    //Set period
    PWM2->PWMPR = 1024;

    //            Repeat once         | Prescaler              | Clock hf            | Set at rollover    |
    PWM3->PWMCR = PWM_PWMCR_REPEAT(1) | PWM_PWMCR_PRESCALER(0) | PWM_PWMCR_CLKSRC(2) | PWM_PWMCR_POUTC(0);
    PWM3->PWMSAR = 1; //First sample

    //Check errors
    if (PWM3->PWMSR && PWM_PWMSR_FWE_MASK)
        PWM3->PWMSR = PWM_PWMSR_FWE_MASK;
    if (PWM3->PWMSR && PWM_PWMSR_ROV_MASK)
        PWM3->PWMSR = PWM_PWMSR_ROV_MASK;
    if (PWM3->PWMSR && PWM_PWMSR_CMP_MASK)
        PWM3->PWMSR = PWM_PWMSR_CMP_MASK;

    //Set period
    PWM3->PWMPR = 1024;
}

void PWM2_Enable() {
    PWM2->PWMCR = PWM2->PWMCR | PWM_PWMCR_EN_MASK;
}

void PWM3_Enable() {
    PWM3->PWMCR = PWM3->PWMCR | PWM_PWMCR_EN_MASK;
}

void PWM2_Disable() {
    PWM2->PWMCR = PWM2->PWMCR & ~PWM_PWMCR_EN_MASK;
}

void PWM3_Disable() {
    PWM3->PWMCR = PWM3->PWMCR & ~PWM_PWMCR_EN_MASK;
}

void PWM2_SetWidth(uint16_t width) {
    if (width <= 1024) {
        PWM2->PWMSAR = width;
    }
}

void PWM3_SetWidth(uint16_t width) {
    if (width <= 1024) {
        PWM3->PWMSAR = width;
    }
}
