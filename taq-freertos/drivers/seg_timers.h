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

#ifndef FREERTOS_COMMON_SEG_TIMERS_H_
#define FREERTOS_COMMON_SEG_TIMERS_H_

#include "device_imx.h"
#include "pin_mux.h"

void PWM_Init();

void PWM2_Enable();
void PWM3_Enable();

void PWM2_Disable();
void PWM3_Disable();

void PWM2_SetWidth(uint16_t width);
void PWM3_SetWidth(uint16_t width);
#endif /* FREERTOS_COMMON_SEG_TIMERS_H_ */
