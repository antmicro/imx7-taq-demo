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

#ifndef FREERTOS_MOTOR_DRIVER_H_
#define FREERTOS_MOTOR_DRIVER_H_

//Initializes PWMs and sets GPIOs
void Motors_Init(void);

//values from -1024 to 1024 are directly translated to PWM values and direction
void Motors_SetSpeed(short leftSpeed, short rightSpeed);

#endif /* FREERTOS_MOTOR_DRIVER_H_ */
