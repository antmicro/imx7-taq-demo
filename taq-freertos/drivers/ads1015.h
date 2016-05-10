/*
 * Copyright (c) 2015-2016 Antmicro Ltd.
 *
 * This work is based on Adafruit driver for the ADS1015/ADS1115 ADC
 * K.Townsend (Adafruit Industries)
 * https://github.com/adafruit/Adafruit_ADS1X15
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
 */

#ifndef ADS1015_H
#define ADS1015_H

uint16_t ads1015_read(uint8_t channel);

#endif
