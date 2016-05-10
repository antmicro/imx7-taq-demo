/*
 * Copyright (c) 2015-2016 Antmicro Ltd.
 *
 * This work is based on the Adafruit driver for the ADS1015/ADS1115 ADC
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

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ads1015.h"
#include "i2c_xfer.h"
#include "board.h"
#include "debug_console_imx.h"

#define ADS1015_CONVERT               0x00
#define ADS1015_CONFIG                0x01
#define ADS1015_LOWTHRESH             0x02
#define ADS1015_HITHRESH              0x03

#define ADS1015_CONFIG_OS_SINGLE    0x8000
#define ADS1015_CONFIG_OS_BUSY      0x0000
#define ADS1015_CONFIG_OS_NOTBUSY   0x8000

#define ADS1015_CONFIG_MUX_SINGLE_0 0x4000
#define ADS1015_CONFIG_MUX_SINGLE_1 0x5000
#define ADS1015_CONFIG_MUX_SINGLE_2 0x6000
#define ADS1015_CONFIG_MUX_SINGLE_3 0x7000

#define ADS1015_CONFIG_PGA_6_144V   0x0000
#define ADS1015_CONFIG_PGA_4_096V   0x0200
#define ADS1015_CONFIG_PGA_2_048V   0x0400
#define ADS1015_CONFIG_PGA_1_024V   0x0600
#define ADS1015_CONFIG_PGA_0_512V   0x0800
#define ADS1015_CONFIG_PGA_0_256V   0x0A00

#define ADS1015_CONFIG_MODE_CONTIN  0x0000
#define ADS1015_CONFIG_MODE_SINGLE  0x0100

#define ADS1015_CONFIG_DR_128SPS    0x0000
#define ADS1015_CONFIG_DR_250SPS    0x0020
#define ADS1015_CONFIG_DR_490SPS    0x0040
#define ADS1015_CONFIG_DR_920SPS    0x0060
#define ADS1015_CONFIG_DR_1600SPS   0x0080
#define ADS1015_CONFIG_DR_2400SPS   0x00A0
#define ADS1015_CONFIG_DR_3300SPS   0x00C0

#define ADS1015_CONFIG_CMODE_TRAD   0x0000
#define ADS1015_CONFIG_CMODE_WINDOW 0x0010

#define ADS1015_CONFIG_CPOL_ACTVLOW 0x0000
#define ADS1015_CONFIG_CPOL_ACTVHI  0x0008

#define ADS1015_CONFIG_CLAT_NONLAT  0x0000
#define ADS1015_CONFIG_CLAT_LATCH   0x0004

#define ADS1015_CONFIG_CQUE_1CONV   0x0000
#define ADS1015_CONFIG_CQUE_2CONV   0x0001
#define ADS1015_CONFIG_CQUE_4CONV   0x0002
#define ADS1015_CONFIG_CQUE_NONE    0x0003

uint16_t ads1015_read(uint8_t channel)
{
    uint8_t dataBuffer[4];
    uint16_t result;
    uint16_t config;

    if(channel > 3)
        return 0;

    config = ADS1015_CONFIG_CQUE_NONE    |
             ADS1015_CONFIG_CLAT_NONLAT  |
             ADS1015_CONFIG_CPOL_ACTVLOW |
             ADS1015_CONFIG_CMODE_TRAD   |
             ADS1015_CONFIG_DR_1600SPS   |
             ADS1015_CONFIG_MODE_SINGLE  |
             ADS1015_CONFIG_PGA_6_144V   |
             ADS1015_CONFIG_OS_SINGLE;

    channel += 4;
    config |= (((uint16_t)channel) << 12);

    dataBuffer[0] = (config >> 8) & 0xff;
    dataBuffer[1] = config & 0xff;

    I2C_XFER_SendDataBlocking(BOARD_I2C_ADS1015_ADDR, ADS1015_CONFIG, dataBuffer, 2);

    vTaskDelay(1);

    I2C_XFER_ReceiveDataBlocking(BOARD_I2C_ADS1015_ADDR, ADS1015_CONVERT, dataBuffer, 2);

    result = ((uint16_t)dataBuffer[0] << 8) | dataBuffer[1];
    result >>= 4;

    return result;
}
