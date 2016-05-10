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

#include <stdbool.h>
#include "FreeRTOS.h"
#include "pca9685.h"
#include "i2c_xfer.h"
#include "board.h"
#include "debug_console_imx.h"

#define PCA9685_REG_MODE1_DEFAULT_VALUE 0x11

#define PCA9685_REG_MODE1         0x00
#define PCA9685_REG_MODE2         0x01
#define PCA9685_REG_SUBADR1       0x02
#define PCA9685_REG_SUBADR2       0x03
#define PCA9685_REG_SUBADR3       0x04
#define PCA9685_REG_ALLCALLADR    0x05
#define PCA9685_REG_LED0_ON_L     0x06
#define PCA9685_REG_LED0_ON_H     0x07
#define PCA9685_REG_LED0_OFF_L    0x08
#define PCA9685_REG_LED0_OFF_H    0x09

#define PCA9685_REG_ALL_LED_ON_L  0xFA
#define PCA9685_REG_ALL_LED_ON_H  0xFB
#define PCA9685_REG_ALL_LED_OFF_L 0xFC
#define PCA9685_REG_ALL_LED_OFF_H 0xFD
#define PCA9685_REG_PRE_SCALE     0xFE

#define PCA9685_REG_MODE1_ALLCALL   0
#define PCA9685_REG_MODE1_SUB3      1
#define PCA9685_REG_MODE1_SUB2      2
#define PCA9685_REG_MODE1_SUB1      3
#define PCA9685_REG_MODE1_SLEEP     4
#define PCA9685_REG_MODE1_AI        5
#define PCA9685_REG_MODE1_EXTCLK    6
#define PCA9685_REG_MODE1_RESTART   7

bool pca9685_init(uint16_t frequency) {
    uint8_t dataBuffer[4];

    if (!I2C_XFER_ReceiveDataBlocking(BOARD_I2C_PCA9685_ADDR, PCA9685_REG_MODE1, dataBuffer, 1)) {
        return false;
    } else {
        if (dataBuffer[0] != PCA9685_REG_MODE1_DEFAULT_VALUE) {
            return false;
        }
    }

    dataBuffer[0] = (25000000L / (long) (4096 * frequency)) - 1;
    // 0x79 for 50Hz

    I2C_XFER_SendDataBlocking(BOARD_I2C_PCA9685_ADDR, PCA9685_REG_PRE_SCALE, dataBuffer, 1);

    I2C_XFER_ReceiveDataBlocking(BOARD_I2C_PCA9685_ADDR, PCA9685_REG_MODE1, dataBuffer, 1);
    dataBuffer[0] &= ~(1 << PCA9685_REG_MODE1_SLEEP);
    dataBuffer[0] |= (1 << PCA9685_REG_MODE1_AI);
    I2C_XFER_SendDataBlocking(BOARD_I2C_PCA9685_ADDR, PCA9685_REG_MODE1, dataBuffer, 1);

    return true;
}

void pca9685_set_output(uint8_t output, uint16_t duty) {
    uint8_t dataBuffer[4];
    uint16_t led_off;

    // PWM registers are 12 bits, so trim the argument in case of wrong value
    duty &= 0x0fff;

    //LED_ON = duty
    //LED_OFF = 4095 - duty

    dataBuffer[0] = duty & 0x00ff;
    dataBuffer[1] = (duty >> 8) & 0x00ff;

    led_off = 4095 - duty;
    dataBuffer[2] = led_off & 0x00ff;
    dataBuffer[3] = (led_off >> 8) & 0x00ff;

    I2C_XFER_SendDataBlocking(BOARD_I2C_PCA9685_ADDR, PCA9685_REG_LED0_ON_L + (4 * output), dataBuffer, 4);
}
