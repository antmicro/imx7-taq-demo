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
#include "i2c_xfer.h"
#include "mpu6000.h"
#include "board.h"
#include "debug_console_imx.h"

bool mpu6000_init(void) {
    uint8_t dataBuffer[1];

    // check the ID of the MPU-6000
    if (!I2C_XFER_ReceiveDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_WHOAMI, dataBuffer, 1)) {
        PRINTF("MPU6000 not found!\n");
        return false;
    }

    if (dataBuffer[0] != BOARD_I2C_MPU6000_ADDR) {
        PRINTF("MPU6000 IS is frong (expected: 0x%02x received: 0x%02x)\n", BOARD_I2C_MPU6000_ADDR, dataBuffer[0]);
        return false;
    }

    // select clock source
    if (!I2C_XFER_ReceiveDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_PWR_MGMT_1, dataBuffer, 1)) {
        PRINTF("Faile dto read MPU_REG_PWR_MGMT_1\n");
        return false;
    }

    dataBuffer[0] |= MPU_REG_PWRMGMT_1_PLL_X_CLK;
    if (!I2C_XFER_SendDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_PWR_MGMT_1, dataBuffer, 1)) {
        PRINTF("Can't set MPU_REG_PWR_MGMT_1\n");
        return false;
    }

    // configure gyro (250 deg/s)
    dataBuffer[0] = 0x00;
    if (!I2C_XFER_SendDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_GYRO_CONFIG, dataBuffer, 1)) {
        PRINTF("Can't set MPU_REG_GYRO_CONFIG\n");
        return false;
    }

    // configure accelerometer (2g)
    dataBuffer[0] = 0x00;
    if (!I2C_XFER_SendDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_ACCEL_CONFIG, dataBuffer, 1)) {
        PRINTF("Can't set MPU_REG_ACCEL_CONFIG\n");
        return false;
    }

    // turn off sleep mode
    if (!I2C_XFER_ReceiveDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_PWR_MGMT_1, dataBuffer, 1)) {
        PRINTF("Faile dto read MPU_REG_PWR_MGMT_1\n");
        return false;
    }

    dataBuffer[0] &= ~MPU_REG_PWRMGMT_1_SLEEP;
    if (!I2C_XFER_SendDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_PWR_MGMT_1, dataBuffer, 1)) {
        PRINTF("Can't set MPU_REG_PWR_MGMT_1\n");
        return false;
    }
    return true;
}

bool mpu6000_read_accel(accel_t* acc) {
    if (!acc)
        return false;

    uint8_t dataBuffer[6];

    if (!I2C_XFER_ReceiveDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_ACCEL_XOUT_H, dataBuffer, 6)) {
        PRINTF("Failed to read MPU_REG_PWR_MGMT_1\n");
        return false;
    }

    acc->x = (((int16_t) dataBuffer[0]) << 8) | dataBuffer[1];
    acc->y = (((int16_t) dataBuffer[2]) << 8) | dataBuffer[3];
    acc->z = (((int16_t) dataBuffer[4]) << 8) | dataBuffer[5];

    return true;
}

bool mpu6000_read_gyro(gyro_t* gyro) {
    if (!gyro)
        return false;

    uint8_t dataBuffer[6];

    if (!I2C_XFER_ReceiveDataBlocking(BOARD_I2C_MPU6000_ADDR, MPU_GYRO_XOUT_H, dataBuffer, 6)) {
        PRINTF("Faile dto read MPU_REG_PWR_MGMT_1\n");
        return false;
    }

    gyro->x = (((int16_t) dataBuffer[0]) << 8) | dataBuffer[1];
    gyro->y = (((int16_t) dataBuffer[2]) << 8) | dataBuffer[3];
    gyro->z = (((int16_t) dataBuffer[4]) << 8) | dataBuffer[5];

    return true;
}
