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

#ifndef MPU6000_h
#define MPU6000_h

#define MPU_PWR_MGMT_1       0x6B
#define MPU_PWR_MGMT_2       0x6B
#define MPU_WHOAMI           0x75
#define MPU_GYRO_CONFIG      0x1B
#define MPU_ACCEL_CONFIG     0x1C
#define MPU_ACCEL_XOUT_H     0x3B
#define MPU_ACCEL_XOUT_L     0x3C
#define MPU_ACCEL_YOUT_H     0x3D
#define MPU_ACCEL_YOUT_L     0x3E
#define MPU_ACCEL_ZOUT_H     0x3F
#define MPU_ACCEL_ZOUT_L     0x40
#define MPU_GYRO_XOUT_H      0x43
#define MPU_GYRO_XOUT_L      0x44
#define MPU_GYRO_YOUT_H      0x45
#define MPU_GYRO_YOUT_L      0x46
#define MPU_GYRO_ZOUT_H      0x47
#define MPU_GYRO_ZOUT_L      0x48

/* Power management and clock selection */
#define MPU_REG_PWRMGMT_1_IMU_RST      0x80
#define MPU_REG_PWRMGMT_1_SLEEP        0x40
#define MPU_REG_PWRMGMT_1_CYCLE        0x20
#define MPU_REG_PWRMGMT_1_TEMP_DIS     0x08
#define MPU_REG_PWRMGMT_1_INTERN_CLK   0x00
#define MPU_REG_PWRMGMT_1_PLL_X_CLK    0x01
#define MPU_REG_PWRMGMT_1_PLL_Y_CLK    0x02
#define MPU_REG_PWRMGMT_1_PLL_Z_CLK    0x03
#define MPU_REG_PWRMGMT_1_STOP_CLK     0x07
#define MPU_REG_PWRMGMT_1_STOP_CLK     0x07

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accel_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} gyro_t;

bool mpu6000_init(void);
bool mpu6000_read_accel(accel_t* acc);
bool mpu6000_read_gyro(gyro_t* gyro);

#endif
