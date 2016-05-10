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
#include <math.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "seg_timers.h"
#include "debug_console_imx.h"
#include "gpio_pins.h"
#include "i2c_xfer.h"
#include "mpu6000.h"
#include "motor_driver.h"
#include "kalman.h"
#include "pid.h"
#include "pca9685.h"
#include "ads1015.h"
#include "semphr.h"
#include "MCIMX7D_M4.h"
#include "mu_imx.h"
#include "plat_porting.h"
#include "semphr.h"
#include "string.h"
#include "assert.h"
#include "rpmsg/rpmsg.h"
#include <stdio.h>

#define APP_MU_IRQ_PRIORITY 3
#define MAX_STRING_SIZE 496         /* Maximum size to hold the data A7 gives */
#define STRING_BUFFER_CNT 3

volatile int enc1Position = 0, enc1PreviousPosition = 0;
volatile int enc2Position = 0, enc2PreviousPosition = 0;
volatile int enc1Speed = 0, enc2Speed = 0;
float kp = 56.0, ki = 43.0, kd = 26.0, kpv = 54.0, kiv = 45.0, kdv = 1; //Initial PID settings
float Setpoint, Input, Output, Setpointv, DesiredSetpointv, Inputv, Outputv;
float measuredAngle;
float angulatAcceleration;

int Voltage = 0;

static volatile int handPos1 = 0;
static volatile int handPos2 = 0;

int32_t driveOffset = 0;
TickType_t previousTime = 0, currentTime = 0;

PIDState PID1, //Angle
        PID2; //Speed

#define W32(a, b) (*(volatile unsigned int   *)(a)) = b
#define R32(a)    (*(volatile unsigned int   *)(a))

SemaphoreHandle_t i2cMutex = NULL;
static struct remote_device *rdev;
static struct rpmsg_channel *app_chnl;
static char strVar[STRING_BUFFER_CNT][MAX_STRING_SIZE + 1];
static uint8_t app_idx = 0;
static uint8_t handler_idx = 0;
static SemaphoreHandle_t app_sema;

/* rpmsg_rx_callback will call into this for a channel creation event*/
static void rpmsg_channel_created(struct rpmsg_channel *rp_chnl) {
    app_chnl = rp_chnl;
    xSemaphoreGiveFromISR(app_sema, NULL);
}

static void rpmsg_channel_deleted(struct rpmsg_channel *rp_chnl) {
    rpmsg_destroy_ept(rp_chnl->rp_ept);
}

static void rpmsg_read_cb(struct rpmsg_channel *rp_chnl, void *data, int len, void * priv, unsigned long src) {
    /*
     * Temporarily Disable MU Receive Interrupt to avoid master
     * sending too many messages and remote will fail to keep pace
     * to consume
     */
    MU_DisableRxFullInt(MU0_B, MU_RPMSG_CHANNEL);
    /*
     * Copy to next app string buffer
     */
    assert(len <= MAX_STRING_SIZE);
    memcpy((void*) strVar[handler_idx], data, len);
    /*
     * Add trailing '\0'
     */
    strVar[handler_idx][len] = 0;
    handler_idx = (handler_idx + 1) % STRING_BUFFER_CNT;
    xSemaphoreGiveFromISR(app_sema, NULL);
}

/*!
 * Sets a servo position
 * @param id - servo ID (0 - 3)
 * @param value - servo angle (0 - 180)
 * @return
 */
bool setServo(uint8_t id, uint16_t value) {
    if (value > 180) {
        return false;
    }

    if (id > 3) {
        return false;
    }

    /*
     0 deg   - 50/2048
     180 deg - 270/2048
     */
    value *= 220;
    value /= 180;
    value += 50;

    if (xSemaphoreTake(i2cMutex, 300)) {
        pca9685_set_output(id, value);
        xSemaphoreGive(i2cMutex);
    }

    return true;
}

/*!
 * Reads and converts data from GP2YA60SZLF
 * @return Distance [cm] / -1 when error
 */
float getDistance() {
    if (xSemaphoreTake(i2cMutex, 500)) {
        uint16_t adc = ads1015_read(0);

        float Voltage = ((float) adc * 3.3) / 1023.0;
        float Distance = 24.062 * pow(Voltage, (-1.618));

        xSemaphoreGive(i2cMutex);

        if (Distance > 1000 || Distance < 0) {
            Distance = 1000;
        }
        return Distance;
    }
    return -1;
}

/*!
 * Voltage monitor task
 */
void VoltageTask(void *pvParameters) {
    uint16_t adc;

    while (1) {
        if (xSemaphoreTake(i2cMutex, 100)) {
            adc = ads1015_read(1);
            xSemaphoreGive(i2cMutex);
            Voltage = (int) (((float) adc / 83) * 100);
        }

        //Adjust PID settings
        if (Voltage > 1190) {
            kp = 56.0;
            ki = 43.0;
            kd = 26.0;
            kpv = 54.0;
            kiv = 45.0;
            kdv = 1;
        } else {
            kp = 55.0;
            ki = 38.0;
            kd = 28.0;
            kpv = 58.0;
            kiv = 43.0;
            kdv = 1;
        }

        vTaskDelay(500);
    }
}

/*!
 * Set PID parameters via a uart interface. For the test purposes only
 */
void UARTControlTask(void *pvParameters) {
    while (1) {
        bool setPid = true;
        char key = debug_getchar();
        switch (key) {
        case 'q':
            kp += 1;
            break;
        case 'a':
            kp -= 1;
            break;
        case 'w':
            ki += 1;
            break;
        case 's':
            ki -= 1;
            break;
        case 'e':
            kd += 1;
            break;
        case 'd':
            kd -= 1;
            break;

        case 'r':
            kpv += 1;
            break;
        case 'f':
            kpv -= 1;
            break;
        case 't':
            kiv += 1;
            break;
        case 'g':
            kiv -= 1;
            break;
        case 'y':
            kdv += 1;
            break;
        case 'h':
            kdv -= 1;
            break;

        case 'i':
            driveOffset = 0;
            DesiredSetpointv = 7;
            setPid = false;
            SetTunings(&PID1, kp, ki / 10, kd / 100.0);
            SetTunings(&PID2, kpv / 100.0, 0, kdv / 10000.0);
            break;
        case 'k':
            driveOffset = 0;
            DesiredSetpointv = -7;
            setPid = false;
            SetTunings(&PID1, kp, ki / 10, kd / 100.0);
            SetTunings(&PID2, kpv / 100.0, 0, kdv / 10000.0);
            break;
        case 'j':
            driveOffset = -50;
            DesiredSetpointv = 0;
            setPid = false;
            break;
        case 'l':
            driveOffset = 50;
            DesiredSetpointv = 0;
            setPid = false;
            break;
        case 'z':
            driveOffset = 0;
            DesiredSetpointv = 0;
            setPid = false;
            SetTunings(&PID1, kp, ki, kd / 100.0);
            SetTunings(&PID2, kpv / 100.0, kiv / 100.0, kdv / 10000.0);
            break;
        case '7':
            handPos1 += 10;
            setServo(0, handPos1);
            break;
        case '1':
            handPos1 -= 10;
            setServo(0, handPos1);
            break;
        case '9':
            handPos2 += 10;
            setServo(1, handPos2);
            break;
        case '3':
            handPos2 -= 10;
            setServo(1, handPos2);
            break;
        }

        if (setPid) {
            SetTunings(&PID1, kp, ki, kd / 100.0);
            SetTunings(&PID2, kpv / 100.0, kiv / 100.0, kdv / 10000.0);
            PRINTF("%d %d %d; %d %d %d\r\n", (int32_t) kp, (int32_t) ki, (int32_t) kd, (int32_t) kpv, (int32_t) kiv, (int32_t) kdv);
        }

        vTaskDelay(100);
    }
}

/*!
 * Inter-core communication task
 */
void CommandTask(void *pvParameters) {
    char command[20];
    char result[50];
    int param1, param2;

    if (xSemaphoreTake(i2cMutex, 100)) {
        pca9685_init(50);
        xSemaphoreGive(i2cMutex);
    }

    MU_Init(BOARD_MU_BASE_ADDR);
    NVIC_SetPriority(BOARD_MU_IRQ_NUM, APP_MU_IRQ_PRIORITY);
    NVIC_EnableIRQ(BOARD_MU_IRQ_NUM);

    app_sema = xSemaphoreCreateCounting(STRING_BUFFER_CNT + 1, 0);

    rpmsg_init(0, &rdev, rpmsg_channel_created, rpmsg_channel_deleted, rpmsg_read_cb, RPMSG_MASTER);

    xSemaphoreTake(app_sema, portMAX_DELAY);

    while (1) {
        xSemaphoreTake(app_sema, portMAX_DELAY);

        if ((strlen(strVar[app_idx]) == 2) && (strVar[app_idx][0] == 0xd) && (strVar[app_idx][1] == 0xa)) {
            PRINTF("Get New Line From A7 From Slot %d\r\n", app_idx);
        } else {

            switch (strVar[app_idx][0]) {
            case '?':
                sscanf(strVar[app_idx], "?%s", command);

                if (0 == strcmp(command, "angle")) {
                    sprintf(result, "?angle:%d\n", (int) measuredAngle);
                    rpmsg_send(app_chnl, result, strlen(result));
                } else if (0 == strcmp(command, "speedLeft")) {
                    sprintf(result, "?speedLeft:%d\n", enc1Speed);
                    rpmsg_send(app_chnl, result, strlen(result));
                } else if (0 == strcmp(command, "speedRight")) {
                    sprintf(result, "?speedRight:%d\n", enc2Speed);
                    rpmsg_send(app_chnl, result, strlen(result));
                } else if (0 == strcmp(command, "distance")) {
                    sprintf(result, "?distance:%d\n", (int) getDistance());
                    rpmsg_send(app_chnl, result, strlen(result));
                } else if (0 == strcmp(command, "voltage")) {
                    sprintf(result, "?voltage:%d\n", (Voltage));
                    rpmsg_send(app_chnl, result, strlen(result));
                } else {
                    sprintf(result, "wrong command\n");
                    rpmsg_send(app_chnl, result, strlen(result));
                }

                break;
            case '!':
                sscanf(strVar[app_idx], "!%[^:\n]:%d:%d", command, &param1, &param2);

                int8_t commandValid = false;

                if (0 == strcmp(command, "move")) {
                    if (param1 <= 20 && param1 >= -20) {
                        DesiredSetpointv = param1;
                        commandValid = true;
                        SetTunings(&PID1, kp, ki / 15, kd / 100.0);
                        SetTunings(&PID2, kpv / 100.0, 0, kdv / 10000.0);
                    }
                } else if (0 == strcmp(command, "turnLeft")) {
                    driveOffset = -abs(param1);
                    commandValid = true;
                } else if (0 == strcmp(command, "turnRight")) {
                    driveOffset = abs(param1);
                    commandValid = true;
                } else if (0 == strcmp(command, "stop")) {
                    DesiredSetpointv = 0;
                    driveOffset = 0;
                    commandValid = true;
                    SetTunings(&PID1, kp, ki, kd / 100.0);
                    SetTunings(&PID2, kpv / 100.0, kiv / 100.0, kdv / 10000.0);
                } else if (0 == strcmp(command, "servo")) {
                    if (param1 >= 0 && param1 <= 3 && param2 >= 0 && param2 <= 180) {
                        setServo(param1, param2);
                        commandValid = true;
                    }
                }

                if (commandValid) {
                    sprintf(result, "%s:ok\n", command);
                    rpmsg_send(app_chnl, result, strlen(result));
                } else {
                    sprintf(result, "%s:error\n", command);
                    rpmsg_send(app_chnl, result, strlen(result));
                }

                break;
            default:
                sprintf(result, "wrong command\n");
                rpmsg_send(app_chnl, result, strlen(result));
                break;
            }
        }

        app_idx = (app_idx + 1) % STRING_BUFFER_CNT;
        MU_EnableRxFullInt(MU0_B, MU_RPMSG_CHANNEL);
    }
}

/*!
 * PID control task
 */
const double RAD_TO_DEG = 180 / 3.1415;
void ControlTask(void *pvParameters) {
    accel_t acc;
    gyro_t gyro;

    PID_Init(&PID1, &Input, &Output, &Setpoint, kp, ki, kd / 100.0, REVERSE);
    PID_Init(&PID2, &Inputv, &Outputv, &Setpointv, kpv / 100.0, kiv / 100.0, kdv / 10000.0, DIRECT);

    if (xSemaphoreTake(i2cMutex, 100)) {
        mpu6000_init();
        xSemaphoreGive(i2cMutex);
    }

    Motors_Init();
    Motors_SetSpeed(0, 0);

    previousTime = xTaskGetTickCount();

    if (xSemaphoreTake(i2cMutex, 100)) {
        mpu6000_read_accel(&acc);
        mpu6000_read_gyro(&gyro);
        xSemaphoreGive(i2cMutex);
    }

    double mAngle = atan2(acc.y, acc.z) * RAD_TO_DEG;
    double mAccel = gyro.x / 131.0; //deg/s

    Kalman_SetInitialValues(mAngle, -mAccel);

    //PID1 settings
    SetOutputLimits(&PID1, -650, 650);
    SetSampleTime(&PID1, 1);
    SetMode(&PID1, AUTOMATIC);
    Input = 0;
    Setpoint = 0;

    //PID2 settings
    SetOutputLimits(&PID2, -15, 15);
    SetSampleTime(&PID2, 1);
    SetMode(&PID2, AUTOMATIC);
    Inputv = 0;
    Setpointv = 0;
    DesiredSetpointv = 0;

    vTaskDelay(100);

    while (1) {

        //Read MPU6000
        if (xSemaphoreTake(i2cMutex, 10)) {
            mpu6000_read_accel(&acc);
            mpu6000_read_gyro(&gyro);
            xSemaphoreGive(i2cMutex);
        }

        double accKat = atan2(acc.y, acc.z) * RAD_TO_DEG;
        double gyroV = gyro.x / 131.0; //deg/s

        //Kalman filter
        TickType_t currentTime = xTaskGetTickCount();
        double dt = (double) (currentTime - previousTime) / 1000.0;
        measuredAngle = Kalman_Filter(accKat, gyroV, dt);
        previousTime = currentTime;

        //Compute speed
        enc1Speed = enc1Position - enc1PreviousPosition;
        enc2Speed = enc2Position - enc2PreviousPosition;
        enc1PreviousPosition = enc1Position;
        enc2PreviousPosition = enc2Position;

        //Set inputs
        Input = measuredAngle;
        Inputv = (enc1Speed + enc2Speed);

        //Compute PIDs
        if (abs(measuredAngle) < 25.0) {
            Compute(&PID2);
            const int balanceOffset = -4; // balance offset - fast regulation at the beginning
            Setpoint = Outputv + balanceOffset;

            Compute(&PID1);
            Motors_SetSpeed(Output + driveOffset, Output - driveOffset);
        } else {
            Motors_SetSpeed(0, 0);
            Setpointv = 0;
            DesiredSetpointv = 0;
            driveOffset = 0;
        }

        //Smooth setpoint change
        if (abs(Setpointv - DesiredSetpointv) >= 0.1) {
            if (Setpointv > DesiredSetpointv) {
                Setpointv -= 0.05;
            } else if (Setpointv < DesiredSetpointv) {
                Setpointv += 0.05;
            }
        } else {
            Setpointv = DesiredSetpointv;
        }

        vTaskDelay(2);
    }
}

/*!
 * @brief Main function
 */
int main(void) {
    //Initialize demo application pins setting and clock setting.
    hardware_init();

    //I2C communication
    i2c_init_config_t i2cInitConfig = { .clockRate = get_i2c_clock_freq(BOARD_I2C_BASEADDR), .baudRate = 400000u, .slaveAddress = 0x00 };

    /* Initialize I2C module with I2C init structure. */
    I2C_XFER_Config(&i2cInitConfig);

    i2cMutex = xSemaphoreCreateMutex();

    xTaskCreate(VoltageTask, "Test task", 500, NULL, tskIDLE_PRIORITY+2, NULL);
    xTaskCreate(UARTControlTask, "PID task", 500, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(ControlTask, "Control task", 600, NULL, tskIDLE_PRIORITY+3, NULL);
    xTaskCreate(CommandTask, "Command Task", 500, NULL, tskIDLE_PRIORITY+4, NULL);

    PRINTF("TAQ firmware started\r\n");

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    // Should never reach this point.
    while (true)
        ;
}

/*!
 * IRQ handlers - measure speed and encoders position.
 */

void BOARD_ENC_HANDLER(void) {

    if (GPIO_IsIntPending(gpioEnc1A.base, gpioEnc1A.pin)) {
        uint8_t stateA = GPIO_ReadPinInput(gpioEnc1A.base, gpioEnc1A.pin);
        uint8_t stateB = GPIO_ReadPinInput(gpioEnc1B.base, gpioEnc1B.pin);

        if (stateA == stateB) {
            enc1Position += 1;
        } else {
            enc1Position -= 1;
        }

        GPIO_ClearStatusFlag(gpioEnc1A.base, gpioEnc1A.pin);
    }

    if (GPIO_IsIntPending(gpioEnc2A.base, gpioEnc2A.pin)) {
        uint8_t stateA = GPIO_ReadPinInput(gpioEnc2A.base, gpioEnc2A.pin);
        uint8_t stateB = GPIO_ReadPinInput(gpioEnc2B.base, gpioEnc2B.pin);

        if (stateA == stateB) {
            enc2Position += 1;
        } else {
            enc2Position -= 1;
        }

        GPIO_ClearStatusFlag(gpioEnc2A.base, gpioEnc2A.pin);
    }
}

void BOARD_MU_HANDLER(void) {
    //calls into rpmsg_handler provided by middleware
    rpmsg_handler();
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
