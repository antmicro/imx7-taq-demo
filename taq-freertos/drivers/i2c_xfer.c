/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "device_imx.h"
#include "i2c_imx.h"
#include "board.h"

typedef struct _i2c_state {
    const uint8_t*    cmdBuff;         /*!< The buffer of I2C command. */
    const uint8_t*    txBuff;          /*!< The buffer of data being sent.*/
    uint8_t*          rxBuff;          /*!< The buffer of received data. */
    uint32_t          cmdSize;         /*!< The remaining number of commands to be transmitted. */
    uint32_t          txSize;          /*!< The remaining number of bytes to be transmitted. */
    uint32_t          rxSize;          /*!< The remaining number of bytes to be received. */
    bool              isBusy;          /*!< True if there is an active transmission. */
    uint32_t          operateDir;      /*!< Overall I2C bus operating direction. */
    uint32_t          currentDir;      /*!< Current Data transfer direction. */
    uint32_t          currentMode;     /*!< Current I2C Bus role of this module. */
    SemaphoreHandle_t xSemaphore;      /*!< I2C internal synchronize semaphore. */
} i2c_state_t;

/* I2C runtime state structure */
static volatile i2c_state_t i2cState;

void I2C_XFER_Config(i2c_init_config_t* initConfig)
{
    /* Initialize I2C state structure content. */
    i2cState.cmdBuff = 0;
    i2cState.txBuff = 0;
    i2cState.rxBuff = 0;
    i2cState.cmdSize = 0;
    i2cState.txSize = 0;
    i2cState.rxSize = 0;
    i2cState.isBusy = false;
    i2cState.operateDir = i2cDirectionReceive;
    i2cState.currentDir = i2cDirectionReceive;
    i2cState.currentMode = i2cModeSlave;
    i2cState.xSemaphore = xSemaphoreCreateBinary();

    /* Initialize I2C baud rate, mode, transfer direction and slave address. */
    I2C_Init(BOARD_I2C_BASEADDR, initConfig);

    /* Set I2C Interrupt priority */
    NVIC_SetPriority(BOARD_I2C_IRQ_NUM, 3);

    /* Call core API to enable the IRQ. */
    NVIC_EnableIRQ(BOARD_I2C_IRQ_NUM);

    /* Finally, enable the I2C module */
    I2C_Enable(BOARD_I2C_BASEADDR);
}

bool I2C_XFER_SendDataBlocking(uint8_t devAddr, uint8_t regAddr, const uint8_t* txBuffer, uint32_t txSize)
{
    if ((i2cState.isBusy) || (0 == txSize))
        return false;

    uint8_t cmdBuff[2];
    cmdBuff[0] = (devAddr << 1);
    cmdBuff[1] = regAddr;

    /* Initialize i2c transfer struct */
    i2cState.cmdBuff = cmdBuff;
    i2cState.cmdSize = 2;
    i2cState.txBuff = txBuffer;
    i2cState.txSize = txSize;
    i2cState.isBusy = true;
    i2cState.operateDir = i2cDirectionTransmit;

    /* Clear I2C interrupt flag to avoid spurious interrupt */
    I2C_ClearStatusFlag(BOARD_I2C_BASEADDR, i2cStatusInterrupt);

    if (I2C_GetStatusFlag(BOARD_I2C_BASEADDR, i2cStatusBusBusy))
    {
        /* Reset i2c transfer state. */
        i2cState.operateDir = i2cDirectionReceive;
        i2cState.isBusy = false;
        return false;
    }

    /* Set I2C work under Tx mode */
    I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionTransmit);
    i2cState.currentDir = i2cDirectionTransmit;

    /* Switch to Master Mode and Send Start Signal. */
    I2C_SetWorkMode(BOARD_I2C_BASEADDR, i2cModeMaster);
    i2cState.currentMode = i2cModeMaster;

    if (0 != i2cState.cmdSize)
    {
        I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.cmdBuff);
        i2cState.cmdBuff++;
        i2cState.cmdSize--;
    }
    else
    {
        I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.txBuff);
        i2cState.txBuff++;
        i2cState.txSize--;
    }

    /* Enable I2C interrupt, subsequent data transfer will be handled in ISR. */
    I2C_SetIntCmd(BOARD_I2C_BASEADDR, true);

    /* Wait until send data finish. */
    xSemaphoreTake(i2cState.xSemaphore, portMAX_DELAY);

    return true;
}

uint32_t I2C_XFER_GetSendStatus(void)
{
    return i2cState.txSize;
}

bool I2C_XFER_ReceiveDataBlocking(uint8_t devAddr, uint8_t regAddr, uint8_t* rxBuffer, uint32_t rxSize)
{
    if ((i2cState.isBusy) || (0 == rxSize))
        return false;

    uint8_t cmdBuff[3];
    cmdBuff[0] = (devAddr << 1);
    cmdBuff[1] = regAddr;
    cmdBuff[2] = (devAddr << 1) + 1;

    /* Initialize i2c transfer struct */
    i2cState.cmdBuff = cmdBuff;
    i2cState.cmdSize = 3;
    i2cState.rxBuff = rxBuffer;
    i2cState.rxSize = rxSize;
    i2cState.isBusy = true;
    i2cState.operateDir = i2cDirectionReceive;

    /* Clear I2C interrupt flag to avoid spurious interrupt */
    I2C_ClearStatusFlag(BOARD_I2C_BASEADDR, i2cStatusInterrupt);

    if (I2C_GetStatusFlag(BOARD_I2C_BASEADDR, i2cStatusBusBusy))
    {
        /* Reset i2c transfer state. */
        i2cState.operateDir = i2cDirectionReceive;
        i2cState.isBusy = false;
        return false;
    }

    /* Set I2C work under Tx mode */
    I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionTransmit);
    i2cState.currentDir = i2cDirectionTransmit;

    /* Switch to Master Mode and Send Start Signal. */
    I2C_SetWorkMode(BOARD_I2C_BASEADDR, i2cModeMaster);
    i2cState.currentMode = i2cModeMaster;

    /* Is there command to be sent before receive data? */
    if (0 != i2cState.cmdSize)
    {
        if (1 == i2cState.cmdSize)
            I2C_SendRepeatStart(BOARD_I2C_BASEADDR);
        I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.cmdBuff);
        i2cState.cmdBuff++;
        i2cState.cmdSize--;
    }
    else
    {
        /* Change to receive state. */
        I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionReceive);
        i2cState.currentDir = i2cDirectionReceive;

        if (1 == rxSize)
            /* Send Nack */
            I2C_SetAckBit(BOARD_I2C_BASEADDR, false);
        else
            /* Send Ack */
            I2C_SetAckBit(BOARD_I2C_BASEADDR, true);
        /* dummy read to clock in 1st byte */
        I2C_ReadByte(BOARD_I2C_BASEADDR);
    }

    /* Enable I2C interrupt, subsequent data transfer will be handled in ISR. */
    I2C_SetIntCmd(BOARD_I2C_BASEADDR, true);

    /* Wait until receive data finish. */
    xSemaphoreTake(i2cState.xSemaphore, portMAX_DELAY);

    return true;
}

uint32_t I2C_XFER_GetReceiveStatus(void)
{
    return i2cState.rxSize;
}

void BOARD_I2C_HANDLER(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Clear interrupt flag. */
    I2C_ClearStatusFlag(BOARD_I2C_BASEADDR, i2cStatusInterrupt);

    /* Exit the ISR if no transfer is happening for this instance. */
    if (!i2cState.isBusy)
        return;

    if (i2cModeMaster == i2cState.currentMode)
    {
        if (i2cDirectionTransmit == i2cState.currentDir)
        {
            if ((I2C_GetStatusFlag(BOARD_I2C_BASEADDR, i2cStatusReceivedAck)) ||
                ((0 == i2cState.txSize) && (0 == i2cState.cmdSize)))
            {
                if ((i2cDirectionTransmit == i2cState.operateDir) ||
                    (I2C_GetStatusFlag(BOARD_I2C_BASEADDR, i2cStatusReceivedAck)))
                {
                    /* Switch to Slave mode and Generate a Stop Signal. */
                    I2C_SetWorkMode(BOARD_I2C_BASEADDR, i2cModeSlave);
                    i2cState.currentMode = i2cModeSlave;

                    /* Switch back to Rx direction. */
                    I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionReceive);
                    i2cState.currentDir = i2cDirectionReceive;

                    /* Close I2C interrupt. */
                    I2C_SetIntCmd(BOARD_I2C_BASEADDR, false);
                    /* Release I2C Bus. */
                    i2cState.isBusy = false;
                    xSemaphoreGiveFromISR(i2cState.xSemaphore, &xHigherPriorityTaskWoken);
                }
                else
                {
                    /* Switch back to Rx direction. */
                    I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionReceive);
                    i2cState.currentDir = i2cDirectionReceive;

                    if (1 == i2cState.rxSize)
                        /* Send Nack */
                        I2C_SetAckBit(BOARD_I2C_BASEADDR, false);
                    else
                        /* Send Ack */
                        I2C_SetAckBit(BOARD_I2C_BASEADDR, true);
                    /* dummy read to clock in 1st byte */
                    *i2cState.rxBuff = I2C_ReadByte(BOARD_I2C_BASEADDR);
                }
            }
            else
            {
                if (0 != i2cState.cmdSize)
                {
                    if ((1 == i2cState.cmdSize) && (i2cDirectionReceive == i2cState.operateDir))
                        I2C_SendRepeatStart(BOARD_I2C_BASEADDR);
                    I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.cmdBuff);
                    i2cState.cmdBuff++;
                    i2cState.cmdSize--;
                }
                else
                {
                    I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.txBuff);
                    i2cState.txBuff++;
                    i2cState.txSize--;
                }
            }
        }
        else
        {
            /* Normal read operation. */
            if (2 == i2cState.rxSize)
                /* Send Nack */
                I2C_SetAckBit(BOARD_I2C_BASEADDR, false);
            else
                /* Send Nack */
                I2C_SetAckBit(BOARD_I2C_BASEADDR, true);

            if (1 == i2cState.rxSize)
            {
                /* Switch back to Tx direction to avoid additional I2C bus read. */
                I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionTransmit);
                i2cState.currentDir = i2cDirectionTransmit;
            }
            *i2cState.rxBuff = I2C_ReadByte(BOARD_I2C_BASEADDR);
            i2cState.rxBuff++;
            i2cState.rxSize--;

            /* receive finished. */
            if (0 == i2cState.rxSize)
            {
                /* Switch to Slave mode and Generate a Stop Signal. */
                I2C_SetWorkMode(BOARD_I2C_BASEADDR, i2cModeSlave);
                i2cState.currentMode = i2cModeSlave;

                /* Switch back to Rx direction. */
                I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionReceive);
                i2cState.currentDir = i2cDirectionReceive;

                /* Close I2C interrupt. */
                I2C_SetIntCmd(BOARD_I2C_BASEADDR, false);
                /* Release I2C Bus. */
                i2cState.isBusy = false;
                /* Release I2C Sem4 */
                xSemaphoreGiveFromISR(i2cState.xSemaphore, &xHigherPriorityTaskWoken);
            }
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
