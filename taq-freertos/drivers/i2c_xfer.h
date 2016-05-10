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
#ifndef __I2C_XFER_H__
#define __I2C_XFER_H__

#include <stdint.h>
#include <stdbool.h>
#include "i2c_imx.h"

/* Function prototypes */
#if defined(__cplusplus)
extern "C" {
#endif

void I2C_XFER_Config(i2c_init_config_t* initConfig);
bool I2C_XFER_SendDataBlocking(uint8_t devAddr, uint8_t regAddr, const uint8_t* txBuffer, uint32_t txSize);
uint32_t I2C_XFER_GetSendStatus(void);
bool I2C_XFER_ReceiveDataBlocking(uint8_t devAddr, uint8_t regAddr, uint8_t* rxBuffer, uint32_t rxSize);
uint32_t I2C_XFER_GetReceiveStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_XFER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
