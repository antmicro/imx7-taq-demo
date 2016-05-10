/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2015-2016 Antmicro Ltd.
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

#include <assert.h>
#include "gpio_pins.h"
#include "MCIMX7D_M4.h"

gpio_config_t gpioEnc1A = {
    "ENC1A",                                          /* name */
    &IOMUXC_SW_MUX_CTL_PAD_SAI2_TX_SYNC,              /* muxReg */
    5,                                                /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_SYNC,              /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_SYNC_PS(2) |        /* padConfig */
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_SYNC_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_SYNC_HYS_MASK,
    GPIO6,                                            /* base */
    19                                                /* pin */
};
gpio_init_t Enc1A = {
    .pin           = 19, //pin number
    .direction     = gpioDigitalInput,
    .interruptMode = gpioIntRisingEdge
};


gpio_config_t gpioEnc1B = {
    "ENC1B",                                         /* name */
    &IOMUXC_SW_MUX_CTL_PAD_SAI2_TX_BCLK,             /* muxReg */
    5,                                               /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_BCLK,             /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_BCLK_PS(2) |       /* padConfig */
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_BCLK_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_BCLK_HYS_MASK,
    GPIO6,                                           /* base */
    20                                               /* pin */
};
gpio_init_t Enc1B = {
    .pin           = 20, //pin number
    .direction     = gpioDigitalInput,
    .interruptMode = gpioNoIntmode
};


gpio_config_t gpioEnc2A = {
    "ENC2A",                                         /* name */
    &IOMUXC_SW_MUX_CTL_PAD_SAI2_RX_DATA,             /* muxReg */
    5,                                               /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_SAI2_RX_DATA,             /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_SAI2_RX_DATA_PS(2) |       /* padConfig */
    IOMUXC_SW_PAD_CTL_PAD_SAI2_RX_DATA_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_SAI2_RX_DATA_HYS_MASK,
    GPIO6,                                           /* base */
    21                                               /* pin */
};
gpio_init_t Enc2A = {
    .pin           = 21, //pin number
    .direction     = gpioDigitalInput,
    .interruptMode = gpioIntRisingEdge
};


gpio_config_t gpioEnc2B = {
    "ENC2B",                                        /* name */
    &IOMUXC_SW_MUX_CTL_PAD_SAI2_TX_DATA,            /* muxReg */
    5,                                              /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_DATA,            /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_DATA_PS(2) |      /* padConfig */
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_DATA_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_SAI2_TX_DATA_HYS_MASK,
    GPIO6,                                          /* base */
    22                                              /* pin */
};
gpio_init_t Enc2B = {
    .pin           = 22, //pin number
    .direction     = gpioDigitalInput,
    .interruptMode = gpioNoIntmode
};


gpio_config_t gpioMoto1A = {
    "MOT1A",                                         /* name */
    &IOMUXC_LPSR_SW_MUX_CTL_PAD_GPIO1_IO02,          /* muxReg */
    0,                                               /* muxConfig */
    &IOMUXC_LPSR_SW_PAD_CTL_PAD_GPIO1_IO02,          /* padReg */
    IOMUXC_LPSR_SW_PAD_CTL_PAD_GPIO1_IO02_PS(3) |    /* padConfig */
    IOMUXC_LPSR_SW_PAD_CTL_PAD_GPIO1_IO02_PE_MASK |
    IOMUXC_LPSR_SW_PAD_CTL_PAD_GPIO1_IO02_HYS_MASK,
    GPIO1,                                           /* base */
    2                                                /* pin */
};
gpio_init_t Moto1A = {
    .pin           = 2, //pin number
    .direction     = gpioDigitalOutput,
    .interruptMode = gpioIntRisingEdge
};

gpio_config_t gpioMoto1B = {
    "MOT1B",                                         /* name */
    &IOMUXC_SW_MUX_CTL_PAD_SD2_RESET_B,              /* muxReg */
    5,                                               /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_SD2_RESET_B,              /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_SD2_RESET_B_PS(3)   |      /* padConfig */
    IOMUXC_SW_PAD_CTL_PAD_SD2_RESET_B_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_SD2_RESET_B_HYS_MASK,
    GPIO5,                                           /* base */
    11                                                /* pin */
};
gpio_init_t Moto1B = {
    .pin           = 11, //pin number
    .direction     = gpioDigitalOutput,
    .interruptMode = gpioIntRisingEdge
};

gpio_config_t gpioMoto2A = {
    "MOT2A",                                         /* name */
    &IOMUXC_SW_MUX_CTL_PAD_EPDC_GDRL,                /* muxReg */
    5,                                               /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL,                /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_PS(3)   |        /* padConfig */
    IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_EPDC_GDRL_HYS_MASK,
    GPIO2,                                           /* base */
    26                                               /* pin */
};
gpio_init_t Moto2A = {
    .pin           = 26, //pin number
    .direction     = gpioDigitalOutput,
    .interruptMode = gpioIntRisingEdge
};

gpio_config_t gpioMoto2B = {
    "MOT2B",                                         /* name */
    &IOMUXC_SW_MUX_CTL_PAD_ECSPI1_MOSI,              /* muxReg */
    5,                                               /* muxConfig */
    &IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI,              /* padReg */
    IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI_PS(3)   |      /* padConfig */
    IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI_PE_MASK |
    IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI_HYS_MASK,
    GPIO4,                                           /* base */
    17                                               /* pin */
};
gpio_init_t Moto2B = {
    .pin           = 17, //pin number
    .direction     = gpioDigitalOutput,
    .interruptMode = gpioIntRisingEdge
};

void configure_platform_gpio(void) {

    /* Encoder 1 */
    GPIO_Init(gpioEnc1A.base, &Enc1A);
    GPIO_Init(gpioEnc1B.base, &Enc1B);

    /* Encoder 2 */
    GPIO_Init(gpioEnc2A.base, &Enc2A);
    GPIO_Init(gpioEnc2B.base, &Enc2B);

    /* Motor Controller 1 */
    GPIO_Init(gpioMoto1A.base, &Moto1A);
    GPIO_Init(gpioMoto1B.base, &Moto1B);

    /* Motor Controller 2 */
    GPIO_Init(gpioMoto2A.base, &Moto2A);
    GPIO_Init(gpioMoto2B.base, &Moto2B);

    /* Clear the interrupt state, this operation is necessary, because the GPIO module maybe confuse
       the first rising edge as interrupt*/
    GPIO_ClearStatusFlag(gpioEnc1A.base, gpioEnc1A.pin);
    GPIO_ClearStatusFlag(gpioEnc2A.base, gpioEnc2A.pin);

    IOMUXC_SAI2_TX_SYNC_SELECT_INPUT = 1;
    IOMUXC_SAI2_TX_BCLK_SELECT_INPUT = 1;

    /* Enable GPIO pin interrupt */
    GPIO_SetPinIntMode(gpioEnc1A.base, gpioEnc1A.pin, true);
    GPIO_SetIntEdgeSelect(gpioEnc1A.base, gpioEnc1A.pin, true); //Active on any edge

    GPIO_SetPinIntMode(gpioEnc2A.base, gpioEnc2A.pin, true);
    GPIO_SetIntEdgeSelect(gpioEnc2A.base, gpioEnc2A.pin, true);

}

/*******************************************************************************
 * EOF
 ******************************************************************************/
