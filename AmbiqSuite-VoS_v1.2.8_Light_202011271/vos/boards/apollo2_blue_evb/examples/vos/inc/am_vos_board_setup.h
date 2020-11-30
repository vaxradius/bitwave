//*****************************************************************************
//
//! @file am_vos_board_setup.h
//!
//! @brief the header file of KWD board setup functions
//!
//*****************************************************************************

//*****************************************************************************
// Copyright (c) 2017, Ambiq Micro
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision v1.2.11 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_VOS_BOARD_SETUP
#define AM_VOS_BOARD_SETUP

#include "am_bsp.h"

//********************************************************************************
// PCM parameters
//********************************************************************************
#define PCM_FRAME_SIZE                  AM_SPP_FRAME_SAMPLES
#define PCM_FRAME_PUSH_OVER             3

//********************************************************************************
// UART & System Log parameters
//********************************************************************************
#define UART0_BUFFER_SIZE               (512 * 2)       // gloabal UART queue size (used by transmit buffered service)
#define UART1_BUFFER_SIZE               (512 * 2)
#define UART_TRANSMIT_BUFFER            256             // size limit to frame added into global UART queue

///********************************************************************************
// Hardware Related Macro
//*********************************************************************************
#if USE_MIKRO_DEV_SHIELD_REV1
    #define LED_SYSTEM                  0

    #define RTT_DUMP_BUTTON             AM_BSP_GPIO_BUTTON0
    #define PUSH_TO_TALK_BUTTON         AM_BSP_GPIO_BUTTON1
    #define MUTE_MIC_BUTTON             AM_BSP_GPIO_BUTTON2

    #define UART0_MODULE                0
    #define UART1_MODULE                1

#if USE_DMIC_MB3 || USE_DMIC_MB3_VM3011
    #define PDM_CLK                     12
    #define PDM_CLK_PIN_CFG             AM_HAL_PIN_12_PDM_CLK
    #define PDM_DATA                    11
    #define PDM_DATA_PIN_CFG            AM_HAL_PIN_11_PDM_DATA
#endif // USE_DMIC_MB3 || USE_DMIC_MB3_VM3011

#if USE_WOS_AMIC_MB1
    #define WOS_MODE_PIN                44
    #define WOS_WAKE_PIN                39
    #define WOS_WAKE_PIN_MODE_SEL       AM_HAL_PIN_39_GPIO
#endif // USE_WOS_AMIC_MB1

#if USE_WOS_AMIC_MB3
    #define WOS_MODE_PIN                4
    #define WOS_WAKE_PIN                19
    #define WOS_WAKE_PIN_MODE_SEL       AM_HAL_PIN_19_GPIO
#endif // USE_WOS_AMIC_MB3

#if USE_DMIC_MB3_VM3011
    #define WOS_WAKE_PIN                19
    #define WOS_WAKE_PIN_MODE_SEL       AM_HAL_PIN_19_GPIO
#endif // USE_DMIC_MB3_VM3011

#if configUSE_GSENSOR || configUSE_OVVP_DOUBLE_TAP
    #define LIS2DW_INT_PIN              13
    #define LIS2DW_INT_PIN_MODE_SEL     AM_HAL_PIN_13_GPIO
#endif // ConfigUSE_GSENSOR || configUSE_OVVP_DOUBLE_TAP

#endif // USE_MIKRO_DEV_SHIELD_REV1

/* Function declaration */
void am_vos_board_init(void);
void am_vos_burst_mode_enable(void);
void am_vos_burst_mode_disable(void);
void am_vos_mic_fifo_flush(void);
void am_vos_mic_enable(void);
void am_vos_mic_disable(void);
void am_vos_wos_mic_enable(void);
void am_vos_wos_mic_disable(void);
void am_vos_gpio_enable_irq(unsigned char gpio);
void am_vos_gpio_disable_irq(unsigned char gpio);

void PDMInit(void);
void PDMDeInit(void);

#endif
