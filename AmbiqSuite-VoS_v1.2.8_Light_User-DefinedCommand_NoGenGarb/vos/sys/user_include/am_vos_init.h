//*****************************************************************************
//
//! @file am_vos_init.h
//!
//! @brief the header file of KWD system initialization functions
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

#ifndef AM_VOS_INIT
#define AM_VOS_INIT

#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include <stdbool.h>
#include "am_app_utils_ring_buffer.h"
#include "am_vos_spp.h"
#include "am_vos_logic.h"

#if configUSE_DSPC_QSD
#include "dspc_qsd.h"
#include "am_vos_spp.h"
#endif // configUSE_DSPC_QSD

#if configUSE_AUDIO_CODEC && (configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ)
#include "sbc.h"
#endif // configUSE_AUDIO_CODEC && (configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ)

#if USE_MAYA
#define HEART_BEAT_PERIOD               pdMS_TO_TICKS(3000)
#else // USE_MAYA
#define HEART_BEAT_PERIOD               pdMS_TO_TICKS(1000)
#endif // USE_MAYA

#define RTT_RECORDER_RUNNING            pdMS_TO_TICKS(300)

#define GSENSOR_PERIOD                  pdMS_TO_TICKS(2000) 

#define LONG_PRESS_TIMER_PERIOD_MS     100
#define LONG_PRESS_EFFECTIVE_MS        4000

//*****************************************************************************
//
// App ring buffer list structure typedefs
//
//*****************************************************************************
// this enum list contains all app related ring buffers between AM_APP_RINGBUFF_NONE and AM_APP_RINGBUFF_MAX
// where each index corresponds to a ring buffer
typedef enum
{
    AM_APP_RINGBUFF_NONE = 0, // The enum must begin with this value as named.
#if configUSE_RTT_RECORDER
    AM_APP_RINGBUFF_RTT_STREAM,
#endif // configUSE_RTT_RECORDER
    AM_APP_RINGBUFF_MAX // The enum must end with this value as named.
} am_app_utils_ring_buffer_enum_t;

#if configUSE_RTT_RECORDER
    #define RTT_BUFFER_LENGTH               (128*1024)
#endif // configUSE_RTT_RECORDER

//*****************************************************************************
//
// Voice on SPOT global status structure
//
//*****************************************************************************
typedef struct _VosSysInfo {
    uint8_t     ui8KwdDetectedFlag;             // 1: the key word is detected; 0: no key word detected.
    uint8_t     ui8ButtonDownFlag;              // 1: the button is pushed down; 0: no button is pushed.
    uint8_t     ui8PushTalkFlag;                // trig for push-to-talk.
    uint8_t     ui8ProvideSpeechFlag;           // trig for conversation talk.

    uint32_t    ui32StreamingTimeOutCount;      // Voice streaming timeout counter
    bool        bWwdEnabled;

#if defined (AM_PART_APOLLO2)
#if ConfigUSE_LOG_UART0 || configUSE_PRINTF_UART0
    uint8_t     pui8UartRxBuf0[UART0_BUFFER_SIZE];
    uint8_t     pui8UartTxBuf0[UART0_BUFFER_SIZE];
#endif // ConfigUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_LOG_UART1
    uint8_t     pui8UartRxBuf1[UART1_BUFFER_SIZE];
    uint8_t     pui8UartTxBuf1[UART1_BUFFER_SIZE];
#endif // configUSE_LOG_UART1
    bool        bUARTPacketReceived;
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    uint32_t    ui32LastError;
    void        *pvUartHandle;
    uint8_t     pui8UartTxBuf[256];
    uint8_t     pui8UartRxBuf[2];

#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
    void        *pvPDMHandle;
    uint32_t    ui32PDMDataBuf[PCM_FRAME_SIZE * SAMPLE_32BIT];
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    void        *pvADCHandle;                                           // ADC Device Handle.
    uint32_t    ui32ADCSampleBuf[ADC_SAMPLE_COUNT * ADC_MIC_CHANNELS];  // ADC DMA target address
    bool        bADCDMAError;                                           // ADC DMA error flag.
    
    high_pass_filterType        sAmicLeftHPFilter;
    high_pass_filterType        sAmicRightHPFilter;

    float       pfAmicLeftArray[ADC_SAMPLE_COUNT];
    float       pfAmicRightArray[ADC_SAMPLE_COUNT];
    float       pfAmicLeftFilteredBuff[ADC_SAMPLE_COUNT];
    float       pfAmicRightFilteredBuff[ADC_SAMPLE_COUNT];
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB

    am_hal_burst_avail_e        eBurstModeAvailable;
    am_hal_burst_mode_e         eBurstMode;

#if USE_MAYA
    enum_logic_power_states_t   eLogicPowerState;
#endif // USE_MAYA

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    void        *pvGSensorHdl;
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

#if configUSE_AUDIO_CODEC && (configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ)
    sbc_t       sBluezSBCInstance;
#endif // configUSE_AUDIO_CODEC && (configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ)

#if configUSE_RTT_RECORDER
    uint8_t     pui8RttRecorderBuffer[RTT_BUFFER_LENGTH];
#endif // configUSE_RTT_RECORDER

#if configUSE_AMVOS_AMA
    int32_t     i32DelayedSample;
    int32_t     i32EndingSampleCnt;
    int32_t     i32PreRollDelayMs;
#else // configUSE_AMVOS_AMA
    uint8_t     ui8SeqNumber;
#endif // configUSE_AMVOS_AMA

#if configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER
    uint8_t     ui8RecordStartFlag;              // 1: the start of RTT recorder; 0: stop recording
#endif // configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER

#if configUSE_WOS
    uint8_t     ui8WosStatus;                   // 0: Noisy status; 1: Quiet status
    uint8_t     ui8WosActiveFlag;               // Start pdm > WOS_TIMEOUT_COUNT * 5 ms when power-up
    int32_t     i32WosActiveCount;
    uint8_t     ui8WoSDetectFlag;
#endif // configUSE_WOS
    
#if configUSE_DSPC_QSD
    DSPC_QSD            sQsdHandle;
    eVosQSDStatus_t     eQsdStatus;
    uint16_t            ui16QsdTimeoutCnt;
    uint16_t            ui16QsdOffDelayCnt;     // "QSD off" means QSD detected quiet environment 
                                                // and turns AWE processing off
#endif // configUSE_DSPC_QSD

    uint32_t    ui32SysHeapSize;
} VosSysInfo;

extern VosSysInfo g_sVosSys; 

extern am_app_utils_ring_buffer_t am_KWD_ring_buffers[];

extern void am_vos_sys_init(void);
extern void am_vos_reset_detected_flag(void);
extern void am_vos_board_init(void);

#endif // AM_VOS_INIT
