//*****************************************************************************
//
//! @file am_vos_audio.h
//!
//! @brief header file of audio processing work
//
//*****************************************************************************

//*****************************************************************************
//
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

#ifndef AM_VOS_AUDIO_H
#define AM_VOS_AUDIO_H

#include "am_vos_sys_config.h"
#include "am_vos_spp.h"


//********************************************************************************
// Audio pre & post buffer parameters
//********************************************************************************
#define AUDIO_KWD_TIMEOUT_S             8                            // 8 seconds timeout.

#define AUDIO_PREROLL_TIME_MS           500 

//
// QSD and WOS parameters
//
#if configUSE_DSPC_QSD
  #define QSD_TIMEOUT_MS                3000

#if configUSE_AIC_LITE_LAYOUT
  #define QSD_TIMEOUT_COUNT             (QSD_TIMEOUT_MS / 10)   // 1 count is ~10ms, 1 sec = 100 x 10 ms

#else // configUSE_AIC_LITE_LAYOUT
  #define QSD_TIMEOUT_COUNT             (QSD_TIMEOUT_MS / 5)    // 1 count is ~5ms, 1 sec = 200 x 5 ms
#endif // configUSE_AIC_LITE_LAYOUT

  #define QSD_TIMEOUT_ADJ_THRESHOLD     20                      // 20 seconds = QSD_TIMEOUT_MS * QSD_TIMEOUT_ADJ_THRESHOLD
  #define USE_QSD_DEBUG                 1
#endif // configUSE_DSPC_QSD

#if configUSE_WOS
#if configUSE_DSPC_QSD || configUSE_RetuneDSP_VAD
  #define WOS_TIMEOUT_SEC               1.5              

#else // configUSE_DSPC_QSD || configUSE_RetuneDSP_VAD
  #define WOS_TIMEOUT_SEC               10
#endif // configUSE_DSPC_QSD || configUSE_RetuneDSP_VAD

  #define WOS_TIMEOUT_COUNT             (WOS_TIMEOUT_SEC * AM_SPP_SAMPLE_FREQ / AM_SPP_FRAME_SAMPLES)
#endif // configUSE_WOS

//********************************************************************************
// WWE parameters
//********************************************************************************
#if configUSE_RetuneDSP_VS
#define WWE_INPUT_FRAME_LENTH_SAMPLES   200
#elif configUSE_OAL_AID || configUSE_Cyberon_Spotter
#define WWE_INPUT_FRAME_LENTH_SAMPLES   160
#ifdef AM_VOS_DSPOTTER
#undef WWE_INPUT_FRAME_LENTH_SAMPLES
#define WWE_INPUT_FRAME_LENTH_SAMPLES   480
#endif
#else 
#define WWE_INPUT_FRAME_LENTH_SAMPLES   240
#endif // configUSE_RetuneDSP_VS, configUSE_OAL_AID

//extern void am_vos_AWE_instance_init(void);
void am_vos_audio_handler(int32_t *nLRSample);
void am_vos_audio_flush_ring_buffer(void);

uint8_t am_vos_engine_init(void);
void am_vos_engine_process(int16_t *pi16InputBuffer, int16_t i16InputLength);

void am_vos_audio_buffer_rewind(void);
void am_vos_audio_reset_flag_and_buffer(void);

void am_vos_CMD_LED_display(void);
void am_vos_THF_mode_change(uint8_t mode);

void am_vos_qsd_wos_threshold_to_high(void);
void am_vos_qsd_wos_threshold_to_low(void);

void am_vos_audio_wwd_enable(void);
void am_vos_audio_wwd_disable(void);

//*****************************************************************************
// Tuning Driver function declaration
//*****************************************************************************

extern void UARTMsgInit(void);

extern void CheckForUARTPacketReady(void);

extern void UART0SendReply(void);

#if configUSE_OVVP_DOUBLE_TAP
extern short Acc[3];
#endif // configUSE_OVVP_DOUBLE_TAP

#endif // AM_VOS_AUDIO_H
