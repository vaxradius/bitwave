//*****************************************************************************
//
//! @file am_vos_ama.h
//!
//! @brief header file of AMA protocol handler
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

#ifndef AM_VOS_AMA_H
#define AM_VOS_AMA_H

#include "accessories.pb.h"
#include "am_vos_sys_config.h"

#if configUSE_AMVOS_AMA

#define AMA_TRANSPORT_HEADER_VERSION_MASK       0xF000
#define AMA_TRANSPORT_HEADER_STEAM_ID_MASK      0x0F80
#define AMA_TRANSPORT_HEADER_LENGTH_TYPE_MASK   0x01

typedef struct _AmaDevInfo {
    char        serial_number[20];
    char        name[16];
    char        device_type[14];
    uint8_t     ui8AudioFormat;
} AmaDevInfo;

typedef struct _AmaSpeechInfo {
    uint8_t     ui8PushTalkFlag;

    bool        bPreRollEnabled;
    int32_t     i32PreRollDelayMs;
    int32_t     i32DelayedSample;
    int32_t     i32EndingSampleCnt;
} AmaSpeechInfo;

// AMA Event callback
typedef void (*amaEvtCback_t)(Command cmd, void* pMsg);

//*****************************************************************************
// External function declaration
//*****************************************************************************
bool am_vos_ama_reset_connection_send(void);

extern bool am_vos_ama_tx_ver_exchange_send(void);
extern bool am_vos_ama_start_speech_send(AmaSpeechInfo *pSpeechInfo);
extern bool am_vos_ama_get_state_rsp_send(State *pInput);
extern bool am_vos_ama_rsp_send(uint8_t tag, uint8_t err);
extern int am_vos_ama_rx_handler(uint8_t *data, uint16_t len);
extern bool am_vos_ama_evt_cback_register(amaEvtCback_t cback);
extern void am_vos_ama_status_reset(void);
extern void am_vos_ama_event_callback(Command cmd, void* pMsg);
extern void am_vos_ama_devinfo_set(AmaDevInfo *pInfo);
extern bool am_vos_ama_isready(void);
#endif // configUSE_AMVOS_AMA

#endif // AM_VOS_AMA_H
