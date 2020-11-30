//*****************************************************************************
//
//! @file am_vos_codec.h
//!
//! @brief header file of KWD codec
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

#ifndef AM_VOS_CODEC_H
#define AM_VOS_CODEC_H

#include "am_vos_sys_config.h"

#if configUSE_AUDIO_CODEC && (configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ)
#include "sbc.h" 
#endif // configUSE_AUDIO_CODEC && (configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ)


//********************************************************************************
// Codec parameters
//********************************************************************************
    
#define SBC_IN_RING_BUFF_SIZE           256
#define MSBC_IN_RING_BUFF_SIZE          240
#define OPUS_IN_RING_BUFF_SIZE          640

//
// If using SBC compression, select audio transfer compression ratio
// 1:1 = 256000 bps, 4:1 = 64000 bps, 8:1 = 32000 bps, 16:1 = 16000 bps
// mSBC has fixed output rate 57000 bps
//
#define SBC_BLUEZ_COMPRESS_BPS          64000
#define SBC_OUT_RING_BUFF_SIZE          (SBC_BLUEZ_COMPRESS_BPS / 1000)

#define MSBC_BLUEZ_COMPRESS_BPS         57000
#define MSBC_OUT_RING_BUFF_SIZE         (MSBC_BLUEZ_COMPRESS_BPS / 1000)

#define OPUS_IN_FRAME_SIZE              320
#define OPUS_OUT_RING_BUFF_SIZE         80

#if configUSE_SBC_BLUEZ
    #define CODEC_IN_RING_BUFF_SIZE     SBC_IN_RING_BUFF_SIZE
    #define CODEC_OUT_RING_BUFF_SIZE    SBC_OUT_RING_BUFF_SIZE

#elif configUSE_MSBC_BLUEZ
    #define CODEC_IN_RING_BUFF_SIZE     MSBC_IN_RING_BUFF_SIZE
    #define CODEC_OUT_RING_BUFF_SIZE    MSBC_OUT_RING_BUFF_SIZE

#elif configUSE_OPTIM_OPUS
    #define CODEC_IN_RING_BUFF_SIZE     OPUS_IN_RING_BUFF_SIZE
    #define CODEC_OUT_RING_BUFF_SIZE    OPUS_OUT_RING_BUFF_SIZE

#else
    #define CODEC_IN_RING_BUFF_SIZE     256
    #define CODEC_OUT_RING_BUFF_SIZE    64
#endif // configUSE_SBC_BLUEZ, configUSE_MSBC_BLUEZ, configUSE_OPTIM_OPUS

void am_vos_codec_init(void);
void am_vos_codec_encode(void *, const void *, size_t, void *, size_t, int *);

#endif // AM_VOS_CODEC_H
