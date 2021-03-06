
//*****************************************************************************
//
//! @file am_app_utils.h
//!
//! @brief apollo application utils header file 
//!
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
// This is part of revision v1.2.9-459-g671fc0f-wearable-dev-framework of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_APP_UTILS_H
#define AM_APP_UTILS_H

#include <stdint.h>
#include <stdbool.h>
//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"

//*****************************************************************************
//
// Application utils include files.
//
//*****************************************************************************
#include "am_app_utils_ring_buffer.h"
#include "am_app_utils_stdio.h"
#include "am_audio_buffer.h"

/**************************************************************************************************
 Type Macros
**************************************************************************************************/

// System message macro definition 
#define EMPTY_MESSAGE                   1
#define HEARTBEAT_TIMER_MESSAGE         2
#define DOUBLE_TAP_MESSAGE              3
#define MUTE_MIC_MESSAGE                4
#define KEY_WORD_GOT_MESSAGE            0xaa111155

typedef struct _AmVosUtil {
    am_app_utils_stdio_printf_t         pfnPrintf;                                      // function pointer for printf
    uint32_t                            ui32BuffIndx;
    char                                pcStdioBuff[AM_APP_PRINTF_TOTAL_BUFFSIZE];      // buffer for printf
    
    uint8_t                             pui8RingBuffer[BYTES_UNIVERSAL_BUFFER_SIZE];
    am_app_utils_ring_buffer_t          sRingBuf[AM_AUDIO_BUFFER_MAX];

#if configUSE_GSENSOR_MOTION
    TickType_t                          ui32FirstTick;
#endif // configUSE_GSENSOR_MOTION
} AmVosUtil;

extern AmVosUtil g_sAmUtil; 

#endif // AM_APP_UTILS_H
