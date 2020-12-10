//*****************************************************************************
//
//! @file am_vos_task.h
//!
//! @brief header file of application tasks
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

#ifndef AM_VOS_TASK_H
#define AM_VOS_TASK_H

#include <stdbool.h>

#include "am_vos_sys_config.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

//*****************************************************************************
//
// App message type list macro definitions
//
//*****************************************************************************
// this enum list contains all app related ring buffers between AM_APP_RINGBUFF_NONE and AM_APP_RINGBUFF_MAX
// where each index corresponds to a ring buffer
typedef enum
{
    AM_APP_MESSAGE_NONE = 0, // The enum must begin with this value as named.
    AM_APP_MESSAGE_SHORT,
    AM_APP_MESSAGE_LONG,
    AM_APP_MESSAGE_STR,
    AM_APP_MESSAGE_MAX // The enum must end with this value as name} am_app_utils_ring_buffer_enum_t;
} am_app_utils_message_type_enum_t;

#define AM_CRITICAL_BEGIN_VOS                                               \
    if ( 1 )                                                                \
    {                                                                       \
        volatile uint32_t ui32Primask_04172010;                             \
        ui32Primask_04172010 = am_hal_interrupt_master_disable();

#define AM_CRITICAL_END_VOS                                                 \
        am_hal_interrupt_master_set(ui32Primask_04172010);                  \
    }

// Macro define for some empty message

#if configUSE_PRINTF_SWO
extern void am_app_itm_printf_enable(void);
#endif // configUSE_PRINTF_SWO

extern void am_vos_setup_task(void *pvParameters);

extern void uart_task(void *pvParameters);

extern void am_vos_led_task(void *pvParameters);

extern void am_vos_rtt_switch_task(void *pvParameters);

extern void am_vos_uart0_gatekeeper_task(void *pvParameters);

extern void am_vos_stdio_gatekeeper_task(void* pvParameters);

extern void am_vos_audio_processing_task(void *pvParameters);

extern void am_vos_AWE_tick_task(void *pvParameters);

extern void am_vos_buzzer_task(void *pvParameters);
extern void am_vos_gsensor_task(void *pvParameters);
extern void am_vos_logic_task(void *pvParameters);

extern void am_vos_codec_task(void *pvParameters);

extern void am_vos_timer_heart_beat_callback(TimerHandle_t xTimer);

extern void am_vos_timer_gsensor_period_callback(TimerHandle_t xTimer);

#if configUSE_WOS
void am_vos_wos_check_status(void);
extern void am_vos_wos_handler(void);
#endif // configUSE_WOS

#if configUSE_PUSH_TO_TALK
void am_vos_push_to_talk_process(void);
#endif // configUSE_PUSH_TO_TALK

#if configUSE_MUTE_MIC
void am_vos_timer_longpress_callback(TimerHandle_t xTimer);
void am_vos_mute_mic_process(void);
void am_vos_mute_mic_toggle(void);
#endif // configUSE_MUTE_MIC

#endif // AM_VOS_TASK_H
