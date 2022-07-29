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
#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include "am_app_utils.h"
#include "am_app_utils_task.h"

#include "am_vos_task.h"
#include "am_vos_init.h"
#include "am_vos_audio.h"
#include "am_vos_codec.h"
#include "am_vos_ble.h"

#if configUSE_AMVOS_AMA
#include "am_vos_ama.h"
#endif // configUSE_AMVOS_AMA

#if configUSE_SYSVIEWER
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_RTT.h"
#endif // configUSE_SYSVIEWER

#if configUSE_STDIO_PRINTF
#include "am_vos_rtt_recorder.h"
#endif // configUSE_STDIO_PRINTF

#if configUSE_PCM_HP_FILTER
#include "am_app_digital_filter.h"
#endif // configUSE_PCM_HP_FILTER

//*****************************************************************************
//
// KWD application tasks list. User needs to keep this lists name unchaged.
//
//*****************************************************************************

am_app_utils_task_t am_KWD_tasks[AM_APP_MAX_TASK];

VosSysInfo g_sVosSys =
{
    .ui8KwdDetectedFlag = 0,             // 1: the key word is detected; 0: no key word detected.
    .ui8ButtonDownFlag = 0,              // 1: the button is pushed down; 0: no button is pushed.
    .ui8PushTalkFlag = 0,                // trig for AMA push-to-talk.

    .ui32StreamingTimeOutCount = 0,      // Voice streaming timeout counter
    .bWwdEnabled = 0,

#if defined (AM_PART_APOLLO2)
    .bUARTPacketReceived = false,
#endif // AM_PART_APOLLO2
    
#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
    .pvPDMHandle = NULL,
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    .pvADCHandle = NULL;
    .bADCDMAError = false;
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB

#if USE_MAYA
    .eLogicPowerState = APP_LOGIC_POWERING_UP,
#endif // USE_MAYA

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    .pvGSensorHdl = NULL,
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

#if configUSE_AMVOS_AMA
    .ui8ProvideSpeechFlag = 0,           // trig for AMA conversation talk.

    .i32DelayedSample = 0,
    .i32EndingSampleCnt = 0,
    .i32PreRollDelayMs = AUDIO_PREROLL_TIME_MS,
#else // configUSE_AMVOS_AMA
    .ui8SeqNumber = 0,
#endif // configUSE_AMVOS_AMA

#if configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER
    .ui8RecordStartFlag = 0,
//    .ui32RTTSendBytes = 0,
#endif // configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER

#if configUSE_WOS
    .ui8WosStatus = 0,
    .ui8WosActiveFlag = 0,
    .i32WosActiveCount = WOS_TIMEOUT_COUNT,     // Start pdm > WOS_TIMEOUT_COUNT * 5 ms when power-up
    .ui8WoSDetectFlag = 0,
#endif // configUSE_WOS

#if configUSE_DSPC_QSD
    .sQsdHandle.noiseFloorThreshold = -60,
    .sQsdHandle.snrThreshold = 6,
    .eQsdStatus = VOS_QSD_INIT,
    .ui16QsdOffDelayCnt = 0,
    .ui16QsdTimeoutCnt = 0,
#endif // configUSE_DSPC_QSD

    .ui32SysHeapSize = VOS_HEAL_TOTAL_SIZE
};

//*****************************************************************************
//
// KWD application tasks configuration 
//
//*****************************************************************************
static const am_app_utils_task_setup_t g_KWD_TaskSetup[] = 
{
    // TASK ID,                     Task function pointer,              Task name string,           Stack,  Cback,  Priority,   Queue size
    {AM_APP_TASK_LED,               &am_vos_led_task,               "key&led",                  256,    NULL,   2,          4},

#if configUSE_LOG_UART0 || configUSE_PRINTF_UART0
    {AM_APP_TASK_UART0,             &am_vos_uart0_gatekeeper_task,  "uart0_data_print",         256,   NULL,   2,          16},
#endif // configUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_PRINTF_RTT || configUSE_PRINTF_SWO
    {AM_APP_TASK_STDIO,             &am_vos_stdio_gatekeeper_task,  "print_on_rtt_swo",         768,   NULL,   3,          16},  // a high priority task to print
#endif // configUSE_PRINTF_RTT || configUSE_PRINTF_SWO

    {AM_APP_TASK_AUD_PROCESSING,    &am_vos_audio_processing_task,  "audio_process_task",         1024,   NULL,   5,          16},

#if configUSE_AUDIO_CODEC
    {AM_APP_TASK_CODEC,             &am_vos_codec_task,                 "audio_stream_compress",    2048,   NULL,   4,          16},
#endif // configUSE_AUDIO_CODEC

#if configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER
    {AM_APP_TASK_RTT_SWITCH,        &am_vos_rtt_switch_task,        "rtt_switch",               128,    NULL,   1,          8},
#endif // configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER
    
#if configUSE_BLE
    {AM_APP_BLE,                    &am_vos_ble_task,                   "ble_task",                 768,   NULL,   4,          16},
#endif // configUSE_BLE

#if USE_MAYA

#if configUSE_BUZZER
    {AM_APP_TASK_BUZZER,            &am_vos_buzzer_task,            "buzzer_task",              64,     NULL,   2,          4},
#endif // configUSE_BUZZER

    {AM_APP_TASK_LOGIC,             &am_vos_logic_task,             "logic_task",               256,    NULL,   1,          4},
#endif // USE_MAYA

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    {AM_APP_TASK_GSENSOR,           &am_vos_gsensor_task,           "gsensor_task",             256,    NULL,   1,          4},
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
};

#define KWD_TASK_INIT_COUNT     (sizeof(g_KWD_TaskSetup)/sizeof(am_app_utils_task_setup_t))

//*****************************************************************************
//
// KWD application tasks list. User needs to keep this lists name unchaged.
//
//*****************************************************************************

TimerHandle_t am_KWD_timers[AM_APP_MAX_TIMER];
//*****************************************************************************
//
// KWD application timers configuration
//
//*****************************************************************************
static const am_app_utils_timer_setup_t g_KWD_TimerSetup[] =
{
    {AM_APP_TIMER_HEART_BEAT, "HeartBeat", HEART_BEAT_PERIOD,  pdTRUE, &am_vos_timer_heart_beat_callback},
    #if 0 //configUSE_MUTE_MIC
        {AM_APP_TIMER_LONG_PRESS, "ButtonLongPress", LONG_PRESS_TIMER_PERIOD_MS, pdFALSE, &am_vos_timer_longpress_callback},
    #endif // configUSE_MUTE_MIC

    #if configUSE_GSENSOR_MOTION
        {AM_APP_TIMER_GSENSOR_PERIOD, "GsensorTimer", GSENSOR_PERIOD, pdTRUE, &am_vos_timer_gsensor_period_callback}
    #endif // configUSE_GSENSOR_MOTION
};

#define KWD_TIMER_INIT_COUNT     (sizeof(g_KWD_TimerSetup)/sizeof(am_app_utils_timer_setup_t))

//******************************************************************************
//KWD system init function
//*****************************************************************************

void am_vos_sys_init(void)
{
    am_vos_board_init();

#if configUSE_STDIO_PRINTF
    am_app_utils_stdio_printf_init((am_app_utils_stdio_printf_t)am_vos_printf);
    //am_app_utils_stdio_printf_init((am_app_utils_stdio_printf_t)am_hal_itm_print);
#endif // configUSE_STDIO_PRINTF

#if configUSE_AWE
    // QSD isn't working for now    
    am_spp_init(&g_sSppInput, &g_sSppOutput);
#endif // configUSE_AWE

    //
    // audio codec initialization
    //
#if configUSE_AUDIO_CODEC
    am_vos_codec_init();
#endif // configUSE_AUDIO_CODEC

#if configUSE_RTT_RECORDER
    am_vos_rtt_init(g_sVosSys.pui8RttRecorderBuffer, RTT_BUFFER_LENGTH);
#endif // configUSE_RTT_RECORDER
    
#if USE_UNIVERSAL_AUDIO_BUFFER
    //
    // init audio buffers
    //
    am_audio_buffer_init();
#endif // USE_UNIVERSAL_AUDIO_BUFFER

#if configUSE_PCM_HP_FILTER    
    //
    // apps digital filter init
    // we need 2 instances of IIR because we have 2 channels
    //
    high_pass_filter_init(&(g_sVosSys.sAmicLeftHPFilter));
    high_pass_filter_init(&(g_sVosSys.sAmicRightHPFilter));
#endif // configUSE_PCM_HP_FILTER
    
    //
    // create application tasks 
    //
    am_app_utils_task_init();
    am_app_utils_task_create_all_tasks(g_KWD_TaskSetup, KWD_TASK_INIT_COUNT);
    
    am_app_utils_timer_create_all_timers(g_KWD_TimerSetup, KWD_TIMER_INIT_COUNT);

    // Enable system heart beat LED
    xTimerStart(am_KWD_timers[AM_APP_TIMER_HEART_BEAT], 0);

#if configUSE_GSENSOR_MOTION
    xTimerStart(am_KWD_timers[AM_APP_TIMER_GSENSOR_PERIOD], 0);
#endif // configUSE_GSENSOR_MOTION

    am_vos_mic_enable();

#if configUSE_SYSVIEWER
    //
    // Configure the SEGGER SystemView Interface.
    //
    SEGGER_RTT_Init();      //explicitly call this function to init RTT block
    SEGGER_SYSVIEW_Conf();
#endif // configUSE_SYSVIEWER
}
