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
// This is part of revision 1.2.11 of the AmbiqSuite Development Package.
//
//*****************************************************************************

///*******************************************************************************
//*
//*               System configurtion
//*               -------------------
//*
//********************************************************************************
//*     am_vos_sys_config.h
//********************************************************************************
//*
//*     Description:  Platform Interface Header File
//*
//*******************************************************************************/

#ifndef __AM_VOS_SYS_CONFIG_H__
#define __AM_VOS_SYS_CONFIG_H__

#define VOS_AMA_FW_VER_STRING                     "1.2.8"

// ---------------------------------------
// User Hardware Selection
// ---------------------------------------

/* Select an EVB Board */
#define USE_APOLLO2_EVB                            0
#define USE_APOLLO2_BLUE_EVB                       0
#define USE_APOLLO3_BLUE_EVB                       1    // Apollo3Blue & Apollo3BluePlus
#define USE_MAYA                                   0    // Apollo3Blue Button board

// Select Mikro Board(s)used and MikroBUS slots
#define USE_DMIC_MB3                               1    // Dual DMIC on Mikrobus slot 3
#define USE_DMIC_MB3_VM3011                        0    // Single DMIC on Mikrobus slot 3 (Vesper VM3011)

#define USE_AMIC_MB2                               0    // Sigle AMIC on Mikrobus slot 2
#define USE_AMIC_MB3                               0    // Sigle AMIC on Mikrobus slot 3
#define USE_AMIC_MB3_VMB                           0    // Dual AMIC on Mikrobus slot 3 (Vesper Mikrobus board)

#define USE_WOS_AMIC_MB1                           0
#define USE_WOS_AMIC_MB3                           0

#define USE_GPIO_FXL6408_MB3                       0
#define USE_EM9304_MIKRO_MB2                       0    // Apollo 2 EVB + EM9304 Clik shield

/* Select Shield Board */
#define USE_MIKRO_DEV_SHIELD_REV1                  1

/* Select PDM Clock Speed */
#define USE_PDM_CLK_750KHZ                         1
#define USE_PDM_CLK_1_5MHZ                         0

//*******************************************************************************
// System level functional module selection
//*******************************************************************************

#define configUSE_SYSVIEWER             0
#define configUSE_SYS_LOG               0
#define configUSE_RTT_RECORDER          0
#define configUSE_AMU2S_RECORDER        0
#define configUSE_STDIO_PRINTF          1

#if defined (AM_VOS_DSPC)
#define configUSE_AWE                   1               // DSPC AWE frame work switch
#else // AM_VOS_DSPC
#define configUSE_AWE                   0
#endif // AM_VOS_DSPC

#if defined (AM_VOS_THF)
    #define configUSE_Sensory_THF       1               // Sensory detection lib
#else // AM_VOS_THF
    #define configUSE_Sensory_THF       0
#endif // AM_VOS_THF

#if defined (AM_VOS_RDSP)
    #define configUSE_RetuneDSP_VS      1               // RetuneDSP VoiceSpot lib
#else // AM_VOS_RDSP
    #define configUSE_RetuneDSP_VS      0
#endif // AM_VOS_RDSP

#if defined (AM_VOS_CSPOTTER) || defined (AM_VOS_CSPOTTER_CES)
    #define configUSE_Cyberon_Spotter   1               // Cyberon detection lib
#else // AM_VOS_CSPOTTER
    #define configUSE_Cyberon_Spotter   0
#endif // AM_VOS_CSPOTTER

#if defined (AM_VOS_AID)
    #define configUSE_OAL_AID           1               // OpenAI Lab detection lib 
#else // AM_VOS_AID
    #define configUSE_OAL_AID           0
#endif // AM_VOS_AID

#define configUSE_BLE                   1
#define configUSE_AUDIO_CODEC           1
#define configUSE_LEDs                  1

#define configUSE_GSENSOR               0
#define configUSE_OVVP_DOUBLE_TAP       0

#define configUSE_PUSH_TO_TALK          1
#define configUSE_MUTE_MIC              1               // Use CTRL_BUTTON4 as mute mic function

//********************************************************************************
// AWE module configuration
//********************************************************************************
#if configUSE_AWE
    #define configUSE_ALEXA_QUAL_LAYOUT        1
    #define configUSE_AIC_LITE_LAYOUT          0
#endif // configUSE_AWE

#define configUSE_DSPC_QSD              0               // Enable QSD

#if configUSE_Sensory_THF
    #define configUSE_CMD_Phrase        0
    #define configUSE_THF_LPSD          0
#if configUSE_CMD_Phrase
    #define configUSE_THF_Static_Alloc  1               // If CMD phrase is enabled, Static mem allocation is recommended.
#else // configUSE_CMD_Phrase
    #define configUSE_THF_Static_Alloc  0
#endif // configUSE_CMD_Phrase
#endif // configUSE_Sensory_THF

#if configUSE_RetuneDSP_VS
#define configUSE_RetuneDSP_VAD         1
#endif // configUSE_RetuneDSP_VS

#if configUSE_CMD_Phrase
    #define USE_CMD_FullFitness         0
    #define USE_CMD_ReducedFitness_1    0
    #define USE_CMD_ReducedFitness_2    0
    #define USE_CMD_MUSIC               1
#endif // configUSE_CMD_Phrase

#if configUSE_OAL_AID
    #define configUSE_CMD_Phrase        1
#endif // configUSE_OAL_AID

#if USE_WOS_AMIC_MB1 || USE_WOS_AMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
    #define configUSE_WOS               1               // Wake on sound can be enabled in these board condition.

    #if USE_WOS_AMIC_MB1
        #define configUSE_Enhanced_WoS  0               // WoS monitoring peridically
    #endif // USE_WOS_AMIC_MB1
#else
    #define configUSE_WOS               0               // Disable wake on sound
#endif // USE_WOS_AMIC_MB1 || USE_WOS_AMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

//********************************************************************************
// Audio Codec module configuration
//********************************************************************************
#if configUSE_AUDIO_CODEC
    #define configUSE_SBC_BLUEZ                 0       // Codec selection. You could only choose 1
    #define configUSE_MSBC_BLUEZ                0
    #define configUSE_OPTIM_OPUS                1
#endif // configUSE_AUDIO_CODEC

#if configUSE_MSBC_BLUEZ
    #define AM_AMA_AUDIO_FORMAT                 3       // MSBC
#elif configUSE_OPTIM_OPUS
    #define AM_AMA_AUDIO_FORMAT                 1       // OPUS_32KBPS
#else
    #define AM_AMA_AUDIO_FORMAT                 0       // format
#endif // configUSE_MSBC_BLUEZ, configUSE_OPTIM_OPUS

//********************************************************************************
// BLE module configuration
//********************************************************************************
#if configUSE_BLE
#if (configUSE_Sensory_THF || configUSE_RetuneDSP_VS) && (!configUSE_WOS)
    #define configUSE_PREROLL                   1       // Audio preroll function, like Amazon CBWWV.
#else // (configUSE_Sensory_THF || configUSE_RetuneDSP_VS) && (!configUSE_WOS)
    #define configUSE_PREROLL                   0
#endif // (configUSE_Sensory_THF || configUSE_RetuneDSP_VS) && (!configUSE_WOS)

    #define configUSE_AMVOS_AMA                 1
    #define configUSE_BLE_SECURE_CONNECTION     1       // Using BLE with secured connection.
    #define configUSE_BLE_WATCHDOG              1       // Using watchdog timeout feature to recover connection.
    #define configUSE_BLE_Measure_Throughput    0
    #define configUSE_BLE_ERROR_SW_RESET        0
    #define AM_AMA_DEVICE_TYPE_STR              "A3U3B4RTI76K2J"        // Ambiq Mirco ID
#endif // configUSE_BLE

#define USE_UNIVERSAL_AUDIO_BUFFER              1

//********************************************************************************
// RTT recorder module configuration
//********************************************************************************
#if configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER
    #define configUSE_RECORD_RAW_PCM            1       // Select which data be recorded
    #define configUSE_RECORD_FULL_FILTER        0
#endif // configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER
//********************************************************************************
// System log module configuration
//********************************************************************************
#if configUSE_SYS_LOG
    #define configUSE_LOG_UART0                 1       // This is mutex with AWE_TUNING and BLE
    #define configUSE_LOG_UART1                 0
#endif // configUSE_SYS_LOG

//********************************************************************************
// std IO sub module configuration
//********************************************************************************
#define configUSE_Debug_Log_Print               1

#if configUSE_STDIO_PRINTF
    #define configUSE_PRINTF_UART0              0
    #define configUSE_PRINTF_RTT                0
    #define configUSE_PRINTF_SWO                1

    #define am_app_printf(...)                  am_app_utils_stdio_printf(__VA_ARGS__)

#if configUSE_Debug_Log_Print
    #define AM_APP_LOG_DEBUG(...)               am_app_utils_stdio_printf(1, __VA_ARGS__)
#else // configUSE_Debug_Log_Print
    #define AM_APP_LOG_DEBUG(...)
#endif // configUSE_Debug_Log_Print

    #define AM_APP_LOG_WARNING(...)             am_app_utils_stdio_printf(2, __VA_ARGS__)
    #define AM_APP_LOG_INFO(...)                am_app_utils_stdio_printf(3, __VA_ARGS__)

#else // configUSE_STDIO_PRINTF
    #define am_app_printf(...)
    #define AM_APP_LOG_DEBUG(...)
    #define AM_APP_LOG_WARNING(...)
    #define AM_APP_LOG_INFO(...)
#endif // configUSE_STDIO_PRINTF

//********************************************************************************
// Heap Memory parameters
//********************************************************************************

#define VOS_HEAP_SYSTEM                 (10 * 1024)              // Minimum Memory size that needed for audio task stack and scheduler

#if configUSE_AWE
    #define VOS_HEAP_PREPROCESSING      (2 * 1024)              // AWE Tick task stack (2352 bytes at VoS 1.1.3)
#else // configUSE_AWE
    #define VOS_HEAP_PREPROCESSING      0
#endif // configUSE_AWE

#if configUSE_Sensory_THF && (!configUSE_THF_Static_Alloc)
    #define VOS_HEAP_WWD_ENGINE         (15 * 1024)             // 12864 bytes (CMD phrase) 11672 bytes (64K Alexa kwd) at VoS 1.1.3
#elif configUSE_Cyberon_Spotter
    #define VOS_HEAP_WWD_ENGINE         (8 * 1024)
#elif configUSE_RetuneDSP_VS
    #define VOS_HEAP_WWD_ENGINE         (40 * 1024)
#elif configUSE_OAL_AID
    #define VOS_HEAP_WWD_ENGINE         (16 * 1024)
#else
    #define VOS_HEAP_WWD_ENGINE         0
#endif // configUSE_Sensory_THF, configUSE_Cyberon_Spotter, configUSE_RetuneDSP_VS, configUSE_OAL_AID

#if configUSE_AUDIO_CODEC
    #define VOS_HEAP_CODEC              (10 * 1024)              // Audio Codec task stack (4528 bytes at VoS 1.1.3)
#else // configUSE_AUDIO_CODEC
    #define VOS_HEAP_CODEC              0
#endif // configUSE_AUDIO_CODEC

#if configUSE_BLE
    #define VOS_HEAP_BLE                (3 * 1024)              // This value is depend on RadioQueue Size
#else // configUSE_BLE
    #define VOS_HEAP_BLE                0
#endif // configUSE_BLE

#if configUSE_LOG_UART0 || configUSE_PRINTF_UART0
    #define VOS_HEAP_UART               (2 * 1024)              // UART task stack (4528 bytes at VoS 1.1.3)
#else // configUSE_LOG_UART0 || configUSE_PRINTF_UART0
    #define VOS_HEAP_UART               0
#endif // configUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_PRINTF_RTT || configUSE_PRINTF_SWO
    #define VOS_HEAP_RTT_SWO            (4 * 1024)              // SWD STDIO task stack (4528 bytes at VoS 1.1.3)
#else // configUSE_PRINTF_RTT || configUSE_PRINTF_SWO
    #define VOS_HEAP_RTT_SWO            0
#endif // configUSE_PRINTF_RTT || configUSE_PRINTF_SWO

#if configUSE_SYSVIEWER
    #define VOS_HEAP_SYSTEMVIEW         (16 * 1024)
#else // configUSE_SYSVIEWER
    #define VOS_HEAP_SYSTEMVIEW         0
#endif // configUSE_SYSVIEWER

#define VOS_HEAL_TOTAL_SIZE             (VOS_HEAP_SYSTEM + VOS_HEAP_PREPROCESSING + VOS_HEAP_WWD_ENGINE + VOS_HEAP_CODEC + VOS_HEAP_BLE + VOS_HEAP_UART + VOS_HEAP_RTT_SWO + VOS_HEAP_SYSTEMVIEW)


#endif // __AM_VOS_SYS_CONFIG_H__
