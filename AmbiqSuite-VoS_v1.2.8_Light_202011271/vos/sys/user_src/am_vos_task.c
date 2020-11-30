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

//*****************************************************************************
// Global includes for this project.
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
#include "am_vos_logic.h"

#if configUSE_AMVOS_AMA
#include "am_vos_ama.h"
#endif // configUSE_AMVOS_AMA

#if configUSE_Sensory_THF
#include "SensoryLib.h"
#include "am_vos_SensoryTHF.h"
#endif // configUSE_Sensory_THF

#if configUSE_Cyberon_Spotter
#ifdef AM_VOS_DSPOTTER
#include "am_vos_DSpotter.h"
#else
#include "am_vos_CSpotter.h"
#endif
#endif // configUSE_Cyberon_Spotter

#if configUSE_RetuneDSP_VS
#include "am_vos_RetuneDSP.h"
#endif // configUSE_RetuneDSP_VS

#if USE_MAYA
#include "am_app_utils_buzzer.h"
#include "am_app_utils_gsensor.h"
#endif // USE_MAYA

#if configUSE_OVVP_DOUBLE_TAP
#include "am_devices_lis2dw12.h"
#endif // configUSE_OVVP_DOUBLE_TAP

#if USE_GPIO_FXL6408_MB3
#include "am_devices_fxl6408.h"
#endif // USE_GPIO_FXL6408_MB3

#if configUSE_LOG_UART0 || configUSE_PRINTF_UART0
//*****************************************************************************
//
// Serial communication task to transmit data.
//
//*****************************************************************************

void am_vos_uart0_gatekeeper_task(void *pvParameters)
{
    am_app_utils_task_queue_element_t QueueElement;
    uint8_t transmit_buff[UART_TRANSMIT_BUFFER];
    int32_t transmit_length = 0;
    uint32_t index = 0;
#if defined (AM_PART_APOLLO2)
    const TickType_t xDelay1ms = pdMS_TO_TICKS(1);
    uint32_t ui32RxSize = 0;
    uint32_t ui32TxSize = 0;
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    uint32_t ui32BytesWritten = 0;
    am_hal_uart_transfer_t sUartWrite;
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

    while(1)
    {
        am_app_utils_task_read(AM_APP_TASK_UART0, &QueueElement);

        switch(QueueElement.ui32MessageType)
        {
            case AM_APP_MESSAGE_SHORT:
                transmit_length = 4;
                for(index=0; index < 4; index++)
                {
                    transmit_buff[index] = *((uint8_t*)&QueueElement.info.ui32Note+index);
                }
#if defined (AM_PART_APOLLO2)
                for(index=0; index < transmit_length; index++)
                {
                    am_hal_uart_char_transmit_buffered(0, transmit_buff[index]);
                }
#endif // AM_PART_APOLLO2
#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
                    ui32BytesWritten = 0;
                    sUartWrite.ui32Direction = AM_HAL_UART_WRITE;
                    sUartWrite.pui8Data = (uint8_t *) transmit_buff;
                    sUartWrite.ui32NumBytes = transmit_length;
                    sUartWrite.ui32TimeoutMs = 0;
                    sUartWrite.pui32BytesTransferred = &ui32BytesWritten;

                    am_hal_uart_transfer(g_sVosSys.pvUartHandle, &sUartWrite);
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

                break;

            case AM_APP_MESSAGE_LONG:
                transmit_length = (int32_t)QueueElement.info.ui32Length;
                while(transmit_length > 0)
                {
#if defined (AM_PART_APOLLO2)
                    while(ui32TxSize > (UART0_BUFFER_SIZE - UART_TRANSMIT_BUFFER))
                    {
                        vTaskDelay(xDelay1ms);
                        am_hal_uart_get_status_buffered(UART0_MODULE, &ui32RxSize, &ui32TxSize);

                    }
#endif // AM_PART_APOLLO2

                    if(transmit_length < UART_TRANSMIT_BUFFER)
                    {
                        AM_CRITICAL_BEGIN_VOS;
//                        am_app_utils_ring_buffer_pop(QueueElement.pDataBuffer, &transmit_buff,
//                                                    transmit_length, FALSE);
                        am_app_utils_ring_buffer_pop(QueueElement.pDataBuffer, &transmit_buff,
                                                        transmit_length);

                        AM_CRITICAL_END_VOS;
#if defined (AM_PART_APOLLO2)
                        for(index=0; index < transmit_length; index++)
                        {
                            am_hal_uart_char_transmit_buffered(0, transmit_buff[index]);
                        }
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
                        //
                        // Print the string via the UART.
                        //
                        ui32BytesWritten = 0;
                        sUartWrite.ui32Direction = AM_HAL_UART_WRITE;
                        sUartWrite.pui8Data = (uint8_t *) transmit_buff;
                        sUartWrite.ui32NumBytes = transmit_length;
                        sUartWrite.ui32TimeoutMs = 0;
                        sUartWrite.pui32BytesTransferred = &ui32BytesWritten;

                        am_hal_uart_transfer(g_sVosSys.pvUartHandle, &sUartWrite);

#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
                        transmit_length = 0;
                    }
                    else
                    {
                        AM_CRITICAL_BEGIN_VOS;
                        am_app_utils_ring_buffer_pop(QueueElement.pDataBuffer,
                                                        &transmit_buff, UART_TRANSMIT_BUFFER);

                        AM_CRITICAL_END_VOS;
#if defined (AM_PART_APOLLO2)
                        for(index=0; index < UART_TRANSMIT_BUFFER; index++)
                        {
                            am_hal_uart_char_transmit_buffered(0, transmit_buff[index]);
                        }
                        transmit_length -= UART_TRANSMIT_BUFFER;
                    }

                    am_hal_uart_get_status_buffered(UART0_MODULE, &ui32RxSize, &ui32TxSize);
                }
                break;

            case AM_APP_MESSAGE_STR:
                transmit_length = (int32_t)strlen(&(g_sAmUtil.pcStdioBuff[QueueElement.info.ui32Indx]));
                for(index=0; index < transmit_length; index++)
                {
                    transmit_buff[index] = g_sAmUtil.pcStdioBuff[QueueElement.info.ui32Indx + index];
                }

                for(index=0; index < transmit_length; index++)
                {
                    am_hal_uart_char_transmit_buffered(0, transmit_buff[index]);
                }
                break;

            default:
                break;
        }
    }

}
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
                        ui32BytesWritten = 0;
                        sUartWrite.ui32Direction = AM_HAL_UART_WRITE;
                        sUartWrite.pui8Data = (uint8_t *) transmit_buff;
                        sUartWrite.ui32NumBytes = transmit_length;
                        sUartWrite.ui32TimeoutMs = 0;
                        sUartWrite.pui32BytesTransferred = &ui32BytesWritten;

                        am_hal_uart_transfer(g_sVosSys.pvUartHandle, &sUartWrite);

                        transmit_length -= UART_TRANSMIT_BUFFER;
                    }
                }
                break;

            case AM_APP_MESSAGE_STR:
                transmit_length = (int32_t)strlen(&(g_sAmUtil.pcStdioBuff[QueueElement.info.ui32Indx]));
                for(index=0; index < transmit_length; index++)
                {
                    transmit_buff[index] = g_sAmUtil.pcStdioBuff[QueueElement.info.ui32Indx + index];
                }

                ui32BytesWritten = 0;
                sUartWrite.ui32Direction = AM_HAL_UART_WRITE;
                sUartWrite.pui8Data = (uint8_t *) transmit_buff;
                sUartWrite.ui32NumBytes = transmit_length;
                sUartWrite.ui32TimeoutMs = 0;
                sUartWrite.pui32BytesTransferred = &ui32BytesWritten;

                am_hal_uart_transfer(g_sVosSys.pvUartHandle, &sUartWrite);

                break;

            default:
                break;
        }
    }
}
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

#endif // configUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_PRINTF_RTT || configUSE_PRINTF_SWO
//*****************************************************************************
//
// Standard IO task to print data on configured interface
//
//*****************************************************************************
void am_vos_stdio_gatekeeper_task(void* pvParameters)
{
    am_app_utils_task_queue_element_t QueueElement;
    uint8_t transmit_buff[AM_APP_PRINTF_BUFFSIZE*AM_APP_STDIO_BUFFNUM + 8 + 7 + 4]; // extra 19 bytes for color coding, RTT (BG = 8bytes, text = 7bytes, reset = 4bytes)
    int32_t transmit_length = 0;
    uint32_t index = 0;


    // during init
    // rtt print examples:
//    am_util_stdio_printf("Example: Use AM_APP_LOG_DEBUG(); for Debug log print. \n");
//    am_util_stdio_printf("Example: Use AM_APP_LOG_INFO(); for Info log print. \n");
//    am_util_stdio_printf("Example: Use AM_APP_LOG_WARNING(); for Warning log print. \n");

//    AM_APP_LOG_DEBUG("BG coding length = %d, Text coding length = %d, Ctrl reset coding length = %d \n", strlen(RTT_CTRL_BG_BLACK),  strlen(RTT_CTRL_TEXT_BRIGHT_GREEN), strlen(RTT_CTRL_RESET));

    while(1)
    {
        am_app_utils_task_read(AM_APP_TASK_STDIO, &QueueElement);
        uint8_t color_offset = 0;
        switch(QueueElement.ui32MessageType)
        {
            case AM_APP_MESSAGE_STR:
#if configUSE_PRINTF_RTT
                switch(QueueElement.Source)
                {
                    case 0:
                      // plain text, write
                      memcpy(transmit_buff, RTT_CTRL_RESET, strlen(RTT_CTRL_RESET));
                      color_offset = strlen(RTT_CTRL_RESET);
                      break;
                    case 1:
                      // debug, green, black background
                      memcpy(transmit_buff, RTT_CTRL_BG_BLACK, strlen(RTT_CTRL_BG_BLACK));
                      memcpy(transmit_buff + strlen(RTT_CTRL_BG_BLACK), RTT_CTRL_TEXT_BRIGHT_GREEN, strlen(RTT_CTRL_TEXT_BRIGHT_GREEN));
                      color_offset = strlen(RTT_CTRL_BG_BLACK) + strlen(RTT_CTRL_TEXT_BRIGHT_GREEN);
                      break;
                    case 2:
                      // warning, white, red background
                      memcpy(transmit_buff, RTT_CTRL_BG_RED, strlen(RTT_CTRL_BG_RED));
                      memcpy(transmit_buff + strlen(RTT_CTRL_BG_RED), RTT_CTRL_TEXT_BRIGHT_WHITE, strlen(RTT_CTRL_TEXT_BRIGHT_WHITE));
                      color_offset = strlen(RTT_CTRL_BG_RED) + strlen(RTT_CTRL_TEXT_BRIGHT_WHITE);
                      break;
                    case 3:
                      // info, cyan, black background
                      memcpy(transmit_buff, RTT_CTRL_BG_BLACK, strlen(RTT_CTRL_BG_BLACK));
                      memcpy(transmit_buff + strlen(RTT_CTRL_BG_BLACK), RTT_CTRL_TEXT_BRIGHT_CYAN, strlen(RTT_CTRL_TEXT_BRIGHT_CYAN));
                      color_offset = strlen(RTT_CTRL_BG_BLACK) + strlen(RTT_CTRL_TEXT_BRIGHT_CYAN);
                      break;
                    default:
                      // plain text, write
                      memcpy(transmit_buff, RTT_CTRL_RESET, strlen(RTT_CTRL_RESET));
                      color_offset = strlen(RTT_CTRL_RESET);
                      break;
//                    SEGGER_RTT_Write(0, transmit_buff, transmit_length);
                }
#endif // configUSE_PRINTF_RTT

#if configUSE_PRINTF_SWO
                color_offset = 0;
#endif // configUSE_PRINTF_SWO

                transmit_length = (int32_t)strlen(&(g_sAmUtil.pcStdioBuff[QueueElement.info.ui32Indx]));

                configASSERT(transmit_length < AM_APP_PRINTF_BUFFSIZE);

                for(index=0; index < transmit_length; index++)
                {
                    transmit_buff[index + color_offset] = g_sAmUtil.pcStdioBuff[QueueElement.info.ui32Indx + index];
                }
                transmit_buff[transmit_length + color_offset] = NULL;
#if configUSE_PRINTF_RTT
                SEGGER_RTT_printf(0, (const char *)transmit_buff);
#endif // configUSE_PRINTF_RTT

#if configUSE_PRINTF_SWO
                am_util_stdio_printf((const char *)transmit_buff);
#endif // configUSE_PRINTF_SWO
                break;
            default:
                break;
        }

    }


}
#endif // configUSE_PRINTF_RTT || configUSE_PRINTF_SWO

//*****************************************************************************
//
// LED task to indicate external events, such as heart beat and key word detected.
//
//*****************************************************************************
void am_vos_led_task(void *pvParameters)
{
    am_app_utils_task_queue_element_t QueueElement;

    // power up swirl
    //am_vos_logic_led_swirl(1);

    while(1)
    {
        am_app_utils_task_read(AM_APP_TASK_LED, &QueueElement);
        if(QueueElement.info.ui32Note == KEY_WORD_GOT_MESSAGE)
        {
#if configUSE_LEDs
            am_vos_logic_led_swirl(0);
#endif // configUSE_LEDs
        }
        else if(QueueElement.info.ui32Note == HEARTBEAT_TIMER_MESSAGE)
        {
#if configUSE_OVVP_DOUBLE_TAP
            //AM_APP_LOG_INFO("Acc[0]: %d Acc[1]: %d, Acc[2]: %d\n", Acc[0], Acc[1], Acc[2]);
            //AM_APP_LOG_INFO("three_tap_counter: %d two_tap_counter: %d\n", three_tap_counter, two_tap_counter);
#endif // configUSE_OVVP_DOUBLE_TAP
#if configUSE_LEDs
#if USE_MAYA
            if(am_vos_is_connected())
            {
                am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0xFF);
                vTaskDelay(pdMS_TO_TICKS(20));
                am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0x0);
            }
            else
            {
                am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0xFF);
                vTaskDelay(pdMS_TO_TICKS(10));
                am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0x0);
                vTaskDelay(pdMS_TO_TICKS(150));
                am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0xFF);
                vTaskDelay(pdMS_TO_TICKS(10));
                am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0x0);
            }
#else // USE_MAYA
            if(am_vos_is_connected())
            {
                am_devices_led_toggle(am_bsp_psLEDs, LED_SYSTEM);
            }
            else
            {
                am_devices_led_on(am_bsp_psLEDs, LED_SYSTEM);
                vTaskDelay(pdMS_TO_TICKS(10));
                am_devices_led_off(am_bsp_psLEDs, LED_SYSTEM);
                vTaskDelay(pdMS_TO_TICKS(150));
                am_devices_led_on(am_bsp_psLEDs, LED_SYSTEM);
                vTaskDelay(pdMS_TO_TICKS(10));
                am_devices_led_off(am_bsp_psLEDs, LED_SYSTEM);
            }
#endif // USE_MAYA
#endif // configUSE_LEDs

#if configUSE_BLE_Measure_Throughput
            if(g_ui32AmaDataSentLength > 0)
            {
                AM_APP_LOG_INFO("[AMA] Data sent %d B/s\n", g_ui32AmaDataSentLength * g_ui32AmaDataCnfCnt / g_ui32AmaDataSentCnt);
                g_ui32AmaDataSentLength = 0;
                g_ui32AmaDataSentCnt -= g_ui32AmaDataCnfCnt;
                g_ui32AmaDataCnfCnt = 0;
            }
#endif // configUSE_BLE_Measure_Throughput
        }
        else if(QueueElement.info.ui32Note == DOUBLE_TAP_MESSAGE)
        {
#if configUSE_LEDs
            am_vos_logic_led_swirl(2);
#endif // configUSE_LEDs
        }
#if configUSE_MUTE_MIC
        else if(QueueElement.info.ui32Note == MUTE_MIC_MESSAGE)
        {
            am_vos_mute_mic_toggle();
        }
#endif // configUSE_MUTE_MIC
#if USE_MAYA
        const TickType_t xDelay200ms = pdMS_TO_TICKS(200);

        // enable button0 interrupt.
        if (QueueElement.Source == AM_APP_ISR_GPIO)
        {
          vTaskDelay(xDelay200ms);
          am_vos_gpio_enable_irq(AM_BSP_GPIO_BUTTON0);
        }
#endif // USE_MAYA
    }

}
//****************************************************************************
//
// RTT switch task to turn on/off RTT recorder
//
//****************************************************************************
#if configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER
void am_vos_rtt_switch_task(void *pvParameters)
{
    uint32_t record_switch = 1;
    uint8_t switch_times = 0;
    const TickType_t xDelay100ms = pdMS_TO_TICKS(100);

    while(1)
    {
        //
        //Key anti jitter
        //
#if defined (AM_PART_APOLLO2)
        record_switch = am_hal_gpio_input_bit_read(RTT_DUMP_BUTTON);          // set low to turn off LED
#endif // AM_PART_APOLLO2
#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
        am_hal_gpio_state_read(RTT_DUMP_BUTTON, AM_HAL_GPIO_INPUT_READ, &record_switch);
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
        if((record_switch==0)&&(switch_times<10))
            switch_times++;
        else if((record_switch==1)&&(switch_times<=2))
            switch_times = 0;
        //
        //Key action is confirmed
        //
        if((switch_times>2)&&(record_switch==1))
        {
            if(g_sVosSys.ui8RecordStartFlag == 0)
                g_sVosSys.ui8RecordStartFlag = 1;
            else
                g_sVosSys.ui8RecordStartFlag = 0;

            switch_times = 0;

            if(g_sVosSys.ui8RecordStartFlag == 1)
            {
                xTimerChangePeriod(am_KWD_timers[AM_APP_TIMER_HEART_BEAT], RTT_RECORDER_RUNNING, 0);
            }
            else
            {
                xTimerChangePeriod(am_KWD_timers[AM_APP_TIMER_HEART_BEAT], HEART_BEAT_PERIOD, 0);
            }
        }
        vTaskDelay(xDelay100ms);
    }

}
#endif // configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER

//*****************************************************************************
//
// Audio task to execute the SPP and Wakeword engine
//
//*****************************************************************************
void am_vos_audio_processing_task(void *pvParameters)
{
    am_app_utils_task_queue_element_t QueueElement;
    int32_t in32LRSample[AM_SPP_FRAME_SAMPLES];

    //am_app_KWD_print_info();
#if USE_GPIO_FXL6408_MB3
    am_app_utils_fxl6408_init();
#endif // USE_GPIO_FXL6408_MB3

#if configUSE_Sensory_THF || configUSE_Cyberon_Spotter || configUSE_RetuneDSP_VS || configUSE_OAL_AID
    if(am_vos_engine_init())
    {
        AM_APP_LOG_WARNING("Wakeword Detection Engine Init Fail!\n");
    }

    am_vos_audio_wwd_enable();
#endif // configUSE_Sensory_THF || configUSE_Cyberon_Spotter || configUSE_RetuneDSP_VS || configUSE_OAL_AID

#if configUSE_DSPC_QSD
    dspc_qsd_config(&(g_sVosSys.sQsdHandle), 16000.0f, AM_SPP_FRAME_SAMPLES);
#endif // configUSE_DSPC_QSD

    while(1)
    {
        am_app_utils_task_read(AM_APP_TASK_AUD_PROCESSING, &QueueElement);

        switch(QueueElement.ui32MessageType)
        {
            case AM_APP_MESSAGE_SHORT:
#if configUSE_PUSH_TO_TALK
                if(QueueElement.info.ui32Note == KEY_WORD_GOT_MESSAGE)
                    am_vos_streaming_push_to_talk();
#endif // configUSE_PUSH_TO_TALK
                break;

            case AM_APP_MESSAGE_LONG:
                am_app_utils_ring_buffer_pop(QueueElement.pDataBuffer, in32LRSample, QueueElement.info.ui32Length);

#if configUSE_WOS && (!configUSE_DSPC_QSD)
                if(g_sVosSys.bWwdEnabled == true)
                {
                    am_vos_wos_check_status();
                }
#endif // configUSE_WOS && (!configUSE_DSPC_QSD)
                break;

            default:
                break;
        }

#if configUSE_DSPC_QSD
        int32_t qsdFlag;
        qsdFlag = dspc_qsd_process_fract16(&(g_sVosSys.sQsdHandle), (short int *)in32LRSample, 2);

        if (qsdFlag || g_sVosSys.ui8KwdDetectedFlag || g_sVosSys.ui8PushTalkFlag || g_sVosSys.ui8ProvideSpeechFlag)
        {
            // Place the samples in the AWE processing buffers
            am_vos_audio_handler(in32LRSample);
            g_sVosSys.ui16QsdOffDelayCnt = QSD_TIMEOUT_COUNT;
            if(g_sVosSys.eQsdStatus != VOS_QSD_NOISY) {
#if USE_QSD_DEBUG
                AM_APP_LOG_INFO("QN\n");
#endif // USE_QSD_DEBUG
                g_sVosSys.eQsdStatus = VOS_QSD_NOISY;
            }
        }
        else
        {
            //
            // QSD needs a delayed turning off
            //
            if(g_sVosSys.ui16QsdOffDelayCnt)
            {
                g_sVosSys.ui16QsdOffDelayCnt--;
                // Place the samples in the sound pre-processing buffers
                am_vos_audio_handler(in32LRSample);
                if(g_sVosSys.eQsdStatus != VOS_QSD_DELAYED) {
#if USE_QSD_DEBUG
                    AM_APP_LOG_INFO("QD\n");
#endif // USE_QSD_DEBUG
                    g_sVosSys.eQsdStatus = VOS_QSD_DELAYED;
                }
            }
            else
            {
                if(g_sVosSys.eQsdStatus == VOS_QSD_DELAYED)
                {
                    AM_APP_LOG_INFO("QQ\n");
                    g_sVosSys.eQsdStatus = VOS_QSD_QUIET;
                    g_sVosSys.ui16QsdTimeoutCnt++;
                    if(g_sVosSys.ui16QsdTimeoutCnt >= QSD_TIMEOUT_ADJ_THRESHOLD)
                    {
                        g_sVosSys.ui16QsdTimeoutCnt = 0;
                        am_vos_qsd_wos_threshold_to_high();
                    }
                }
                  
#if configUSE_WOS
                am_vos_wos_check_status();
#endif // configUSE_WOS
            }
        }
#else // configUSE_DSPC_QSD
        am_vos_audio_handler(in32LRSample);
#endif // configUSE_DSPC_QSD

     }
}

#if configUSE_AUDIO_CODEC
//*****************************************************************************
//
// Audio codec task
//
//*****************************************************************************
void am_vos_codec_task(void *pvParameters)
{
    am_app_utils_task_queue_element_t QueueElement;

    int8_t codecInputBuffer[CODEC_IN_RING_BUFF_SIZE];
    uint8_t codecOutputBuffer[CODEC_OUT_RING_BUFF_SIZE];

    uint32_t codecInBufIndex = 0;
    uint32_t codecInBufRemaining = CODEC_IN_RING_BUFF_SIZE;
    uint32_t codecInBufBytesToPop = 0;

    int8_t *p_CodecInBuf = codecInputBuffer;
    uint8_t *p_CodecOutBuf = codecOutputBuffer;

    int32_t i32CompressedLen;
    uint32_t ui32StreamLen;

    uint8_t ui8EncoderLoopCount = 0;

    while(1)
    {
        am_app_utils_task_read(AM_APP_TASK_CODEC, &QueueElement);

        switch(QueueElement.Source)
        {
            case AM_APP_TASK_AUD_PROCESSING:

                AM_CRITICAL_BEGIN_VOS;
                ui32StreamLen = am_app_utils_get_ring_buffer_status(&(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_MONO]));

                AM_CRITICAL_END_VOS;

                while(ui32StreamLen)
                {
                    //
                    // Attempt to clear mono buffer
                    //
                    if(ui32StreamLen > codecInBufRemaining)
                    {
                        codecInBufBytesToPop = codecInBufRemaining;
                    }
                    else
                    {
                        codecInBufBytesToPop = ui32StreamLen;
                    }

                    am_audio_buffer_nested_pop(AM_AUDIO_BUFFER_MONO, &codecInputBuffer[codecInBufIndex], codecInBufBytesToPop);
                    if(codecInBufRemaining > codecInBufBytesToPop)
                    {
                        codecInBufRemaining -= codecInBufBytesToPop;
                        codecInBufIndex += codecInBufBytesToPop;
                    }
                    else
                    {
                        // equal, or less...though there should not be case for less...
                        // reset index and remaining
                        codecInBufRemaining = CODEC_IN_RING_BUFF_SIZE;
                        codecInBufIndex = 0;
                    }

                    if(codecInBufIndex == 0)
                    {

                        vTaskSuspendAll();
                        ui8EncoderLoopCount++;

#if 0 //(VOS_configUSE_AMU2S_RECORDER && VOS_configUSE_RECORD_FULL_FILTER)
//                        amu2s_send(Amu2s_spp, codecInputBuffer, AMU2S_SPP_BYTES);
#endif //(VOS_configUSE_AMU2S_RECORDER && VOS_configUSE_RECORD_FULL_FILTER)

#if configUSE_MSBC_BLUEZ || configUSE_SBC_BLUEZ
                        am_vos_codec_encode(&(g_sVosSys.sBluezSBCInstance), p_CodecInBuf,
                                                    CODEC_IN_RING_BUFF_SIZE, p_CodecOutBuf, CODEC_OUT_RING_BUFF_SIZE, (int32_t *)&i32CompressedLen);
#endif // configUSE_MSBC_BLUEZ || configUSE_SBC_BLUEZ

#if configUSE_OPTIM_OPUS
                        am_vos_codec_encode(NULL, p_CodecInBuf, CODEC_IN_RING_BUFF_SIZE, p_CodecOutBuf, CODEC_OUT_RING_BUFF_SIZE, &i32CompressedLen);
#endif // configUSE_OPTIM_OPUS
                        am_audio_buffer_nested_push(AM_AUDIO_BUFFER_ENCODED, p_CodecOutBuf, CODEC_OUT_RING_BUFF_SIZE);

                        if(am_audio_universal_buffer_status_check(g_sAmUtil.sRingBuf)==false)
                        {
//                            AM_APP_LOG_DEBUG("There is over-write in universal buffer when pushing to ENCODED!\n\r");
                            AM_APP_LOG_DEBUG("SB RH: %d, SB WT: %d, OV: %d\n",
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_STEREO].ui32BufferHead_read,
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_STEREO].ui32BufferTail_write,
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_STEREO].ui32OverWriting);
                            AM_APP_LOG_DEBUG("MONO RH: %d, WT: %d, OV: %d\n",
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_MONO].ui32BufferHead_read,
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_MONO].ui32BufferTail_write,
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_MONO].ui32OverWriting);
                            AM_APP_LOG_DEBUG("EB RH: %d, EB WT: %d, OV: %d \n",
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_ENCODED].ui32BufferHead_read,
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_ENCODED].ui32BufferTail_write,
                                             g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_ENCODED].ui32OverWriting);
                        }

                        ui8EncoderLoopCount = 0;
                        xTaskResumeAll();

                        if(am_app_utils_get_ring_buffer_status(&(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_ENCODED])) >= BLE_DATA_BUFFER_SIZE)
                        {
#if configUSE_BLE
                            am_vos_ble_stream_send();
#else // configUSE_BLE
                            am_vos_audio_flush_ring_buffer();
#endif // configUSE_BLE
                        }
                    }

                    AM_CRITICAL_BEGIN_VOS;
                    ui32StreamLen = am_app_utils_get_ring_buffer_status(&(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_MONO]));
                    AM_CRITICAL_END_VOS;

                    if((ui32StreamLen < CODEC_IN_RING_BUFF_SIZE)||(ui8EncoderLoopCount > 1))
                    {
                        ui8EncoderLoopCount = 0;
                    }
                }
                break;

            default:
                break;
        }
    }
}
#endif // configUSE_AUDIO_CODEC

//*****************************************************************************
//
// Software timer callback functions
//
//*****************************************************************************
void am_vos_timer_heart_beat_callback(TimerHandle_t xTimer)
{
    am_app_utils_task_send(AM_APP_TASK_NONE, AM_APP_TASK_LED,
                            AM_APP_MESSAGE_SHORT, HEARTBEAT_TIMER_MESSAGE, NULL);
}

#if configUSE_MUTE_MIC
static bool bMicStatus = true; // true = mic on, false = mic off

void am_vos_mute_mic_toggle(void)
{
    // disable interrupt on this pin
    AM_CRITICAL_BEGIN_VOS;
    // button pressed, process to mute or unmute the mic
    if(bMicStatus == true)
    {
        // mics are on, mute them
        //
        // Notes: if the system has push-to-talk, we should keep the
        // functionality of the system, e.g. ble connection
        // Otherwise, we should be able to shut down everything and go to deep sleep
        //

#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
        PDMDeInit();
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

        am_vos_reset_detected_flag();
        am_vos_audio_flush_ring_buffer();

        bMicStatus = false;

        AM_APP_LOG_WARNING("== MICs are muted! ==\n");
    }
    else
    {
#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
        // mics are off, unmute them
        PDMInit();
#if defined (AM_PART_APOLLO2)
        am_hal_pdm_enable();
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
        if(g_sVosSys.pvPDMHandle)
            am_hal_pdm_enable(g_sVosSys.pvPDMHandle);
        pdm_trigger_dma();
#endif // AM_PART_APOLLO2 || AM_PART_APOLLO3 || AM_PART_APOLLO3P
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

        am_vos_reset_detected_flag();
        am_vos_audio_flush_ring_buffer();

        bMicStatus = true;
        AM_APP_LOG_WARNING("== MICs are unmuted! ==\n");
    }
    AM_CRITICAL_END_VOS;
    am_vos_gpio_enable_irq(MUTE_MIC_BUTTON);
}
#endif // configUSE_MUTE_MIC

#if USE_MAYA
#if configUSE_BUZZER
void am_vos_buzzer_task(void *pvParameters)
{
    am_app_utils_task_queue_element_t QueueElement;

    while(1)
    {
        am_app_utils_task_read(AM_APP_TASK_BUZZER, &QueueElement);
        am_app_utils_buzzer_process();
    }
}
#endif // configUSE_BUZZER

void am_vos_logic_task(void *pvParameters)
{
    am_app_utils_task_queue_element_t QueueElement;

    g_sVosSys.eLogicPowerState = APP_LOGIC_POWERING_UP;
//    // power up swirl
//    am_vos_logic_led_swirl(1);

    g_sVosSys.eLogicPowerState = APP_LOGIC_POWER_ON;

    while(1)
    {
        am_app_utils_task_read(AM_APP_TASK_LOGIC, &QueueElement);
#if configUSE_LONG_PRESS_BTN1
        if(logic_check_button_long(MAYA_LOGIC_BUTTON, BTN_LONG_PRESS_PERIOD_MS) >= BTN_LONG_PRESS_PERIOD_MS/BTN_CHECK_CYCLE_MS - 1)
#endif // configUSE_LONG_PRESS_BTN1
        {
            // there is a button press
            // switching off
            g_sVosSys.eLogicPowerState = APP_LOGIC_POWERING_DOWN;
            am_app_utils_buzzer_power_down();
            //AM_APP_LOG_WARNING("== Button Power Down! ==\n");
            am_vos_logic_system_power_off();

            g_sVosSys.eLogicPowerState = APP_LOGIC_POWER_OFF;
            // reinitialize the interrupt
            am_vos_logic_button_init();
  //        while(1)
  //        {
            am_hal_interrupt_master_disable();

            //
            // Only keep flash0 and DTCM0 power on to save power.
            //
            PWRCTRL->MEMPWREN =
                (_VAL2FLD(PWRCTRL_MEMPWREN_DTCM, PWRCTRL_MEMPWREN_DTCM_GROUP0DTCM0) |
                 _VAL2FLD(PWRCTRL_MEMPWREN_FLASH0, PWRCTRL_MEMPWREN_FLASH0_EN));

            SCB->SCR |= _VAL2FLD(SCB_SCR_SLEEPDEEP, 1);
            __DSB();
            #define SYNC_READ       0x5FFF0000

            //
            // CAUTION!!! It is unsafe. Read the value to 0x10001000.
            // The sram only keep 8k size to 0x10002000.
            // It's OK under low level optimization, the behavior may different under high lever optimization.
            //
//            if ( 1 )
//            {
                *((uint32_t*)0x10001000) = *((uint32_t*)SYNC_READ);
                //*(&g_ui32BusWriteFlush) = *pui32Flush;
//            }


            //
            // Execute the sleep instruction.
            //
            __WFI();
            //
            // Upon wake, execute the Instruction Sync Barrier instruction.
            //
            __ISB();

            //
            // Wake up from deep sleep, power on sram all. There must have no function call, or it will invoke stack region
            // that has been cleared by sram power down operation.
            //
            PWRCTRL->MEMPWREN =
                (AM_HAL_PWRCTRL_MEMEN_SRAM_384K    |
                 _VAL2FLD(PWRCTRL_MEMPWREN_FLASH0, PWRCTRL_MEMPWREN_FLASH0_EN));

            //
            // 6 nop delay.
            //
            __asm (
            " nop\n"
            " nop\n"
            " nop\n"
            " nop\n"
            " nop\n"
            " nop\n"
            );

            //
            // clear STIME configuration.
            //
            CTIMER->STCFG = 0;

            //
            // SWPOI software reset.
            //
            RSTGEN->SWPOI =
                _VAL2FLD(RSTGEN_SWPOI_SWPOIKEY, RSTGEN_SWPOI_SWPOIKEY_KEYVALUE);

            while(1);
        }
#if configUSE_LONG_PRESS_BTN1
        else
        {
            am_vos_mute_mic_toggle();
            am_vos_logic_button_init();
        }
#endif // configUSE_LONG_PRESS_BTN1
    }
}
#endif // USE_MAYA

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
void am_vos_gsensor_task(void *pvParameters)
{
    am_app_utils_task_queue_element_t QueueElement;

#if configUSE_OVVP_DOUBLE_TAP
    uint32_t ui32CurTwoTapCnt = 0;
    static unsigned char sensorchip_id;

    sensorchip_id = am_devices_lis2dw12_device_id_get(&g_sACCEL_DW);

    AM_APP_LOG_INFO("[AM-VoS] GSensor ID = 0x%X\n", sensorchip_id);
    if(sensorchip_id != 0x44)   // Check the sensor chip id is for ST lis2dw12
    {
        AM_APP_LOG_WARNING("LIS2DW init fail!!\n");     // sensor error
    }
    am_devices_lis2dw12_config(&g_sACCEL_DW);
    read_acc_run_ovvp();        //Read the sensor and do the OVVP and double click algorithm.

    am_util_id_t sIdDevice;

    am_util_id_device(&sIdDevice);
    AM_APP_LOG_INFO("Vendor Name: %s\n", sIdDevice.pui8VendorName);
    AM_APP_LOG_INFO("Device type: %s\n", sIdDevice.pui8DeviceName);
    AM_APP_LOG_INFO("Device Info:\n\tPart number: 0x%08X\n"
                         "\tRevision: 0x%X (Rev%c%c)\n",
                         sIdDevice.sMcuCtrlDevice.ui32ChipPN,
                         sIdDevice.sMcuCtrlDevice.ui32ChipRev,
                         sIdDevice.ui8ChipRevMaj, sIdDevice.ui8ChipRevMin );
    AM_APP_LOG_INFO("\tChip ID 0: 0x%X\n", sIdDevice.sMcuCtrlDevice.ui32ChipID0);
    AM_APP_LOG_INFO("\tChip ID 1: 0x%X\n", sIdDevice.sMcuCtrlDevice.ui32ChipID1);
    AM_APP_LOG_INFO("\tFlash size: %d\n", sIdDevice.sMcuCtrlDevice.ui32FlashSize);
    AM_APP_LOG_INFO("\tSRAM size: %d\n\n", sIdDevice.sMcuCtrlDevice.ui32SRAMSize);
#endif // configUSE_OVVP_DOUBLE_TAP

    while(1)
    {
        //am_app_utils_gsensor_power_down();
        //am_app_utils_task_suspend(AM_APP_TASK_GSENSOR);
        am_app_utils_task_read(AM_APP_TASK_GSENSOR, &QueueElement);

        switch(QueueElement.Source)
        {
            case AM_APP_ISR_GPIO:
#if configUSE_GSENSOR_MOTION
                am_app_util_gsensor_button2_disble_handler();
#endif // configUSE_GSENSOR_MOTION
#if configUSE_OVVP_DOUBLE_TAP
                //bf_energy_chk();//Use this function without Mic data
                //clk_intv=400;
                clktrilvl = 250;

                read_acc_run_ovvp();    // Read the sensor and do the OVVP and double click algorithm.
                if(two_tap_counter > ui32CurTwoTapCnt)
                {
                    ui32CurTwoTapCnt = two_tap_counter;
                    AM_APP_LOG_INFO("[AM-VoS] Double Tap Detected = %d\n", two_tap_counter);
                    am_app_utils_task_send(AM_APP_TASK_GSENSOR, AM_APP_TASK_LED,
                            AM_APP_MESSAGE_SHORT, DOUBLE_TAP_MESSAGE, NULL);
                }
#endif // configUSE_OVVP_DOUBLE_TAP
                break;

            // FROM itself.
            case AM_APP_TASK_GSENSOR:
#if configUSE_GSENSOR_MOTION
                am_app_util_gsensor_powerdown();
#endif // configUSE_GSENSOR_MOTION
                break;

            default:
                break;

        }
    }
}
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP

//*****************************************************************************
//
// Simple button detection function with debounce
//
//*****************************************************************************
static bool am_vos_check_button_gpio(uint32_t gpio)
{
    uint64_t pinVal = 0;

    //
    // Debounce for 100 ms.
    // We're triggered for rising edge - so we expect a consistent HIGH here
    //
#if defined (AM_PART_APOLLO2)
    for(uint8_t i = 0; i < 5 ; i++)
    {
        am_util_delay_ms(20);
        pinVal = (am_hal_gpio_input_read() & AM_HAL_GPIO_BIT(gpio));

        if(!pinVal)
        {
            return false;
        }
    }
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    for(uint8_t i = 0; i < 5 ; i++)
    {
        am_util_delay_ms(20);
        pinVal = am_hal_gpio_input_read(gpio);

        if(!pinVal)
        {
            return false;
        }
    }
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

    return true;
}

void am_vos_push_to_talk_process(void)
{
    // disable interrupt on this pin
    am_vos_gpio_disable_irq(PUSH_TO_TALK_BUTTON);

    if(am_vos_check_button_gpio(PUSH_TO_TALK_BUTTON))
    {
        // button pressed, send a msg to trigger push to talk
        am_app_utils_task_send_fromISR(AM_APP_ISR_GPIO, AM_APP_TASK_AUD_PROCESSING, AM_APP_MESSAGE_SHORT, KEY_WORD_GOT_MESSAGE, NULL);
    }
}

#if 0
static bool bMicStatus = true; // true = mic on, false = mic off

bool bLongPressStatus = false;  // button not pressed
uint16_t ui16LongPressCount = 0;

void mute_mic_long_press_handler(void)
{
    if(bLongPressStatus == false) // enter from ISR
    {
        if(am_vos_check_button_gpio(MUTE_MIC_BUTTON))
        {
            am_vos_gpio_disable_irq(MUTE_MIC_BUTTON);

            // pressed, start a 100ms timer
            xTimerStartFromISR(am_KWD_timers[AM_APP_TIMER_LONG_PRESS], 0);
            xTimerChangePeriodFromISR(am_KWD_timers[AM_APP_TIMER_LONG_PRESS], pdMS_TO_TICKS(LONG_PRESS_TIMER_PERIOD_MS), 0);
            bLongPressStatus = true;
            ui16LongPressCount++;
        }
    }
    else // enter from timer callback
    {
        ui16LongPressCount++;
        if(ui16LongPressCount > (LONG_PRESS_EFFECTIVE_MS/LONG_PRESS_TIMER_PERIOD_MS))
        {
            // long press over effective period, take action
            AM_APP_LOG_WARNING("== Long press effective! ==\n");
            am_vos_mute_mic_process();

            ui16LongPressCount = 0;
            bLongPressStatus = false;

            am_vos_gpio_enable_irq(MUTE_MIC_BUTTON);
        }
        else
        {
            // keep counting
            if(mute_mic_check_button(MUTE_MIC_BUTTON))
            {
                // pressed, start a 100ms timer
                xTimerStartFromISR(am_KWD_timers[AM_APP_TIMER_LONG_PRESS], 0);
                xTimerChangePeriodFromISR(am_KWD_timers[AM_APP_TIMER_LONG_PRESS], pdMS_TO_TICKS(LONG_PRESS_TIMER_PERIOD_MS), 0);
            }
            else
            {
                // button released
                AM_APP_LOG_WARNING("== Button released! pressed for %d ms ==\n", (ui16LongPressCount*LONG_PRESS_TIMER_PERIOD_MS));

                ui16LongPressCount = 0;
                bLongPressStatus = false;

                am_vos_gpio_enable_irq(MUTE_MIC_BUTTON);
            }
        }
    }
}

void am_vos_timer_longpress_callback(TimerHandle_t xTimer)
{
    mute_mic_long_press_handler();
}
#endif // #if 0

#if configUSE_MUTE_MIC
void am_vos_mute_mic_process(void)
{
    am_vos_gpio_disable_irq(MUTE_MIC_BUTTON);

    if(am_vos_check_button_gpio(MUTE_MIC_BUTTON))
    {
        // button pressed, send a msg to trigger push to talk
        am_app_utils_task_send_fromISR(AM_APP_ISR_GPIO, AM_APP_TASK_LED, AM_APP_MESSAGE_SHORT, MUTE_MIC_MESSAGE, NULL);
    }
    else
    {
        am_vos_gpio_enable_irq(MUTE_MIC_BUTTON);
    }
}
#endif // configUSE_MUTE_MIC

//*****************************************************************************
//
// WOS ISR
//
//*****************************************************************************
#if configUSE_WOS
void
am_vos_wos_handler(void)
{
//    if(g_sVosSys.i16WosActiveFlag <= 0)
//    {
        AM_APP_LOG_INFO("WN\n");

        am_vos_gpio_disable_irq(WOS_WAKE_PIN);
        am_vos_wos_mic_enable();

        g_sVosSys.ui8WoSDetectFlag = 1;
#if configUSE_OVVP_DOUBLE_TAP
        am_vos_gpio_enable_irq(LIS2DW_INT_PIN);
#endif // configUSE_OVVP_DOUBLE_TAP
//    }
        if(g_sVosSys.ui8WosStatus == 1)
        {
            g_sVosSys.i32WosActiveCount = WOS_TIMEOUT_COUNT;
            g_sVosSys.ui8WosActiveFlag = 1;
        }
        else if(g_sVosSys.ui8WosStatus == 0)
        {
            g_sVosSys.i32WosActiveCount = 0;
            g_sVosSys.ui8WosActiveFlag = 0;
        }
}

//static uint8_t curPin = 0;

void
am_vos_wos_check_status(void)
{
#if configUSE_Enhanced_WoS
#if defined (AM_PART_APOLLO2)
    uint64_t ui64PinState = 0;

    ui64PinState = am_hal_gpio_input_read();

#if (!USE_DMIC_MB3_VM3011)
    if(!((ui64PinState >> WOS_MODE_PIN) & 0x01 ))  //If VM1010 is in normal mode
    {
        am_hal_gpio_out_bit_set(WOS_MODE_PIN); //set high to enable WoS mode
        return;
    }
#endif // (!USE_DMIC_MB3_VM3011)

//    if (g_sVosSys.i16WosActiveFlag % WOS_TIMEOUT_SKIP)  //Only perform WoS check once every WOS_TIMEOUT_SKIP cycles
//    {
//        g_sVosSys.i16WosActiveFlag--;
//        return;
//    }
#if (!USE_DMIC_MB3_VM3011)
    if((ui64PinState >> WOS_WAKE_PIN) & 0x01 )
    {
        g_sVosSys.i32WosActiveCount = ((WOS_TIMEOUT_COUNT / WOS_TIMEOUT_SKIP) * WOS_TIMEOUT_SKIP  - 1);
        am_hal_gpio_out_bit_clear(WOS_MODE_PIN); //set low to enter normal mode
        return;
    }
#endif // (!USE_DMIC_MB3_VM3011)
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    uint32_t ui32PinState = 0;

#if (!USE_DMIC_MB3_VM3011)
    am_hal_gpio_state_read(WOS_MODE_PIN, AM_HAL_GPIO_INPUT_READ, &ui32PinState);

    if(!ui32PinState)
    {
        am_hal_gpio_state_write(WOS_MODE_PIN, AM_HAL_GPIO_OUTPUT_SET);        //set high to enable WoS mode
        return;
    }
#endif // (!USE_DMIC_MB3_VM3011)

//    if (g_sVosSys.i16WosActiveFlag % WOS_TIMEOUT_SKIP)  //Only perform WoS check once every WOS_TIMEOUT_SKIP cycles
//    {
//        g_sVosSys.i16WosActiveFlag--;
//        return;
//    }

    am_hal_gpio_state_read(WOS_WAKE_PIN, AM_HAL_GPIO_INPUT_READ, &ui32PinState);

#if (!USE_DMIC_MB3_VM3011)
    if(ui32PinState)
    {
        g_sVosSys.i32WosActiveCount = ((WOS_TIMEOUT_COUNT / WOS_TIMEOUT_SKIP) * WOS_TIMEOUT_SKIP  - 1);
        am_hal_gpio_state_write(WOS_MODE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);        //set low to enter normal mode
        return;
    }
#endif // (!USE_DMIC_MB3_VM3011)
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
#endif // configUSE_Enhanced_WoS

//    uint32_t ui32PinState = 0;
//
//    am_hal_gpio_state_read(WOS_WAKE_PIN, AM_HAL_GPIO_INPUT_READ, &ui32PinState);
//    if(ui32PinState)
//    {
//        if(!curPin)
//        {
//            curPin = 1;
//            AM_APP_LOG_INFO("1 ");
//        }
//    }
//    else
//    {
//        if(curPin)
//        {
//            curPin = 0;
//            AM_APP_LOG_INFO("0 ");
//        }
//    }

    if((g_sVosSys.i32WosActiveCount > 0) && (g_sVosSys.ui8WosActiveFlag == 1))
    {
        g_sVosSys.i32WosActiveCount--;
    }
    else if((g_sVosSys.i32WosActiveCount <= 0) && (g_sVosSys.ui8WosActiveFlag == 1))
    {
        AM_APP_LOG_INFO("WQ\n");
        //
        // enter into quiet status
        //
        g_sVosSys.ui8WosStatus = 1;
#if configUSE_OVVP_DOUBLE_TAP
        am_vos_gpio_disable_irq(LIS2DW_INT_PIN);
#endif // configUSE_OVVP_DOUBLE_TAP

        am_vos_wos_mic_disable();
        am_vos_gpio_enable_irq(WOS_WAKE_PIN);
        g_sVosSys.i32WosActiveCount = 0;
        g_sVosSys.ui8WosActiveFlag = 0;
    }
}
#endif // configUSE_WOS
