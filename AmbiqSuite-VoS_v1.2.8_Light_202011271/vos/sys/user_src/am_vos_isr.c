//*****************************************************************************
//
//! @file am_app_KWD_isr.c
//!
//! @brief ISR functions of KWD application
//!
//! These functions are required by the RTOS for ticking, sleeping, and basic
//! error checking.
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
#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include "hci_api.h"

#include "am_app_utils.h"
#include "am_app_utils_task.h"

#include "am_vos_task.h"
#include "am_vos_init.h"
#include "am_vos_spp.h"
#include "am_vos_ble.h"

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
#include "hci_drv_apollo3.h"

#if USE_MAYA
#include "am_vos_logic.h"

#if configUSE_BUZZER
#include "am_app_utils_buzzer.h"
#endif // configUSE_BUZZER
#endif // USE_MAYA

#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

#if configUSE_PCM_HP_FILTER
#include "am_app_digital_filter.h"
#endif // configUSE_PCM_HP_FILTER

#if configUSE_RTT_RECORDER
#include "am_vos_rtt_recorder.h"
#endif // configUSE_RTT_RECORDER

#if configUSE_AMU2S_RECORDER
#include "am_devices_amu2s.h"
#endif // configUSE_AMU2S_RECORDER

#if configUSE_OVVP_DOUBLE_TAP
#include "am_devices_lis2dw12.h"
#endif // configUSE_OVVP_DOUBLE_TAP

#if USE_DMIC_MB3_VM3011 || configUSE_WOS
#include "am_devices_vm3011.h"
#endif // USE_DMIC_MB3_VM3011 || configUSE_WOS

#if defined (AM_PART_APOLLO2)
#include "amvos_api.h"
#endif // AM_PART_APOLLO2

//*****************************************************************************
//
// Interrupt handler for the CTIMER module.
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
#if configUSE_SYSVIEWER
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER

    uint32_t ui32Status;
    //
    // Check the timer interrupt status.
    //
    ui32Status = am_hal_ctimer_int_status_get(false);
    am_hal_ctimer_int_clear(ui32Status);
#if USE_MAYA && configUSE_BUZZER
    if(ui32Status & BUZZER_PWM_TIMER_INT)
    {
        am_app_utils_buzzer_interrupt_routine();
    }
#endif // USE_MAYA && configUSE_BUZZER

    //
    // Run handlers for the various possible timer events.
    //
    am_hal_ctimer_int_service(ui32Status);

#if configUSE_SYSVIEWER
    traceISR_EXIT();
#endif // configUSE_SYSVIEWER
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
am_gpio_isr(void)
{
#if defined (AM_PART_APOLLO2)

    uint64_t ui64Status;

    //
    // Read and clear the GPIO interrupt status.
    //
    ui64Status = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64Status);

#if configUSE_SYSVIEWER
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER

#if configUSE_BLE
    //
    // Check to see if this was a wakeup event falserom the BLE radio.
    //
    if ( ui64Status & AM_HAL_GPIO_BIT(AM_BSP_GPIO_EM9304_INT) )
    {
        am_vos_ble_data_ready_evt_set();
    }
#endif // configUSE_BLE

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    if((ui64Status >> LIS2DW_INT_PIN) & 0x01 )
    {
        am_app_utils_task_send_fromISR(AM_APP_ISR_GPIO, AM_APP_TASK_GSENSOR, AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);
    }
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    //
    // Call the individual pin interrupt handlers for any pin that triggered an
    // interrupt.
    //
    am_hal_gpio_int_service(ui64Status);
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3)

    uint64_t ui64Status;

    //
    // Read and clear the GPIO interrupt status.
    //
    am_hal_gpio_interrupt_status_get(false, &ui64Status);
    am_hal_gpio_interrupt_clear(ui64Status);

#if USE_MAYA
    if((ui64Status >> MAYA_LOGIC_BUTTON) & 0x01 )
    {
        am_vos_logic_button_process();
    }
#endif // USE_MAYA

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    if((ui64Status >> LIS2DW_INT_PIN) & 0x01 )
    {
        am_app_utils_task_send_fromISR(AM_APP_ISR_GPIO, AM_APP_TASK_GSENSOR, AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);
    }
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    
    //
    // Call the individual pin interrupt handlers for any pin that triggered an
    // interrupt.
    //
    am_hal_gpio_interrupt_service(ui64Status);
#if configUSE_SYSVIEWER
    traceISR_EXIT();
#endif // configUSE_SYSVIEWER
#endif // AM_PART_APOLLO3
#if defined (AM_PART_APOLLO3P)
    AM_HAL_GPIO_MASKCREATE(GpioIntStatusMask);

    am_hal_gpio_interrupt_status_get(false, pGpioIntStatusMask);
    am_hal_gpio_interrupt_clear(pGpioIntStatusMask);

    //
    // Call the individual pin interrupt handlers for any pin that triggered an
    // interrupt.
    //
    am_hal_gpio_interrupt_service(pGpioIntStatusMask);
#endif // AM_PART_APOLLO3P
}

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
//*****************************************************************************
//
// Interrupt handler for BLE
//
//*****************************************************************************
void
am_ble_isr(void)
{
#if configUSE_SYSVIEWER
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER
    HciDrvIntService();

    // Signal radio task to run

    WsfTaskSetReady(0, 0);
#if configUSE_SYSVIEWER
    traceISR_EXIT();
#endif // configUSE_SYSVIEWER
}
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

//*****************************************************************************
//
// PDM Interrupt Service Routine (ISR)
//
//*****************************************************************************
#if defined (AM_PART_APOLLO2) && (USE_DMIC_MB3 || USE_DMIC_MB3_VM3011)
void am_pdm_isr(void)
{
    uint32_t ui32Status;
    uint32_t ui32FIFOCount;
    ui32Status = am_hal_pdm_int_status_get(false);
    uint16_t sampleIndex = 0;
    int32_t i32PDMSampleStereo[PCM_FRAME_SIZE];

#if configUSE_SYSVIEWER 
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER
    //
    // Grab the FIFO depth
    //
    ui32FIFOCount = am_hal_pdm_fifo_depth_read();

    //
    // When Threshold causes ISR, Grab samples from packed FIFO and make sure we don't underflow
    //
    if((ui32Status == AM_HAL_PDM_INT_FIFO) && (ui32FIFOCount >= PCM_FRAME_SIZE))
    {
        for (sampleIndex = 0; sampleIndex < PCM_FRAME_SIZE; sampleIndex ++)
        {
            i32PDMSampleStereo[sampleIndex] = (int32_t)am_hal_pdm_fifo_data_read();
#if configUSE_OVVP_DOUBLE_TAP			
            sll = i32PDMSampleStereo[sampleIndex] >> 16;             // Right channel
            bf_energy();
#endif // configUSE_OVVP_DOUBLE_TAP
        }
#if configUSE_WOS && USE_DMIC_MB3_VM3011
        if(g_sVosSys.ui8WoSDetectFlag)
        {
            g_sVosSys.ui8WoSDetectFlag = 0;
            memset(i32PDMSampleStereo, 0, (g_ui32WosDiscardTimeUS/1000)*(AM_SPP_SAMPLE_FREQ/1000)*SAMPLE_32BIT);
        }
        else
        {
            am_audio_buffer_push(AM_AUDIO_BUFFER_STEREO, i32PDMSampleStereo, PCM_FRAME_SIZE * SAMPLE_32BIT);
            am_app_utils_task_send_fromISR(AM_APP_ISR_PDM, AM_APP_TASK_AUD_PROCESSING, AM_APP_MESSAGE_LONG, SAMPLE_32BIT*PCM_FRAME_SIZE, &(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_STEREO]));
        }
#else // configUSE_WOS && USE_DMIC_MB3_VM3011
        am_audio_buffer_push(AM_AUDIO_BUFFER_STEREO, i32PDMSampleStereo, PCM_FRAME_SIZE * SAMPLE_32BIT);
        am_app_utils_task_send_fromISR(AM_APP_ISR_PDM, AM_APP_TASK_AUD_PROCESSING, AM_APP_MESSAGE_LONG, SAMPLE_32BIT*PCM_FRAME_SIZE, &(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_STEREO]));
#endif // configUSE_WOS && USE_DMIC_MB3_VM3011

#if configUSE_RTT_RECORDER && configUSE_RECORD_RAW_PCM
        //
        // Record the raw PCM data and send over RTT
        //
        if(g_sVosSys.ui8RecordStartFlag == 1)
            am_vos_rtt_record((void*)i32PDMSampleStereo, PCM_FRAME_SIZE * SAMPLE_32BIT); 
#endif // configUSE_RTT_RECORDER && configUSE_RECORD_RAW_PCM

#if configUSE_AMU2S_RECORDER && configUSE_RECORD_RAW_PCM
        if(g_sVosSys.ui8RecordStartFlag == 1)
            amu2s_send(Amu2s_pcm, i32PDMSampleStereo, PCM_FRAME_SIZE * SAMPLE_32BIT);
#endif // configUSE_AMU2S_RECORDER && configUSE_RECORD_RAW_PCM
    }
    if(ui32Status & (AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF))
    {
        am_hal_pdm_fifo_flush(); 
        AM_APP_LOG_DEBUG("am_hal_pdm_fifo_flush();\n");
    }
    // Clear PDM Interrupt (write to clear).
    am_hal_pdm_int_clear(AM_HAL_PDM_INT_FIFO | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF);

#if configUSE_SYSVIEWER
    traceISR_EXIT();
#endif // configUSE_SYSVIEWER
}
#endif // AM_PART_APOLLO2 && (USE_DMIC_MB3 || USE_DMIC_MB3_VM3011)

#if (defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)) && (USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011)
void am_pdm0_isr(void)
{
    uint32_t ui32Status;
    //bool bQueueReValue;

#if configUSE_SYSVIEWER
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER

    //
    // Read the interrupt status.
    //
    am_hal_pdm_interrupt_status_get(g_sVosSys.pvPDMHandle, &ui32Status, true);
    am_hal_pdm_interrupt_clear(g_sVosSys.pvPDMHandle, ui32Status);

    // Test code for PDM wakeup time measurement
    //am_hal_gpio_state_write(LED_SYSTEM, AM_HAL_GPIO_OUTPUT_TOGGLE);
    //
    // Once our DMA transaction completes, we will disable the PDM and send a
    // flag back down to the main routine. Disabling the PDM is only necessary
    // because this example only implemented a single buffer for storing FFT
    // data. More complex programs could use a system of multiple buffers to
    // allow the CPU to run the FFT in one buffer while the DMA pulls PCM data
    // into another buffer.
    //
    if (ui32Status & AM_HAL_PDM_INT_DCMP)
    {
        // trigger next traction
        PDMn(0)->DMATOTCOUNT = AM_SPP_FRAME_SAMPLES * SAMPLE_32BIT;  // FIFO unit in bytes

#if configUSE_OVVP_DOUBLE_TAP
        for (int i = 0; i < PCM_FRAME_SIZE; i++)
        {
            sll = g_sVosSys.ui32PDMDataBuf[i] >> 16;             // Right channel
            bf_energy();
        }
#endif // configUSE_OVVP_DOUBLE_TAP

#if configUSE_WOS
        if(g_sVosSys.ui8WoSDetectFlag)
        {
            g_sVosSys.ui8WoSDetectFlag = 0;
            memset(g_sVosSys.ui32PDMDataBuf, 0, (g_ui32WosDiscardTimeUS/1000)*(AM_SPP_SAMPLE_FREQ/1000)*SAMPLE_32BIT);
        }
        else
        {
            am_audio_buffer_push(AM_AUDIO_BUFFER_STEREO, g_sVosSys.ui32PDMDataBuf, PCM_FRAME_SIZE * SAMPLE_32BIT);
            am_app_utils_task_send_fromISR(AM_APP_ISR_PDM, AM_APP_TASK_AUD_PROCESSING, AM_APP_MESSAGE_LONG, SAMPLE_32BIT*PCM_FRAME_SIZE, &(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_STEREO]));
        }
#else // configUSE_WOS
        am_audio_buffer_push(AM_AUDIO_BUFFER_STEREO, g_sVosSys.ui32PDMDataBuf, PCM_FRAME_SIZE * SAMPLE_32BIT);
        am_app_utils_task_send_fromISR(AM_APP_ISR_PDM, AM_APP_TASK_AUD_PROCESSING, AM_APP_MESSAGE_LONG, SAMPLE_32BIT*PCM_FRAME_SIZE, &(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_STEREO]));
#endif // configUSE_WOS

#if configUSE_RTT_RECORDER && configUSE_RECORD_RAW_PCM
        //
        // Record the raw PCM data and send over RTT
        //
        if(g_sVosSys.ui8RecordStartFlag == 1)
           am_vos_rtt_record((void*)g_sVosSys.ui32PDMDataBuf, PCM_FRAME_SIZE * SAMPLE_32BIT); 
#endif // configUSE_RTT_RECORDER && configUSE_RECORD_RAW_PCM
#if configUSE_AMU2S_RECORDER && configUSE_RECORD_RAW_PCM
        if(g_sVosSys.ui8RecordStartFlag == 1)
            amu2s_send(Amu2s_pcm, g_sVosSys.ui32PDMDataBuf, PCM_FRAME_SIZE * SAMPLE_32BIT);
#endif // configUSE_AMU2S_RECORDER && configUSE_RECORD_RAW_PCM
    }
    else if(ui32Status & (AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF))
    {
        am_hal_pdm_fifo_flush(g_sVosSys.pvPDMHandle);
        AM_APP_LOG_DEBUG("am_hal_pdm_fifo_flush();\n");
    }

    //
    // When Threshold causes ISR, Grab samples from packed FIFO and make sure we don't underflow
    //
#if configUSE_SYSVIEWER
    traceISR_EXIT();	// Should be comment out when it is Apollo 3
#endif // configUSE_SYSVIEWER

}
#endif // (AM_PART_APOLLO3 || AM_PART_APOLLO3P) && (USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011)

#if USE_OUTPUT_BUFFER_UART1
//*****************************************************************************
//
// Interrupt handler for UART1
//
//*****************************************************************************
void
am_uart1_isr(void)
{
#if configUSE_SYSVIEWER
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER
    BaseType_t xHigherPriorityTaskWoken;
    uint32_t ui32Status;

    //
    // Read and save the interrupt status, but clear out the status register.
    //
    ui32Status = AM_REGn(UART, 1, MIS);
    AM_REGn(UART, 1, IEC) = ui32Status;

    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_TX | AM_HAL_UART_INT_RX))
    {
        am_hal_uart_service_buffered_timeout_save(UART1_MODULE, ui32Status);
    }
    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_RX))
    {
        g_bUARTPacketReceived = true;
    }
    // set event flag to unblock tasks
//    xEventGroupSetBitsFromISR(xKWDEventHandle, 0x04, &xHigherPriorityTaskWoken);

#if configUSE_SYSVIEWER
    traceISR_EXIT();
#endif // configUSE_SYSVIEWER
}
#endif // USE_OUTPUT_BUFFER_UART1

//*****************************************************************************
//
// Interrupt handler for UART0
//
//*****************************************************************************
#if defined (AM_PART_APOLLO2)
void
am_uart_isr(void)
{
#if configUSE_SYSVIEWER
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER
    uint32_t ui32Status;

    //
    // Read and save the interrupt status, but clear out the status register.
    //
    ui32Status = AM_REGn(UART, 0, MIS);
    AM_REGn(UART, 0, IEC) = ui32Status;

    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_TX | AM_HAL_UART_INT_RX))
    {
        am_hal_uart_service_buffered(UART0_MODULE, ui32Status);
    }
    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_RX))
    {
        g_sVosSys.bUARTPacketReceived = true;
    }

#if configUSE_SYSVIEWER
    traceISR_EXIT();
#endif // configUSE_SYSVIEWER
}
#endif // AM_PART_APOLLO2

#if 0 // refer to uart_task.c
void
am_uart_isr(void)
{
#if configUSE_SYSVIEWER
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(g_sVosSys.pvUartHandle, &ui32Status, true);
    am_hal_uart_interrupt_clear(g_sVosSys.pvUartHandle, ui32Status);
    am_hal_uart_interrupt_service(g_sVosSys.pvUartHandle, ui32Status, &ui32Idle);

#if configUSE_SYSVIEWER
    traceISR_EXIT();
#endif // configUSE_SYSVIEWER
}
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

#if USE_APOLLO3_BLUE_EVB && (USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB)
void am_adc_isr(void)
{
    uint32_t ui32IntMask;
    uint32_t ui32AdcPcmDataBuffer[ADC_SAMPLE_COUNT];
    int16_t in16PcmCh1Val = 0;
    int16_t in16PcmCh2Val = 0;
    
    uint8_t indx= 0;
#if configUSE_SYSVIEWER
    traceISR_ENTER();
#endif // configUSE_SYSVIEWER

    //
    // Read the interrupt status.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_interrupt_status(g_sVosSys.pvADCHandle, &ui32IntMask, false))
    {
        am_util_stdio_printf("Error reading ADC interrupt status\n");
    }

    //
    // Clear the ADC interrupt.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_interrupt_clear(g_sVosSys.pvADCHandle, ui32IntMask))
    {
        am_util_stdio_printf("Error clearing ADC interrupt status\n");
    }

    //
    // If we got a DMA complete, set the flag.
    //
    if (ui32IntMask & AM_HAL_ADC_INT_DCMP)
    {
        //
        // Re-configure the ADC DMA.
        //
        adc_config_dma();
#if configUSE_PCM_HP_FILTER

        for(indx=0; indx<ADC_SAMPLE_COUNT; indx++)
        {
            in16PcmCh1Val = ((g_sVosSys.ui32ADCSampleBuffer[indx * ADC_MIC_CHANNELS] >> 6) & 0x00003FFF)-8192;    //0.75: 8192
            in16PcmCh2Val = ((g_sVosSys.ui32ADCSampleBuffer[indx * ADC_MIC_CHANNELS+1] >> 6) & 0x00003FFF)-8192;    //0.75: 8192
            g_sVosSys.pfAmicLeftArray[indx] = (float)in16PcmCh1Val;
            g_sVosSys.pfAmicRightArray[indx] = (float)in16PcmCh2Val;
        }
        high_pass_filter_filterBlock(&(g_sVosSys.sAmicLeftHPFilter), g_sVosSys.pfAmicLeftArray, g_sVosSys.pfAmicLeftFilteredBuff, ADC_SAMPLE_COUNT);
        high_pass_filter_filterBlock(&(g_sVosSys.sAmicRightHPFilter), g_sVosSys.pfAmicRightArray, g_sVosSys.pfAmicRightFilteredBuff, ADC_SAMPLE_COUNT);

        for(indx=0; indx<ADC_SAMPLE_COUNT; indx++)
        {

            ui32AdcPcmDataBuffer[indx] = 0;
            in16PcmCh1Val = (int16_t)(g_sVosSys.pfAmicLeftFilteredBuff[indx]);
            in16PcmCh2Val = (int16_t)(g_sVosSys.pfAmicRightFilteredBuff[indx]);
            ui32AdcPcmDataBuffer[indx] = (((uint16_t)in16PcmCh1Val) << 16) | ((uint16_t)in16PcmCh2Val);
        }
#else // configUSE_PCM_HP_FILTER
        for(indx=0; indx<ADC_SAMPLE_COUNT; indx++)
        {
            ui32AdcPcmDataBuffer[indx] = 0;
#if (ADC_MIC_CHANNELS == 1)
            in16PcmCh1Val = ((g_sVosSys.ui32ADCSampleBuffer[indx] >> 6) & 0x00003FFF)-8192;     // 0.75: 8192
            ui32AdcPcmDataBuffer[indx] = (((uint16_t)in16PcmCh1Val) << 16);             // Single channel is right as a default.
#elif (ADC_MIC_CHANNELS == 2)
            in16PcmCh1Val = ((g_sVosSys.ui32ADCSampleBuffer[indx * ADC_MIC_CHANNELS] >> 6) & 0x00003FFF)-8192;    // 0.75: 8192
            in16PcmCh2Val = ((g_sVosSys.ui32ADCSampleBuffer[indx * ADC_MIC_CHANNELS+1] >> 6) & 0x00003FFF)-8192;    // 0.75: 8192
            ui32AdcPcmDataBuffer[indx] = (((uint16_t)in16PcmCh1Val) << 16) | ((uint16_t)in16PcmCh2Val);
#endif // ADC_MIC_CHANNELS == 1, ADC_MIC_CHANNELS == 2
        }
#endif // configUSE_PCM_HP_FILTER
        am_audio_buffer_push(AM_AUDIO_BUFFER_STEREO, ui32AdcPcmDataBuffer, ADC_SAMPLE_COUNT*ANALOG_MIC_DATA_BYTES);
        am_app_utils_task_send_fromISR(AM_APP_ISR_ADC, AM_APP_TASK_AUD_PROCESSING, AM_APP_MESSAGE_LONG, SAMPLE_32BIT*PCM_FRAME_SIZE, &(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_STEREO]));
#if configUSE_RTT_RECORDER && configUSE_RECORD_RAW_PCM
        if(g_sVosSys.ui8RecordStartFlag == 1)
        {
            am_vos_rtt_record((void*)ui32AdcPcmDataBuffer, ADC_SAMPLE_COUNT*ANALOG_MIC_DATA_BYTES);
        }
#endif // configUSE_RTT_RECORDER && configUSE_RECORD_RAW_PCM
#if configUSE_AMU2S_RECORDER && configUSE_RECORD_RAW_PCM
        if(g_sVosSys.ui8RecordStartFlag == 1)
        {
            amu2s_send(Amu2s_pcm, ui32AdcPcmDataBuffer, AMU2S_PCM_BYTES);
        }
#endif // configUSE_AMU2S_RECORDER && configUSE_RECORD_RAW_PCM
    }

    //
    // If we got a DMA error, set the flag.
    //
    if (ui32IntMask & AM_HAL_ADC_INT_DERR)
    {
        g_sVosSys.bADCDMAError = true;
    }

#if configUSE_SYSVIEWER
    traceISR_EXIT();
#endif // configUSE_SYSVIEWER

}
#endif // USE_APOLLO3_BLUE_EVB && (USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB)

//*****************************************************************************
//
// Sleep function called from FreeRTOS IDLE task.
// Do necessary application specific Power down operations here
// Return 0 if this function also incorporates the WFI, else return value same
// as idleTime
//
//*****************************************************************************
uint32_t am_freertos_sleep(uint32_t idleTime)
{
#if (configUSE_WOS && USE_DMIC_MB3_VM3011)
    am_vos_wos_debug_gpio_set(g_ui32SleepGpioNum);
#endif
    
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

#if (configUSE_WOS && USE_DMIC_MB3_VM3011)
    am_vos_wos_debug_gpio_clear(g_ui32SleepGpioNum);
#endif
    return 0;
}

//*****************************************************************************
//
// Recovery function called from FreeRTOS IDLE task, after waking up from Sleep
// Do necessary 'wakeup' operations here, e.g. to power up/enable peripherals etc.
//
//*****************************************************************************
void am_freertos_wakeup(uint32_t idleTime)
{
    return;
}


//*****************************************************************************
//
// FreeRTOS debugging functions.
//
//*****************************************************************************
void
vApplicationMallocFailedHook(void)
{
    //
    // Called if a call to pvPortMalloc() fails because there is insufficient
    // free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    // internally by FreeRTOS API functions that create tasks, queues, software
    // timers, and semaphores.  The size of the FreeRTOS heap is set by the
    // configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
    //
    while (1);
}

void
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    //
    // Run time stack overflow checking is performed if
    // configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    // function is called if a stack overflow is detected.
    //
    while (1)
    {
        __asm("BKPT #0\n") ; // Break into the debugger
    }
}

