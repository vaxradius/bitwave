//*****************************************************************************
//
//! @file radio_task.c
//!
//! @brief Task to handle radio operation.
//!
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
// This is part of revision 1.2.11 of the AmbiqSuite Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
// Global includes for this project.
//
//*****************************************************************************
#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

//*****************************************************************************
//
// WSF standard includes.
//
//*****************************************************************************
#include "wsf_types.h"
#include "wsf_buf.h"

//*****************************************************************************
//
// Includes for operating the ExactLE stack.
//
//*****************************************************************************
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_drv_apollo.h"

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
#include "hci_drv_apollo3.h"
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_bsp.h"

//*****************************************************************************
//
// Includes for the VoS.
//
//*****************************************************************************
#include "am_app_utils.h"

#include "am_vos_task.h"
#include "am_vos_init.h"
#include "am_vos_audio.h"
#include "am_vos_codec.h"
#include "am_vos_ble.h"

#include "amvos_api.h"
#include "app_ui.h"

#if configUSE_AMVOS_AMA
#include "am_vos_ama.h"
#endif // configUSE_AMVOS_AMA

//*****************************************************************************
//
// WSF buffer pools.
//
//*****************************************************************************
#define WSF_BUF_POOLS               (4 + 2)

// Important note: the size of g_pui32BufMem should includes both overhead of internal
// buffer management structure, wsfBufPool_t (up to 16 bytes for each pool), and pool
// description (e.g. g_psPoolDescriptors below).

// Memory for the buffer pool
// extra AMOTA_PACKET_SIZE bytes for OTA handling
static uint32_t g_pui32BufMem[(1024 + 512 + 520*4) / sizeof(uint32_t)];
//static uint32_t g_pui32BufMem[(WSF_BUF_POOLS*16 + 16*8 + 32*4 + 64*6 + 128*2 + 256*2 + 520*4) / sizeof(uint32_t)];

// Default pool descriptor.
static wsfBufPoolDesc_t g_psPoolDescriptors[WSF_BUF_POOLS] =
{
    {  16,  8 },
    {  32,  4 },
    {  64,  6 },
    { 128,  2 },
    { 256,  2 },
    { 520,  4 },        //517 bytes buffer for 496bytes of MTU
};

#if defined (AM_PART_APOLLO2)
void am_vos_ble_data_ready_evt_set(void)
{
    WsfSetEvent(g_sVosBle.hBleDataReadyHandlerId, BLE_DATA_READY_EVENT);
    return;
}

//*****************************************************************************
//
// Tracking variable for the scheduler timer.
//
//*****************************************************************************
void am_vos_data_ready_handler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if (FALSE == HciDataReadyISR())
    {
        // trigger event again to handle pending data from BLE controller
        am_vos_ble_data_ready_evt_set();
    }
}

//*****************************************************************************
//
// Interrupt handler for the CTS pin
//
//*****************************************************************************
void
am_vos_radio_cts_handler(void)
{
    // Signal radio task to run
    WsfTaskSetReady(0,0);
}
#endif // AM_PART_APOLLO2

//*****************************************************************************
//
// Initialization for the ExactLE stack.
//
//*****************************************************************************
void
exactle_stack_init(void)
{
    wsfHandlerId_t handlerId;
#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    uint16_t       wsfBufMemLen;
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
    //
    // Set up timers for the WSF scheduler.
    //
    WsfOsInit();
    WsfTimerInit();

    //
    // Initialize a buffer pool for WSF dynamic memory needs.
    //
#if defined (AM_PART_APOLLO2)
    WsfBufInit(sizeof(g_pui32BufMem), (uint8_t *)g_pui32BufMem, WSF_BUF_POOLS,
               g_psPoolDescriptors);
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    wsfBufMemLen = WsfBufInit(sizeof(g_pui32BufMem), (uint8_t *)g_pui32BufMem, WSF_BUF_POOLS,
               g_psPoolDescriptors);

    if (wsfBufMemLen > sizeof(g_pui32BufMem))
    {
        am_util_debug_printf("Memory pool is too small by %d\r\n",
                             wsfBufMemLen - sizeof(g_pui32BufMem));
    }
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
    //
    // Initialize the WSF security service.
    //
    SecInit();
    SecAesInit();
    SecCmacInit();
    SecEccInit();

    //
    // Set up callback functions for the various layers of the ExactLE stack.
    //
    handlerId = WsfOsSetNextHandler(HciHandler);
    HciHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(DmHandler);
    DmDevVsInit(0);
    DmAdvInit();
    DmConnInit();
    DmConnSlaveInit();
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
    L2cSlaveHandlerInit(handlerId);
    L2cInit();
    L2cSlaveInit();

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();
    AttcInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmprInit();
    SmprScInit();
    HciSetMaxRxAclLen(251);//(500);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);

#if configUSE_BLE
    handlerId = WsfOsSetNextHandler(AmVosHandler);
    AmVosHandlerInit(handlerId);
#endif // configUSE_BLE

#if defined (AM_PART_APOLLO2)
    g_sVosBle.hBleDataReadyHandlerId = WsfOsSetNextHandler(am_vos_data_ready_handler);
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    handlerId = WsfOsSetNextHandler(HciDrvHandler);
    HciDrvHandlerInit(handlerId);
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
}

//*****************************************************************************
//
// Perform initial setup for the radio task.
//
//*****************************************************************************
void
RadioTaskSetup(void)
{
    //    am_util_debug_printf("RadioTask: setup\r\n");

#if defined (AM_PART_APOLLO2)
    //
    // Enable 32KHz Clockout to EM9304
    //
    // QT board configuration
//#if USE_NICKEL_BOARD
//    am_hal_gpio_pin_config(4, AM_HAL_PIN_4_CLKOUT);
//#endif
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    NVIC_SetPriority(BLE_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

    //
    // Boot the radio.
    //
    HciDrvRadioBoot(1);
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************

// triggers a ble task event to send data
// return true if failed
#if configUSE_BLE
bool am_vos_ble_cmd_send(uint8_t *buf, uint32_t length)
{
    sRadioCmdQueue_t bleMessageCmd;

    memset(bleMessageCmd.cmd_buf, 0, length);
    memcpy(bleMessageCmd.cmd_buf, buf, length);

    bleMessageCmd.len = length;

    if(xQueueSend(g_sVosBle.hRadioCmdQueue, &bleMessageCmd, 0) == NULL)
    {
        AM_APP_LOG_WARNING("queue send fail in tx!\r\n");
        return false;
    }

    WsfTaskSetReady(0,0);

    return true;

}

bool am_vos_ble_stream_send(void)
{
    sRadioQueue_t bleStreamTrigger;

    //fill audio data header
    bleStreamTrigger.vos_buf[0] = 0x00;

    bleStreamTrigger.len = 0;

    if(xQueueSend(g_sVosBle.hRadioQueue, &bleStreamTrigger, 0) == NULL)
    {
        return true;
    }

    WsfTaskSetReady(0,0);
    return false;

}

sRadioQueue_t gRadioQueue;       // queue element defined globally to avoid using the stack
sRadioCmdQueue_t gRadioCmdQueue;       // queue element defined globally to avoid using the stack

//uint8_t bleAmVosSendingBuffer[256];   // sized to fit a max MTU value
bool am_vos_ble_nextdata_check(uint8_t** buf, uint32_t* len)
{
    if(xQueueReceive(g_sVosBle.hRadioCmdQueue, &gRadioCmdQueue, 0))
    {
        //
        // check command queue first for higher priority
        //
        *buf = gRadioCmdQueue.cmd_buf;
        *len = gRadioCmdQueue.len;
        return true;
    }
    else if(xQueueReceive(g_sVosBle.hRadioQueue, &gRadioQueue, 0))
    {
        if((gRadioQueue.len == 0) && (gRadioQueue.vos_buf[0] == 0x00))
        {
                return false;
        }
        else
        {
            // use queued data
            *buf = gRadioQueue.vos_buf;
            *len = gRadioQueue.len;
            return true;
        }
    }
    else
    {
        return false;
    }
}
#endif // configUSE_BLE

//*****************************************************************************
//
//! @brief am_vos_streaming_push_to_talk - Manual trigger and start voice command streamming.
//! 
//! 
//! 
//! @return None.
//*****************************************************************************
void am_vos_streaming_push_to_talk(void)
{
#if configUSE_PUSH_TO_TALK
    if(g_sVosSys.bWwdEnabled == true)
    {
        // push to talk event
#if configUSE_WOS
        am_vos_wos_handler();
#endif // configUSE_WOS

        am_vos_audio_reset_flag_and_buffer();
        g_sVosSys.ui8PushTalkFlag = 1;

        AM_APP_LOG_DEBUG("[AM-VoS] Disable WakeWord Detection\n");
        g_sVosSys.bWwdEnabled = false;

        AM_APP_LOG_INFO("[AM-VoS] Push to talk!\n");
    }
    else
    {
        // barge-in not allowed
        am_vos_gpio_enable_irq(PUSH_TO_TALK_BUTTON);
    }
#endif // configUSE_PUSH_TO_TALK
}

void am_vos_streaming_provide_speech(void)
{
#if configUSE_WOS
    am_vos_wos_handler();
#endif // configUSE_WOS

    am_vos_reset_detected_flag();

    am_vos_audio_wwd_disable();
    g_sVosSys.ui8ProvideSpeechFlag = 1;
}


//*****************************************************************************
//
//! @brief am_kwd_streaming_stop - Force stop voice command streamming.
//! 
//! 
//! 
//! @return None.
//*****************************************************************************
void am_vos_streaming_stop(void)
{
    // Note: count variable is used in AWEProcessing() at am_pdm_isr,
    // is it safe to change? or need to handle with mutex?
    // This function will be called by SmartVoice task in VENDOR_L project.
    am_vos_reset_detected_flag();

    // Lewis : Need to confirm later.
}

void am_vos_ble_task(void *pvParameters)
{
    //
    // Initialize the main ExactLE stack.
    //
    exactle_stack_init();

#if defined (AM_PART_APOLLO2)
    //
    // Register an interrupt handler for BLE event
    //
    am_hal_gpio_int_register(AM_BSP_GPIO_EM9304_INT, am_vos_radio_cts_handler);

    //
    // Enable BLE data ready interrupt
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
#endif // AM_PART_APOLLO2

    g_sVosBle.hRadioQueue = xQueueCreate(4, sizeof( sRadioQueue_t ));          //4 * 15ms = 60 ms
    g_sVosBle.hRadioCmdQueue = xQueueCreate(16, sizeof( sRadioCmdQueue_t ));
  
    //
    // Start the "AmVos" profile.
    //
#if configUSE_BLE
    AmVosStart();
#if configUSE_AMVOS_AMA
    am_vos_ama_evt_cback_register(am_vos_ama_event_callback);
#endif // configUSE_AMVOS_AMA
#endif // configUSE_BLE

    while (1)
    {
#if configUSE_BLE && configUSE_AUDIO_CODEC
        uint8_t* pBuf;
        uint32_t len;

        if(am_vos_is_tx_ready())
        {
            uint32_t data_len = am_app_utils_get_ring_buffer_status(&(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_ENCODED]));
            
            if(am_vos_ble_nextdata_check(&pBuf, &len))
            {
                AmVosBleSend(pBuf, len);
                //AM_APP_LOG_INFO(" 2 ");
            }
           
            else if(data_len >= BLE_DATA_BUFFER_SIZE) //57) //
            {
                uint8_t ui8AmVosTxBuf[BLE_DATA_BUFFER_SIZE + 3];
                //AM_APP_LOG_INFO(" 3 ");
                am_vos_ble_tx_packet_encap(ui8AmVosTxBuf, BLE_DATA_BUFFER_SIZE);
                
                AmVosBleSend(ui8AmVosTxBuf, BLE_DATA_BUFFER_SIZE + 3);
            }
        }
#endif // configUSE_BLE && configUSE_AUDIO_CODEC


        //
        // Calculate the elapsed time from our free-running timer, and update
        // the software timers in the WSF scheduler.
        //
        wsfOsDispatcher();
    }

}

int am_vos_ble_rx_handler(uint8_t *data, uint16_t len)
{
    if(len)
    {
        AM_APP_LOG_DEBUG("[RX data]: ");
        for(int i = 0; i < len; i++)
          AM_APP_LOG_DEBUG("0x%x ", data[i]);
        AM_APP_LOG_DEBUG("\n");
    }

    return true;
}

void am_vos_ble_queue_reset(void)
{
    xQueueReset(g_sVosBle.hRadioQueue);
    xQueueReset(g_sVosBle.hRadioCmdQueue);
}

void am_vos_ble_tx_packet_encap(uint8_t *buf, uint16_t len)
{
    AM_CRITICAL_BEGIN_VOS;
#if configUSE_AMVOS_AMA
    buf[0] = 0x10;
    buf[1] = 0x80;  //AMA_TRANSPORT_DATA_TYPE_VOICE
    buf[2] = len;
#else // configUSE_AMVOS_AMA
    buf[0] = 0x00;
    buf[1] = g_sVosSys.ui8SeqNumber++;
    buf[2] = len;
#endif // configUSE_AMVOS_AMA
    am_audio_buffer_pop(AM_AUDIO_BUFFER_ENCODED, &buf[3], len);
    AM_CRITICAL_END_VOS;
}

bool am_vos_is_connected(void)
{
#if configUSE_BLE
#if configUSE_AMVOS_AMA
    if((g_sVosBle.ui8AmVosConnId != DM_CONN_ID_NONE) && am_vos_ama_isready())
#else // configUSE_AMVOS_AMA
    if(g_sVosBle.ui8AmVosConnId != DM_CONN_ID_NONE)
#endif // configUSE_AMVOS_AMA
    {
        return true;
    }
    else
        return false;
#else // configUSE_BLE
    return false;
#endif // configUSE_BLE
}

bool am_vos_is_tx_ready(void)
{
#if configUSE_BLE
    if(g_sVosBle.ui8VosTxBusy == 0)
        return true;
#endif // configUSE_BLE
    return false;
}

