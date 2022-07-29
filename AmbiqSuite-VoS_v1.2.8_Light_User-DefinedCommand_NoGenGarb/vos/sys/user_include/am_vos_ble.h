//*****************************************************************************
//
//! @file radio_task.h
//!
//! @brief Functions and variables related to the radio task.
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

#ifndef RADIO_TASK_H
#define RADIO_TASK_H

#define USE_POWER_SAVE_MODE             1		// If USE_POWER_SAVE_MODE is 0, keep alive packet is sending and BLE connection parameter is always keep setting value.

#define USE_BLE_TX_POWER_SET            0

/*! Event types for ble rx data handler */
#define BLE_DATA_READY_EVENT                   0x01      /*! Trigger Rx data path */

//********************************************************************************
// BLE parameters
//********************************************************************************
#if configUSE_OPTIM_OPUS
#define BLE_DATA_BUFFER_SIZE            CODEC_OUT_RING_BUFF_SIZE        // 2 times for Apollo2Blue, 4 times for Apollo3Blue due to SRAM resource difference
#else // configUSE_OPTIM_OPUS
#define BLE_DATA_BUFFER_SIZE            CODEC_OUT_RING_BUFF_SIZE * 2    // 2 times for Apollo2Blue, 4 times for Apollo3Blue due to SRAM resource difference
#endif // configUSE_OPTIM_OPUS

#define AMA_COMMAND_BUFFER_SIZE         64                              // command + header length   

typedef struct{
    uint8_t vos_buf[4]; //dummy
    uint32_t len;
    uint32_t reserved;
}sRadioQueue_t;

typedef struct{
    uint8_t cmd_buf[AMA_COMMAND_BUFFER_SIZE];
    uint32_t len;
}sRadioCmdQueue_t;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void RadioTaskSetup(void);
extern void RadioTask(void *pvParameters);

extern void am_vos_ble_hw_init(void);
extern int am_vos_ble_rx_handler(uint8_t *data, uint16_t len);
extern bool am_vos_ble_stream_send(void);
extern bool am_vos_ble_nextdata_check(uint8_t** buf, uint32_t* len);
extern void am_vos_ble_task(void *pvParameters);

extern void am_vos_streaming_push_to_talk(void);
extern void am_vos_streaming_stop(void);
extern void am_vos_streaming_provide_speech(void);

extern bool am_vos_is_connected(void);
extern bool am_vos_is_tx_ready(void);
extern void am_vos_ble_tx_packet_encap(uint8_t *buf, uint16_t len);
extern bool am_vos_ble_cmd_send(uint8_t *buf, uint32_t length);
extern void am_vos_ble_tx_staus_set(uint8_t val);
extern void am_vos_ble_data_ready_evt_set(void);

#if (!configUSE_BLE)
extern void HciDrvRadioShutdown(void);
#endif

#endif // RADIO_TASK_H
