// ****************************************************************************
//
//  amvos_api.h
//! @file
//!
//! @brief Ambiq Micro's demonstration of AMVOS service.
//!
//! @{
//
// ****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2019, Ambiq Micro
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
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
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
// This is part of revision 2.3.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AMVOS_API_H
#define AMVOS_API_H

#include "FreeRTOS.h"
#include "queue.h"
#include "wsf_timer.h"
#include "dm_api.h"

#include "am_vos_sys_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

#ifndef AMVOS_CONN_MAX
#define AMVOS_CONN_MAX                  1
#endif

#if configUSE_AMVOS_AMA
#define AMA_VID                 0x005A      /* Company ID i.e. EM Micro */
#define ACCESSORY_COLOR_INDEX   0x0         /* Accessory color - TBD */
#endif // configUSE_AMVOS_AMA

typedef struct _VosBleInfo {
    wsfHandlerId_t      ui8AmVosHandlerId;      // WSF handler ID
    dmConnId_t          ui8AmVosConnId;

    uint8_t             ui8VosTxBusy;

    QueueHandle_t       hRadioQueue;
    QueueHandle_t       hRadioCmdQueue;

#if configUSE_AMVOS_AMA
    bool                bAmaComSecured;         // flag indicating channel secured status
    bool                bAmaSlaveSecReq;        // flag indicating a slave security request is needed or not
    bool                bAmaSetupFlag;          // channel setup (firmware exchange) command sent or not
#endif // configUSE_AMVOS_AMA

#if defined (AM_PART_APOLLO2)
    wsfHandlerId_t      hBleDataReadyHandlerId;
#endif // AM_PART_APOLLO2

#if configUSE_BLE_WATCHDOG
    wsfTimer_t          sAmvosTxTimer;         // timeout timer to monitor ble controller tx activity
    wsfTimer_t          sAmvosRspTimer;        // timeout timer to monitor APP responsiveness
    wsfTimer_t          sAmvosTxCccTimer;      // timeout timer to monitor AMA setup
    wsfTimer_t          sAmvosTestTimer;       // timeout timer to monitor APP responsiveness
#endif // configUSE_BLE_WATCHDOG

#if configUSE_BLE_Measure_Throughput
    uint32_t            ui32AmaDataSentLength;
    uint32_t            ui32AmaDataSentCnt;
    uint32_t            ui32AmaDataCnfCnt;
#endif // configUSE_BLE_Measure_Throughput

} VosBleInfo;

extern VosBleInfo g_sVosBle;

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/
/*************************************************************************************************/
/*!
 *  \fn     AmvosStart
 *
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmVosStart(void);

/*************************************************************************************************/
/*!
 *  \fn     AmVosHandlerInit
 *
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for App.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmVosHandlerInit(wsfHandlerId_t handlerId);

/*************************************************************************************************/
/*!
 *  \fn     AmVosHandler
 *
 *  \brief  WSF event handler for the application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmVosHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

void AmVosBleSend(uint8_t * buf, uint32_t len);

#ifdef __cplusplus
};
#endif

#endif /* AMVOS_API_H */
