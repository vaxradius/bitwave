//*****************************************************************************
//
//! @file svc_amvos.c
//!
//! @brief AM data transfer protocol service implementation
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2018, Ambiq Micro
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
// This is part of revision 1.2.12 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "am_vos_sys_config.h"

#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "bstream.h"
#include "svc_ch.h"
#include "svc_cfg.h"
#include "svc_amvos.h"

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
//#define SOME_MACRO              42          //!< This is the answer

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
//uint32_t g_ui32Stuff;

/**************************************************************************************************
 Static Variables
**************************************************************************************************/
/* UUIDs */
static const uint8_t svcRxUuid[] = {ATT_UUID_AMVOS_RX};
static const uint8_t svcTxUuid[] = {ATT_UUID_AMVOS_TX};

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* AMVOS service declaration */
static const uint8_t amvosSvc[] = {ATT_UUID_AMVOS_SERVICE};
static const uint16_t amvosLenSvc = sizeof(amvosSvc);

/* AMVOS RX characteristic */ 
static const uint8_t amvosRxCh[] = {ATT_PROP_WRITE, UINT16_TO_BYTES(AMVOS_RX_HDL), ATT_UUID_AMVOS_RX};
static const uint16_t amvosLenRxCh = sizeof(amvosRxCh);

/* AMVOS TX characteristic */ 
static const uint8_t amvosTxCh[] = {ATT_PROP_NOTIFY | ATT_PROP_READ, UINT16_TO_BYTES(AMVOS_TX_HDL), ATT_UUID_AMVOS_TX};
static const uint16_t amvosLenTxCh = sizeof(amvosTxCh);

/* AMVOS RX data */
/* Note these are dummy values */
static const uint8_t amvosRx[] = {0};
static const uint16_t amvosLenRx = sizeof(amvosRx);

/* AMVOS TX data */
/* Note these are dummy values */
static const uint8_t amvosTx[] = {0};
static const uint16_t amvosLenTx = sizeof(amvosTx);

/* Proprietary data client characteristic configuration */
static uint8_t amvosTxChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t amvosLenTxChCcc = sizeof(amvosTxChCcc);

/* Attribute list for AMVOS group */
static const attsAttr_t amvosList[] =
{
    {
        attPrimSvcUuid, 
        (uint8_t *) amvosSvc,
        (uint16_t *) &amvosLenSvc, 
        sizeof(amvosSvc),
        0,
        ATTS_PERMIT_READ
    },
    {
        attChUuid,
        (uint8_t *) amvosRxCh,
        (uint16_t *) &amvosLenRxCh,
        sizeof(amvosRxCh),
        0,
        ATTS_PERMIT_READ
    },
    {
        svcRxUuid,
        (uint8_t *) amvosRx,
        (uint16_t *) &amvosLenRx,
        ATT_VALUE_MAX_LEN,
        (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
        ATTS_PERMIT_WRITE
        //ATTS_PERMIT_WRITE_ENC
    },
    {
        attChUuid,
        (uint8_t *) amvosTxCh,
        (uint16_t *) &amvosLenTxCh,
        sizeof(amvosTxCh),
        0,
        ATTS_PERMIT_READ
    },
    {
        svcTxUuid,
        (uint8_t *) amvosTx,
        (uint16_t *) &amvosLenTx,
        sizeof(amvosTx), //ATT_VALUE_MAX_LEN,
#if configUSE_AMVOS_AMA
            (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN),
            ATTS_PERMIT_READ
#else // configUSE_AMVOS_AMA
            0,  //(ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN),
            0	//ATTS_PERMIT_READ
#endif // configUSE_AMVOS_AMA
    },
    {
        attCliChCfgUuid,
        (uint8_t *) amvosTxChCcc,
        (uint16_t *) &amvosLenTxChCcc,
        sizeof(amvosTxChCcc),
        ATTS_SET_CCC,
        (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE)
    }
};

/* AMVOS group structure */
static attsGroup_t svcAmvosGroup =
{
    NULL,
    (attsAttr_t *) amvosList,
    NULL,
    NULL,
    AMVOS_START_HDL,
    AMVOS_END_HDL
};

/*************************************************************************************************/
/*!
 *  \fn     SvcAmvosAddGroup
 *        
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcAmvosAddGroup(void)
{
    AttsAddGroup(&svcAmvosGroup);
}

/*************************************************************************************************/
/*!
 *  \fn     SvcAmvosRemoveGroup
 *        
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcAmvosRemoveGroup(void)
{
    AttsRemoveGroup(AMVOS_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \fn     SvcAmvosCbackRegister
 *        
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcAmvosCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
    svcAmvosGroup.readCback = readCback;
    svcAmvosGroup.writeCback = writeCback;
}
