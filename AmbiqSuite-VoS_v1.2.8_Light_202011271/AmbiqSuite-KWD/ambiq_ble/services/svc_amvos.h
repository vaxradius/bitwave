//*****************************************************************************
//
//! @file svc_amdvos.h
//!
//! @brief AmbiqMicro Data Transfer Protocol service definition
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
#ifndef SVC_AMVOS_H
#define SVC_AMVOS_H

//
// Put additional includes here if necessary.
//


#ifdef __cplusplus
extern "C"
{
#endif
//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

#if configUSE_AMVOS_AMA
    /* AMA services */
    #define ATT_UUID_AMVOS_SERVICE          0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x03, 0xFE, 0x00, 0x00
           
    /* Amvos characteristics */
    #define ATT_UUID_AMVOS_RX               0x76, 0x30, 0xF8, 0xDD, 0x90, 0xA3, 0x61, 0xAC, 0xA7, 0x43, 0x05, 0x30, 0x77, 0xB1, 0x4E, 0xF0
    #define ATT_UUID_AMVOS_TX               0x0B, 0x42, 0x82, 0x1F, 0x64, 0x72, 0x2F, 0x8A, 0xB4, 0x4B, 0x79, 0x18, 0x5B, 0xA0, 0xEE, 0x2B

    // AM VoS Service
    #define AMVOS_START_HDL               0x0800//0x300
    #define AMVOS_END_HDL                 (AMVOS_MAX_HDL - 1)
#else
    /*! Base UUID:  00002760-08C2-11E1-9073-0E8AC72EXXXX */
    #define ATT_UUID_AMBIQ_BASE             0x2E, 0xC7, 0x8A, 0x0E, 0x73, 0x90, \
                                                0xE1, 0x11, 0xC2, 0x08, 0x60, 0x27, 0x00, 0x00

    /*! Macro for building Ambiq UUIDs */
    #define ATT_UUID_AMBIQ_BUILD(part)      UINT16_TO_BYTES(part), ATT_UUID_AMBIQ_BASE

    /*! Partial amvos service UUIDs */
    #define ATT_UUID_AMVOS_SERVICE_PART     0x1011

    /*! Partial amvos rx characteristic UUIDs */
    #define ATT_UUID_AMVOS_RX_PART          0x0011

    /*! Partial amvos tx characteristic UUIDs */
    #define ATT_UUID_AMVOS_TX_PART          0x0012
                                                 
    /* Amvos services */
    #define ATT_UUID_AMVOS_SERVICE          ATT_UUID_AMBIQ_BUILD(ATT_UUID_AMVOS_SERVICE_PART)

    /* Amvos characteristics */
    #define ATT_UUID_AMVOS_RX               ATT_UUID_AMBIQ_BUILD(ATT_UUID_AMVOS_RX_PART)
    #define ATT_UUID_AMVOS_TX               ATT_UUID_AMBIQ_BUILD(ATT_UUID_AMVOS_TX_PART)

    // AM VoS Service
    #define AMVOS_START_HDL               0x0800//0x300
    #define AMVOS_END_HDL                 (AMVOS_MAX_HDL - 1)
#endif

/* AMVOS Service Handles */
enum
{
  AMVOS_SVC_HDL = AMVOS_START_HDL,     /* AMVOS service declaration */
  AMVOS_RX_CH_HDL,                     /* AMVOS write command characteristic */ 
  AMVOS_RX_HDL,                        /* AMVOS write command data */
  AMVOS_TX_CH_HDL,                     /* AMVOS notify characteristic */ 
  AMVOS_TX_HDL,                        /* AMVOS notify data */
  AMVOS_TX_CH_CCC_HDL,                 /* AMVOS notify client characteristic configuration */
  AMVOS_MAX_HDL
};


//*****************************************************************************
//
// External variable definitions
//
//*****************************************************************************
//extern uint32_t g_ui32Stuff;

//*****************************************************************************
//
// Function definitions.
//
//*****************************************************************************
void SvcAmvosAddGroup(void);
void SvcAmvosRemoveGroup(void);
void SvcAmvosCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

#ifdef __cplusplus
}
#endif

#endif // SVC_AMVOS_H
