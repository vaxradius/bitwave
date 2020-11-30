// ****************************************************************************
//
//  amvos_main.c
//! @file
//!
//! @brief Ambiq Micro's demonstration of AMVOS service.
//!
//! @{
//
// ****************************************************************************

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
#include "am_vos_board_setup.h"

#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "wsf_types.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "hci_api.h"
#include "bstream.h"
#include "dm_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "app_hw.h"
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "gatt_api.h"

#include "hci_drv_apollo.h"

#include "am_app_utils.h"
#include "am_vos_init.h"
#include "am_vos_audio.h"
#include "am_vos_codec.h"
#include "am_vos_ble.h"

#include "amvos_api.h"
#include "svc_amvos.h"

#ifdef USE_BLE_OTA
#include "amotas_api.h"
#include "svc_amotas.h"
#endif // USE_BLE_OTA

#if configUSE_AMVOS_AMA
#include "am_vos_ama.h"
#include "hci_core.h"

extern hciCoreCb_t hciCoreCb;
#endif // configUSE_AMVOS_AMA

#include "am_util.h"
#include "crc32.h"

#if USE_BLE_TX_POWER_SET
#include "hci_drv_em9304.h"
#endif // USE_BLE_TX_POWER_SET

#include "am_audio_buffer.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event starting value */
#define AMVOS_MSG_START               0xA0

/*! WSF message event enumeration */
enum
{
#ifdef USE_BLE_OTA
  AMOTA_RESET_TIMER_IND = AMVOS_MSG_START,      // AMOTA reset timer expired
  AMOTA_DISCONNECT_TIMER_IND,                   // AMOTA disconnect timer expired
  AMVOS_TX_TIMEOUT_TIMER_IND,                   // ble controller tx event timeout timer
#else // USE_BLE_OTA
  AMVOS_TX_TIMEOUT_TIMER_IND = AMVOS_MSG_START, // ble controller tx event timeout timer
#endif // USE_BLE_OTA
  AMVOS_RSP_TIMER_START_IND,                    // event to trigger a wsfTimer from other context 
  AMVOS_RSP_TIMEOUT_TIMER_IND,                  // app response event timeout timer
  AMVOS_TX_CCC_TIMEOUT_TIMER_IND,               // app setup event timeout timer
  AMVOS_TEST_TIMEOUT_TIMER_IND,                 // test timeout timer
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! Application message type */
typedef union
{
  wsfMsgHdr_t     hdr;
  dmEvt_t         dm;
  attsCccEvt_t    ccc;
  attEvt_t        att;
} amvosMsg_t;

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t amvosAdvCfg =
{
  {60000,     0,     0},                  /*! Advertising durations in ms */
  {  800,   800,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t amvosSlaveCfg =
{
  AMVOS_CONN_MAX,                           /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t amvosSecCfg =
{
#if configUSE_BLE_SECURE_CONNECTION
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
#else // configUSE_BLE_SECURE_CONNECTION
  DM_AUTH_BOND_FLAG,
#endif // configUSE_BLE_SECURE_CONNECTION
  0,                                      /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  FALSE                                   /*! TRUE to initiate security upon connection */
};


static appUpdateCfg_t amvosUpdateCfg =
{
  1000,//3000,                                   /*! Connection idle period in ms before attempting
                                              //connection parameter update; set to zero to disable */
  6,    //6,                                      /*! 7.5ms */
  12,   //15,                                     /*! 15ms */
  0,                                      /*! Connection latency */
  400, //2000, //600,                     /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/*! SMP security parameter configuration */
static const smpCfg_t amvosSmpCfg =
{
  3000,                                   /*! 'Repeated attempts' timeout in msec */
  SMP_IO_NO_IN_NO_OUT,                    /*! I/O Capability */
  7,                                      /*! Minimum encryption key length */
  16,                                     /*! Maximum encryption key length */
  3,                                      /*! Attempts to trigger 'repeated attempts' timeout */
  0,                                      /*! Device authentication requirements */
};

#ifdef USE_BLE_OTA
/*! AMOTAS configuration */
static const AmotasCfg_t vosAmotaCfg =
{
    0
};
#endif // USE_BLE_OTA
/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

// RAM buffers to be used
uint8_t amvosAdvDataDisc[31];
uint8_t amvosScanDataDisc[31];

#if configUSE_AMVOS_AMA
/*! advertising data, discoverable mode */
static const uint8_t amvosAdvDataDiscDefault[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! tx power */
  3,                                      /*! length */
  DM_ADV_TYPE_16_UUID,                    /*! AD type */
  UINT16_TO_BYTES(0xFE03),

  /*! service UUID list */
  0x17,                                    /*! length */
  DM_ADV_TYPE_SERVICE_DATA,                /*! AD type */
  UINT16_TO_BYTES(0xFE03),
  UINT16_TO_BYTES(AMA_VID),                /* Vendor ID assigned by BT */
  UINT16_TO_BYTES(0x0001),                 /* Alexa Built-in Headphone */
  ACCESSORY_COLOR_INDEX,                   /* Accessory color index */
  0,
  0,
  0,
  0,
  0, 0, 0, 0, 0
};
#else // configUSE_AMVOS_AMA
/*! advertising data, discoverable mode */
static const uint8_t amvosAdvDataDiscDefault[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! tx power */
  2,                                      /*! length */
  DM_ADV_TYPE_TX_POWER,                   /*! AD type */
  0,                                      /*! tx power */

  3,
  DM_ADV_TYPE_APPEARANCE,                 // used to identify kwd_ble
  0x55,
  0xAA,

  /*! service UUID list */
  3,                                      /*! length */
  DM_ADV_TYPE_16_UUID,                    /*! AD type */
  UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE),

  17,                                      /*! length */
  DM_ADV_TYPE_128_UUID,                    /*! AD type */
  ATT_UUID_AMVOS_SERVICE
};
#endif // configUSE_AMVOS_AMA

/*! scan data, discoverable mode */
static const uint8_t amvosScanDataDiscDefault[] =
{
  /*! device name */
  11,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'V',
  'o',
  'S',
  '-',
  'A',
  'M',
  'A',
  '-',
  'L',
  'E'
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  AMVOS_GATT_SC_CCC_IDX,                  /*! GATT service, service changed characteristic */
  AMVOS_TX_CCC_IDX,                       /*! AMVOS service, tx characteristic */
#ifdef USE_BLE_OTA
  AMVOS_AMOTAS_TX_CCC_IDX,                /*! AMOTA service, tx characteristic */
#endif // USE_BLE_OTA
  AMVOS_NUM_CCC_IDX
};

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t amvosCccSet[AMVOS_NUM_CCC_IDX] =
{
  /* cccd handle                value range               security level */
  {GATT_SC_CH_CCC_HDL,          ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* AMVOS_GATT_SC_CCC_IDX */
  {AMVOS_TX_CH_CCC_HDL,         ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* AMVOS_TX_CCC_IDX */
#ifdef USE_BLE_OTA
  {AMOTAS_TX_CH_CCC_HDL,        ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* AMOTA_AMOTAS_TX_CCC_IDX */
#endif // USE_BLE_OTA
};

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

VosBleInfo g_sVosBle =
{
    .ui8AmVosConnId = DM_CONN_ID_NONE,

#if configUSE_AMVOS_AMA
    .bAmaComSecured = false,
    .bAmaSlaveSecReq = false,
    .bAmaSetupFlag = false,
#endif // configUSE_AMVOS_AMA

#if configUSE_BLE_Measure_Throughput
    .ui32AmaDataSentLength = 0,
    .ui32AmaDataSentCnt = 0,
    .ui32AmaDataCnfCnt = 0,
#endif // configUSE_BLE_Measure_Throughput
};

#if configUSE_BLE_WATCHDOG
#define AMVOS_TX_TIMER_START()                                                  \
    do                                                                          \
    {                                                                           \
      WsfTimerStartMs(&(g_sVosBle.sAmvosTxTimer), (10000));                                \
      WsfTaskSetReady(0,0);                                                     \
    } while (0)

#define AMVOS_TX_TIMER_STOP()                                                   \
    do                                                                          \
    {                                                                           \
      WsfTimerStop(&(g_sVosBle.sAmvosTxTimer));                                            \
      WsfTaskSetReady(0,0);                                                     \
    } while (0)

#define AMVOS_TX_TIMER_RESTART()                                                \
    do                                                                          \
    {                                                                           \
        WsfTimerStop(&(g_sVosBle.sAmvosTxTimer));                                          \
        WsfTimerStartMs(&(g_sVosBle.sAmvosTxTimer), (10000));                              \
        WsfTaskSetReady(0,0);                                                   \
    } while (0)

#define AMVOS_TX_CCC_TIMER_START()                                              \
    do                                                                          \
    {                                                                           \
      WsfTimerStartMs(&(g_sVosBle.sAmvosTxCccTimer), (20000));                             \
      WsfTaskSetReady(0,0);                                                     \
    } while (0)

void AMVOS_TX_CCC_TIMER_STOP(void)
{                                                                        
      WsfTimerStop(&(g_sVosBle.sAmvosTxCccTimer));
      WsfTaskSetReady(0,0);
}

#define AMVOS_TX_CCC_TIMER_RESTART()                                            \
    do                                                                          \
    {                                                                           \
        WsfTimerStop(&(g_sVosBle.sAmvosTxCccTimer));                                       \
        WsfTimerStartMs(&(g_sVosBle.sAmvosTxCccTimer), (20000));                           \
        WsfTaskSetReady(0,0);                                                   \
    } while (0)

        
void AMVOS_RSP_TIMER_START(void)
{
//    WsfTimerStartSec(&g_AmvosRspTimer, (10));
    amvosMsg_t *pMsg;
    uint16_t len;
    len = sizeof(wsfMsgHdr_t);
    if ((pMsg = WsfMsgAlloc(len)) != NULL)
    {
        pMsg->hdr.event = AMVOS_RSP_TIMER_START_IND;
        WsfMsgSend(g_sVosBle.ui8AmVosHandlerId, pMsg);
    }
    WsfTaskSetReady(0,0); 
    //AM_APP_LOG_DEBUG("---- AMVOS_RSP_TIMER_START(), tick = %d\n", xTaskGetTickCount());
}
      
#define AMVOS_RSP_TIMER_STOP()                                                  \
    do                                                                          \
    {                                                                           \
      if(g_sVosBle.sAmvosRspTimer.isStarted == true)                                     \
      {                                                                         \
        WsfTimerStop(&(g_sVosBle.sAmvosRspTimer));                                         \
        AM_APP_LOG_DEBUG("---- AMVOS_RSP_TIMER_STOP(), tick = %d\n", xTaskGetTickCount());                      \
        WsfTaskSetReady(0,0);                                                   \
      }                                                                         \
    } while (0)

void AMVOS_RSP_TIMER_RESTART(void)
{
    if(g_sVosBle.sAmvosRspTimer.isStarted == true)
    {
        WsfTimerStop(&(g_sVosBle.sAmvosRspTimer));
    }
    WsfTimerStartSec(&(g_sVosBle.sAmvosRspTimer), (10));
    WsfTaskSetReady(0,0); 
    AM_APP_LOG_DEBUG("---- AMVOS_RSP_TIMER_RESTART(), tick = %d\n", xTaskGetTickCount());
}

#define AMVOS_TEST_TIMER_START()                                                \
    do { WsfTimerStartMs(&(g_sVosBle.sAmvosTestTimer), (10000));                           \
        AM_APP_LOG_DEBUG("---- test timer start, tick = %d\n", xTaskGetTickCount()); \
    } while (0)

#define AMVOS_TEST_TIMER_STOP()                                                 \
    do { WsfTimerStop(&(g_sVosBle.sAmvosTestTimer)); } while (0)

void AMVOS_TEST_TIMER_RESTART(void)
{
    WsfTimerStop(&(g_sVosBle.sAmvosTestTimer));
    WsfTimerStartMs(&(g_sVosBle.sAmvosTestTimer), (10000));
}
#else // configUSE_BLE_WATCHDOG
void AMVOS_TX_CCC_TIMER_STOP(void)
{
    return;
}

void AMVOS_RSP_TIMER_START(void)
{
    return;
}
#endif // configUSE_BLE_WATCHDOG

// index is the starting point of the local name, local name only in advData
static bool amvosSetLocalName(uint8_t* pAdvData, uint8_t* pLocalName, uint8_t len, uint8_t index)
{
    if(index+len+1 > 31)
    {
        // max adv data is 31 byte long
        return false;
    }

    // set parameter length
    pAdvData[index] = len + 1;
    // set parameter type
    pAdvData[index+1] = DM_ADV_TYPE_LOCAL_NAME;

    // set local name
    for(uint8_t i = 0; i < len; i++)
    {
        pAdvData[i+2+index] = pLocalName[i];
    }

    return true;
}

void amvosKwdSetDemoName(void)
{
    uint8_t test_bdaddress[6];
    uint8_t ble_device_name[20] = "VoS-";    //local name = device name
    uint8_t * pBda;
    uint8_t index = 4;

    //fixme: read bd address and print out
    pBda = HciGetBdAddr();
    BdaCpy(test_bdaddress, pBda);

    pBda = (uint8_t*)Bda2Str(test_bdaddress);
    AM_APP_LOG_INFO("[AM-VoS] Local Device BD Address: ");
    AM_APP_LOG_INFO(Bda2Str(test_bdaddress));
    AM_APP_LOG_INFO("\n");

    // build demo name here
    // 1st letter is board variant
#if USE_MAYA
    ble_device_name[index++] = 'M';
#elif USE_APOLLO2_QT
    ble_device_name[index++] = 'Q';
#else
    ble_device_name[index++] = 'E';
#endif // USE_MAYA, USE_APOLLO2_QT, EVBs


    // 3rd letter is wake-on-sound variant
#if configUSE_WOS
    ble_device_name[index++] = 'W';   // Wake on Sound enabled
#else // configUSE_WOS
    ble_device_name[index++] = 'A';   // A for always listening...
#endif // configUSE_WOS

#ifdef USE_BLE_OTA
    ble_device_name[index++] = 'O';
#endif // USE_BLE_OTA

#if configUSE_AMVOS_AMA
    ble_device_name[index++] = '-';

    ble_device_name[index++] = 'A';
    ble_device_name[index++] = 'M';
    ble_device_name[index++] = 'A';
#endif // configUSE_AMVOS_AMA
    // a hyphen...
    ble_device_name[index++] = '-';

    // take the last 4 hex digit
    ble_device_name[index++] = pBda[8];
    ble_device_name[index++] = pBda[9];
    ble_device_name[index++] = pBda[10];
    ble_device_name[index++] = pBda[11];

#if configUSE_AMVOS_AMA
    AmaDevInfo devinfo =
    {
      .ui8AudioFormat = AM_AMA_AUDIO_FORMAT,
      .device_type = AM_AMA_DEVICE_TYPE_STR
    };

    strcpy(devinfo.serial_number, (char const *)pBda);
    if(index < 17)
        strncpy(devinfo.name, (char const *)ble_device_name, index);
    else
        strncpy(devinfo.name, (char const *)ble_device_name, 16);

    am_vos_ama_devinfo_set(&devinfo);
#endif // configUSE_AMVOS_AMA
    
    // set local name here:
    amvosSetLocalName(amvosScanDataDisc, ble_device_name, index, 0);
    //SvcCoreSetDevName(ble_device_name, index);
}

/*************************************************************************************************/
/*!
 *  \fn     amvosDmCback
 *
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amvosDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t *pMsg;
  uint16_t len;

  len = DmSizeOfEvt(pDmEvt);

  if ((pMsg = WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pMsg, pDmEvt, len);
    WsfMsgSend(g_sVosBle.ui8AmVosHandlerId, pMsg);
  }
}

bool amvosTxChannelIsAvailable(void)
{
  return(DM_CONN_ID_NONE != AppConnIsOpen());
}

void AmVosBleSend(uint8_t * buf, uint32_t len)
{
    if(amvosTxChannelIsAvailable())
    {
        // simply tries to send notification
        AttsHandleValueNtf(g_sVosBle.ui8AmVosConnId, AMVOS_TX_HDL, len, buf);   // connId always group 0 since support only 1 connection.
        g_sVosBle.ui8VosTxBusy = 1;
        //AM_APP_LOG_INFO(" 1 ");

#if configUSE_BLE_Measure_Throughput
        g_ui32AmaDataSentLength += len;
#endif // configUSE_BLE_Measure_Throughput

#if configUSE_BLE_WATCHDOG
        AMVOS_TX_TIMER_RESTART();
#endif // configUSE_BLE_WATCHDOG
    }
}

/*************************************************************************************************/
/*!
 *  \fn     amvosAttCback
 *
 *  \brief  Application ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amvosAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(g_sVosBle.ui8AmVosHandlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     amvosCccCback
 *
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amvosCccCback(attsCccEvt_t *pEvt)
{
  attsCccEvt_t  *pMsg;
  appDbHdl_t    dbHdl;

  /* if CCC not set from initialization and there's a device record */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE))
  {
    /* store value in device database */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }

  if ((pMsg = WsfMsgAlloc(sizeof(attsCccEvt_t))) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attsCccEvt_t));
    WsfMsgSend(g_sVosBle.ui8AmVosHandlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     amvosProcCccState
 *
 *  \brief  Process CCC state change.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amvosProcCccState(amvosMsg_t *pMsg)
{
  AM_APP_LOG_INFO("[AM-VoS] ccc state ind value:%d handle:0x%X idx:%d\n", pMsg->ccc.value, pMsg->ccc.handle, pMsg->ccc.idx);

  /* AMVOS TX CCC */
  if (pMsg->ccc.idx == AMVOS_TX_CCC_IDX)
  {
    if (pMsg->ccc.value == ATT_CLIENT_CFG_NOTIFY)
    {
      // notify enabled
      g_sVosBle.ui8AmVosConnId = (dmConnId_t) pMsg->ccc.hdr.param;
      AM_APP_LOG_INFO("[AM-VoS] connId : %d\r\n", pMsg->ccc.hdr.param);
    }
    else
    {
      g_sVosBle.ui8AmVosConnId = DM_CONN_ID_NONE;
    }
    return;
  }

#ifdef USE_BLE_OTA
  /* AMOTAS TX CCC */
  if (pMsg->ccc.idx == AMVOS_AMOTAS_TX_CCC_IDX)
  {
    if (pMsg->ccc.value == ATT_CLIENT_CFG_NOTIFY)
    {
        // notify enabled
        amotas_start((dmConnId_t) pMsg->ccc.hdr.param, 
            AMOTA_RESET_TIMER_IND, AMOTA_DISCONNECT_TIMER_IND,
            AMVOS_AMOTAS_TX_CCC_IDX);

        AM_APP_LOG_INFO("[AM-VoS] MIC (PDM/ADC) disable\n");

#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
        PDMDeInit();    // PDM deinit to solving performance issue with VoS Framework.
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
        ADCDeInit();    // ADC deinit to solving performance issue with VoS Framework.
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    }
    else
    {
        // notify disabled
        amotas_stop((dmConnId_t) pMsg->ccc.hdr.param);
    }
    return;
  }
#endif // USE_BLE_OTA
}

/*************************************************************************************************/
/*!
 *  \fn     amvosClose
 *
 *  \brief  Perform UI actions on connection close.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amvosClose(amvosMsg_t *pMsg)
{
#ifdef USE_BLE_OTA
    /* stop amotas */
    amotas_conn_close((dmConnId_t) pMsg->hdr.param);
#endif // USE_BLE_OTA
}

/*************************************************************************************************/
/*!
 *  \fn     amvosSetup
 *
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amvosSetup(amvosMsg_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(amvosAdvDataDisc), (uint8_t *) amvosAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(amvosScanDataDisc), (uint8_t *) amvosScanDataDisc);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(amvosAdvDataDisc), (uint8_t *) amvosAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(amvosScanDataDisc), (uint8_t *) amvosScanDataDisc);

  /* start advertising; automatically set connectable/discoverable mode and bondable mode */
  AppAdvStart(APP_MODE_DISCOVERABLE);
}

/*************************************************************************************************/
/*!
 *  \fn     amvosBtnCback
 *
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amvosBtnCback(uint8_t btn)
{
  dmConnId_t      connId;

  /* button actions when connected */
  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      case APP_UI_BTN_1_SHORT:
        break;

      case APP_UI_BTN_1_MED:
        break;

      case APP_UI_BTN_1_LONG:
        AppConnClose(connId);
        break;

      case APP_UI_BTN_2_SHORT:
        break;

      default:
        break;
    }
  }
  /* button actions when not connected */
  else
  {
    switch (btn)
    {
      case APP_UI_BTN_1_SHORT:
        /* start or restart advertising */
        AppAdvStart(APP_MODE_AUTO_INIT);
        break;

      case APP_UI_BTN_1_MED:
        /* enter discoverable and bondable mode mode */
        AppSetBondable(TRUE);
        AppAdvStart(APP_MODE_DISCOVERABLE);
        break;

      case APP_UI_BTN_1_LONG:
        /* clear bonded device info and restart advertising */
        AppDbDeleteAllRecords();
        AppAdvStart(APP_MODE_AUTO_INIT);
        break;

      default:
        break;
    }
  }
}

void amvosConnParameterReqSend(void)
{
	  
    hciConnSpec_t connSpec;
    connSpec.connIntervalMin = amvosUpdateCfg.connIntervalMin;//(15/1.25);//(30/1.25);
    connSpec.connIntervalMax = amvosUpdateCfg.connIntervalMax;//(15/1.25);//(30/1.25);
    connSpec.connLatency = amvosUpdateCfg.connLatency;//0;
    connSpec.supTimeout = amvosUpdateCfg.supTimeout;
    connSpec.minCeLen = 0;
    connSpec.maxCeLen = 0xffff; //fixme
    DmConnUpdate(1, &connSpec);
}

#if configUSE_AMVOS_AMA
void amvosConnIntervalUpdate(uint8_t platform)
{
    if(platform == 1)
    {
        pAppUpdateCfg->connIntervalMax = 12;
        amvosUpdateCfg.connIntervalMax = 12;
        AM_APP_LOG_INFO("[AMA] iOS connection parameter setup\n");
    }
    else if(platform == 2)
    {
        pAppUpdateCfg->connIntervalMax = 99;
        amvosUpdateCfg.connIntervalMax = 99;
        AM_APP_LOG_INFO("[AMA] Android connection parameter setup\n");
    }
}
#endif // configUSE_AMVOS_AMA

uint8_t
amvos_write_cback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                       uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
    if (handle == AMVOS_RX_HDL)
    {
#if configUSE_AMVOS_AMA
#if configUSE_BLE_WATCHDOG
        AMVOS_RSP_TIMER_STOP(); // stop app response timer
#endif // configUSE_BLE_WATCHDOG
        am_vos_ama_rx_handler(pValue, len);
#else // configUSE_AMVOS_AMA
        am_vos_ble_rx_handler(pValue, len);
#endif // configUSE_AMVOS_AMA
    }
    return ATT_SUCCESS;
}

//*****************************************************************************
//
// Connection Open event
//
//*****************************************************************************
static void
amvosOpen(dmEvt_t *pMsg)
{
    hciLeConnCmplEvt_t *evt = (hciLeConnCmplEvt_t*) pMsg;

    AM_APP_LOG_INFO("[AM-VoS] Connection opened\n");
    AM_APP_LOG_INFO("handle = 0x%x\t", evt->handle);
    AM_APP_LOG_INFO("role = 0x%x\n", evt->role);
    AM_APP_LOG_INFO("addrMSB = %02x%02x%02x%02x%02x%02x\t", evt->peerAddr[0], evt->peerAddr[1], evt->peerAddr[2]);
    AM_APP_LOG_INFO("addrLSB = %02x%02x%02x%02x%02x%02x\n", evt->peerAddr[3], evt->peerAddr[4], evt->peerAddr[5]);
    AM_APP_LOG_INFO("connInterval = %d x 1.25 ms\n", evt->connInterval);
    AM_APP_LOG_INFO("connLatency = %d\t", evt->connLatency);
    AM_APP_LOG_INFO("supTimeout = %d ms\n\n", evt->supTimeout * 10);
    
    if(evt->connInterval > amvosUpdateCfg.connIntervalMax)
        amvosConnParameterReqSend();
}

//*****************************************************************************
//
// Connection Update event
//
//*****************************************************************************
static void
amvosConnUpdate(dmEvt_t *pMsg)
{
#if configUSE_STDIO_PRINTF
    volatile hciLeConnUpdateCmplEvt_t *evt = (hciLeConnUpdateCmplEvt_t*) pMsg;

    AM_APP_LOG_INFO("[AM-VoS] Connection update status = 0x%x\n", evt->status);
    AM_APP_LOG_INFO("handle = 0x%x\t", evt->handle);
    AM_APP_LOG_INFO("connInterval = %d x 1.25 ms\n", evt->connInterval);
    AM_APP_LOG_INFO("connLatency = %d\t", evt->connLatency);
    AM_APP_LOG_INFO("supTimeout = %d ms\n\n", evt->supTimeout * 10);
#endif // configUSE_STDIO_PRINTF
}

/*************************************************************************************************/
/*!
 *  \fn     amvosProcMsg
 *
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void amvosProcMsg(amvosMsg_t *pMsg)
{
    uint8_t uiEvent = APP_UI_NONE;
    static uint8_t retry_cnt = 0;
    
    switch(pMsg->hdr.event)
    {
#ifdef USE_BLE_OTA
        case AMOTA_RESET_TIMER_IND:
            amotas_proc_msg(&pMsg->hdr);
            break;
            
        case AMOTA_DISCONNECT_TIMER_IND:
            amotas_proc_msg(&pMsg->hdr);
            break;
#endif // USE_BLE_OTA

#if configUSE_BLE_WATCHDOG
#if configUSE_AMVOS_AMA
        case AMVOS_RSP_TIMER_START_IND:
            AMVOS_RSP_TIMER_RESTART();
            break;
            
        case AMVOS_TX_TIMEOUT_TIMER_IND:   // stuck somehow...

            AM_APP_LOG_DEBUG("---- stuck in TX event... hciCoreCb.availBufs = %d\n", hciCoreCb.availBufs);

            // When it times out, pretty much we have to
            // reset/reboot controller and initialize HCI
            // layer and SPI transport layer again.

            // stop the app response timeout timer
            AMVOS_RSP_TIMER_STOP();

            // clear ama channel flags
            g_sVosBle.ui8AmVosConnId = DM_CONN_ID_NONE;
            am_vos_ama_status_reset();
            
            //
            // VOS-5, reset BLE controller here instead of hciCmdTimeout
            //
#if configUSE_BLE_ERROR_SW_RESET
#if defined(AM_PART_APOLLO2)
            am_hal_reset_por();
#elif defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
            am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOR, 0);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P

#else // configUSE_BLE_ERROR_SW_RESET
            HciDrvRadioShutdown();
        
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_STOP, 0);
            //vTaskDelay(pdMS_TO_TICKS(10));
            am_util_delay_ms(200);
            am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
        
            HciDrvRadioBoot(1);
            DmDevReset();
#endif // configUSE_BLE_ERROR_SW_RESET
            break;

        case AMVOS_RSP_TIMEOUT_TIMER_IND:
            // timeout for APP response, close the current opened connection
            AM_APP_LOG_DEBUG("---- app not responding timeout, disconnect... tick = %d\n", xTaskGetTickCount());
            // clear ama channel flags
            g_sVosBle.ui8AmVosConnId = DM_CONN_ID_NONE;
            am_vos_ama_status_reset();
            
            AppConnClose(AppConnIsOpen());
            break;
            
        case AMVOS_TX_CCC_TIMEOUT_TIMER_IND:
            // TX CCC timed out, disconnect...
#if configUSE_BLE_ERROR_SW_RESET
            AM_APP_LOG_INFO("---- CCC not set timeout, reset board...\n");
#if defined(AM_PART_APOLLO2)
            am_hal_reset_por();
#elif defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
            am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOR, 0);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P

#else // configUSE_BLE_ERROR_SW_RESET
            AppConnClose(AppConnIsOpen());
            AM_APP_LOG_INFO("---- CCC not set timeout, disconnect...\n");
            AMVOS_TX_CCC_TIMER_STOP();
#endif // configUSE_BLE_ERROR_SW_RESET
            break;
        
        case AMVOS_TEST_TIMEOUT_TIMER_IND:
    //        AM_APP_LOG_DEBUG("---- test timer timeout, tick = %d\n", xTaskGetTickCount());
            AMVOS_TEST_TIMER_RESTART();
            break;
#endif // configUSE_AMVOS_AMA
#endif // configUSE_BLE_WATCHDOG

#if configUSE_BLE_Measure_Throughput
        case AMVOS_MEAS_TP_TIMER_IND:
            showThroughput();
            break;
#endif // configUSE_BLE_Measure_Throughput

        case ATTS_HANDLE_VALUE_CNF:
#if configUSE_BLE_Measure_Throughput
            g_ui32AmaDataCnfCnt++;
#endif // configUSE_BLE_Measure_Throughput
		
#if configUSE_BLE && configUSE_AUDIO_CODEC
            if((pMsg->att.handle == AMVOS_TX_HDL)&&(pMsg->hdr.status == ATT_SUCCESS))
            {
                uint8_t* pui8CommadBuffer = NULL;
                uint32_t ui32CommandLen = 0;

#if configUSE_BLE_WATCHDOG
                AMVOS_TX_TIMER_STOP();
#endif // configUSE_BLE_WATCHDOG
                if(am_vos_ble_nextdata_check(&pui8CommadBuffer, &ui32CommandLen))
                {
                    AmVosBleSend(pui8CommadBuffer, ui32CommandLen);
                }
                else
                {     
                    uint32_t data_len = am_app_utils_get_ring_buffer_status(&(g_sAmUtil.sRingBuf[AM_AUDIO_BUFFER_ENCODED]));
                    if(data_len >= BLE_DATA_BUFFER_SIZE)
                    {
                        uint8_t ui8AmVosTxBuf[BLE_DATA_BUFFER_SIZE + 3];
                        am_vos_ble_tx_packet_encap(ui8AmVosTxBuf, BLE_DATA_BUFFER_SIZE);
                        
                        AmVosBleSend(ui8AmVosTxBuf, BLE_DATA_BUFFER_SIZE + 3);
#if configUSE_BLE_Measure_Throughput
                        AM_APP_LOG_INFO("%d ", g_ui32AmaDataSentCnt - g_ui32AmaDataCnfCnt);
#endif // configUSE_BLE_Measure_Throughput
                    }
                    else
                    {
                        g_sVosBle.ui8VosTxBusy = 0;
                    }
                }
            }
#endif // configUSE_BLE && configUSE_AUDIO_CODEC
            break;

        case ATTS_CCC_STATE_IND:
            amvosProcCccState(pMsg);
#if configUSE_AMVOS_AMA
            if(pMsg->ccc.handle == AMVOS_TX_CH_CCC_HDL)
            {
#if configUSE_BLE_WATCHDOG
                AMVOS_TX_CCC_TIMER_STOP();
#endif // configUSE_BLE_WATCHDOG
                if(g_sVosBle.bAmaComSecured == true)
                {
                    AM_APP_LOG_INFO("[AM-VoS] AMA TX CCC enabled.\n");
                    AM_APP_LOG_INFO("[AMA] Version Exchange Send.\n");
                    am_vos_ama_tx_ver_exchange_send();
#if configUSE_BLE_WATCHDOG
                    AMVOS_TX_CCC_TIMER_RESTART();
#endif // configUSE_BLE_WATCHDOG
                    AM_APP_LOG_INFO("[AMA] AMVOS_TX_CCC_TIMER_RESTART();\n");
                    g_sVosBle.bAmaSetupFlag = true;
                }
                else
                {
                    // channel not secured
                    // probably iOS APP
                    /* request security */
                    AM_APP_LOG_INFO("[AM-VoS] Request for security from slave.\n");
                    g_sVosBle.bAmaSlaveSecReq = true;
                    AppSlaveSecurityReq(AppConnIsOpen());
                }
            }
#endif // configUSE_AMVOS_AMA
            break;

        case DM_RESET_CMPL_IND:
            AM_APP_LOG_INFO("[AM-VoS] DM_RESET_CMPL_IND\n");
            
            // clear ama channel flags
            g_sVosBle.ui8AmVosConnId = DM_CONN_ID_NONE;
#if configUSE_AMVOS_AMA
            am_vos_ama_status_reset();
#endif // configUSE_AMVOS_AMA
            AttsCalculateDbHash();
            DmSecGenerateEccKeyReq();
            amvosKwdSetDemoName();
            amvosSetup(pMsg);

#if USE_BLE_TX_POWER_SET
            HciVsEM_SetRfPowerLevelEx(TX_POWER_LEVEL_PLUS_6P2_dBm);
#endif // USE_BLE_TX_POWER_SET

            uiEvent = APP_UI_RESET_CMPL;
            break;

        case DM_ADV_START_IND:
            uiEvent = APP_UI_ADV_START;
            break;

        case DM_ADV_STOP_IND:
            uiEvent = APP_UI_ADV_STOP;
            break;

        case DM_CONN_OPEN_IND:
            AppSetBondable(TRUE);
            amvosOpen((dmEvt_t *)&pMsg->hdr);

#ifdef USE_BLE_OTA
            amotas_proc_msg(&pMsg->hdr);
#endif // USE_BLE_OTA

            //AM_APP_LOG_INFO("[AM-VoS] DmConnSetDataLen()\n");
            //DmConnSetDataLen(1, 251, 0x848);
            uiEvent = APP_UI_CONN_OPEN;
            retry_cnt = 0;
        
#if configUSE_BLE_WATCHDOG
            AMVOS_TEST_TIMER_RESTART();
#endif // configUSE_BLE_WATCHDOG

            break;

        case ATT_MTU_UPDATE_IND:
            AM_APP_LOG_INFO("[AM-VoS] ATT_MTU_UPDATE_IND AttGetMtu(), return = %d pMsg->att.mtu = %d\n", AttGetMtu(1), pMsg->att.mtu);
            if(pMsg->att.mtu < (BLE_DATA_BUFFER_SIZE+3))
            {
                if(retry_cnt < 5)
                {
                    retry_cnt++;
                    AttcMtuReq(1, 247);
                    AM_APP_LOG_INFO("[AM-VoS] AttcMtuReq retry_cnt = %d\n", retry_cnt);
                }
            }
            else
            {
                // once succeeded, clear retry count and enable next time to retry
                retry_cnt = 0;
                AM_APP_LOG_INFO("[AM-VoS] DmConnSetDataLen() %d\n", pMsg->att.mtu);
                DmConnSetDataLen(1, pMsg->att.mtu, 0x848);
            }
            break;

        case DM_CONN_DATA_LEN_CHANGE_IND:
            am_util_debug_printf("[AM-VoS] DM_CONN_DATA_LEN_CHANGE_IND: status = %d, max RX len = %d, max TX len = %d \n", pMsg->dm.dataLenChange.hdr.status, pMsg->dm.dataLenChange.maxRxOctets, pMsg->dm.dataLenChange.maxTxOctets);
            break;

        case DM_CONN_CLOSE_IND:
#if configUSE_BLE_WATCHDOG
            AMVOS_TX_TIMER_STOP();
            AMVOS_RSP_TIMER_STOP();
#endif // configUSE_BLE_WATCHDOG
            AM_APP_LOG_INFO("[AM-VoS] DM_CONN_CLOSE_IND reason = 0x%02x\n", pMsg->dm.connClose.reason);

            amvosClose(pMsg);
            uiEvent = APP_UI_CONN_CLOSE;
            am_vos_audio_flush_ring_buffer();
            am_vos_audio_wwd_enable();
          
#if configUSE_AMVOS_AMA
            g_sVosBle.bAmaComSecured = false;
            g_sVosBle.bAmaSlaveSecReq = false;
            g_sVosBle.bAmaSetupFlag = false;
            am_vos_ama_status_reset();
#endif // configUSE_AMVOS_AMA

            g_sVosBle.ui8AmVosConnId = DM_CONN_ID_NONE;

            //AppAdvStart(APP_MODE_AUTO_INIT);
            break;

        case DM_CONN_UPDATE_IND:
            amvosConnUpdate((dmEvt_t *)&pMsg->hdr);
#if 0
            if(pMsg->dm.connUpdate.connInterval > amvosUpdateCfg.connIntervalMax)
            {
                // retry
                amvosConnParameterReqSend();
            }
#endif
            break;

        case DM_SEC_PAIR_CMPL_IND:
#if configUSE_BLE_SECURE_CONNECTION
            DmSecGenerateEccKeyReq();
#endif // configUSE_BLE_SECURE_CONNECTION
            uiEvent = APP_UI_SEC_PAIR_CMPL;
            AM_APP_LOG_INFO("[AM-VoS] DM_SEC_PAIR_CMPL_IND.\n");
            break;

        case DM_SEC_PAIR_FAIL_IND:
#if configUSE_BLE_SECURE_CONNECTION
            DmSecGenerateEccKeyReq();
#endif // configUSE_BLE_SECURE_CONNECTION
            uiEvent = APP_UI_SEC_PAIR_FAIL;
            AM_APP_LOG_DEBUG("[AM-VoS] DM_SEC_PAIR_FAIL_IND. auth=0x%x status=0x%x\n", pMsg->dm.pairCmpl.auth, pMsg->hdr.status);
            //AppConnClose(AppConnIsOpen());
            break;

        case DM_SEC_ENCRYPT_IND:
            uiEvent = APP_UI_SEC_ENCRYPT;
#if configUSE_AMVOS_AMA
            AM_APP_LOG_INFO("[AM-VoS] DM_SEC_ENCRYPT_IND \n");
            g_sVosBle.bAmaComSecured = true;   // channel encrypted
            //AM_APP_LOG_INFO("[AM-VoS] g_sVosBle.ui8AmVosConnId = %d AppConnIsOpen() = %d\n", g_sVosBle.ui8AmVosConnId, AppConnIsOpen());
            if(AttsCccEnabled(AppConnIsOpen(), AMVOS_TX_CCC_IDX))
            {
#if configUSE_BLE_WATCHDOG
                AMVOS_TX_CCC_TIMER_STOP();
#endif // configUSE_BLE_WATCHDOG
                if(g_sVosBle.bAmaSetupFlag == false)
                {
                    AM_APP_LOG_INFO("[AM-VoS] AMA TX CCC enabled.\n");
                    AM_APP_LOG_INFO("[AMA] Version Exchange Send.\n");
                    am_vos_ama_tx_ver_exchange_send();
                    g_sVosBle.bAmaSetupFlag = true;
#if configUSE_BLE_WATCHDOG
                    AMVOS_TX_CCC_TIMER_RESTART();
#endif // configUSE_BLE_WATCHDOG
                    AM_APP_LOG_INFO("[AMA] AMVOS_TX_CCC_TIMER_RESTART();\n");
                }
            }
            else
            {
#if configUSE_BLE_WATCHDOG
                AMVOS_TX_CCC_TIMER_START();
#endif // configUSE_BLE_WATCHDOG
                AM_APP_LOG_INFO("[AMA] AMVOS_TX_CCC_TIMER_START();\n");
            }
#endif // configUSE_AMVOS_AMA
            break;

        case DM_SEC_ENCRYPT_FAIL_IND:
            uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
            //AppConnClose(AppConnIsOpen());
            //g_sVosBle.ui8AmVosConnId = DM_CONN_ID_NONE;
            break;

        case DM_SEC_AUTH_REQ_IND:
            AM_APP_LOG_INFO("[AM-VoS] DM_SEC_AUTH_REQ_IND \n");
            AppHandlePasskey(&pMsg->dm.authReq);
            break;
        case DM_SEC_PAIR_IND:
            AM_APP_LOG_INFO("[AM-VoS] DM_SEC_PAIR_IND \n");
            
            break;
        case DM_SEC_ECC_KEY_IND:
            DmSecSetEccKey(&pMsg->dm.eccMsg.data.key);
            break;

        case DM_SEC_COMPARE_IND:
            AM_APP_LOG_INFO("[AM-VoS] DM_SEC_COMPARE_IND \n");
            AppHandleNumericComparison(&pMsg->dm.cnfInd);
            break;
      
        case DM_VENDOR_SPEC_CMD_CMPL_IND:
            {
#if defined(AM_PART_APOLLO) || defined(AM_PART_APOLLO2)

                uint8_t *param_ptr = &pMsg->dm.vendorSpecCmdCmpl.param[0];

                switch (pMsg->dm.vendorSpecCmdCmpl.opcode)
                {
                    case 0xFC20: //read at address
                    {
                        uint32_t read_value;

                        BSTREAM_TO_UINT32(read_value, param_ptr);
                        AM_APP_LOG_INFO("[AM-VoS] VSC 0x%0x complete status %x param %x",
                          pMsg->dm.vendorSpecCmdCmpl.opcode,
                          pMsg->hdr.status,
                          read_value);
                    }

                    break;
                    default:
                          AM_APP_LOG_INFO("[AM-VoS] VSC 0x%0x complete status %x",
                              pMsg->dm.vendorSpecCmdCmpl.opcode,
                              pMsg->hdr.status);
                    break;
                }
#endif // AM_PART_APOLLO, AM_PART_APOLLO2
            }
            break;

        default:
            break;
    }

    if (uiEvent != APP_UI_NONE)
    {
        AppUiAction(uiEvent);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     AmVosHandlerInit
 *
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmVosHandlerInit(wsfHandlerId_t handlerId)
{
    AM_APP_LOG_INFO("[AM-VoS] AmVosHandlerInit\n");

    /* store handler ID */
    g_sVosBle.ui8AmVosHandlerId = handlerId;

    /* Set configuration pointers */
    pAppAdvCfg = (appAdvCfg_t *) &amvosAdvCfg;
    pAppSlaveCfg = (appSlaveCfg_t *) &amvosSlaveCfg;
    pAppSecCfg = (appSecCfg_t *) &amvosSecCfg;
    pAppUpdateCfg = (appUpdateCfg_t *) &amvosUpdateCfg;

    /* Initialize application framework */
    AppSlaveInit();
    AppServerInit();

    /* Set stack configuration pointers */
    pSmpCfg = (smpCfg_t *) &amvosSmpCfg;

#ifdef USE_BLE_OTA
    /* initialize amota service server */
    amotas_init(handlerId, (AmotasCfg_t *) &vosAmotaCfg);
#endif // USE_BLE_OTA

#if configUSE_BLE_Measure_Throughput
    measTpTimer.handlerId = handlerId;
    measTpTimer.msg.event = AMVOS_MEAS_TP_TIMER_IND;
#endif // configUSE_BLE_Measure_Throughput
    
#if configUSE_BLE_WATCHDOG
    g_sVosBle.sAmvosTxTimer.handlerId = handlerId;
    g_sVosBle.sAmvosTxTimer.msg.event = AMVOS_TX_TIMEOUT_TIMER_IND;
    
    g_sVosBle.sAmvosRspTimer.handlerId = handlerId;
    g_sVosBle.sAmvosRspTimer.msg.event = AMVOS_RSP_TIMEOUT_TIMER_IND;

    g_sVosBle.sAmvosTxCccTimer.handlerId = handlerId;
    g_sVosBle.sAmvosTxCccTimer.msg.event = AMVOS_TX_CCC_TIMEOUT_TIMER_IND;
    
    g_sVosBle.sAmvosTestTimer.handlerId = handlerId;
    g_sVosBle.sAmvosTestTimer.msg.event = AMVOS_TEST_TIMEOUT_TIMER_IND;
#endif // configUSE_BLE_WATCHDOG
}

/*************************************************************************************************/
/*!
 *  \fn     AmVosHandler
 *
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmVosHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    if (pMsg != NULL)
    {
        //AM_APP_LOG_INFO("Amvos got evt 0x%x", pMsg->event);

        if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END)
        {
            /* process advertising and connection-related messages */
            AppSlaveProcDmMsg((dmEvt_t *) pMsg);

            /* process security-related messages */
            AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
        }

        /* perform profile and user interface-related operations */
        amvosProcMsg((amvosMsg_t *) pMsg);
    }
}

/*************************************************************************************************/
/*!
 *  \fn     AmVosStart
 *
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AmVosStart(void)
{
    /* Register for stack callbacks */
    DmRegister(amvosDmCback);
    DmConnRegister(DM_CLIENT_ID_APP, amvosDmCback);
    AttRegister(amvosAttCback);
    AttConnRegister(AppServerConnCback);
    AttsCccRegister(AMVOS_NUM_CCC_IDX, (attsCccSet_t *) amvosCccSet, amvosCccCback);

    /* Register for app framework callbacks */
    AppUiBtnRegister(amvosBtnCback);

    // set up adv data
    memcpy(amvosAdvDataDisc, amvosAdvDataDiscDefault, sizeof(amvosAdvDataDiscDefault));
    memcpy(amvosScanDataDisc, amvosScanDataDiscDefault, sizeof(amvosScanDataDiscDefault));

    /* Initialize attribute server database */
    SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
    SvcCoreAddGroup();
    SvcDisAddGroup();
    SvcAmvosCbackRegister(NULL, amvos_write_cback);
    SvcAmvosAddGroup();

    /* Set Service Changed CCCD index. */
    GattSetSvcChangedIdx(AMVOS_GATT_SC_CCC_IDX);

#ifdef USE_BLE_OTA
    SvcAmotasCbackRegister(NULL, amotas_write_cback);
    SvcAmotasAddGroup();
#endif // USE_BLE_OTA

    /* Reset the device */
    DmDevReset();
}
