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

#include "accessories.pb.h"

#include "am_app_utils_stdio.h"
#include "am_vos_init.h"
#include "am_vos_ama.h"

#if configUSE_AMVOS_AMA
void vos_ama_proc_notify_speech(SpeechState state)
{
    switch(state)
    {
        case SpeechState_IDLE :
            AM_APP_LOG_INFO("[AMA] Cmd NOTIFY_SPEECH_STATE recv IDLE\n");
            break;

        case SpeechState_LISTENING :
            AM_APP_LOG_INFO("[AMA] Cmd NOTIFY_SPEECH_STATE recv LISTENING\n");
            break;

        case SpeechState_PROCESSING :
            AM_APP_LOG_INFO("[AMA] Cmd NOTIFY_SPEECH_STATE recv PROCESSING\n");
            break;

        case SpeechState_SPEAKING :
            AM_APP_LOG_INFO("[AMA] Cmd NOTIFY_SPEECH_STATE recv SPEAKING\n");
            break;

        default :  
            AM_APP_LOG_INFO("[AMA] Cmd NOTIFY_SPEECH_STATE recv unknown %d\n", state);
            break;
    }
    return;
}
void vos_ama_proc_get_state(uint32_t feature)
{
    State sState = {0,};

    sState.feature = feature;

    switch(feature)
    {
        case 0x130:     // Bluetooth A2DP Enabled
            sState.which_value = sizeof(bool);
            sState.value.boolean = false;              // A2DP is disabled
            break;

        case 0x131:     // Bluetooth HFP Enabled
            sState.which_value = sizeof(bool);
            sState.value.boolean = false;              // HFP is disabled
            break;

        case 0x132:     // Bluetooth A2DP Connected
            sState.which_value = sizeof(bool);
            sState.value.boolean = false;              // A2DP is not connected
            break;

        case 0x133:     // Bluetooth HFP Connected
            sState.which_value = sizeof(bool);
            sState.value.boolean = false;              // HFP is not connected
            break;

        case 0x203:     // DND (Do not disturb) mode
            sState.which_value = sizeof(bool);
            sState.value.boolean = false;              // DND is disabled
            break;

        case 0x204:     // Privacy mode
            sState.which_value = sizeof(bool);
            sState.value.boolean = false;              // Privacy mode is disabled
            break;

        default:  
            break;
    }

    if(sState.which_value)
        am_vos_ama_get_state_rsp_send(&sState);
    else
        am_vos_ama_rsp_send(Command_GET_STATE, ErrorCode_UNSUPPORTED);      // if unknown feature was requested.

    return;
}

void vos_ama_proc_set_state(State *pState)
{
    switch(pState->feature)
    {
        case 0x400 :    // AVRCP Override
            if(pState->value.boolean == 0)
            {
                AM_APP_LOG_DEBUG("[AMA] Regular AVRCP commands are being sent.\n");
            }
            else if(pState->value.boolean == 1)
            {
                AM_APP_LOG_DEBUG("[AMA] Accessory is sending Media control commands.\n");
            }
            am_vos_ama_rsp_send(Command_GET_STATE, ErrorCode_SUCCESS);
            break;

        default :
            AM_APP_LOG_DEBUG("[AMA] Unknown feature 0x%X (Not supported)\n", pState->feature);
            am_vos_ama_rsp_send(Command_GET_STATE, ErrorCode_UNSUPPORTED);
            break;
    }
    return;
}

void vos_ama_proc_sync_state(State *pState)
{
    switch(pState->feature)
    {
        case 0x135 :    // Bluetooth A2DP Active
            if(pState->value.boolean == 0)
            {
                AM_APP_LOG_DEBUG("[AMA] A2DP is not active\n");
            }
            else if(pState->value.boolean == 1)
            {
                AM_APP_LOG_DEBUG("[AMA] A2DP is active\n");
            }
            break;

        case 0x136 :    // Bluetooth HFP Active
            if(pState->value.boolean == 0)
            {
                AM_APP_LOG_DEBUG("[AMA] HFP is not active\n");
            }
            else if(pState->value.boolean == 1)
            {
                AM_APP_LOG_DEBUG("[AMA] HFP is active\n");
            }
            break;

        case 0x203 :
            if(pState->value.integer == 0)      // Device Network Connectivity Status
            {
                AM_APP_LOG_DEBUG("[AMA] Network is in good condition\n");
            }
            else if(pState->value.integer == 1)
            {
                AM_APP_LOG_DEBUG("[AMA] Network is slow\n");
            }
            else if(pState->value.integer == 2)
            {
                AM_APP_LOG_DEBUG("[AMA] Network is not available\n");
            }
            else
            {
                AM_APP_LOG_DEBUG("[AMA] Invailid value: %d\n", pState->value.integer);
            }
            break;

        default :
            break;
    }
    am_vos_ama_rsp_send(Command_SYNCHRONIZE_STATE, ErrorCode_SUCCESS);

    return;
}

void am_vos_ama_event_callback(Command cmd, void* pMsg)
{

    ProvideSpeech *pProvideSpeech;
    StopSpeech *pStopSpeech;
    EndpointSpeech *pEndpointSpeech;
    NotifySpeechState *pSpeechState;
    GetState *pGetState;
    SetState *pSetState;
    SynchronizeState *pSyncState;

    switch(cmd)
    {
        case Command_PROVIDE_SPEECH :
            pProvideSpeech = (ProvideSpeech *)pMsg;
            AM_APP_LOG_INFO("[AMA] Cmd PROVIDE_SPEECH dialog = %d\n", pProvideSpeech->dialog.id);
            break;

        case Command_STOP_SPEECH :
            pStopSpeech = (StopSpeech *)pMsg;
            AM_APP_LOG_INFO("[AMA] Cmd STOP_SPEECH dialog %d err_no %d recv\n", pStopSpeech->dialog.id, pStopSpeech->error_code);
            break;

        case Command_ENDPOINT_SPEECH :
            pEndpointSpeech = (EndpointSpeech *)pMsg;
            AM_APP_LOG_INFO("[AMA] Cmd ENDPOINT_SPEECH dialog %d recv\n", pEndpointSpeech->dialog.id);
            break;

        case Command_NOTIFY_SPEECH_STATE :
            pSpeechState = (NotifySpeechState *)pMsg;
            vos_ama_proc_notify_speech(pSpeechState->state);
            break;
            
        case Command_GET_DEVICE_INFORMATION :
            AM_APP_LOG_INFO("[AMA] Cmd GET_DEVICE_INFORMATION recv\n");
            break;

        case Command_GET_DEVICE_CONFIGURATION :
            AM_APP_LOG_INFO("[AMA] Cmd GET_DEVICE_CONFIGURATION recv\n");
            break;

        case Command_START_SETUP :
            AM_APP_LOG_INFO("[AMA] Cmd START_SETUP recv\n");
            break;

        case Command_COMPLETE_SETUP :
            AM_APP_LOG_INFO("[AMA] Cmd COMPLETE_SETUP recv\n");
            break;

        case Command_KEEP_ALIVE :
            AM_APP_LOG_INFO("[AMA] Cmd KEEP_ALIVE recv\n");
            break;
            
        case Command_SYNCHRONIZE_SETTINGS :
            AM_APP_LOG_INFO("[AMA] Cmd SYNCHRONIZE_SETTINGS recv\n");
            break;

        case Command_GET_STATE :
            pGetState = (GetState *)pMsg;
            AM_APP_LOG_INFO("[AMA] Cmd GET_STATE recv feature 0x%x\n", pGetState->feature);
            vos_ama_proc_get_state(pGetState->feature);
            break;

        case Command_SET_STATE :
            pSetState = (SetState *)pMsg;
            AM_APP_LOG_INFO("[AMA] Cmd Command_SET_STATE recv feature 0x%x value %d\n",
                            pSetState->state.feature, pSetState->state.which_value == 1?
                              pSetState->state.value.boolean : pSetState->state.value.integer);
            vos_ama_proc_set_state(&(pSetState->state));
            break;

        case Command_SYNCHRONIZE_STATE :
            pSyncState = (SynchronizeState *)pMsg;
            AM_APP_LOG_INFO("[AMA] Cmd SYNCHRONIZE_STATE recv feature 0x%x value %d\n",
                            pSyncState->state.feature, pSyncState->state.which_value == 1?
                              pSyncState->state.value.boolean : pSyncState->state.value.integer);

            vos_ama_proc_sync_state(&(pSyncState->state));
            break;

        default :
            //AM_APP_LOG_INFO("[AMA] Cmd %d recv\n", cmd);
            break;
    }
}
#endif
