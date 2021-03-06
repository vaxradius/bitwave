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
// This is part of revision v1.2.12 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "am_vos_sys_config.h"

#include "FreeRTOS.h"

#include "am_vos_init.h"
#include "am_vos_codec.h"

#if configUSE_AUDIO_CODEC

#if configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ
#include "sbc.h" 
#endif // configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ

#if configUSE_OPTIM_OPUS
#include "ae_api.h"
#endif // configUSE_OPTIM_OPUS

//*****************************************************************************
//
// Audio codec init
//
//*****************************************************************************

void am_vos_codec_init(void)
{
#if configUSE_SBC_BLUEZ
    sbc_encode_init(&(g_sVosSys.sBluezSBCInstance), 0);  //0: SBC
#endif // configUSE_SBC_BLUEZ

#if configUSE_MSBC_BLUEZ
    sbc_encode_init(&(g_sVosSys.sBluezSBCInstance), 1);  //1: MSBC
#endif // configUSE_MSBC_BLUEZ

#if configUSE_OPTIM_OPUS
    /* initialize the audio encoder */
    audio_enc_init(0);
#endif // configUSE_OPTIM_OPUS
}
//*****************************************************************************
//
// Audio codec encoder
//
//*****************************************************************************

void am_vos_codec_encode(void *codecInstance, const void *p_InputBuf, size_t input_len,
			                void *p_OutputBuf, size_t output_len, int *p_CompressedLen)
{
    uint32_t ui32EncoderReturn;

#if configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ
    ui32EncoderReturn = sbc_encoder_encode((sbc_t*)codecInstance, p_InputBuf, input_len, 
                                            p_OutputBuf, output_len, (int32_t *)p_CompressedLen);
    configASSERT(ui32EncoderReturn == CODEC_IN_RING_BUFF_SIZE);
#endif // configUSE_SBC_BLUEZ || configUSE_MSBC_BLUEZ

#if configUSE_OPTIM_OPUS
    ui32EncoderReturn = audio_enc_encode_frame((int16_t*)p_InputBuf, OPUS_IN_FRAME_SIZE, (unsigned char*)p_OutputBuf);
    *p_CompressedLen = (int32_t)ui32EncoderReturn;
#endif // configUSE_OPTIM_OPUS
}

#endif // configUSE_AUDIO_CODEC
