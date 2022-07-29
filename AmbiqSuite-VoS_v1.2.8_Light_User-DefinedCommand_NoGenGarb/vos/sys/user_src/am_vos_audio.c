
#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include "am_app_utils.h"
#include "am_app_utils_task.h"

#include "am_vos_task.h"
#include "am_vos_init.h"
#include "am_vos_spp.h"
#include "am_vos_audio.h"
#include "am_vos_codec.h"
#include "am_vos_ble.h"

#if configUSE_AWE
#include "am_vos_awe.h"
#endif // configUSE_AWE

#if USE_MAYA && configUSE_BUZZER
#include "am_app_utils_buzzer.h"
#endif // USE_MAYA && configUSE_BUZZER

#if configUSE_AMVOS_AMA
#include "am_vos_ama.h"
#endif // configUSE_AMVOS_AMA

#if configUSE_RTT_RECORDER
#include "am_vos_rtt_recorder.h"
#endif // configUSE_RTT_RECORDER

#if configUSE_AMU2S_RECORDER
#include "am_devices_amu2s.h"
#endif // configUSE_AMU2S_RECORDER

#if USE_DMIC_MB3_VM3011 || configUSE_WOS
#include "am_devices_vm3011.h"
#endif // USE_DMIC_MB3_VM3011 || configUSE_WOS

#if configUSE_Sensory_THF || configUSE_Cyberon_Spotter || configUSE_RetuneDSP_VS || configUSE_OAL_AID
int16_t g_i16WWDInputBuff[WWE_INPUT_FRAME_LENTH_SAMPLES];
#endif // configUSE_Sensory_THF || configUSE_Cyberon_Spotter || configUSE_RetuneDSP_VS || configUSE_OAL_AID

//
// small ring buffer used between SPP and WWE
//
static uint8_t s_SppToWweBuff[AM_SPP_FRAME_SAMPLES * SAMPLE_16BIT * 8];
am_app_utils_ring_buffer_t SppToWwe_ringbuff = 
{
    .pui8Data = s_SppToWweBuff,
    .ui32BufferTail_write = 0,
    .ui32BufferHead_read = 0,
    .ui32OverWriting = 0,
    .ui32Capacity = AM_SPP_FRAME_SAMPLES * SAMPLE_16BIT * 8
};

//*****************************************************************************
//
// Apollo SPP I/O structure configuration
//
//*****************************************************************************
am_spp_input_buff_t g_sSppInput =
{
    .ui32SppInChNum = AM_SPP_IN_CHANNEL_NUM,
    .ui32SppInDataFormat = AM_SPP_IN_DATA_FORMAT,
    .ui32SppInBlockSize = AM_SPP_FRAME_SAMPLES,
};

am_spp_output_buff_t g_sSppOutput =
{
    .ui32SppOutChNum = AM_SPP_OUT_CHANNEL_NUM,
    .ui32SppOutDataFormat = AM_SPP_OUT_DATA_FORMAT,
    .ui32SppOutBlockSize = AM_SPP_FRAME_SAMPLES,
};

//
// This process only take 1-channel data into WWE
//
void am_vos_stereo_to_mono_proc(int32_t *nLRSample, int16_t *nMonoSample)
{
    int32_t nSample;

    for (nSample = 0; nSample < AM_SPP_FRAME_SAMPLES; nSample++)
    {
        // Without voice pre-processing(e.g. Beamforming), using right MIC's data as a default.
        //nMonoSample[nSample] = nLRSample[nSample] >> 16;                   // Right channel
        nMonoSample[nSample] = nLRSample[nSample] & 0xFFFF;              // Left channel
    }
}

//-----------------------------------------------------------------------------
// METHOD:  am_vos_audio_handler
// PURPOSE: Pass new samples to SPP / WWD engine for processing
//-----------------------------------------------------------------------------

void am_vos_audio_handler(int32_t *nLRSample)
{   
#if configUSE_AWE
    int32_t nSample = 0;
    int16_t* pin16LeftChPtr;
    int16_t* pin16RightChPtr;
    pin16LeftChPtr = (int16_t*)g_sSppInput.SppInputArray[0];
    pin16RightChPtr = (int16_t*)g_sSppInput.SppInputArray[1];
    //
    // PCM data to left channel and right channel
    //
    for(uint32_t ui32ChIdx=0; ui32ChIdx<g_sSppInput.ui32SppInChNum; ui32ChIdx++)
    {
        for (nSample = 0; nSample < AM_SPP_FRAME_SAMPLES; nSample++)
        {
            if(ui32ChIdx==0)
            {
                pin16LeftChPtr[nSample] = (nLRSample[nSample])& 0xFFFF;
            }
            else if(ui32ChIdx==1)
            {
                pin16RightChPtr[nSample] = (nLRSample[nSample]>>16)& 0xFFFF;
            }
        }
    }
    //
    // audio data push into spp
    //
    am_spp_input_push(&g_sSppInput);
    //
    // spp data process
    //
    am_spp_process_handler();
    //
    // spp processed data pop out
    //
    am_spp_output_pop(&g_sSppOutput);
#else // configUSE_AWE
    am_vos_stereo_to_mono_proc(nLRSample, (int16_t*)g_sSppOutput.SppOutputArray[0]);
#endif // configUSE_AWE

    am_app_utils_ring_buffer_push(&SppToWwe_ringbuff, g_sSppOutput.SppOutputArray[0], AM_SPP_FRAME_SAMPLES * AM_SPP_OUT_DATA_FORMAT, false);

#if configUSE_Sensory_THF || configUSE_Cyberon_Spotter || configUSE_RetuneDSP_VS || configUSE_OAL_AID
    
    if(g_sVosSys.bWwdEnabled == true)
    {
        if(am_app_utils_get_ring_buffer_status(&SppToWwe_ringbuff) >= WWE_INPUT_FRAME_LENTH_SAMPLES * AM_SPP_OUT_DATA_FORMAT)
        {
            am_app_utils_ring_buffer_pop(&SppToWwe_ringbuff, g_i16WWDInputBuff, WWE_INPUT_FRAME_LENTH_SAMPLES * AM_SPP_OUT_DATA_FORMAT);
            am_vos_engine_process(g_i16WWDInputBuff, WWE_INPUT_FRAME_LENTH_SAMPLES);
        }
    }
    else
    {
        // 
        // we do not need to run keyword detection algorithm
        // flush the buffer
        //
        am_app_utils_flush_ring_buffer(&SppToWwe_ringbuff);
    }
#endif // configUSE_Sensory_THF || configUSE_Cyberon_Spotter || configUSE_RetuneDSP_VS || configUSE_OAL_AID

#if configUSE_RECORD_FULL_FILTER
#if configUSE_RTT_RECORDER
    //
    // Record the raw PCM data and send over RTT
    //
    if(g_sVosSys.ui8RecordStartFlag == 1)
        am_vos_rtt_record((void*)g_sSppOutput.SppOutputArray[0], AM_SPP_FRAME_SAMPLES * AM_SPP_OUT_DATA_FORMAT); 

#elif configUSE_AMU2S_RECORDER
    //
    // Record the raw PCM data and send over RTT
    //
    if(g_sVosSys.ui8RecordStartFlag == 1)
        amu2s_send(Amu2s_spp, void*)g_sSppOutput.SppOutputArray[0], AM_SPP_FRAME_SAMPLES * AM_SPP_OUT_DATA_FORMAT);
#endif // configUSE_RTT_RECORDER, configUSE_RECORD_FULL_FILTER
#endif // configUSE_RECORD_FULL_FILTER

#if USE_UNIVERSAL_AUDIO_BUFFER
    //am_audio_buffer_push(AM_AUDIO_BUFFER_MONO, g_sSppOutput.SppOutputArray[0], AWE_FRAME_SIZE*BYTES_PER_DSPC_SAMPLE);
    am_audio_buffer_nested_push(AM_AUDIO_BUFFER_MONO, g_sSppOutput.SppOutputArray[0], AM_SPP_FRAME_SAMPLES * AM_SPP_OUT_DATA_FORMAT);
#endif // USE_UNIVERSAL_AUDIO_BUFFER
    
    if (g_sVosSys.ui8KwdDetectedFlag || g_sVosSys.ui8PushTalkFlag || g_sVosSys.ui8ProvideSpeechFlag)
    {
#if configUSE_DSPC_QSD || configUSE_WOS
        am_vos_qsd_wos_threshold_to_low();
#endif // configUSE_DSPC_QSD || configUSE_WOS

#if configUSE_AWE
        am_vos_awe_scnr_off();
#endif // configUSE_AWE                                             
        if (g_sVosSys.ui32StreamingTimeOutCount < AUDIO_KWD_TIMEOUT_S * AM_SPP_SAMPLE_FREQ)
        {
#if configUSE_BLE
            if(g_sVosSys.ui32StreamingTimeOutCount == 0)
            {
#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
                // burst to 96MHz (only supported with Apollo 3)
                am_vos_burst_mode_enable();
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
                am_vos_audio_buffer_rewind();  // in conversation mode, dial back only for 100ms...
            
#if configUSE_AMVOS_AMA
                // keyword detected, disable barge-in before Alexa understands us
                if (am_vos_is_connected())
                {
                    AM_APP_LOG_INFO("[AM-VoS] Disable WakeWord Detection\n");                    
                    am_vos_audio_wwd_disable();

                    if (!g_sVosSys.ui8ProvideSpeechFlag)
                    {
                        AmaSpeechInfo sAmaSpeechInfo = 
                        {
                            .ui8PushTalkFlag = g_sVosSys.ui8PushTalkFlag,
                            .bPreRollEnabled = configUSE_PREROLL,
                            .i32PreRollDelayMs = g_sVosSys.i32PreRollDelayMs,
                            .i32DelayedSample = g_sVosSys.i32DelayedSample,
                            .i32EndingSampleCnt = g_sVosSys.i32EndingSampleCnt
                        };
                        am_vos_ama_start_speech_send(&sAmaSpeechInfo);
                    }
                }
                else
                {
                    AM_APP_LOG_WARNING("---- Not connected to Alexa App.----\n");
                }
#endif // configUSE_AMVOS_AMA
            }
#endif // configUSE_BLE
#if configUSE_AMVOS_AMA
            if (am_vos_is_connected())
#endif // configUSE_AMVOS_AMA
            {
#if configUSE_AUDIO_CODEC
                am_app_utils_task_send(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_CODEC,
                                        AM_APP_MESSAGE_LONG, AM_SPP_OUT_DATA_FORMAT*AM_SPP_FRAME_SAMPLES, 
                                        NULL);  
#endif // configUSE_AUDIO_CODEC
            }
            g_sVosSys.ui32StreamingTimeOutCount += AM_SPP_FRAME_SAMPLES;
        }
        else
            am_vos_audio_reset_flag_and_buffer();
    }
}

void am_vos_reset_detected_flag()
{
    g_sVosSys.ui8KwdDetectedFlag = 0;
    g_sVosSys.ui8PushTalkFlag = 0;
    g_sVosSys.ui8ProvideSpeechFlag = 0;
    g_sVosSys.ui32StreamingTimeOutCount = 0;

#if configUSE_WOS && USE_DMIC_MB3_VM3011
    am_vos_wos_debug_gpio_clear(g_ui32WosGpioNum);
#endif // configUSE_WOS && USE_DMIC_MB3_VM3011

    am_vos_audio_wwd_enable();

#if configUSE_AWE
    am_vos_awe_scnr_on();
#endif // configUSE_AWE

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
#if (!configUSE_AIC_LITE_LAYOUT)
    // AIC layout requires burst mode always on
    am_vos_burst_mode_disable();
#endif // (!configUSE_AIC_LITE_LAYOUT)
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
}

void am_vos_audio_flush_ring_buffer(void)
{
#if USE_UNIVERSAL_AUDIO_BUFFER
    am_audio_buffer_init();
#endif // USE_UNIVERSAL_AUDIO_BUFFER
}

void am_vos_audio_buffer_rewind(void)
{
    AM_CRITICAL_BEGIN_VOS;
#if configUSE_AMVOS_AMA && configUSE_PREROLL
    am_audio_buffer_rewind(AM_AUDIO_BUFFER_MONO, ((g_sVosSys.i32DelayedSample * SAMPLE_16BIT + SAMPLE_16BIT* (AM_SPP_SAMPLE_FREQ/1000) * AUDIO_PREROLL_TIME_MS) / CODEC_IN_RING_BUFF_SIZE + 1) * CODEC_IN_RING_BUFF_SIZE); // start index + 500ms preroll, 32000bytes of samples = 1000ms
#else // configUSE_AMVOS_AMA && configUSE_PREROLL
    am_audio_buffer_rewind(AM_AUDIO_BUFFER_MONO, (3200 / CODEC_IN_RING_BUFF_SIZE + 1) * CODEC_IN_RING_BUFF_SIZE); // rewind 100ms voice to overcome the detection delay
#endif // configUSE_AMVOS_AMA && configUSE_PREROLL
    AM_CRITICAL_END_VOS;
}

void am_vos_audio_reset_flag_and_buffer(void)
{
    am_vos_audio_flush_ring_buffer();
    am_vos_reset_detected_flag();

#if configUSE_PUSH_TO_TALK
    am_vos_gpio_enable_irq(PUSH_TO_TALK_BUTTON);
#endif // configUSE_PUSH_TO_TALK
}

void am_vos_qsd_wos_threshold_to_high(void)
{
  //
  //  Change QSD and WoS threshold to higher (SNR) value until max.
  //
}

void am_vos_qsd_wos_threshold_to_low(void)
{
  //
  //  Change QSD and WoS threshold to lower (SNR) value until min.
  //
}

void am_vos_audio_wwd_enable(void)
{
    if(g_sVosSys.bWwdEnabled == false)
        AM_APP_LOG_DEBUG("[AM-VoS] Enable WakeWord Detection\n");
    g_sVosSys.bWwdEnabled = true;
    
    return;
}

void am_vos_audio_wwd_disable(void)
{
    if(g_sVosSys.bWwdEnabled == true)
        AM_APP_LOG_DEBUG("[AM-VoS] Disable WakeWord Detection\n");
    g_sVosSys.bWwdEnabled = false;

    return;
}
