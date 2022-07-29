//*****************************************************************************
//
//! @file am_spp_platform.h
//!
//! @brief header file of am_spp_platform.c
//! All configurations of SPP module
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef __AM_SPP_PLATFORM_H__
#define __AM_SPP_PLATFORM_H__
//
// configuration header file
//
#include "am_vos_sys_config.h"

#include "am_util.h"

//*****************************************************************************
//
// Apollo platform configuration
//
//*****************************************************************************
#define AM_SPP_LOG(...)                                             am_util_stdio_printf(__VA_ARGS__)

//
// AWE error check handler
//
extern void spp_error_handler(uint32_t ui32ErrorStatus);
#define SPP_CHECK_ERRORS(x)                                                       \
    if ((x) != 0)                                                                 \
    {                                                                             \
        spp_error_handler(x);                                                     \
    }

//
// DSPC layout selection
// There're some pre-designed layout for test and demo
// Note: please make sure the selection here is ALIGNED WITH the selected AWB file in IAR project 
//
//#define AM_configUSE_AIC_ORIG_LAYOUT                                0
//#define AM_configUSE_AIC_LITE_LAYOUT                                1
//#define AM_configUSE_VOS_QUAL_LAYOUT                                0
//#define AM_configUSE_BF_SCNR2_LAYOUT                                0
//
// audio data common format
//
#define SAMPLE_8BIT                                                 1
#define SAMPLE_16BIT                                                2
#define SAMPLE_32BIT                                                4

//*****************************************************************************
//
// AWE layout and apollo platform related configuration
// No need of manual modification
//
//*****************************************************************************
//
// awe layout related parameters
//
#if configUSE_AWE
#if configUSE_AIC_LITE_LAYOUT
#define AWE_FRAME_SIZE_SAMPLES                                      160
#define AWE_SAMPLE_FREQ                                             16000
#define AWE_INPUT_CH_NUM                                            2
#define AWE_OUTPUT_CH_NUM                                           1

#elif configUSE_ALEXA_QUAL_LAYOUT
#define AWE_FRAME_SIZE_SAMPLES                                      80
#define AWE_SAMPLE_FREQ                                             16000
#define AWE_INPUT_CH_NUM                                            2
#define AWE_OUTPUT_CH_NUM                                           1

#else
//
// if self-defined layout used, please input the related params
//
#define AWE_FRAME_SIZE_SAMPLES                                      
#define AWE_SAMPLE_FREQ                                             
#define AWE_INPUT_CH_NUM                                            
#define AWE_OUTPUT_CH_NUM                                           
#endif // configUSE_AIC_LITE_LAYOUT, configUSE_ALEXA_QUAL_LAYOUT

#endif // configUSE_AWE

//
// apollo platform parameters
//
#if configUSE_AWE
#define AM_SPP_FRAME_SAMPLES                                        AWE_FRAME_SIZE_SAMPLES
#define AM_SPP_SAMPLE_FREQ                                          AWE_SAMPLE_FREQ

#elif configUSE_RetuneDSP_VS
#define AM_SPP_FRAME_SAMPLES                                        200
#define AM_SPP_SAMPLE_FREQ                                          16000    

#else
#define AM_SPP_FRAME_SAMPLES                                        80
#define AM_SPP_SAMPLE_FREQ                                          16000    
#endif // configUSE_AWE, configUSE_RetuneDSP_VS

#define AM_SPP_IN_SAMPLE_BYTES                                      SAMPLE_32BIT
#define AM_SPP_OUT_SAMPLE_BYTES                                     SAMPLE_16BIT
#define AM_SPP_IN_CHANNEL_NUM                                       2
#define AM_SPP_IN_DATA_FORMAT                                       SAMPLE_16BIT
#define AM_SPP_OUT_CHANNEL_NUM                                      1
#define AM_SPP_OUT_DATA_FORMAT                                      SAMPLE_16BIT

//
// SPP input data config structure
//
typedef struct
{
    //
    // SPP input data channel number
    //
    const uint32_t ui32SppInChNum;
    const uint32_t ui32SppInDataFormat;
    const uint32_t ui32SppInBlockSize;
    uint8_t SppInputArray[AM_SPP_IN_CHANNEL_NUM][AM_SPP_IN_DATA_FORMAT*AM_SPP_FRAME_SAMPLES];
}am_spp_input_buff_t;

//
// SPP output data config structure
//
typedef struct
{
    //
    // SPP input data channel number
    //
    const uint32_t ui32SppOutChNum;
    const uint32_t ui32SppOutDataFormat;
    const uint32_t ui32SppOutBlockSize;
    uint8_t SppOutputArray[AM_SPP_OUT_CHANNEL_NUM][AM_SPP_OUT_DATA_FORMAT*AM_SPP_FRAME_SAMPLES];
}am_spp_output_buff_t;

#if configUSE_DSPC_QSD
//
// QSD state structure
//
typedef enum
{
    VOS_QSD_INIT = 0,
    VOS_QSD_NOISY,
    VOS_QSD_DELAYED,
    VOS_QSD_QUIET
} eVosQSDStatus_t;
#endif // configUSE_DSPC_QSD

extern volatile bool g_bSppInDataReady;

extern int16_t g_pin16SppLeftInBuff[AM_SPP_FRAME_SAMPLES];
extern int16_t g_pin16SppRightInBuff[AM_SPP_FRAME_SAMPLES];
extern int16_t g_pin16SppOutBuff[AM_SPP_FRAME_SAMPLES];

extern am_spp_input_buff_t g_sSppInput;
extern am_spp_output_buff_t g_sSppOutput;

extern void am_spp_init(am_spp_input_buff_t* spp_in, am_spp_output_buff_t* spp_out);
extern void am_spp_input_push(am_spp_input_buff_t* spp_input);
extern void am_spp_output_pop(am_spp_output_buff_t* spp_output);
extern void am_spp_process_handler(void);

#endif // __AM_SPP_PLATFORM_H__
