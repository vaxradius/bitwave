//*****************************************************************************
//
//! @file am_app_utils.c
//!
//! @brief A few printf-style functions for freeRTOS application use
//!
//! Functions for performing printf-style operations without dynamic memory
//! allocation.
//
//*****************************************************************************
#include "am_app_utils.h"

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
AmVosUtil g_sAmUtil =
{
    .pfnPrintf = NULL,
    .ui32BuffIndx = 0,
#if configUSE_GSENSOR_MOTION
    .ui32FirstTick = 0,
#endif // configUSE_GSENSOR_MOTION
};
