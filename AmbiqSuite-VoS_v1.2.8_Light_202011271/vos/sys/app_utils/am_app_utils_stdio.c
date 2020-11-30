//*****************************************************************************
//
//! @file am_app_utils_stdio.c
//!
//! @brief A few printf-style functions for freeRTOS application use
//!
//! Functions for performing printf-style operations without dynamic memory
//! allocation.
//
//*****************************************************************************
#include "am_util.h"
#include "am_app_utils.h"

#include "am_app_utils_stdio.h"

//*****************************************************************************
//
//! @brief Sets the interface for printf calls.
//!
//! @param pfnPrintFunc - Function pointer to be used to print to interface
//!
//! This function initializes the global print function which is used for
//! printf. This allows users to define their own printf interface and pass it
//! in as an am_app_utils_stdio_print_t type. 
//!
//! @return None.
//
//*****************************************************************************
void
am_app_utils_stdio_printf_init(am_app_utils_stdio_printf_t pfnPrintFunc)
{
    g_sAmUtil.pfnPrintf = pfnPrintFunc;
}

//*****************************************************************************
//
//! @brief A lite version of printf()
//!
//! @param *pcFmt - Pointer to formatter string
//!
//!
//!
//! @return uint32_t representing the number of characters printed.
//
// *****************************************************************************
uint32_t
am_app_utils_stdio_printf(uint8_t print_type, const char *pcFmt, ...)
{
    uint32_t ui32NumChars;
    uint8_t print_buff[AM_APP_PRINTF_BUFFSIZE]; // local buffer to handle the data

    AM_CRITICAL_BEGIN;
    
    configASSERT(strlen(pcFmt) < AM_APP_PRINTF_BUFFSIZE);
    
    //
    // Convert to the desired string.
    //
    va_list pArgs;
    va_start(pArgs, pcFmt);
    ui32NumChars = am_util_stdio_vsprintf((char *)print_buff, pcFmt, pArgs);
    va_end(pArgs);

    if(g_sAmUtil.ui32BuffIndx + ui32NumChars > (AM_APP_PRINTF_TOTAL_BUFFSIZE - 1))
    {
        g_sAmUtil.ui32BuffIndx = 0;
    }

    // move data into global print buffer
#if configUSE_PRINTF_UART0
    if(print_type == 1) {
        strcpy((char *)(&(g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx])), "\033[1;32m");
        memcpy((char *)(&(g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx + 7])), (char *)print_buff, ui32NumChars);
        strcpy((char *)(&(g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx + 7 + ui32NumChars])), "\033[0m");
        ui32NumChars += 11;
    }
    else if(print_type == 2) {
        strcpy((char *)((&g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx])), "\033[1;31m");
        memcpy((char *)((&g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx + 7])), (char *)print_buff, ui32NumChars);
        strcpy((char *)((&g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx + 7 + ui32NumChars])), "\033[0m");
        ui32NumChars += 11;
    }
    else {
        memcpy((char *)(&(g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx])), (char *)print_buff, ui32NumChars);
    }
#else // configUSE_PRINTF_UART0
    memcpy((char *)(&(g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx])), (char *)print_buff, ui32NumChars);
#endif // configUSE_PRINTF_UART0
    
    g_sAmUtil.pcStdioBuff[g_sAmUtil.ui32BuffIndx + ui32NumChars] = NULL;

    //
    // This is where we print the buffer to the configured interface.
    //
    if(g_sAmUtil.pfnPrintf)
        g_sAmUtil.pfnPrintf(g_sAmUtil.ui32BuffIndx, print_type);
    
    g_sAmUtil.ui32BuffIndx += ui32NumChars + 1;   //add 1 byte NULL
    
    AM_CRITICAL_END;

    //
    // return the number of characters printed.
    //
    return ui32NumChars;
}


