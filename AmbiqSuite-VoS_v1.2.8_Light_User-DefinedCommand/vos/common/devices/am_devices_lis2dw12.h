//*****************************************************************************/
//
//! @file am_devices_LIS2DW12.h
//!
//! @brief Driver to interface with the LIS2DW12
//!
//! These functions implement the LIS2DW12 support routines for use on Ambiq
//! Micro MCUs.
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup LIS2DW12 SPI Device Control for the LIS2DW12 External Accelerometer
//! @ingroup devices
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2015, Ambiq Micro
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
// This is part of revision 1.0.4 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_DEVICES_LIS2DW12_H
#define AM_DEVICES_LIS2DW12_H

#ifdef __cplusplus
extern "C"
{
#endif

#define AM_BSP_LIS2DW12_IOM     0
#define AM_BSP_LIS2DW12_CS      2
#define LIS2DW12_SAMPLE_SIZE    32

//*****************************************************************************
//
// Structure for holding information about the LIS2DW12
//
//*****************************************************************************
typedef struct
{
    //
    // SPI or I2C mode.
    //
    bool bMode;
    //
    // Module number to use for IOM access.
    //
    uint32_t ui32IOMModule;

    //
    // Chip Select number to use for IOM access.
    //
    uint32_t ui32ChipSelect;

    //
    // Address for I2C communication. (unused in SPI mode)
    //
    uint32_t ui32Address;

    // Number of samples to collect before interrupt.  1 sample consists of
    // 6 bytes (2 bytes/axis)
    uint32_t ui32Samples;
}
am_devices_lis2dw12_t;

#if configUSE_OVVP_DOUBLE_TAP
extern am_devices_lis2dw12_t g_sACCEL_DW;
extern unsigned short clktrilvl;
extern uint8_t three_tap_counter, two_tap_counter;
extern int32_t  sll;
extern short s_flag;

extern void bf_energy(void);
extern void read_acc_run_ovvp(void);
#endif

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void sensor_en_wire3_spi(am_devices_lis2dw12_t *psDevice);
extern void am_devices_lis2dw12_config(am_devices_lis2dw12_t *psDevice);

extern void am_devices_lis2ds12_config(am_devices_lis2dw12_t *psDevice);

extern void am_devices_lis2dw12_config(am_devices_lis2dw12_t *psDevice);

extern void am_devices_lis2dw12_sample_get(am_devices_lis2dw12_t *psDevice,
                               uint32_t *psData,
                               am_hal_iom_callback_t pfnCallback);

extern uint8_t am_devices_lis2dw12_reg_read(am_devices_lis2dw12_t *psDevice,
                                           uint8_t ui8Register);

extern void am_devices_lis2dw12_reg_block_read(am_devices_lis2dw12_t *psDevice,
                                              uint8_t ui8StartRegister,
                                              uint32_t *pui32Values, uint32_t
                                              ui32NumBytes,
                                              am_hal_iom_callback_t
                                              pfnCallback);

extern uint8_t am_devices_lis2dw12_reg_write(am_devices_lis2dw12_t *psDevice,
                                         uint8_t ui8Register, uint8_t ui8Value);

extern void am_devices_lis2dw12_reg_block_write(
                                         am_devices_lis2dw12_t *psDevice,
                                         uint8_t ui8StartRegister,
                                         uint32_t *pui32Values,
                                         uint32_t ui32NumBytes,
                                         am_hal_iom_callback_t pfnCallback);

extern uint8_t am_devices_lis2dw12_device_id_get(
                                         am_devices_lis2dw12_t *psDevice);

extern uint8_t am_devices_lis2dw12_init(void);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_LIS2DW12_H
