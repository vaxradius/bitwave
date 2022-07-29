//*****************************************************************************
//
//! @file am_devices_vm3011.h
//!
//! @brief Vesper VM3011 driver.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro
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
// This is part of revision 2.4.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_DEVICES_VM3011_H
#define AM_DEVICES_VM3011_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Global definitions for vm3011 commands
//
//*****************************************************************************
typedef struct  // Future use - not currently used for Apollo3.
{
    union
    {
        volatile  uint8_t Add_0;
        struct
        {
            volatile uint8_t   wdt_enable:  1;
            volatile uint8_t   wdt_dly:     2;
            volatile uint8_t   dout_raw:    1;

        } addr0_b;
    } U0;

    union
    {
        volatile  uint8_t Add_1;
        struct
        {
            volatile uint8_t   wos_pga_gain:  5;

        } addr1_b;
    } U1;

    union
    {
        volatile  uint8_t Add_2;
        struct
        {
            volatile uint8_t   wos_lpf:  2;
            volatile uint8_t   wos_hpf:  2;

        } addr2_b;
    } U2;
    
    union
    {
        volatile  uint8_t Add_3;
        struct
        {
            volatile uint8_t   wos_pga_min_thr:  5;
            volatile uint8_t   fast_mode_cnt:    2;

        } addr3_b;
    } U3;

    union
    {
        volatile  uint8_t Add_4;
        struct
        {
            volatile uint8_t   wos_pga_max_thr:  5;
            volatile uint8_t   wos_rms:          1;

        } addr4_b;
    } U4;
}am_hal_vm3011_config_t;

void am_devices_vm3011_init(void);
void am_devices_vm3011_config(void);
void am_devices_vm3011_deinit(void);
void am_devices_vm3011_test(void);

#if configUSE_WOS
extern void am_vos_wos_debug_gpio_set(uint32_t);
extern void am_vos_wos_debug_gpio_clear(uint32_t);

extern uint32_t g_ui32WosDiscardTimeUS;
extern uint32_t g_ui32WosGpioNum;
extern uint32_t g_ui32SleepGpioNum;
#endif // configUSE_WOS

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_VM3011_H
