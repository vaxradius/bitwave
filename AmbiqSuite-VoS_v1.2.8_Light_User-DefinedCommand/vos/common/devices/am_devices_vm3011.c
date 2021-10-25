//*****************************************************************************
//
//! @file am_devices_vm3011.c
//!
//! @brief Vesper Vm3011 driver.
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"

#include "am_app_utils_stdio.h"
#include "am_vos_sys_config.h"

#include "am_devices_vm3011.h"

#if configUSE_WOS
#pragma location=0x10010000
uint32_t g_ui32WosDiscardTimeUS = 5000;
#pragma location=0x10010004
uint32_t g_ui32WosGpioNum = 34;
#pragma location=0x10010008
uint32_t g_ui32SleepGpioNum = 35;
#endif // configUSE_WOS

#if USE_DMIC_MB3_VM3011
//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
#define     VM3011_I2C_IOM              1
#define     AM_DEVICES_VESPER_SLAVE_ID  0x60

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
uint32_t    DMATCBBuffer_VM3011[256];
void*       g_IOMHandle;
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

//Relevant VM3011 register settings
//From Vesper.
const am_hal_vm3011_config_t g_sFarFieldConfig =
{
    .U0.addr0_b.wdt_enable = 0,
    .U0.addr0_b.wdt_dly =    0,
    .U0.addr0_b.dout_raw =   1
};

const am_hal_vm3011_config_t g_sNearFieldConfig =
{
  0
};

const uint8_t am_hal_vm3011_config_s[5][2] =
{
    0x0, 0x0,     //reg0
    0x2, 0xc,     //reg2
    0x3, 0x40,    //reg3
    0x4, 0x1f,    //reg4
    0x5, 0x2,     //reg5

};


//*****************************************************************************
//
// I2C configurations
//
//*****************************************************************************
am_hal_iom_config_t     g_sIomCfg =
{
#if defined (AM_PART_APOLLO2)
    .ui32InterfaceMode = AM_HAL_IOM_I2CMODE,
    .ui32ClockFrequency = AM_HAL_IOM_400KHZ, //AM_HAL_IOM_400KHZ,

    .ui8WriteThreshold = 12,
    .ui8ReadThreshold = 120,
#elif (defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P))
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_400KHZ,

    .pNBTxnBuf          = &DMATCBBuffer_VM3011[0],
    .ui32NBTxnBufLength = sizeof(DMATCBBuffer_VM3011) / 4
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P

};

void am_devices_vm3011_init(void)
{
#if defined (AM_PART_APOLLO2)
    //
    // Configure the IOM pins.
    //
    am_bsp_iom_i2c_pins_enable(VM3011_I2C_IOM);

    am_hal_gpio_pin_config(8, AM_HAL_PIN_8_M1SCL | AM_HAL_GPIO_PULLUP);
    am_hal_gpio_pin_config(9, AM_HAL_PIN_9_M1SDA | AM_HAL_GPIO_PULLUP);
	    
    am_hal_iom_pwrctrl_enable(VM3011_I2C_IOM);
    am_hal_iom_config(VM3011_I2C_IOM, &g_sIomCfg);

    //
    // Attach the IOM1 queue system to our allocated memory.
    //
    am_hal_iom_int_enable(VM3011_I2C_IOM, AM_HAL_IOM_INT_THR | AM_HAL_IOM_INT_CMDCMP);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER1);
    am_hal_interrupt_master_enable();
    
    //
    // Enable the IOM
    //
    am_hal_iom_enable(VM3011_I2C_IOM);
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(VM3011_I2C_IOM, AM_HAL_IOM_I2C_MODE);

    //
    // initialize IOM
    //
    am_hal_iom_initialize(VM3011_I2C_IOM, &g_IOMHandle);
    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(g_IOMHandle, &g_sIomCfg);
    am_hal_iom_enable(g_IOMHandle);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
static int32_t
am_device_command_write(void *pHandle, uint8_t ui8DevAddr, uint32_t ui32InstrLen,
                        uint32_t ui32Instr, bool bCont,
                        uint32_t *pData, uint32_t ui32NumBytes)
{
#if defined (AM_PART_APOLLO2)
    am_hal_iom_status_e ret;
    am_hal_iom_buffer(1) sData;
    uint8_t *ui8Data = (uint8_t *)pData;

    for(int i = 0; i < ui32NumBytes; i++)
        sData.bytes[i] = *(ui8Data++);

    ret = am_hal_iom_i2c_write(VM3011_I2C_IOM, ui8DevAddr, sData.words, ui32NumBytes, AM_HAL_IOM_OFFSET((uint8_t)ui32Instr));

    if(ret)
        return -1;

#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_hal_iom_transfer_t Transaction;

    //
    // Create the transaction.
    //
    Transaction.ui32InstrLen    = ui32InstrLen;
    Transaction.ui32Instr       = ui32Instr;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32TxBuffer   = pData;
    Transaction.uPeerInfo.ui32I2CDevAddr = ui8DevAddr;
    Transaction.bContinue       = bCont;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Execute the transction over IOM.
    //
    if (am_hal_iom_blocking_transfer(pHandle, &Transaction))
    {
        return -1;
    }
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
    return 0;
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
static int32_t
am_device_command_read(void *pHandle, uint8_t ui8DevAddr, uint32_t ui32InstrLen, uint32_t ui32Instr,
                       bool bCont, uint32_t *pData, uint32_t ui32NumBytes)
{
#if defined (AM_PART_APOLLO2)
    am_hal_iom_status_e ret;
    am_hal_iom_buffer(1) sData;
    uint8_t *ui8Data = (uint8_t *)pData;

    sData.bytes[0] = *ui8Data;
    
    ret = am_hal_iom_i2c_read(VM3011_I2C_IOM, ui8DevAddr, sData.words, ui32NumBytes, AM_HAL_IOM_OFFSET((uint8_t)ui32Instr));

    if(ret)
        return -1;

    for(int i = 0; i < ui32NumBytes; i++)
         *(ui8Data++) = sData.bytes[i];
    
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_hal_iom_transfer_t  Transaction;

    //
    // Create the transaction.
    //
    Transaction.ui32InstrLen    = ui32InstrLen;
    Transaction.ui32Instr       = ui32Instr;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32RxBuffer   = pData;
    Transaction.uPeerInfo.ui32I2CDevAddr = ui8DevAddr;
    Transaction.bContinue       = bCont;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Execute the transction over IOM.
    //
    if (am_hal_iom_blocking_transfer(pHandle, &Transaction))
    {
        return -1;
    }
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
    return 0;
}

//*****************************************************************************
int32_t am_vesper_i2c_blocking_write(uint32_t ui32WriteAddress, uint32_t value)
{

    //
    // Write the data to the device.
    //
#if defined (AM_PART_APOLLO2)
    if (am_device_command_write(NULL, AM_DEVICES_VESPER_SLAVE_ID, 1, ui32WriteAddress,
                            false, &value, 1))
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    if (am_device_command_write(g_IOMHandle, AM_DEVICES_VESPER_SLAVE_ID, 1, ui32WriteAddress,
                            false, &value, 1))
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
    {
        return -1;
    }

    //
    // Return the status.
    //
    return 0;
}

int32_t am_vesper_i2c_read_value(uint32_t addr, uint32_t *value)
{
    //
    // Send the command sequence to read the value.
    //
#if defined (AM_PART_APOLLO2)
    if (am_device_command_read(NULL, AM_DEVICES_VESPER_SLAVE_ID, 1, addr,
                           false, value, 1))
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    if (am_device_command_read(g_IOMHandle, AM_DEVICES_VESPER_SLAVE_ID, 1, addr,
                           false, value, 1))
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
    {
        return -1;
    }

    //
    // Return the status.
    //
    return 0;
}

void am_devices_vm3011_config()
{
    uint8_t i;
    //uint8_t value;

    for(i=0; i<5; i++)
      am_vesper_i2c_blocking_write(am_hal_vm3011_config_s[i][0],am_hal_vm3011_config_s[i][1]);
}

void am_devices_vm3011_deinit()
{
#if defined (AM_PART_APOLLO2)
    //
    // Disable the IOM.
    //
    am_hal_iom_disable(VM3011_I2C_IOM);

    //
    // Disable power to and uninitialize the IOM instance.
    //
    am_hal_iom_pwrctrl_disable(VM3011_I2C_IOM);

    am_bsp_iom_i2c_pins_disable(VM3011_I2C_IOM);
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    if ( VM3011_I2C_IOM > AM_REG_IOM_NUM_MODULES )
    {
        return;
    }

    //
    // Disable the IOM.
    //
    am_hal_iom_disable(g_IOMHandle);

    //
    // Disable power to and uninitialize the IOM instance.
    //
    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);

    am_hal_iom_uninitialize(g_IOMHandle);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}

uint32_t g_value;
void am_devices_vm3011_test()
{
    //uint32_t preValue = 0x55;
    uint32_t pValue = 0xAA;

    am_vesper_i2c_read_value(0x0, &pValue);
    //preValue = pValue;
    //am_vesper_i2c_blocking_write(0x0, 0x08);
    am_vesper_i2c_read_value(0x0, &pValue);
    //am_vesper_i2c_blocking_write(0x0, preValue);
    //am_vesper_i2c_read_value(0x0, &pValue);

    am_vesper_i2c_read_value(0x1, &pValue);
    am_vesper_i2c_read_value(0x2, &pValue);
    am_vesper_i2c_read_value(0x3, &pValue);
    am_vesper_i2c_read_value(0x4, &pValue);
    am_vesper_i2c_read_value(0x5, &g_value);

    return;
}

void am_vos_wos_debug_gpio_set(uint32_t gpio_num)
{
#if configUSE_WOS && USE_APOLLO3_BLUE_EVB
    if(gpio_num < AM_HAL_GPIO_MAX_PADS)
    {
        am_hal_gpio_pinconfig(gpio_num, g_AM_HAL_GPIO_OUTPUT);
        am_hal_gpio_state_write(gpio_num, AM_HAL_GPIO_OUTPUT_SET);
    }

    if(gpio_num == g_ui32WosGpioNum)
        AM_APP_LOG_INFO("DW\n");
#endif // configUSE_WOS && USE_APOLLO3_BLUE_EVB
}

void am_vos_wos_debug_gpio_clear(uint32_t gpio_num)
{
#if configUSE_WOS && USE_APOLLO3_BLUE_EVB
    if(gpio_num < AM_HAL_GPIO_MAX_PADS)
    {
        am_hal_gpio_state_write(gpio_num, AM_HAL_GPIO_OUTPUT_CLEAR);
    }

    if(gpio_num == g_ui32WosGpioNum)
        AM_APP_LOG_INFO("UP\n");
#endif // configUSE_WOS && USE_APOLLO3_BLUE_EVB
}

//*****************************************************************************
//
// Interrupt handler for IOM1
//
//*****************************************************************************
#if defined (AM_PART_APOLLO2)
void
am_iomaster1_isr(void)
{
    uint32_t ui32Status;

    ui32Status = am_hal_iom_int_status_get(1, true);

    am_hal_iom_int_clear(1, ui32Status);

    am_hal_iom_queue_service(1, ui32Status);
}
#endif // AM_PART_APOLLO2
#endif // USE_DMIC_MB3_VM3011
