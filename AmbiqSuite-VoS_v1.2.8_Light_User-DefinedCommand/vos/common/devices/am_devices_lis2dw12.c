//*****************************************************************************
//
//! @file am_app_utils_gsensor.c
//!
//! @brief buzzer interface for Maya
//!
//! Functions for performing buzzer actions
//
//*****************************************************************************
#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include "am_util_delay.h"
#include "am_app_utils.h"

#include "am_vos_init.h"

#include "am_devices_lis2dw12.h"
#include "am_app_utils_gsensor.h"

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
#if configUSE_OVVP_DOUBLE_TAP
am_devices_lis2dw12_t g_sACCEL_DW =
{
    .ui32IOMModule = AM_BSP_LIS2DW12_IOM,
    .ui32ChipSelect = AM_BSP_LIS2DW12_CS,
    .ui32Samples = LIS2DW12_SAMPLE_SIZE
};

//*****************************************************************************
//
// Variables
//
//*****************************************************************************

short Acc[3];
uint32_t slv;
int32_t  sll;
#endif // configUSE_OVVP_DOUBLE_TAP

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
//*****************************************************************************
//
//! @brief Reads an internal register in the lis2dw12.
//!
//! @param psDevice is a pointer to a device structure describing the lis2dw12.
//! @param ui32Register is the address of the register to read.
//! @param ui32Value is the value to read to the register.
//!
//! This function performs a read to an lis2dw12 register over the serial bus.
//!
//! @return
//
//*****************************************************************************
uint8_t
am_devices_lis2dw12_reg_read(am_devices_lis2dw12_t *psDevice,
                             uint8_t ui8Register)
{
#if defined (AM_PART_APOLLO2)
    uint8_t ui8Offset;
    am_hal_iom_buffer(1) sData;

    //
    // Build the SPI offset and the data buffer.
    //
    ui8Offset = 0x80 | ui8Register;

    //
    // Send the read to the bus using the polled API.
    //
    am_hal_iom_spi_read(psDevice->ui32IOMModule, psDevice->ui32ChipSelect,
                        sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));

    //
    // Return the retrieved data.
    //
    return sData.bytes[0];

#elif defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    am_hal_iom_transfer_t Transaction;
    am_hal_iom_buffer(1) sData;

#if USE_MAYA
    Transaction.uPeerInfo.ui32I2CDevAddr = GET_I2CADDR(AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(LIS2DW_MEMS_I2C_ADDRESS));
    Transaction.ui32Instr       = ui8Register;
//    Transaction.ui8Priority     = 1;        // High priority for now.
#else // USE_MAYA
    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_LIS2DW12_CS;
    Transaction.ui32Instr       = ui8Register | 0x80;
#endif // USE_MAYA
    Transaction.ui32InstrLen    = 1;
    Transaction.ui32NumBytes    = 1;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.pui32RxBuffer   = (uint32_t *)sData.words;

    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Start the transaction.
    //
    if (am_hal_iom_blocking_transfer(g_sVosSys.pvGSensorHdl, &Transaction))
    {
        return 1;
    }

    //
    // Return the status.
    //
    return (uint8_t)(*(Transaction.pui32RxBuffer));
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}

//*****************************************************************************
//
//! @brief Reads a block of internal registers in the lis2dw12.
//!
//! @param psDevice is a pointer to a device structure describing the lis2dw12.
//! @param ui32StartRegister is the address of the first register to read.
//! @param pui32Values is the byte-packed array where the read data will go.
//! @param ui32NumBytes is the total number of 8-bit registers to read.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a read to a block of lis2dw12 registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_read_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the spi read will be polled.
//!
//! @return
//
//*****************************************************************************
void
am_devices_lis2dw12_reg_block_read(am_devices_lis2dw12_t *psDevice,
                                   uint8_t ui8StartRegister,
                                   uint32_t *pui32Values,
                                   uint32_t ui32NumBytes,
                                   am_hal_iom_callback_t pfnCallback)
{
#if defined (AM_PART_APOLLO2)
    uint8_t ui8Offset;
    
    //
    // Build the SPI offset for writing a block of registers from the
    // user-supplied start point.
    //
    ui8Offset = 0x80 | ui8StartRegister;
    
    //
    // Check to see if the callback pointer is valid.
    //
    if (pfnCallback)
    {
        //
        // If so, use a non-blocking call with a callback.
        //
        am_hal_iom_spi_read_nb(psDevice->ui32IOMModule,
                               psDevice->ui32ChipSelect,
                               pui32Values, ui32NumBytes,
                               AM_HAL_IOM_OFFSET(ui8Offset),
                               pfnCallback);
    }
    else
    {
        //
        // Otherwise, use a polled call.
        //
        am_hal_iom_spi_read(psDevice->ui32IOMModule,
                            psDevice->ui32ChipSelect,
                            pui32Values, ui32NumBytes,
                            AM_HAL_IOM_OFFSET(ui8Offset));
    }
#endif // AM_PART_APOLLO2

#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    am_hal_iom_transfer_t Transaction;
    am_hal_iom_buffer(50) sData;
    
#if USE_MAYA
    Transaction.uPeerInfo.ui32I2CDevAddr = GET_I2CADDR(AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(LIS2DW_MEMS_I2C_ADDRESS));
#else // USE_MAYA
    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_LIS2DW12_CS;
#endif // USE_MAYA
    Transaction.ui32Instr       = ui8StartRegister | 0x80;
    Transaction.ui32InstrLen    = 1;
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.pui32RxBuffer   = (uint32_t *)sData.words;

    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Start the transaction.
    //
    am_hal_iom_blocking_transfer(g_sVosSys.pvGSensorHdl, &Transaction);
    memcpy(pui32Values, sData.bytes, ui32NumBytes);
    return;
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
}

//*****************************************************************************
//
//! @brief Writes an internal register in the lis2dw12.
//!
//! @param psDevice is a pointer to a device structure describing the lis2dw12.
//! @param ui32Register is the address of the register to write.
//! @param ui32Value is the value to write to the register.
//!
//! This function performs a write to an lis2dw12 register over the serial bus.
//!
//! @return
//
//*****************************************************************************
uint8_t
am_devices_lis2dw12_reg_write(am_devices_lis2dw12_t *psDevice,
                              uint8_t ui8Register, uint8_t ui8Value)
{
#if defined (AM_PART_APOLLO2)
    uint8_t ui8Offset;
    am_hal_iom_buffer(1) sData;
    
    //
    // Build the SPI offset and the data buffer.
    //
    ui8Offset = ui8Register;
    sData.bytes[0] = ui8Value;
    
    //
    // Send the write to the bus using the polled API.
    //
    am_hal_iom_spi_write(psDevice->ui32IOMModule, psDevice->ui32ChipSelect,
                         sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));
#endif // AM_PART_APOLLO2

#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    am_hal_iom_buffer(1) sData;
    sData.bytes[0] = ui8Value;
    
    am_hal_iom_transfer_t Transaction;

    //
    // Set up the IOM transaction.
    //
#if USE_MAYA
    Transaction.uPeerInfo.ui32I2CDevAddr = GET_I2CADDR(AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(LIS2DW_MEMS_I2C_ADDRESS));
//    Transaction.ui8Priority     = 1;        // High priority for now.
#else
    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_LIS2DW12_CS;
#endif

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = ui8Register;
    Transaction.ui32NumBytes    = 1;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.pui32TxBuffer   = (uint32_t *)sData.words;

    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Start the transaction.
    //
    if (am_hal_iom_blocking_transfer(g_sVosSys.pvGSensorHdl, &Transaction))
    {
        return 1;
    }
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
    return 0;
}

uint8_t am_devices_lis2dw12_init(void)
{
#if defined (AM_PART_APOLLO2)
    am_hal_iom_config_t sGsensorIomConfig =
    {
        .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
        .ui32ClockFrequency = AM_HAL_IOM_8MHZ,
        .bSPHA = 0,
        .bSPOL = 0,
        .ui8WriteThreshold = 0,
        .ui8ReadThreshold = 60,
    };
    
    am_bsp_pin_enable(IOM0_SCK);
    am_bsp_pin_enable(IOM0_MISO);
    am_bsp_pin_enable(IOM0_MOSI);
    //am_hal_gpio_pin_config(42, AM_HAL_PIN_42_M0nCE0);
    am_hal_gpio_pin_config(44, AM_HAL_PIN_44_M0nCE2);
    am_hal_iom_pwrctrl_enable(AM_BSP_LIS2DW12_IOM);
    am_hal_iom_config(AM_BSP_LIS2DW12_IOM, &sGsensorIomConfig);

    // Turn on the IOM for this operation.
    //
    am_hal_iom_enable(AM_BSP_LIS2DW12_IOM);
#endif // AM_PART_APOLLO2

#if USE_APOLLO3_BLUE_EVB
    am_hal_iom_config_t sGsensorIomConfig =
    {
        .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq = AM_HAL_IOM_8MHZ,
        .eSpiMode = AM_HAL_IOM_SPI_MODE_0
        //.ui8WriteThreshold = 0,
        //.ui8ReadThreshold = 60,
    };

    //
    // Configure the IOM pins.
    //
    //am_bsp_iom_pins_enable(AM_BSP_LIS2DW12_IOM, AM_HAL_IOM_SPI_MODE);
    am_hal_gpio_pincfg_t sIom0Cs2PinCfg =
    {
        .uFuncSel            = AM_HAL_PIN_44_NCE44,
        .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
        .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
        .eGPInput            = AM_HAL_GPIO_PIN_INPUT_NONE,
        .eIntDir             = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .uIOMnum             = 0,
        .uNCE                = 2,
        .eCEpol              = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW
    };
    
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_SCK,  g_AM_BSP_GPIO_IOM0_SCK);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MISO, g_AM_BSP_GPIO_IOM0_MISO);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_MOSI, g_AM_BSP_GPIO_IOM0_MOSI);
    am_hal_gpio_pinconfig(44, sIom0Cs2PinCfg);

    //
    // Enable fault detection.
    //
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_fault_capture_enable();
#endif // AM_APOLLO3_MCUCTRL

    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    // HAL Success return is 0
    //
    if (am_hal_iom_initialize(AM_BSP_LIS2DW12_IOM, &(g_sVosSys.pvGSensorHdl)) ||
        am_hal_iom_power_ctrl(g_sVosSys.pvGSensorHdl, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_iom_configure(g_sVosSys.pvGSensorHdl, &sGsensorIomConfig) ||
        am_hal_iom_enable(g_sVosSys.pvGSensorHdl))
    {
        return 1;
    }
#endif // USE_APOLLO3_BLUE_EVB

#if USE_MAYA
    am_hal_iom_config_t g_sGsensorIomI2CConfig =
    {
        .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
        .ui32ClockFreq        = AM_HAL_IOM_400KHZ,
        .eSpiMode             = AM_HAL_IOM_SPI_MODE_0,
    };

    am_hal_iom_initialize(GSENSOR_IOM, &(g_sVosSys.pvGSensorHdl));
    am_hal_iom_power_ctrl(g_sVosSys.pvGSensorHdl, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(g_sVosSys.pvGSensorHdl, &g_sGsensorIomI2CConfig);
//    am_hal_iom_interrupt_enable(g_sVosSys.pvGSensorHdl, AM_HAL_IOM_INT_THR | AM_HAL_IOM_INT_CMDCMP);
    am_hal_iom_enable(g_sVosSys.pvGSensorHdl);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL,  g_AM_BSP_GPIO_IOM1_SCL);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA,  g_AM_BSP_GPIO_IOM1_SDA);

#if configUSE_GSENSOR_MOTION
    am_app_util_gsensor_config();
#endif // configUSE_GSENSOR_MOTION
#endif // USE_MAYA
    am_util_delay_ms(50);
    return 0;
}

// Need to review! (Power consumption is high!)
void am_app_utils_gsensor_deinit(void)
{
    // write 0x00 to CTRL1 to enter power down mode
#if USE_MAYA
    am_devices_lis2dw12_reg_write(NULL, AM_DEVICES_LIS2DW_CTRL_REG1, 0x00);
    if (g_sVosSys.pvGSensorHdl != NULL)
    {

        am_bsp_iom_pins_disable(GSENSOR_IOM, AM_HAL_IOM_I2C_MODE);

        //
        // Disable the IOM.
        //
        am_hal_iom_disable(g_sVosSys.pvGSensorHdl);

        //
        // Disable power to and uninitialize the IOM instance.
        //
        am_hal_iom_power_ctrl(g_sVosSys.pvGSensorHdl, AM_HAL_SYSCTRL_DEEPSLEEP, false);

        am_hal_iom_uninitialize(g_sVosSys.pvGSensorHdl);
    }

    g_sVosSys.pvGSensorHdl = NULL;
    
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL,  g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA,  g_AM_HAL_GPIO_DISABLE);
#endif
}
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
