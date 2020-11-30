
#include "am_vos_sys_config.h"

#include "am_mcu_apollo.h"
#include "am_bsp.h"

#include "am_app_utils.h"

#include "am_devices_fxl6408.h"

//*****************************************************************************
//
// Variables
//
//*****************************************************************************
static void* g_pFxl6408IomHandle = NULL;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************

//
// IOM Configuration.
//
static am_hal_iom_config_t g_sFxl6408IomI2CConfig =
{
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_400KHZ,
    .eSpiMode             = AM_HAL_IOM_SPI_MODE_0,
};


void
fxl6408_iom_byte_read(uint32_t offset, uint8_t *pVal)
{
    am_hal_iom_transfer_t Transaction;
    uint32_t tempBuf = 0;

    //
    // Set up the IOM transaction.
    //
    Transaction.ui8Priority     = 1;        // High priority for now.
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = (offset & 0x0000FFFF);
    Transaction.ui32NumBytes    = 1;
    Transaction.pui32RxBuffer   = (uint32_t *)&tempBuf;
  
    Transaction.uPeerInfo.ui32I2CDevAddr = GET_I2CADDR(AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(FXL6408_GPIOEXT_I2C_ADDRESS));
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.bContinue       = false;

    //
    // Start the transaction.
    //
    am_hal_iom_blocking_transfer(g_pFxl6408IomHandle, &Transaction);
    *pVal = (uint8_t)tempBuf;
}

void
fxl6408_iom_byte_write(uint32_t offset, uint8_t val)
{
    am_hal_iom_transfer_t Transaction;
    uint32_t ui32Buf = val;

    //
    // Set up the IOM transaction.
    //
    Transaction.ui8Priority     = 1;        // High priority for now.
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = (offset & 0x0000FFFF);
    Transaction.ui32NumBytes    = 1;
    Transaction.pui32TxBuffer   = (uint32_t *)&ui32Buf;
  
    Transaction.uPeerInfo.ui32I2CDevAddr = GET_I2CADDR(AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(FXL6408_GPIOEXT_I2C_ADDRESS));
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.bContinue       = false;

    //
    // Start the transaction.
    //
    uint32_t ui32Status = am_hal_iom_blocking_transfer(g_pFxl6408IomHandle, &Transaction);

}

uint32_t
fxl6408_gpio_state_write(uint8_t ui8Pin, am_device_fx6408_write_type_e eWriteType)
{
    uint8_t ui8Mask = 0x1 << ui8Pin;
    uint8_t ui8Temp = 0;
    uint32_t ui32Return = AM_HAL_STATUS_SUCCESS;

    AM_CRITICAL_BEGIN;
    switch ( eWriteType )
    {
        case AM_FX6408_GPIO_OUTPUT_SET:                // Write a one to a GPIO.
            fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_OUT_STATE, &ui8Temp);
            ui8Temp |= ui8Mask;
            fxl6408_iom_byte_write(AM_DEVICES_FXL6408_REG_OUT_STATE, ui8Temp);
            break;

        case AM_FX6408_GPIO_OUTPUT_CLEAR:              // Write a zero to a GPIO.
            fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_OUT_STATE, &ui8Temp);
            ui8Temp &= (!ui8Mask);
            fxl6408_iom_byte_write(AM_DEVICES_FXL6408_REG_OUT_STATE, ui8Temp);
            break;
            
        case AM_FX6408_GPIO_OUTPUT_TRISTATE_ENABLE:    // Enable  a tri-state GPIO.
            fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_OUT_HIGHZ, &ui8Temp);
            ui8Temp &= (!ui8Mask);
            fxl6408_iom_byte_write(AM_DEVICES_FXL6408_REG_OUT_HIGHZ, ui8Temp);
            break;
            
        case AM_FX6408_GPIO_OUTPUT_TRISTATE_DISABLE:   // Disable a tri-state GPIO.
            fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_OUT_HIGHZ, &ui8Temp);
            ui8Temp &= (!ui8Mask);
            fxl6408_iom_byte_write(AM_DEVICES_FXL6408_REG_OUT_HIGHZ, ui8Temp);
            break;
            
        default:
            // Type values were validated on entry.
            // We can't return from here because we're in a critical section.
            ui32Return = AM_HAL_STATUS_INVALID_ARG;
            break;
    }

    AM_CRITICAL_END;

    return ui32Return;
} // am_hal_gpio_state_write()

uint8_t
fxl6408_gpio_state_read(uint8_t ui8Pin)
{
    uint8_t ui8Temp = 0;

    fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_IODIR, &ui8Temp);
    if(ui8Temp & (0x01 << ui8Pin))
    {
        // Current IO direction is output, read GPIO output state.
        fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_OUT_STATE, &ui8Temp);
    }
    else
    {
        // Current IO direction is input, read GPIO input state.
        fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_IN_STATE, &ui8Temp);
    }

    return ui8Temp & (0x01 << ui8Pin);
}

void
fxl6408_gpio_pinconfig(uint8_t ui8Pin, uint8_t ui8Dir)
{
    uint8_t ui8Temp = 0;
    uint8_t ui8Mask = 0x1 << ui8Pin;
    
    fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_IODIR, &ui8Temp);

    if(ui8Dir)
        ui8Temp |= ui8Mask;
    else
        ui8Temp &= (!ui8Mask);

    fxl6408_iom_byte_write(AM_DEVICES_FXL6408_REG_IODIR, ui8Temp);
    
    return;
}

void
am_app_utils_fxl6408_init(void)
{
    uint8_t ui8Temp = 0;
    
    // I2C initialize for FLX6408 GPIO extender.
    am_hal_iom_initialize(FLX6408_IOM, &g_pFxl6408IomHandle);
    am_hal_iom_power_ctrl(g_pFxl6408IomHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(g_pFxl6408IomHandle, &g_sFxl6408IomI2CConfig);
    am_hal_iom_enable(g_pFxl6408IomHandle);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL,  g_AM_BSP_GPIO_IOM1_SCL);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA,  g_AM_BSP_GPIO_IOM1_SDA);

    // Read chip ID to confirm FLX6408 is provided.
    fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_CHIPID, &ui8Temp);
    if(ui8Temp & 0xA0)
        AM_APP_LOG_DEBUG("AM_DEVICES_FXL6408_REG_CHIPID = 0x%02x. \n", ui8Temp);
    else
        AM_APP_LOG_DEBUG("FXL6408 device is not connected! ID = 0x%02x. \n", ui8Temp);

    // Default GPIO configuration for AMIC control lines
    fxl6408_gpio_pinconfig(VM_AMIC_CTRL_THRES1, FXL6408_GPIO_DIR_OUTPUT);
    fxl6408_gpio_pinconfig(VM_AMIC_CTRL_THRES2, FXL6408_GPIO_DIR_OUTPUT);
    fxl6408_gpio_pinconfig(VM_AMIC_CTRL_GAIN, FXL6408_GPIO_DIR_OUTPUT);

    fxl6408_iom_byte_read(AM_DEVICES_FXL6408_REG_IODIR, &ui8Temp);
    AM_APP_LOG_DEBUG("AM_DEVICES_FXL6408_REG_IODIR = 0x%02x. \n", ui8Temp);
}
