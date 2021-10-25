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

#include "am_app_utils.h"
#include "am_app_utils_task.h"
#include "am_app_utils_buzzer.h"
#include "am_devices_lis2dw12.h"
#include "am_app_utils_gsensor.h"

#include "am_vos_task.h"
#include "am_vos_init.h"

#if configUSE_GSENSOR_MOTION
void am_app_utils_gsensor_wakeup_handle()
{
    uint32_t ui32PinState;

    am_hal_gpio_state_read(LIS2DW_INT_PIN, AM_HAL_GPIO_INPUT_READ, &ui32PinState);

    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(LIS2DW_INT_PIN));
    
    //am_app_utils_task_send_fromISR(AM_APP_ISR_GPIO, AM_APP_TASK_GSENSOR, AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);   
}

void am_app_utils_gsensor_wakeup_irq_init(void)
{
    const am_hal_gpio_pincfg_t sGsensorWakePinConfig =
    {
        .uFuncSel       = 3,
        .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
        .eGPRdZero      = AM_HAL_GPIO_PIN_RDZERO_READPIN, //AM_HAL_GPIO_PIN_RDZERO_ZERO, //AM_HAL_GPIO_PIN_RDZERO_ZERO, //AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .eIntDir        = AM_HAL_GPIO_PIN_INTDIR_HI2LO
    };

    am_hal_gpio_pinconfig(LIS2DW_INT_PIN, sGsensorWakePinConfig);
    //am_hal_gpio_interrupt_register(LIS2DW_INT_PIN, am_app_utils_gsensor_wakeup_handle);
    
    NVIC_SetPriority(GPIO_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    am_vos_gpio_enable_irq(LIS2DW_INT_PIN);
    NVIC_EnableIRQ(GPIO_IRQn);
}

void am_app_util_gsensor_config(void)
{
    uint8_t sRegBuffer[6];

    am_devices_lis2dw12_reg_write(NULL, 0x34, 0x20);
    am_devices_lis2dw12_reg_write(NULL, 0x35, 0x60);
    am_devices_lis2dw12_reg_write(NULL, 0x3f, 0x20);
    am_devices_lis2dw12_reg_write(NULL, 0x21, 0x0C);
    
    am_util_delay_us(8);   

    sRegBuffer[0] = 0x20;     // CTRL_REG1  #100hz
    sRegBuffer[1] = 0x00;     // CTRL_REG2
    sRegBuffer[2] = 0x00;     // CTRL_REG3
    sRegBuffer[3] = 0x20;     // CTRL_REG4
    sRegBuffer[4] = 0x00;     // CTRL_REG5
    sRegBuffer[5] = 0x04;     // CTRL_REG6  #FS 2 g LOW_NOISE enabled

    am_devices_lis2dw12_reg_write(NULL, AM_DEVICES_LIS2DW_CTRL_REG1, sRegBuffer[0]);
    am_devices_lis2dw12_reg_write(NULL, AM_DEVICES_LIS2DW_CTRL_REG2, sRegBuffer[1]);
    am_devices_lis2dw12_reg_write(NULL, AM_DEVICES_LIS2DW_CTRL_REG3, sRegBuffer[2]);
    am_devices_lis2dw12_reg_write(NULL, AM_DEVICES_LIS2DW_CTRL_REG4, sRegBuffer[3]);
    am_devices_lis2dw12_reg_write(NULL, AM_DEVICES_LIS2DW_CTRL_REG5, sRegBuffer[4]);
    am_devices_lis2dw12_reg_write(NULL, AM_DEVICES_LIS2DW_CTRL_REG6, sRegBuffer[5]);
}

void am_app_utils_gsensor_power_down(void)
{
    uint8_t result[6];
    uint32_t ui32Temp;
    
    // software reset
    am_devices_lis2dw12_reg_write(NULL, 0x21, 0x44);
    am_util_delay_ms(50);

    // write 0x00 to CTRL1 to enter power down mode
    am_devices_lis2dw12_reg_write(NULL, 0x20, 0x00);
    ui32Temp = am_devices_lis2dw12_reg_read(NULL, 0x20);
    AM_APP_LOG_DEBUG("CTRL1 = 0x%02x. \n", ui32Temp);

    // write 0x00 to CTRL2 to enter power down mode
    am_devices_lis2dw12_reg_write(NULL, 0x21, 0x10);
    ui32Temp = am_devices_lis2dw12_reg_read(NULL, 0x21);
    AM_APP_LOG_DEBUG("CTRL2 = 0x%02x. \n", ui32Temp);

    // write 0x00 to CTRL3 to enter power down mode
    am_devices_lis2dw12_reg_write(NULL, 0x22, 0x2a);
    ui32Temp = am_devices_lis2dw12_reg_read(NULL, 0x22);
    AM_APP_LOG_DEBUG("CTRL3 = 0x%02x. \n", ui32Temp);

    // write 0x00 to CTRL4 to enter power down mode
    am_devices_lis2dw12_reg_write(NULL, 0x23, 0x00);
    ui32Temp = am_devices_lis2dw12_reg_read(NULL, 0x23);
    AM_APP_LOG_DEBUG("CTRL4 = 0x%02x. \n", ui32Temp);

    // write 0x00 to CTRL5 to enter power down mode
    am_devices_lis2dw12_reg_write(NULL, 0x24, 0x00);
    ui32Temp = am_devices_lis2dw12_reg_read(NULL, 0x24);
    AM_APP_LOG_DEBUG("CTRL5 = 0x%02x. \n", ui32Temp);

    // write 0x00 to CTRL6 to enter power down mode
    am_devices_lis2dw12_reg_write(NULL, 0x25, 0x00);
    ui32Temp = am_devices_lis2dw12_reg_read(NULL, 0x25);
    AM_APP_LOG_DEBUG("CTRL6 = 0x%02x. \n", ui32Temp);

    // write 0x00 to CTRL7 to enter power down mode
    am_devices_lis2dw12_reg_write(NULL, 0x3f, 0x40);
    ui32Temp = am_devices_lis2dw12_reg_read(NULL, 0x3f);
    AM_APP_LOG_DEBUG("CTRL7 = 0x%02x. \n", ui32Temp);

    // write 0x00 to WAKE_UP_THS to enter power down mode
    am_devices_lis2dw12_reg_write(NULL, 0x34, 0x40);
    ui32Temp = am_devices_lis2dw12_reg_read(NULL, 0x34);
    AM_APP_LOG_DEBUG("WAKE_UP_THS = 0x%02x. \n", ui32Temp);

    am_util_delay_ms(50);

    result[0] = am_devices_lis2dw12_reg_read(NULL, 0x28);
    result[1] = am_devices_lis2dw12_reg_read(NULL, 0x29);
    result[2] = am_devices_lis2dw12_reg_read(NULL, 0x2a);
    result[3] = am_devices_lis2dw12_reg_read(NULL, 0x2b);
    result[4] = am_devices_lis2dw12_reg_read(NULL, 0x2c);
    result[5] = am_devices_lis2dw12_reg_read(NULL, 0x2d);

    AM_APP_LOG_DEBUG("Lis3mdl X0=%X, X1=%X, Y0=%X, Y1= %X, Z0=%X, Z1=%X\n", result[0], result[1], result[2], result[3], result[4], result[5]);

    am_hal_iom_power_ctrl(g_sVosSys.pvGSensorHdl, AM_HAL_SYSCTRL_DEEPSLEEP, false);
}

void am_app_utils_gsensor_system_power_off(void)
{
    //
    // Disable all interrupts
    //
    // am_hal_interrupt_master_disable();

    for(uint8_t i = 0; i < 32; i++)
    {
        NVIC_DisableIRQ((IRQn_Type)i);         // disable all IRQs
        NVIC_ClearPendingIRQ((IRQn_Type)i);    // clear all pending IRQs
    }
    
    //
    // Suspend all tasks
    // 
    vTaskSuspendAll();

    //
    // Stop Stimer
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR |AM_HAL_STIMER_CFG_FREEZE);

    //
    // Turn off peripherals
    //
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOS);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_SCARD);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_MSPI);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_PDM);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_BLEL);

    //
    // Turn OFF Flash1
    //
    if ( am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_FLASH_512K) )
    {
    }

    //
    // Power down SRAM
    //
    
    //PWRCTRL->MEMPWDINSLEEP_b.SRAMPWDSLP = PWRCTRL_MEMPWDINSLEEP_SRAMPWDSLP_ALLBUTLOWER32K;


    // GPIO12 (PDMC) set to low
    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDMCLK, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_PDMCLK, AM_HAL_GPIO_OUTPUT_CLEAR);

    // GPIO38 (nSPK_POWER) set to low
    am_hal_gpio_pinconfig(BUZZER_CAP_PWR_PIN, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(BUZZER_CAP_PWR_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

    // IOM1 I2C IOs output high
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL, g_AM_HAL_GPIO_OUTPUT);    // scl
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA, g_AM_HAL_GPIO_OUTPUT);    // sda
    am_hal_gpio_pinconfig(LIS2DW_INT_PIN, g_AM_HAL_GPIO_OUTPUT);    // int1
    am_hal_gpio_state_write(AM_BSP_GPIO_IOM1_SCL, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_state_write(AM_BSP_GPIO_IOM1_SDA, AM_HAL_GPIO_OUTPUT_SET);
    //am_hal_gpio_state_write(LIS2DW_INT_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    
    // Disable all other GPIOs
    for(int i = 0; i < AM_BSP_NUM_LEDS; i++)
        am_hal_gpio_pinconfig(am_bsp_psLEDs[i].ui32GPIONumber , g_AM_HAL_GPIO_DISABLE);

    am_hal_gpio_pinconfig(PUSH_TO_TALK_BUTTON, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM_DATA, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(BUZZER_PWM_OUTPUT_PIN, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(WOS_WAKE_PIN, g_AM_HAL_GPIO_DISABLE);    // wake
    am_hal_gpio_pinconfig(WOS_MODE_PIN, g_AM_HAL_GPIO_DISABLE);   // mode

    am_hal_gpio_pinconfig(LIS2DW_INT_PIN, g_AM_HAL_GPIO_DISABLE);    // int1
//    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL, g_AM_HAL_GPIO_DISABLE);    // scl
//    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA, g_AM_HAL_GPIO_DISABLE);    // sda

    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON2, g_AM_HAL_GPIO_DISABLE);   // optional button
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_HAL_GPIO_DISABLE);   // tx0
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_HAL_GPIO_DISABLE);   // rx0
    am_hal_gpio_pinconfig(33, g_AM_HAL_GPIO_DISABLE);   // amic audio out
    am_hal_gpio_pinconfig(AM_BSP_GPIO_ITM_SWO, g_AM_HAL_GPIO_DISABLE);   // swo    
    
    //
    // enable only button interrupt
    //
}

void am_app_utils_gsensor_resetTick(void)
{
    g_sAmUtil.ui32FirstTick = 0;
}    

void am_app_util_gsensor_powerdown(void)
{
    am_app_utils_buzzer_power_down();

    am_util_delay_ms(100);  
    //am_hal_interrupt_master_disable();

    am_app_utils_gsensor_system_power_off();

    am_app_utils_gsensor_wakeup_irq_init();
    

    //am_util_delay_ms(10);
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

    // do not enable interrupt here, since we turned off SRAM
    // reset the system and start over
    //am_hal_flash_delay(FLASH_CYCLES_US(100000));
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(LIS2DW_INT_PIN));
    am_hal_stimer_config(0x0);
    am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOI, NULL);

    while(1);
}

void am_app_gsensor_button2_init(void)
{
    const am_hal_gpio_pincfg_t CFG_CTRL_BUTTON =
    {
        .uFuncSel       = 3,
        .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
        .eGPRdZero      = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .eIntDir        = AM_HAL_GPIO_PIN_INTDIR_BOTH,
        .ePullup        = AM_HAL_GPIO_PIN_PULLUP_WEAK,
    };

    am_hal_gpio_pinconfig(PUSH_TO_TALK_BUTTON, CFG_CTRL_BUTTON);

    NVIC_SetPriority(GPIO_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    am_vos_gpio_enable_irq(PUSH_TO_TALK_BUTTON);
    NVIC_EnableIRQ(GPIO_IRQn);
}

void am_app_util_gsensor_button2_disble_handler(void)
{
    xTimerStop(am_KWD_timers[AM_APP_TIMER_GSENSOR_PERIOD], 0);
    //xTimerDelete(am_KWD_timers[AM_APP_TIMER_GSENSOR_PERIOD], 0);

    // write 0x00 to CTRL1 to enter power down mode
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
    
}

int32_t complement(uint8_t high, uint8_t low)
{
    uint8_t hi = high;
    uint16_t value = (uint16_t)(hi << 8) + low;
    
    if((value & 0x8000) != 0)   
    {
        uint16_t nValue = (~value) + 0x1;
        return (int32_t)((nValue* mg_per_bit/4)) * ((int32_t)(-1));
    }
  
    return (int32_t)(value * mg_per_bit/4);
}

void am_app_utils_gsensor_process(void)
{
    bool bStill = false;
    uint8_t result[6];
    int32_t xAxis, yAxis, zAxis;

    result[0] = am_devices_lis2dw12_reg_read(NULL, 0x28);
    result[1] = am_devices_lis2dw12_reg_read(NULL, 0x29);
    result[2] = am_devices_lis2dw12_reg_read(NULL, 0x2a);
    result[3] = am_devices_lis2dw12_reg_read(NULL, 0x2b);
    result[4] = am_devices_lis2dw12_reg_read(NULL, 0x2c);
    result[5] = am_devices_lis2dw12_reg_read(NULL, 0x2d);

    xAxis = complement(result[1], result[0]);
    yAxis = complement(result[3], result[2]);
    zAxis = complement(result[5], result[4]);  
    
    AM_APP_LOG_DEBUG("Lis3mdl X=%d, Y=%d, Z=%d\n", xAxis, yAxis, zAxis);

    if ((xAxis < X0_plus) && (xAxis > X0_minus) && (yAxis < Y0_plus) && (yAxis > Y0_minus) && (zAxis < Z_max) && (zAxis > Z_min))
        bStill = true;

    // 
    if (bStill)
    {
        if (g_sAmUtil.ui32FirstTick == 0)
            g_sAmUtil.ui32FirstTick = xTaskGetTickCount();
        
        TickType_t xTickNow = xTaskGetTickCount();
        
        if( ((xTickNow - g_sAmUtil.ui32FirstTick)/portTICK_PERIOD_MS) >=  GSENSOR_QUITE_MS) 
            am_app_utils_task_send_fromISR(AM_APP_TASK_GSENSOR, AM_APP_TASK_GSENSOR, AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);
    }
    else
    {
        g_sAmUtil.ui32FirstTick = 0;
    }
    
    //configASSERT(bStill == true);
    //am_app_utils_task_send_xTicksToWait(AM_APP_TASK_GSENSOR, AM_APP_TASK_GSENSOR, AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL, pdMS_TO_TICKS( 20000 ));
    
}

void am_vos_timer_gsensor_period_callback(TimerHandle_t xTimer)
{
    // am_app_utils_task_send(AM_APP_TASK_GSENSOR, AM_APP_TASK_GSENSOR, AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);
    
    am_app_utils_gsensor_process();
}
#endif // configUSE_GSENSOR_MOTION
