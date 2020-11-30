//*****************************************************************************
//
// Copyright (c) 2017, Ambiq Micro
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
// This is part of revision v1.2.11 of the AmbiqSuite Development Package.
//
//*****************************************************************************

//*****************************************************************************
// Global includes for this project.
//
//*****************************************************************************
#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include "am_util.h"
#include "am_devices_led.h"
#include "am_app_utils.h"
#include "am_app_utils_task.h"
#include "am_app_utils_buzzer.h"

#include "am_vos_task.h"
#include "am_vos_init.h"
#include "am_vos_logic.h"

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
#include "am_devices_lis2dw12.h"
#include "am_app_utils_gsensor.h"
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP

void am_vos_logic_led_all_on(void)
{
    //LEDs on
    am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0xFF);
}

void am_vos_logic_led_all_off(void)
{
    //LEDs off
    am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 0x0);
}

void am_vos_logic_cmd_led_on(int8_t i8CmdIndex)
{
    am_vos_logic_led_all_off();

    if(i8CmdIndex == -1)
      return;

    if(i8CmdIndex < AM_BSP_NUM_LEDS)
        am_devices_led_on(am_bsp_psLEDs, i8CmdIndex);
    else
        am_devices_led_on(am_bsp_psLEDs, i8CmdIndex % AM_BSP_NUM_LEDS);
}

void am_vos_logic_led_swirl(uint8_t type)
{
    TickType_t xDelaySwirl = pdMS_TO_TICKS(50);

    am_vos_logic_led_all_off();
    switch(type)
    {
        case 0: // keyword detection (Swirling)
            for(int i = 0; i < (AM_BSP_NUM_LEDS * 2); i++)
            {
                am_devices_led_toggle(am_bsp_psLEDs, i % AM_BSP_NUM_LEDS);
                vTaskDelay(xDelaySwirl);
            }
            break;
        case 1: // power up (Swirling twice)
            for(int i = 0; i < (AM_BSP_NUM_LEDS * 4); i++)
            {
                am_devices_led_toggle(am_bsp_psLEDs, i % AM_BSP_NUM_LEDS);
                am_util_delay_ms(50);
            }
            break;
        case 2: // blinking all LEDs twice (for Maya)
            for(int i = 0; i < (AM_BSP_NUM_LEDS * 2); i++)
            {
                if(i == AM_BSP_NUM_LEDS)
                {
                    vTaskDelay(xDelaySwirl * 2);
                    am_vos_logic_led_all_off();
                    vTaskDelay(xDelaySwirl * 2);
                }
                am_devices_led_toggle(am_bsp_psLEDs, i % AM_BSP_NUM_LEDS);
            }
            vTaskDelay(xDelaySwirl);
          
            break;
        default:
            // do nothing
            break;
    }
    am_vos_logic_led_all_off();
}

//*****************************************************************************
//
// Simple button detection function with debounce
//
//*****************************************************************************
#if USE_MAYA
bool logic_check_button(uint32_t gpio)
{
    uint32_t pinVal = 0;

    for(uint8_t i = 0; i < 3 ; i++)
    {
        pinVal = am_hal_gpio_input_read(gpio);
        if(pinVal)
        {
            return false;
        }
        am_util_delay_us(100);
    }

    return true;
}

uint32_t logic_check_button_long(uint32_t gpio, uint32_t delay)
{
    uint32_t pinVal = 0;
    uint8_t count;
      
    for(count = 0; count < delay/BTN_CHECK_CYCLE_MS; count++)
    {
        pinVal = am_hal_gpio_input_read(gpio);
        if(pinVal)
        {
            return count;
        }
        AM_APP_LOG_DEBUG(".");
        vTaskDelay(pdMS_TO_TICKS(BTN_CHECK_CYCLE_MS));
    }

    return count;
}

void am_vos_logic_button_init(void)
{
    const am_hal_gpio_pincfg_t CFG_CTRL_BUTTON =
    {
        .uFuncSel       = 3,
        .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
        .eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
        .eGPRdZero      = AM_HAL_GPIO_PIN_RDZERO_READPIN,
        .eIntDir        = AM_HAL_GPIO_PIN_INTDIR_HI2LO,
    };

    am_hal_gpio_pinconfig(MAYA_LOGIC_BUTTON, CFG_CTRL_BUTTON);
    NVIC_SetPriority(GPIO_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    am_vos_gpio_enable_irq(MAYA_LOGIC_BUTTON);
    NVIC_EnableIRQ(GPIO_IRQn);
}

void am_vos_logic_button_process(void)
{
    // disable interrupt on this pin

    if(logic_check_button(MAYA_LOGIC_BUTTON))
    {
        if(g_sVosSys.eLogicPowerState == APP_LOGIC_POWER_ON)
        {
            am_hal_gpio_interrupt_disable(AM_HAL_GPIO_BIT(MAYA_LOGIC_BUTTON));
            am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(MAYA_LOGIC_BUTTON));

            // button pressed, send a msg to shut down the system
            am_app_utils_task_send_fromISR(AM_APP_TASK_LOGIC, AM_APP_TASK_LOGIC, AM_APP_MESSAGE_SHORT, EMPTY_MESSAGE, NULL);
        }
    }
}

#if 0
static uint8_t button2_working = 0;
void am_vos_logic_button2_process(void)
{
    //am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));

    if (button2_working == 1)
        return;

    button2_working = 1;
    

    if (logic_check_button2())
    {   
        am_hal_gpio_interrupt_disable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
    
#if configUSE_GSENSOR_MOTION
        am_app_gsensor_button2_process();
#endif // configUSE_GSENSOR_MOTION

#if configUSE_PUSH_TO_TALK
        am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
        // button pressed, send a msg to trigger push to talk
        am_app_utils_task_send_fromISR(AM_APP_TASK_AUD_PROCESSING, AM_APP_TASK_AUD_PROCESSING, AM_APP_MESSAGE_SHORT, KEY_WORD_GOT_MESSAGE, NULL);
#endif // configUSE_PUSH_TO_TALK
    }
    button2_working = 0;
}
#endif

void am_vos_logic_system_power_off(void)
{
#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    am_devices_lis2dw12_reg_write(NULL, AM_DEVICES_LIS2DW_CTRL_REG1, 0x00);
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP

    //
    // Disable all interrupts
    //

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
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);

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
    am_hal_gpio_state_write(BUZZER_CAP_PWR_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);      // power down?

    // IOM1 I2C IOs output high
#if 0 //configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL, g_AM_HAL_GPIO_OUTPUT);    // scl
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA, g_AM_HAL_GPIO_OUTPUT);    // sda

    am_hal_gpio_pinconfig(LIS2DW_INT_PIN, g_AM_HAL_GPIO_OUTPUT);    // int1

    am_hal_gpio_state_write(AM_BSP_GPIO_IOM1_SCL, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_state_write(AM_BSP_GPIO_IOM1_SDA, AM_HAL_GPIO_OUTPUT_SET);
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    
    // Disable all other GPIOs
    for(int i = 0; i < AM_BSP_NUM_LEDS; i++)
        am_hal_gpio_pinconfig(am_bsp_psLEDs[i].ui32GPIONumber , g_AM_HAL_GPIO_DISABLE);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON0, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM_DATA, g_AM_HAL_GPIO_DISABLE);

    am_hal_gpio_pinconfig(BUZZER_PWM_OUTPUT_PIN, g_AM_HAL_GPIO_DISABLE);

    am_hal_gpio_pinconfig(WOS_WAKE_PIN, g_AM_HAL_GPIO_DISABLE);    // wake
    am_hal_gpio_pinconfig(WOS_MODE_PIN, g_AM_HAL_GPIO_DISABLE);   // mode

#if configUSE_GSENSOR
//    am_hal_gpio_pinconfig(LIS2DW_INT_PIN, g_AM_HAL_GPIO_DISABLE);    // int1
//    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SCL, g_AM_HAL_GPIO_DISABLE);    // scl
//    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM1_SDA, g_AM_HAL_GPIO_DISABLE);    // sda
#endif // configUSE_GSENSOR
    
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON2, g_AM_HAL_GPIO_DISABLE);   // optional button
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_HAL_GPIO_DISABLE);   // tx0
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_HAL_GPIO_DISABLE);   // rx0
    am_hal_gpio_pinconfig(33, g_AM_HAL_GPIO_DISABLE);   // amic audio out
    am_hal_gpio_pinconfig(AM_BSP_GPIO_ITM_SWO, g_AM_HAL_GPIO_DISABLE);   // swo

    //
    // enable only button interrupt
    //
}

void am_vos_logic_maya_hw_init(void)
{
#if configUSE_BUZZER
    am_app_utils_buzzer_init();
#else // configUSE_BUZZER
    am_app_utils_buzzer_init_power_down();
#endif // configUSE_BUZZER

    // indicating power up, buzzer and led
    am_app_utils_led_buzzer_power_up();

    am_vos_logic_button_init();
}
#endif // USE_MAYA
