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
// This is part of revision 1.2.11 of the AmbiqSuite Development Package.
//
//*****************************************************************************

/*FILE START*/

/*******************************************************************************
********************************************************************************
*     BoardSetup.c
********************************************************************************
*
*     Description:  Setup board peripherals
*
*******************************************************************************/
#include "am_vos_sys_config.h"
#include "am_vos_board_setup.h"

#include "am_app_utils.h"

#include "am_vos_task.h"
#include "am_vos_init.h"
#include "am_vos_spp.h"
#include "am_vos_ble.h"
#include "am_vos_logic.h"

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
#include "am_devices_lis2dw12.h"
#include "am_app_utils_gsensor.h"
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP

#if USE_DMIC_MB3_VM3011
#include "am_devices_vm3011.h"
#endif // USE_DMIC_MB3_VM3011

#if configUSE_AMU2S_RECORDER
#include "am_devices_amu2s.h"
#endif // configUSE_AMU2S_RECORDER

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
//*****************************************************************************
//
// Configure the ADC.
//
//*****************************************************************************
void
adc_config_dma(void)
{
    am_hal_adc_dma_config_t       ADCDMAConfig;

    //
    // Configure the ADC to use DMA for the sample transfer.
    //
    ADCDMAConfig.bDynamicPriority = true;
    ADCDMAConfig.ePriority = AM_HAL_ADC_PRIOR_SERVICE_IMMED;
    ADCDMAConfig.bDMAEnable = true;
    ADCDMAConfig.ui32SampleCount = ADC_SAMPLE_COUNT * ADC_MIC_CHANNELS;
    ADCDMAConfig.ui32TargetAddress = (uint32_t)g_sVosSys.ui32ADCSampleBuffer;

    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_dma(g_sVosSys.pvADCHandle, &ADCDMAConfig))
    {
        am_util_stdio_printf("Error - configuring ADC DMA failed.\n");
    }
}

void
adc_config(void)
{
    am_hal_adc_config_t           ADCConfig;
    am_hal_adc_slot_config_t      ADCSlotConfig;

    //
    // Initialize the ADC and get the handle.
    //
    if ( AM_HAL_STATUS_SUCCESS != am_hal_adc_initialize(0, &g_sVosSys.pvADCHandle) )
    {
        am_util_stdio_printf("Error - reservation of the ADC instance failed.\n");
    }

    //
    // Power on the ADC.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_power_control(g_sVosSys.pvADCHandle,
                                                          AM_HAL_SYSCTRL_WAKE,
                                                          false) )
    {
        am_util_stdio_printf("Error - ADC power on failed.\n");
    }

    //
    // Set up the ADC configuration parameters. These settings are reasonable
    // for accurate measurements at a low sample rate.
    //
    ADCConfig.eClock             = AM_HAL_ADC_CLKSEL_HFRC;
    ADCConfig.ePolarity          = AM_HAL_ADC_TRIGPOL_RISING;
    ADCConfig.eTrigger           = AM_HAL_ADC_TRIGSEL_SOFTWARE;
    ADCConfig.eReference         = AM_HAL_ADC_REFSEL_INT_1P5;
    ADCConfig.eClockMode         = AM_HAL_ADC_CLKMODE_LOW_LATENCY;
    ADCConfig.ePowerMode         = AM_HAL_ADC_LPMODE0;
    ADCConfig.eRepeat            = AM_HAL_ADC_REPEATING_SCAN;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure(g_sVosSys.pvADCHandle, &ADCConfig))
    {
        am_util_stdio_printf("Error - configuring ADC failed.\n");
    }

    //
    // Set up an ADC slot
    //
    ADCSlotConfig.bEnabled       = false;
    ADCSlotConfig.bWindowCompare = false;
    ADCSlotConfig.eChannel       = AM_HAL_ADC_SLOT_CHSEL_SE0;    // 0
    ADCSlotConfig.eMeasToAvg     = AM_HAL_ADC_SLOT_AVG_1;        // 0
    ADCSlotConfig.ePrecisionMode = AM_HAL_ADC_SLOT_14BIT;        // 0

    am_hal_adc_configure_slot(g_sVosSys.pvADCHandle, 2, &ADCSlotConfig);   // Unused slot
    am_hal_adc_configure_slot(g_sVosSys.pvADCHandle, 3, &ADCSlotConfig);   // Unused slot
    am_hal_adc_configure_slot(g_sVosSys.pvADCHandle, 4, &ADCSlotConfig);   // Unused slot
    am_hal_adc_configure_slot(g_sVosSys.pvADCHandle, 5, &ADCSlotConfig);   // Unused slot
    am_hal_adc_configure_slot(g_sVosSys.pvADCHandle, 6, &ADCSlotConfig);   // Unused slot
    am_hal_adc_configure_slot(g_sVosSys.pvADCHandle, 7, &ADCSlotConfig);   // Unused slot

#if USE_AMIC_MB3_VMB || USE_AMIC_MB3
    ADCSlotConfig.bEnabled       = true;
    ADCSlotConfig.eChannel       = ADC_INPUT_RIGHT_CH;
#else // USE_AMIC_MB3_VMB || USE_AMIC_MB3
    ADCSlotConfig.bEnabled       = false;
    ADCSlotConfig.eChannel       = AM_HAL_ADC_SLOT_CHSEL_SE0;    // Disabled
#endif // USE_AMIC_MB3_VMB || USE_AMIC_MB3
    am_hal_adc_configure_slot(g_sVosSys.pvADCHandle, 0, &ADCSlotConfig);   // Channel 0 is used for right channel.

#if USE_AMIC_MB3_VMB || USE_AMIC_MB2
    ADCSlotConfig.bEnabled       = true;
    ADCSlotConfig.eChannel       = ADC_INPUT_LEFT_CH;
#else // USE_AMIC_MB3_VMB || USE_AMIC_MB2
    ADCSlotConfig.bEnabled       = false;
    ADCSlotConfig.eChannel       = AM_HAL_ADC_SLOT_CHSEL_SE0;    // Disabled
#endif // USE_AMIC_MB3_VMB || USE_AMIC_MB2
    am_hal_adc_configure_slot(g_sVosSys.pvADCHandle, 1, &ADCSlotConfig);  // Channel 1 is used for left channel.

    //
    // Configure the ADC to use DMA for the sample transfer.
    //
    adc_config_dma();

    //
    // For this example, the samples will be coming in slowly. This means we
    // can afford to wake up for every conversion.
    //
    am_hal_adc_interrupt_enable(g_sVosSys.pvADCHandle, AM_HAL_ADC_INT_DERR | AM_HAL_ADC_INT_DCMP );

    //
    // Enable IRQ and make it callable in freeRTOS
    //
    NVIC_SetPriority(ADC_IRQn, 4);
    NVIC_EnableIRQ(ADC_IRQn);
}

void init_timerA3_for_ADC(void)
{
    //
    // Start a timer to trigger the ADC periodically (16000HZ sample rate).
    //
    am_hal_ctimer_config_single(3, AM_HAL_CTIMER_TIMERA,
                                AM_HAL_CTIMER_HFRC_12MHZ    |
                                AM_HAL_CTIMER_FN_REPEAT     );

//  am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA3);
//  am_hal_ctimer_period_set(3, AM_HAL_CTIMER_TIMERA, 375, 186);  //12M/375=32ksps
//  am_hal_ctimer_period_set(3, AM_HAL_CTIMER_TIMERA, 750, 325);  //12M/750=16ksps
    am_hal_ctimer_period_set(3, AM_HAL_CTIMER_TIMERA, 750, 375);

    //
    // Enable the timer A3 to trigger the ADC directly
    //
    am_hal_ctimer_adc_trigger_enable();

    //
    // Start the timer.
    //
    am_hal_ctimer_start(3, AM_HAL_CTIMER_TIMERA);
}
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
#define CHECK_ERRORS(x)                                                       \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        error_handler(x);                                                     \
    }

//*****************************************************************************
//
// Catch HAL errors.
//
//*****************************************************************************
void
error_handler(uint32_t ui32ErrorStatus)
{
    g_sVosSys.ui32LastError = ui32ErrorStatus;

    while (1);
}
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

//-----------------------------------------------------------------------------
// METHOD:  uart_init
// PURPOSE: Setup UART0 and/or UART1
//-----------------------------------------------------------------------------
void uart_init(uint32_t ui32Module)
{
#if configUSE_LOG_UART0 || configUSE_PRINTF_UART0
#if defined (AM_PART_APOLLO2)
    static am_hal_uart_config_t g_sUart0Config =
    {
        .ui32BaudRate    = 460800,
        .ui32DataBits    = AM_HAL_UART_DATA_BITS_8,
        .bTwoStopBits    = false,
        .ui32Parity      = AM_HAL_UART_PARITY_NONE,
        .ui32FlowCtrl    = AM_HAL_UART_FLOW_CTRL_NONE,
    };
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
#endif // configUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_LOG_UART1
    static am_hal_uart_config_t g_sUart1Config =
    {
        .ui32BaudRate    = 460800,
        .ui32DataBits    = AM_HAL_UART_DATA_BITS_8,
        .ui32StopBits    = AM_HAL_UART_ONE_STOP_BIT,
        .ui32Parity      = AM_HAL_UART_PARITY_NONE,
        .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
    };
    if (ui32Module == 1)
    {
        // GW: Added for EM9304 Mikro board support
#if USE_EM9304_MIKRO_MB2
        am_hal_gpio_pin_config(35, AM_HAL_PIN_35_UART1TX);
        am_hal_gpio_pin_config(13, AM_HAL_PIN_13_UART1RX);
#else // USE_EM9304_MIKRO_MB2
        am_hal_gpio_pin_config(39, AM_HAL_PIN_39_UART1TX);
        am_hal_gpio_pin_config(40, AM_HAL_PIN_40_UART1RX);
#endif // USE_EM9304_MIKRO_MB2
    }
#endif // configUSE_UART1

#if defined (AM_PART_APOLLO2)
    am_hal_uart_pwrctrl_enable(ui32Module);     // Power on the selected UART
    am_hal_uart_clock_enable(ui32Module);       // start UART interface and enable the FIFOs.
    am_hal_uart_disable(ui32Module);            // Disable the UART before configuring it.

#if configUSE_LOG_UART0 || configUSE_PRINTF_UART0
    if (ui32Module == 0)
    {
        am_hal_uart_config(ui32Module, &g_sUart0Config);
        am_hal_uart_init_buffered(ui32Module, g_sVosSys.pui8UartRxBuf0, UART0_BUFFER_SIZE,
                                    g_sVosSys.pui8UartTxBuf0, UART0_BUFFER_SIZE);
    }

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pin_config(22, AM_HAL_PIN_22_UART0TX);
    am_hal_gpio_pin_config(17, AM_HAL_PIN_17_UART0RX);
#endif // configUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_LOG_UART1
    if (ui32Module == 1)
    {
          am_hal_uart_config(ui32Module, &g_sUart1Config);
          am_hal_uart_init_buffered(ui32Module, g_sVosSys.pui8UartRxBuf1, UART1_BUFFER_SIZE,
                                    g_sVosSys.pui8UartTxBuf1, UART1_BUFFER_SIZE);
    }
#endif // configUSE_LOG_UART1

    //
    // Configure the UART FIFO.
    //
    am_hal_uart_fifo_config(ui32Module, AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    //*****************************************************************************
    //
    // UART configuration.
    //
    //*****************************************************************************
    const am_hal_uart_config_t g_sUartConfig =
    {
        .ui32BaudRate    = 460800,
        .ui32DataBits    = AM_HAL_UART_DATA_BITS_8,
        .ui32StopBits    = AM_HAL_UART_ONE_STOP_BIT,
        .ui32Parity      = AM_HAL_UART_PARITY_NONE,
        .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
        //
        // Set TX and RX FIFOs to interrupt at half-full.
        //
        .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                           AM_HAL_UART_RX_FIFO_1_2),

        //
        // Buffers
        //
        .pui8TxBuffer = g_sVosSys.pui8UartTxBuf,
        .ui32TxBufferSize = sizeof(g_sVosSys.pui8UartTxBuf),
        .pui8RxBuffer = g_sVosSys.pui8UartRxBuf,
        .ui32RxBufferSize = sizeof(g_sVosSys.pui8UartRxBuf),
    };

    //
    // Initialize the printf interface for UART output.
    //
    CHECK_ERRORS(am_hal_uart_initialize(0, &(g_sVosSys.pvUartHandle)));
    CHECK_ERRORS(am_hal_uart_power_control(g_sVosSys.pvUartHandle, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(g_sVosSys.pvUartHandle, &g_sUartConfig));
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
}

//*****************************************************************************
//
// Enable the UART
//
//*****************************************************************************
void uart_enable(uint32_t ui32Module)
{
#if defined (AM_PART_APOLLO2)
    am_hal_uart_clock_enable(ui32Module);

    am_hal_uart_enable(ui32Module);
    am_hal_uart_int_enable(ui32Module, AM_HAL_UART_INT_RX_TMOUT |
                                       AM_HAL_UART_INT_RX |
                                       AM_HAL_UART_INT_TXCMP);

#if configUSE_LOG_UART0 || configUSE_PRINTF_UART0
    am_hal_gpio_pin_config(22, AM_HAL_PIN_22_UART0TX);
    am_hal_gpio_pin_config(17, AM_HAL_PIN_17_UART0RX);
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_UART0, AM_HAL_INTERRUPT_PRIORITY(6));
#endif // configUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_LOG_UART1
    am_hal_gpio_pin_config(39, AM_HAL_PIN_39_UART1TX);
    am_hal_gpio_pin_config(40, AM_HAL_PIN_40_UART1RX);
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_UART1, AM_HAL_INTERRUPT_PRIORITY(6));
#endif // configUSE_LOG_UART1

    am_hal_interrupt_enable(AM_HAL_INTERRUPT_UART + ui32Module);
#endif // AM_PART_APOLLO2

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    //
    // Enable interrupts.
    //
    NVIC_SetPriority(UART0_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));

    am_hal_interrupt_master_enable();
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P
}

void am_vos_gpio_enable_irq(uint8_t gpio)
{
#if defined (AM_PART_APOLLO2)
    // enable push to talk interrupt again
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(gpio));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(gpio));
#elif defined (AM_PART_APOLLO3)
    // enable push to talk interrupt again
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(gpio));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(gpio));
#elif defined (AM_PART_APOLLO3P)
    // enable push to talk interrupt again
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, gpio));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, gpio));
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}

void am_vos_gpio_disable_irq(uint8_t gpio)
{
#if defined (AM_PART_APOLLO2)
    am_hal_gpio_int_disable(AM_HAL_GPIO_BIT(gpio));
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(gpio));
#elif defined (AM_PART_APOLLO3)
    am_hal_gpio_interrupt_disable(AM_HAL_GPIO_BIT(gpio));
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(gpio));
#elif defined (AM_PART_APOLLO3P)
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_disable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, gpio));
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, gpio));
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}
void am_vos_button_int_register(void)
{
#if defined (AM_PART_APOLLO2)
#if configUSE_RTT_RECORDER
    //am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON0, button0_handler);
#endif // configUSE_RTT_RECORDER
#if configUSE_PUSH_TO_TALK
    am_hal_gpio_int_register(AM_BSP_GPIO_BUTTON1, am_vos_push_to_talk_process);
#endif // configUSE_PUSH_TO_TALK
#if configUSE_MUTE_MIC
    am_hal_gpio_int_register(AM_BSP_GPIO_BUTTON2, am_vos_mute_mic_process);
#endif // configUSE_MUTE_MIC

#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
#if configUSE_RTT_RECORDER
    //am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON0, button0_handler);
#endif // configUSE_RTT_RECORDER
#if configUSE_PUSH_TO_TALK
    am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON1, am_vos_push_to_talk_process);
#endif // configUSE_PUSH_TO_TALK
#if configUSE_MUTE_MIC
    am_hal_gpio_interrupt_register(AM_BSP_GPIO_BUTTON2, am_vos_mute_mic_process);
#endif // configUSE_MUTE_MIC
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}

void am_vos_button_init(void)
{
#if defined (AM_PART_APOLLO2)
    uint32_t ui32GPIONumber;
    for(int i = 0; i < AM_BSP_NUM_BUTTONS; i++)
    {
        ui32GPIONumber = am_bsp_psButtons[i].ui32GPIONumber;

        am_hal_gpio_pin_config(ui32GPIONumber, AM_HAL_PIN_INPUT);
        am_hal_gpio_int_polarity_bit_set(ui32GPIONumber, AM_HAL_GPIO_RISING);
    }
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON0, g_AM_BSP_GPIO_BUTTON0);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON1, g_AM_BSP_GPIO_BUTTON1);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_BUTTON2, g_AM_BSP_GPIO_BUTTON2);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P

    am_vos_gpio_enable_irq(AM_BSP_GPIO_BUTTON0);
    am_vos_gpio_enable_irq(AM_BSP_GPIO_BUTTON1);
    am_vos_gpio_enable_irq(AM_BSP_GPIO_BUTTON2);
}

#if configUSE_WOS
void am_vos_wos_gpio_init(void)
{
#if defined (AM_PART_APOLLO2)
#if (!USE_DMIC_MB3_VM3011)
    am_hal_gpio_pin_config(WOS_MODE_PIN, AM_HAL_PIN_OUTPUT | AM_HAL_PIN_INPUT);
    am_hal_gpio_out_bit_clear(WOS_MODE_PIN); //set low to enter normal mode
#endif // (!USE_DMIC_MB3_VM3011)

    am_hal_gpio_pin_config(WOS_WAKE_PIN, AM_HAL_PIN_INPUT);
    // Configure the GPIO/button interrupt polarity.
    am_hal_gpio_int_polarity_bit_set(WOS_WAKE_PIN, AM_HAL_GPIO_RISING);

    am_hal_gpio_int_register(WOS_WAKE_PIN, am_vos_wos_handler);

#if USE_DMIC_MB3_VM3011
    am_vos_gpio_disable_irq(WOS_WAKE_PIN);
#else // USE_DMIC_MB3_VM3011
    am_vos_gpio_enable_irq(WOS_WAKE_PIN);       // Enable the GPIO/button interrupt.
#endif // USE_DMIC_MB3_VM3011

#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_hal_gpio_pincfg_t sPinCfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    sPinCfg.uFuncSel       = WOS_WAKE_PIN_MODE_SEL, // GPIO
    sPinCfg.eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE;
    sPinCfg.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE;
    sPinCfg.eGPRdZero      = AM_HAL_GPIO_PIN_RDZERO_READPIN;
    sPinCfg.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
    am_hal_gpio_pinconfig(WOS_WAKE_PIN, sPinCfg);

    am_hal_gpio_interrupt_register(WOS_WAKE_PIN, am_vos_wos_handler);

#if USE_DMIC_MB3_VM3011
    am_vos_gpio_disable_irq(WOS_WAKE_PIN);
#else // USE_DMIC_MB3_VM3011
    am_vos_gpio_enable_irq(WOS_WAKE_PIN);
    am_hal_gpio_pinconfig(WOS_MODE_PIN, g_AM_HAL_GPIO_OUTPUT_WITH_READ);
    // uncomment to Set MODE pin to Wake On Sound at power up. Otherwise it will
    // happen anyway when QSD determines a quiet condition exists.
    am_hal_gpio_state_write(WOS_MODE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
#endif // USE_DMIC_MB3_VM3011

#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}
#endif // configUSE_WOS

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
void am_vos_lis2dw_gpio_init(void)
{
#if defined (AM_PART_APOLLO2)
    am_hal_gpio_pin_config(LIS2DW_INT_PIN, AM_HAL_PIN_INPUT);
    am_hal_gpio_int_polarity_bit_set(LIS2DW_INT_PIN, AM_HAL_GPIO_RISING);

    am_vos_gpio_enable_irq(LIS2DW_INT_PIN);
    //am_hal_gpio_pin_config(34, AM_HAL_PIN_OUTPUT);            // Debug purpose
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_hal_gpio_pincfg_t sOvvpPinCfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    sOvvpPinCfg.uFuncSel       = LIS2DW_INT_PIN_MODE_SEL, // GPIO
    sOvvpPinCfg.eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE;
    sOvvpPinCfg.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE;
    sOvvpPinCfg.eGPRdZero      = AM_HAL_GPIO_PIN_RDZERO_READPIN;
    sOvvpPinCfg.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
    am_hal_gpio_pinconfig(LIS2DW_INT_PIN, sOvvpPinCfg);

    am_vos_gpio_enable_irq(LIS2DW_INT_PIN);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP

//-----------------------------------------------------------------------------
// METHOD:  GPIO_Init
// PURPOSE: Setup I/O pins
//-----------------------------------------------------------------------------
//*****************************************************************************
//
// CTRL_BUTTON0 : RTT recorder switch button / Membrain switch button - Push to Talk (Maya)
// CTRL_BUTTON1 : Push To Talk / Power on-off switch (Maya)
// CTRL_BUTTON2 : MIC Mute button
// WOS_WAKE_PIN : Wake on Sound GPIO pin
//
//*****************************************************************************
void GPIO_Init(void)
{
    am_vos_button_int_register();
    am_vos_button_init();

    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);

#if configUSE_WOS
    am_vos_wos_gpio_init();
#endif // configUSE_WOS

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    am_vos_lis2dw_gpio_init();
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP

#if defined (AM_PART_APOLLO2)
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_GPIO, configMAX_SYSCALL_INTERRUPT_PRIORITY/*configMAX_SYSCALL_INTERRUPT_PRIORITY*/); //configKERNEL_INTERRUPT_PRIORITY);

#if (!configUSE_BLE)
    // If EM9304 is using, GPIO IRQ should be enable after EM9304 boot up.
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
#endif // (!configUSE_BLE)
#else // AM_PART_APOLLO2
    NVIC_SetPriority(GPIO_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(GPIO_IRQn);
#endif // AM_PART_APOLLO2
}

#if defined (AM_PART_APOLLO2) && (USE_DMIC_MB3 || USE_DMIC_MB3_VM3011)
//-----------------------------------------------------------------------------
// METHOD:  PDM_Init
// PURPOSE: PDM module configuration
//-----------------------------------------------------------------------------
void PDMInit(void)
{
    //
    // Enable power to PDM module and configure PDM
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PDM);

    //
    // Configure the PDM microphone pins
    //
    am_hal_gpio_pin_config(PDM_CLK, PDM_CLK_PIN_CFG); // Configure GP12 as PDM clock pin output (Need to blue-wire on REV4 Shield)
    am_hal_gpio_pin_config(PDM_DATA, PDM_DATA_PIN_CFG);// Configure GP11/GP23 as PDM data pin for unmodified Shield
    //
    // Configure the PDM module registers
    //
    am_hal_pdm_config_t sPDM_Cfg =
    {
        //
        // uint32_t ui32PDMConfigReg
        // PDM Configuration (PCFG, PDMCFG) register
        // Notes:
        //  Choose from AM_HAL_PDM_PCFG macros.
        //  For completeness, all PCFG fields should be referenced here.
        //
        (
                AM_HAL_PDM_PCFG_LRSWAP_DISABLE      |
                AM_HAL_PDM_PCFG_RIGHT_PGA_P105DB    |	// +10.5dB gain
                AM_HAL_PDM_PCFG_LEFT_PGA_P105DB     |	// +10.5dB gain
                AM_HAL_PDM_PCFG_MCLKDIV_DIV1        |
#if USE_PDM_CLK_750KHZ
#if configUSE_OAL_AID
                AM_HAL_PDM_PCFG_SINC_RATE(48)       |   // OAL AID only support 8 kHz
#else // configUSE_OAL_AID
                AM_HAL_PDM_PCFG_SINC_RATE(24)       |   // over sample rate (decimation) for 750 kHz
#endif // configUSE_OAL_AID
#elif USE_PDM_CLK_1_5MHZ
                AM_HAL_PDM_PCFG_SINC_RATE(48)       |   // over sample rate (decimation) for 1.5Mhz
#endif // USE_PDM_CLK_750KHZ, USE_PDM_CLK_1_5MHZ
                AM_HAL_PDM_PCFG_SOFTMUTE_DISABLE    |
#if configUSE_AWE
                AM_HAL_PDM_PCFG_ADCHPD_ENABLE       |
                AM_HAL_PDM_PCFG_HPCUTOFF(0xB)       |   // set high pass coefficient. Must do this!
#else                                                   // no signal preprocessing present, enable High pass filter
                AM_HAL_PDM_PCFG_ADCHPD_DISABLE      |
                AM_HAL_PDM_PCFG_HPCUTOFF(0xB)       |   // 0x4 is best value for A2x.
#endif // configUSE_AWE
                AM_HAL_PDM_PCFG_PDMCORE_DISABLE
        ),

        //
        // uint32_t ui32VoiceConfigReg
        // PDM Voice Config (VCFG, VOICECFG) register
        // Notes:
        //  Choose from AM_HAL_PDM_VCFG macros.
        //  For completeness, all VCFG fields should be referenced here.
        //  AM_HAL_PDM_IOCLK_xxx also sets AM_REG_PDM_VCFG_IOCLKEN_EN
        //  RSTB is set to NORMAL by am_hal_pdm_enable.
        //
        (
#if USE_PDM_CLK_750KHZ
                AM_HAL_PDM_IOCLK_750KHZ             |   // AM_REG_PDM_VCFG_IOCLKEN_EN
#elif USE_PDM_CLK_1_5MHZ
                AM_HAL_PDM_IOCLK_1_5MHZ             |   // AM_REG_PDM_VCFG_IOCLKEN_EN
#endif // USE_PDM_CLK_750KHZ, USE_PDM_CLK_1_5MHZ
                AM_HAL_PDM_VCFG_RSTB_RESET          |
                AM_HAL_PDM_VCFG_PDMCLK_ENABLE       |
                AM_HAL_PDM_VCFG_I2SMODE_DISABLE     |
                AM_HAL_PDM_VCFG_BCLKINV_DISABLE     |
                AM_HAL_PDM_VCFG_DMICDEL_0CYC        |
                AM_HAL_PDM_VCFG_SELAP_INTERNAL      |
                AM_HAL_PDM_VCFG_PACK_ENABLE         |
                AM_HAL_PDM_VCFG_CHANNEL_STEREO
        ),

        //
        // uint32_t ui32FIFOThreshold
        // The PDM controller will generate a processor interrupt when the number
        // of entries in the FIFO goes *above* this number.
        //
        (PCM_FRAME_SIZE - 1)
    };

    am_hal_pdm_config(&sPDM_Cfg);

    //
    // Make sure interrupts are clear
    //
    am_hal_pdm_int_clear(AM_HAL_PDM_INT_FIFO | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF);
    am_hal_pdm_fifo_flush();

    // Enable interrupts PDM
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_PDM);  //NVIC setting
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_PDM, AM_HAL_INTERRUPT_PRIORITY(4));
    am_hal_pdm_int_enable(AM_HAL_PDM_INT_FIFO | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF);

    //am_hal_pdm_enable(); // enable the PDM mic interface

}
#endif // AM_PART_APOLLO2 && (USE_DMIC_MB3 || USE_DMIC_MB3_VM3011)

#if (defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)) && (USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011)
//*****************************************************************************
//
// Start a transaction to get some number of bytes from the PDM interface.
//
//*****************************************************************************
void
pdm_trigger_dma(void)
{
    //
    // Configure DMA and target address.
    //
    am_hal_pdm_transfer_t sTransfer;
    sTransfer.ui32TargetAddr = (uint32_t ) g_sVosSys.ui32PDMDataBuf;
    sTransfer.ui32TotalCount = (PCM_FRAME_SIZE * SAMPLE_32BIT);

    //
    // Start the data transfer.
    //
    am_hal_pdm_dma_start(g_sVosSys.pvPDMHandle, &sTransfer);
}

// This was a workaround code to measure power consumption. Need to review!!
#if 1
//*****************************************************************************
//
// Structure for handling PDM register state information for power up/down
//
//*****************************************************************************
typedef struct
{
    bool bValid;
}
am_hal_pdm_register_state_t;

//*****************************************************************************
//
// Structure for handling PDM HAL state information.
//
//*****************************************************************************
typedef struct
{
    am_hal_handle_prefix_t prefix;
    am_hal_pdm_register_state_t sRegState;
    uint32_t ui32Module;
}
am_hal_pdm_state_t;

void am_vos_pdm_dma_disable(void *pHandle)
{
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) pHandle;
    uint32_t ui32Module = pState->ui32Module;

    //
    // Disable DMA
    //
    PDMn(ui32Module)->DMACFG_b.DMAEN = PDM_DMACFG_DMAEN_DIS;

}
#endif

//-----------------------------------------------------------------------------
// METHOD:  PDM_Init
// PURPOSE: PDM module configuration
//-----------------------------------------------------------------------------
void PDMInit(void)
{
    // PDM configuration information.
    am_hal_pdm_config_t g_sPdmConfig =
    {
        .eClkDivider = AM_HAL_PDM_MCLKDIV_1,
#if defined (AM_VOS_CSPOTTER_CES)
        .eLeftGain = AM_HAL_PDM_GAIN_P405DB,
        .eRightGain = AM_HAL_PDM_GAIN_P405DB,
#elif defined (AM_VOS_DSPOTTER)
			  .eLeftGain = AM_HAL_PDM_GAIN_P90DB,//AM_HAL_PDM_GAIN_P255DB,
        .eRightGain = AM_HAL_PDM_GAIN_P90DB,//AM_HAL_PDM_GAIN_P255DB,
#else // AM_VOS_CSPOTTER_CES
        .eLeftGain = AM_HAL_PDM_GAIN_P105DB,
        .eRightGain = AM_HAL_PDM_GAIN_P105DB,
#endif // AM_VOS_CSPOTTER_CES
#if USE_PDM_CLK_750KHZ
#if configUSE_OAL_AID
        .ui32DecimationRate = 48,       // OAL AID only support 8 kHz. CLK 750 kHz
#else // configUSE_OAL_AID
        .ui32DecimationRate = 24,       // CLK 750 kHz
#endif // configUSE_OAL_AID
        .ePDMClkSpeed = AM_HAL_PDM_CLK_750KHZ,
#elif USE_PDM_CLK_1_5MHZ
        .ui32DecimationRate = 48,     // CLK 1.5 Mhz
        .ePDMClkSpeed = AM_HAL_PDM_CLK_1_5MHZ,
#endif // USE_PDM_CLK_750KHZ, USE_PDM_CLK_1_5MHZ
        .bHighPassEnable = 0,       // 0 is enabled. (Reversed..)
        .ui32HighPassCutoff = 0xB,
        .bInvertI2SBCLK = 0,
        .ePDMClkSource = AM_HAL_PDM_INTERNAL_CLK,
        .bPDMSampleDelay = 0,
        .bDataPacking = 1,
        .ePCMChannels = AM_HAL_PDM_CHANNEL_STEREO,
        .bLRSwap = 0,
        .bSoftMute = 0,
    };

    configASSERT(g_sVosSys.pvPDMHandle == NULL);

    if (g_sVosSys.pvPDMHandle != NULL)
        return;
    //
    // Initialize, power-up, and configure the PDM.
    //
    am_hal_pdm_initialize(0, &g_sVosSys.pvPDMHandle);
    //
    // Configure the necessary pins.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDMCLK, g_AM_BSP_GPIO_PDMCLK);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM_DATA, g_AM_BSP_GPIO_PDM_DATA);

    am_hal_pdm_power_control(g_sVosSys.pvPDMHandle, AM_HAL_PDM_POWER_ON, false);
    am_hal_pdm_configure(g_sVosSys.pvPDMHandle, &g_sPdmConfig);
    am_hal_pdm_fifo_flush(g_sVosSys.pvPDMHandle);

    //
    // Configure and enable PDM interrupts (set up to trigger on DMA
    // completion).
    //
    am_hal_pdm_interrupt_enable(g_sVosSys.pvPDMHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    NVIC_EnableIRQ(PDM_IRQn);
    NVIC_SetPriority(PDM_IRQn, NVIC_configKERNEL_INTERRUPT_PRIORITY);

    //
    // Enable PDM
    //
    //am_hal_pdm_enable(g_sVosSys.pvPDMHandle);
    //pdm_trigger_dma();

}
#endif // (AM_PART_APOLLO3 || AM_PART_APOLLO3P) && (USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011)

#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
void PDMDeInit(void)
{
#if defined (AM_PART_APOLLO2)
    // Disable interrupts PDM
    am_hal_interrupt_disable(AM_HAL_INTERRUPT_PDM);  //NVIC setting

    // Make sure interrupts are clear
    am_hal_pdm_int_clear(AM_HAL_PDM_INT_FIFO | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF);

    am_hal_pdm_disable();

    am_hal_gpio_pin_config(PDM_CLK, AM_HAL_PIN_DISABLE); // Disable PDM clk pin
    am_hal_gpio_pin_config(PDM_DATA, AM_HAL_PIN_DISABLE);  //Disable PDM data pin

    //Power Down PDM interface to further reduce power
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PDM);

    am_hal_gpio_pin_config(PDM_CLK, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(PDM_CLK);

    am_hal_gpio_pin_config(PDM_DATA, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(PDM_DATA);
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    if(g_sVosSys.pvPDMHandle == NULL)
      return;

    am_hal_pdm_interrupt_clear(g_sVosSys.pvPDMHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    am_hal_pdm_interrupt_disable(g_sVosSys.pvPDMHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    NVIC_DisableIRQ(PDM_IRQn);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDMCLK, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM_DATA, g_AM_HAL_GPIO_DISABLE);

    // This was a workaround code to measure power consumption. Need to review!!
    am_vos_pdm_dma_disable(g_sVosSys.pvPDMHandle);

    am_hal_pdm_disable(g_sVosSys.pvPDMHandle);
    am_hal_pdm_power_control(g_sVosSys.pvPDMHandle, AM_HAL_PDM_POWER_OFF, false);

    am_hal_pdm_deinitialize(g_sVosSys.pvPDMHandle);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDMCLK, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_PDMCLK, AM_HAL_GPIO_OUTPUT_CLEAR);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_PDM_DATA, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(AM_BSP_GPIO_PDM_DATA, AM_HAL_GPIO_OUTPUT_CLEAR);
    
    g_sVosSys.pvPDMHandle = NULL;
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

void ADCInit(void)
{
#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    am_hal_gpio_pincfg_t sPinCfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Set a pin to act as our ADC input
    //
#if USE_AMIC_MB3_VMB || USE_AMIC_MB3
    sPinCfg.uFuncSel = ADC_INPUT_RIGHT_CFG;
    am_hal_gpio_pinconfig(ADC_INPUT_RIGHT_PIN, sPinCfg);
#endif // USE_AMIC_MB3_VMB || USE_AMIC_MB3

#if USE_AMIC_MB3_VMB || USE_AMIC_MB2
    sPinCfg.uFuncSel = ADC_INPUT_LEFT_CFG;
    am_hal_gpio_pinconfig(ADC_INPUT_LEFT_PIN, sPinCfg);
#endif // USE_AMIC_MB3_VMB || USE_AMIC_MB2

    //
    // Configure the ADC
    //
    adc_config();
    //
    // Start the CTIMER A3 for timer-based ADC measurements.
    //
    init_timerA3_for_ADC();
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
}

void ADCDeInit(void)
{
#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    if(g_sVosSys.pvADCHandle == NULL)
      return;

    am_hal_adc_interrupt_clear(g_sVosSys.pvADCHandle, (AM_HAL_ADC_INT_DERR
                                            | AM_HAL_ADC_INT_DCMP));

    am_hal_adc_interrupt_disable(g_sVosSys.pvADCHandle, (AM_HAL_ADC_INT_DERR
                                            | AM_HAL_ADC_INT_DCMP));

    NVIC_DisableIRQ(ADC_IRQn);

#if USE_AMIC_MB3_VMB || USE_AMIC_MB3
    am_hal_gpio_pinconfig(ADC_INPUT_RIGHT_PIN, g_AM_HAL_GPIO_DISABLE);
#endif // USE_AMIC_MB3_VMB || USE_AMIC_MB3

#if USE_AMIC_MB3_VMB || USE_AMIC_MB2
    am_hal_gpio_pinconfig(ADC_INPUT_LEFT_PIN, g_AM_HAL_GPIO_DISABLE);
#endif // USE_AMIC_MB3_VMB || USE_AMIC_MB2

    // This was a workaround code to measure power consumption. Need to review!!
    //am_vos_pdm_dma_disable(g_sVosSys.pvADCHandle);

    am_hal_adc_disable(g_sVosSys.pvADCHandle);
    am_hal_adc_power_control(g_sVosSys.pvADCHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);

    am_hal_adc_deinitialize(g_sVosSys.pvADCHandle);
    g_sVosSys.pvADCHandle = NULL;

    //
    // Stop the timer.
    //
    am_hal_ctimer_stop(3, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_adc_trigger_disable();
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
}

void am_vos_mic_enable(void)
{
#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
#if defined (AM_PART_APOLLO2)
    am_hal_pdm_enable();
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    if(g_sVosSys.pvPDMHandle)
    {
        am_hal_pdm_enable(g_sVosSys.pvPDMHandle);
        pdm_trigger_dma();
    }
    else
        AM_APP_LOG_WARNING("Failed to PDM enable : PDM handle is NULL\n");
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    //
    // Enable the ADC.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_enable(g_sVosSys.pvADCHandle))
    {
        am_util_stdio_printf("Error - enabling ADC failed.\n");
    }

    //
    // Trigger the ADC sampling for the first time manually.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_sw_trigger(g_sVosSys.pvADCHandle))
    {
        am_util_stdio_printf("Error - triggering the ADC failed.\n");
    }
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
}

void am_vos_mic_disable(void)
{
#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
#if defined (AM_PART_APOLLO2)
    am_hal_pdm_disable();
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    if(g_sVosSys.pvPDMHandle == NULL)
    {
        AM_APP_LOG_WARNING("Failed to PDM enable : PDM handle is NULL\n");
        return;
    }

    am_hal_pdm_disable(g_sVosSys.pvPDMHandle);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    if(g_sVosSys.pvADCHandle == NULL)
      return;

    am_hal_adc_disable(g_sVosSys.pvADCHandle);
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
}

void am_vos_wos_mic_enable(void)
{
#if configUSE_WOS && (!USE_DMIC_MB3_VM3011)
#if defined (AM_PART_APOLLO2)
    am_hal_gpio_out_bit_clear(WOS_MODE_PIN);

#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_hal_gpio_state_write(WOS_MODE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
#endif // configUSE_WOS && (!USE_DMIC_MB3_VM3011)

//    am_hal_gpio_interrupt_disable(AM_HAL_GPIO_BIT(WOS_WAKE_PIN));
//    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(WOS_WAKE_PIN));

#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
    PDMInit();

#if defined (AM_PART_APOLLO2)
    am_hal_pdm_enable();

#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    if(g_sVosSys.pvPDMHandle)
    {
        am_hal_pdm_enable(g_sVosSys.pvPDMHandle);
        pdm_trigger_dma();
    }
    else
        AM_APP_LOG_WARNING("Failed to PDM enable : PDM handle is NULL\n");
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    ADCInit();
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
}


void am_vos_wos_mic_disable(void)
{
#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
    PDMDeInit();
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    ADCDeInit();
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB

#if configUSE_WOS && (!USE_DMIC_MB3_VM3011)
#if defined (AM_PART_APOLLO2)
    am_hal_gpio_out_bit_set(WOS_MODE_PIN);

#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_hal_gpio_state_write(WOS_MODE_PIN, AM_HAL_GPIO_OUTPUT_SET);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
#endif // configUSE_WOS && (!USE_DMIC_MB3_VM3011)

//    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(WOS_WAKE_PIN));
//    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(WOS_WAKE_PIN));
}

#if configUSE_PRINTF_SWO
//*****************************************************************************
//
// @brief Enable printing over ITM.
// Override the bsp function
//
//*****************************************************************************
void
am_vos_itm_printf_enable(void)
{
#if defined (AM_PART_APOLLO2)

    am_hal_tpiu_config_t TPIUcfg;

    if ( g_ui32HALflags & AM_HAL_FLAGS_ITMSKIPENABLEDISABLE_M )
    {
        return;
    }

    //
    // Write to the ITM control and status register.
    //
    AM_REGVAL(AM_REG_ITM_TCR_O) =
        AM_WRITE_SM(AM_REG_ITM_TCR_ATB_ID, 0x15)      |
        AM_WRITE_SM(AM_REG_ITM_TCR_TS_FREQ, 1)        |
        AM_WRITE_SM(AM_REG_ITM_TCR_TS_PRESCALE, 1)    |
        AM_WRITE_SM(AM_REG_ITM_TCR_SWV_ENABLE, 1)     |
        AM_WRITE_SM(AM_REG_ITM_TCR_DWT_ENABLE, 0)     |
        AM_WRITE_SM(AM_REG_ITM_TCR_SYNC_ENABLE, 0)    |
        AM_WRITE_SM(AM_REG_ITM_TCR_TS_ENABLE, 0)      |
        AM_WRITE_SM(AM_REG_ITM_TCR_ITM_ENABLE, 1);

    //
    // Enable the ITM and TPIU
    //
    TPIUcfg.ui32SetItmBaud = AM_HAL_TPIU_BAUD_1M;
    am_hal_tpiu_enable(&TPIUcfg);
    am_bsp_pin_enable(ITM_SWO);

    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_hal_itm_print);
    am_hal_itm_enable();

#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)

    am_hal_tpiu_config_t TPIUcfg;

    //
    // Enable the ITM interface and the SWO pin.
    //
    am_hal_itm_enable();

    //
    // Enable the ITM and TPIU
    // Set the BAUD clock for 1M
    //
    TPIUcfg.ui32SetItmBaud = AM_HAL_TPIU_BAUD_1M;
    am_hal_tpiu_enable(&TPIUcfg);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_ITM_SWO, g_AM_BSP_GPIO_ITM_SWO);

    //
    // Attach the ITM to the STDIO driver.
    //
    am_util_stdio_printf_init(am_hal_itm_print);

#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}
#endif // configUSE_PRINTF_SWO

void am_vos_print_system_info(void)
{
    am_util_id_t sIdDevice;

    am_util_stdio_printf("\n====== Ambiq Micro VoS SDK ======\n");

    am_util_id_device(&sIdDevice);
    am_util_stdio_printf("Device type: %s\n", sIdDevice.pui8DeviceName);
    am_util_stdio_printf("Device info:\n\tPart number: 0x%08X\n"
                         "\tRevision: 0x%X (Rev%c%c)\n",
                         sIdDevice.sMcuCtrlDevice.ui32ChipPN,
                         sIdDevice.sMcuCtrlDevice.ui32ChipRev,
                         sIdDevice.ui8ChipRevMaj, sIdDevice.ui8ChipRevMin );
    am_util_stdio_printf("\tChip ID 0: 0x%X\n", sIdDevice.sMcuCtrlDevice.ui32ChipID0);
    am_util_stdio_printf("\tChip ID 1: 0x%X\n", sIdDevice.sMcuCtrlDevice.ui32ChipID1);

    am_util_stdio_printf("Firmware version: ");
    am_util_stdio_printf(VOS_AMA_FW_VER_STRING);
    am_util_stdio_printf("\nLoad modules:");

#if configUSE_AWE
    am_util_stdio_printf(" AWE");
#endif // configUSE_AWE

#if configUSE_Sensory_THF
    am_util_stdio_printf(" THF");
#endif // configUSE_Sensory_THF

#if configUSE_Cyberon_Spotter
#ifdef AM_VOS_DSPOTTER
    am_util_stdio_printf(" DSpotter");
#else
		am_util_stdio_printf(" CSpotter");
#endif
#endif // configUSE_Cyberon_Spotter

#if configUSE_RetuneDSP_VS
    am_util_stdio_printf(" RDSP");
#endif // configUSE_RetuneDSP_VS

#if configUSE_OAL_AID
    am_util_stdio_printf(" AID");
#endif // configUSE_OAL_AID

#if configUSE_AMVOS_AMA
    am_util_stdio_printf(" AMA");
#endif // configUSE_AMVOS_AMA

#if configUSE_MSBC_BLUEZ
    am_util_stdio_printf(" mSBC");
#endif // configUSE_MSBC_BLUEZ

#if configUSE_OPTIM_OPUS
    am_util_stdio_printf(" OPUS");
#endif // configUSE_OPTIM_OPUS

    am_util_stdio_printf("\nSystem Heap: %d KB (%d bytes)", g_sVosSys.ui32SysHeapSize / 1024, g_sVosSys.ui32SysHeapSize);
    am_util_stdio_printf("\n=================================\n\n");

#if defined (AM_PART_APOLLO2)
    am_util_stdio_printf("System reset reason: 0x4000000C = 0x%08x \n", (*(volatile uint32_t*)0x4000000C));
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_util_stdio_printf("System reset reason: 0x4FFFF000 = 0x%08x \n", (*(volatile uint32_t*)0x4FFFF000));
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
}

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
void am_vos_burst_mode_setup(void)
{
    // Check that the Burst Feature is available.
    if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_initialize(&g_sVosSys.eBurstModeAvailable))
    {
        if (AM_HAL_BURST_AVAIL == g_sVosSys.eBurstModeAvailable)
        {
            am_util_stdio_printf("Apollo3 Burst Mode is Available\n");

            // Make sure we are in "Normal" mode.
            if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_disable(&(g_sVosSys.eBurstMode)))
            {
                if (AM_HAL_NORMAL_MODE == g_sVosSys.eBurstMode)
                {
                    am_util_stdio_printf("Apollo3 operating in Normal Mode (48MHz)\n");
                }
            }
            else
            {
                am_util_stdio_printf("Failed to Disable Burst Mode operation\n");
            }
        }
        else
        {
            am_util_stdio_printf("Apollo3 Burst Mode is Not Available\n");
        }
    }
    else
    {
        am_util_stdio_printf("Failed to Initialize for Burst Mode operation\n");
    }
    return;
}

void am_vos_burst_mode_enable(void)
{
    if(AM_HAL_BURST_MODE == g_sVosSys.eBurstMode) {
        //AM_APP_LOG_DEBUG("BST NOP\n");
        return;
    }

    if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_enable(&(g_sVosSys.eBurstMode)))
    {
        if (AM_HAL_BURST_MODE == g_sVosSys.eBurstMode)
        {
            AM_APP_LOG_DEBUG("\n[AM-VoS] Burst Mode (96MHz)\n\n");
        }
    }
    else
    {
        AM_APP_LOG_WARNING("Failed to Enable Burst Mode operation\n");
    }
}

void am_vos_burst_mode_disable(void)
{
    if(AM_HAL_NORMAL_MODE == g_sVosSys.eBurstMode) {
        //AM_APP_LOG_DEBUG("NOR NOP\n");
        return;
    }

    // Make sure we are in "Normal" mode.
    if (AM_HAL_STATUS_SUCCESS == am_hal_burst_mode_disable(&(g_sVosSys.eBurstMode)))
    {
        if (AM_HAL_NORMAL_MODE == g_sVosSys.eBurstMode)
        {
            AM_APP_LOG_DEBUG("\n[AM-VoS] Normal Mode (48MHz)\n\n");
        }
    }
    else
    {
        AM_APP_LOG_WARNING("Failed to Disable Burst Mode operation\n");
    }
}
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

//-----------------------------------------------------------------------------
// METHOD:  ApolloBoardInitialization
// PURPOSE: Setup board peripherals
//-----------------------------------------------------------------------------
void am_vos_board_init(void)
{
    GPIO_Init();

#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
    PDMInit();
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_DMIC_MB3_VM3011
    am_devices_vm3011_init();
    am_devices_vm3011_config();
    am_devices_vm3011_deinit();
#endif // USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    ADCInit();
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB

#if USE_MAYA
    am_vos_logic_maya_hw_init();
#else // USE_MAYA
    am_vos_logic_led_swirl(1);
#endif // USE_MAYA

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    am_devices_lis2dw12_init();
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP

    //
    // Enable debug printf messages using ITM on SWO pin
    //
#if configUSE_PRINTF_SWO
    //init SWO print here
    am_vos_itm_printf_enable();
#endif // configUSE_PRINTF_SWO

#if configUSE_AMU2S_RECORDER
    amu2s_init();
#endif // configUSE_AMU2S_RECORDER

    am_vos_print_system_info();

#if defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    am_vos_burst_mode_setup();
#if configUSE_AIC_LITE_LAYOUT
    am_vos_burst_mode_enable();
#endif // configUSE_AIC_LITE_LAYOUT
#endif // AM_PART_APOLLO3, AM_PART_APOLLO3P

#if configUSE_LOG_UART0 || configUSE_PRINTF_UART0
    uart_init(UART0_MODULE);
    uart_enable(UART0_MODULE);
#endif // configUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_LOG_UART1
    uart_init(UART1_MODULE);
    uart_enable(UART1_MODULE);
#endif // configUSE_LOG_UART1

    //
    // Enable software interrupts
    //
#if defined (AM_PART_APOLLO2)
    am_hal_interrupt_pend_clear(AM_HAL_INTERRUPT_SOFTWARE1);
    am_hal_interrupt_pend_clear(AM_HAL_INTERRUPT_SOFTWARE2);
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_SOFTWARE1, AM_HAL_INTERRUPT_PRIORITY(5));
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_SOFTWARE2, AM_HAL_INTERRUPT_PRIORITY(6));
#endif // AM_PART_APOLLO2

    // master interrupt enable
    am_hal_interrupt_master_enable();

#if configUSE_BLE
    RadioTaskSetup();
#else // configUSE_BLE

#if USE_APOLLO2_BLUE_EVB || USE_APOLLO3_BLUE_EVB || USE_MAYA
    HciDrvRadioShutdown();
#endif // USE_APOLLO2_BLUE_EVB || USE_APOLLO3_BLUE_EVB || USE_MAYA

#endif // configUSE_BLE
}   // End BoardInit

void am_vos_mic_fifo_flush()
{
#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
#if defined (AM_PART_APOLLO2)
    am_hal_pdm_fifo_flush();
#elif defined (AM_PART_APOLLO3) || defined (AM_PART_APOLLO3P)
    if(g_sVosSys.pvPDMHandle)
        am_hal_pdm_fifo_flush(g_sVosSys.pvPDMHandle);
#endif // AM_PART_APOLLO2, AM_PART_APOLLO3, AM_PART_APOLLO3P
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011

#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
}
