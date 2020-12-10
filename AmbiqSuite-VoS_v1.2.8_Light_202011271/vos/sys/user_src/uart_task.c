//*****************************************************************************
//
//! @file hello_world_uart.c
//!
//! @brief A simple "Hello World" example using the UART peripheral.
//!
//! Purpose: This example prints a "Hello World" message with some device info
//! over UART at 115200 baud. To see the output of this program, run AMFlash,
//! and configure the console for UART. The example sleeps after it is done
//! printing.
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
#include "am_util.h"


//#define FLOW_CTRL
#define TEMP_BUFF_SIZE 40

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
void *phUART;
volatile bool g_bRxTimeoutFlag = false;

uint8_t g_pui8TxBuffer[1024];
uint8_t g_pui8RxBuffer[1024];


//*****************************************************************************
//
// UART configuration.
//
//*****************************************************************************
const am_hal_uart_config_t g_sUartConfig =
{
    //
    // Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate = 115200*8,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
#ifdef FLOW_CTRL
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_RTS_CTS,
#else
	.ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
#endif

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                       AM_HAL_UART_RX_FIFO_1_2),

	.pui8TxBuffer = g_pui8TxBuffer,
    .ui32TxBufferSize = sizeof(g_pui8TxBuffer),
    .pui8RxBuffer = g_pui8RxBuffer,
    .ui32RxBufferSize = sizeof(g_pui8RxBuffer),
};

//*****************************************************************************
//
// UART0 interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
	am_hal_uart_interrupt_service(phUART, ui32Status, &ui32Idle);
	
    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_RX))
    {
		g_bRxTimeoutFlag = true;
    }
}


uint16_t uart_buff_send(uint16_t size, const uint8_t *data, uint32_t timeout)
{

    uint32_t ret = AM_HAL_STATUS_SUCCESS;
    uint32_t transfer_size;

    am_hal_uart_transfer_t sUartWrite = {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *)data,
        .ui32NumBytes = size,
        .ui32TimeoutMs = timeout,
        .pui32BytesTransferred = &transfer_size,
    };

    ret = am_hal_uart_transfer(phUART, &sUartWrite);

    if (ret == AM_HAL_STATUS_SUCCESS)
		return transfer_size;
	else
		return 0;
}

uint16_t uart_buff_receive(uint16_t size, uint8_t *data, uint32_t timeout)
{

    uint32_t ret = AM_HAL_STATUS_SUCCESS;
    uint32_t transfer_size;

    am_hal_uart_transfer_t sUartRead = {
        .ui32Direction = AM_HAL_UART_READ,
        .pui8Data = (uint8_t *) data,
        .ui32NumBytes = size,
        .ui32TimeoutMs = timeout,
        .pui32BytesTransferred = &transfer_size,
    };

    ret = am_hal_uart_transfer(phUART, &sUartRead);

    if (ret == AM_HAL_STATUS_SUCCESS)
        return transfer_size;
	else
		return 0;

}


//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
	uint32_t ui32NumBytesRead;
	uint8_t ui8inData[TEMP_BUFF_SIZE];
	const uint8_t ui8outData='$';

	am_hal_gpio_pincfg_t pincfg = {0};

    //
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for UART output.
    //
    am_hal_uart_initialize(0, &phUART);
    am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(phUART, &g_sUartConfig);

    //
    // Enable the UART pins.
    //
    pincfg.uFuncSel = AM_HAL_PIN_1_UART0TX;	
	pincfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_8MA;
    am_hal_gpio_pinconfig(1, pincfg);
	pincfg.uFuncSel = AM_HAL_PIN_2_UART0RX;	
    am_hal_gpio_pinconfig(2, pincfg);

#ifdef FLOW_CTRL	
	pincfg.uFuncSel = AM_HAL_PIN_4_UA0CTS;
	am_hal_gpio_pinconfig(4, pincfg);
	pincfg.uFuncSel = AM_HAL_PIN_3_UA0RTS;	
	pincfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA;
	am_hal_gpio_pinconfig(3, pincfg);
#endif
    //
    // Enable interrupts.
    //
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn));
	am_hal_uart_interrupt_enable(phUART, (AM_HAL_UART_INT_RX | AM_HAL_UART_INT_TX | AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_TXCMP));
    am_hal_interrupt_master_enable();

	//
    // Initialize the printf interface for ITM output
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("UART Buffer\n\n");


	uart_buff_send(1, (const uint8_t *)&ui8outData, AM_HAL_UART_WAIT_FOREVER);


    //
    // Loop forever while sleeping.
    //
    while (1)
    {
		if(g_bRxTimeoutFlag)
		{
			g_bRxTimeoutFlag = false;
			ui32NumBytesRead = uart_buff_receive(TEMP_BUFF_SIZE, ui8inData, 0);
			am_util_stdio_printf("IN %d\n",ui32NumBytesRead);

			for(int i=0; i < ui32NumBytesRead; i++)
				am_util_stdio_printf("%c ",ui8inData[i]);
			am_util_stdio_printf("\n");
			am_util_delay_ms(500);
			uart_buff_send(ui32NumBytesRead, ui8inData, AM_HAL_UART_WAIT_FOREVER);
		}
		//
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
