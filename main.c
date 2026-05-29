

/**
 * main.c
 */
/* Includes-----------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pll.h"
#include "uart.h"
#include "mti630ahrs.h"
#include "tm4c123gh6pm.h"

/* Private prototypes-------------------------------------------------------------------------------*/
static void config(void);
static void updateimu(void);

/* Handles----------------------------------------------------------------------------------*/
PLL_Handle_t pllhandle;
UART_Handle_t uart2handle;

/* IMU Constants----------------------------------------------------------------------------------*/
const char gotomeasurement[] = {0xFA, 0xFF, 0x10, 0x00, 0xF1};
const char gotoconfig[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};

/* Private mti630 ahrs variables--------------------------------------------------------------------*/
UART_frame_manager_t frame_mgr = {
    .buffer_a.complete = 0,
    .buffer_b.complete = 0,
    .p_write = &frame_mgr.buffer_a,
    .p_read = NULL,
    .frame_ready = 0,
    .frame_errors = 0,
};

UART_rx_context_t rx_ctx = {
    .state = SYNC_0,
    .index = 0,
    .timeout_counter = 0
};



int main(void)
{
    config();

	updateimu();

	while(true)
	{
	    PLL_API.delayMs(10);
	}
}


static void updateimu(void)
{
	UART_API.sendString(&uart2handle,gotoconfig,sizeof(gotoconfig));
	PLL_API.delayMs(1000);
	UART_API.sendString(&uart2handle,gotomeasurement,sizeof(gotomeasurement));
	PLL_API.delayMs(1000);

}

void config(void)
{
	PLL_Status_t pllstat = PLL_API.init(&pllhandle,
			MHz80);
	while(pllstat != PLL_STATUS_SUCCESS)
	{

	}

	/////////////////////UART2 Config
	UART_Config_t uart2config = {
		.module = UART_MODULE_2,
		.baudRate = UART_BAUD_115200,
		.clockFreqMHz = 80,
		.enableTx = true,
		.enableRx = true,
		.enableFIFO = true,
		.fifoLevel = UART_FIFO_LEVEL_1_2
	};
	UART_Status_t uart2status = UART_API.init(&uart2handle,
											&uart2config);
	switch (uart2status)
	{
		case UART_STATUS_SUCCESS:
		break;
		case UART_STATUS_INVALID_PARAM:
		break;
		case UART_STATUS_TIMEOUT:
		break;
		default:
		break;
	}
	UART_API.enableInterrupt(&uart2handle, UART_FIFO_LEVEL_1_8);
}

void UART2_isr(void)
{
	uint32_t status = UART2_MIS_R; // Read masked interrupt status
	UART2_ICR_R = status; // Clear the interrupt

	while(!(UART2_FR_R & UART_FR_RXFE)) // Check if RX FIFO is not empty
	{
		uint8_t data = (uint8_t)(UART2_DR_R & 0xFF); // Read received byte

		switch(rx_ctx.state)
		{
			case SYNC_0:
				if(data == 0xFA)
				{
					frame_mgr.p_write->frame[0] = data; // Store first byte of header
					rx_ctx.index = 1; // Move to next index for second byte
					rx_ctx.state = SYNC_1;	// Move to next state
					rx_ctx.timeout_counter = 0; // Reset timeout counter
				}
				break;
			case SYNC_1:
				if(data == 0xFF)
				{
					frame_mgr.p_write->frame[1] = data; // Store second byte of header
					rx_ctx.index = 2; // Move to next index for payload
					rx_ctx.state = RX_PALOAD; // Move to payload receiving state
				}
				else
				{
					// false syncronization, reset
					rx_ctx.state = SYNC_0;
					if(data == 0xFA)
					{
						frame_mgr.p_write->frame[0] = data; // Store first byte of header
						rx_ctx.index = 1; // Move to next index for second byte
						rx_ctx.state = SYNC_1;	// Move to next state
					}
				}
				break;
			case RX_PALOAD:
				frame_mgr.p_write->frame[rx_ctx.index++] = data; // Store received byte
				rx_ctx.index++;
				rx_ctx.timeout_counter = 0; // Reset timeout counter

				if(rx_ctx.index >= MTI_FRAME_SIZE)
				{
					frame_mgr.p_write->complete = 1; // Mark frame as complete
					frame_mgr.frame_ready = 1; // Set frame ready flag for main loop

					//Interchange buffers
					if(frame_mgr.p_write == &frame_mgr.buffer_a)
					{
						frame_mgr.p_write = &frame_mgr.buffer_b; // Switch to other buffer for next frame
						frame_mgr.p_read = &frame_mgr.buffer_a; // Main loop will read from the completed buffer
					}
					else
					{
						frame_mgr.p_write = &frame_mgr.buffer_a; // Switch to other buffer for next frame
						frame_mgr.p_read = &frame_mgr.buffer_b; // Main loop will read from the completed buffer
					}

					frame_mgr.p_write->complete = 0; // Reset complete flag for new frame

					// Reset
					rx_ctx.state = SYNC_0; // Reset state to look for next frame header
					rx_ctx.index = 0; // Reset index for next frame
					rx_ctx.timeout_counter = 0; // Reset timeout counter
				}
				else
				{
					// Timeout: if N bytes bass by witout datas = error
					rx_ctx.timeout_counter++;
					if(rx_ctx.timeout_counter > 200) 
					{
						frame_mgr.frame_errors++; // Increment error count
						rx_ctx.state = SYNC_0; // Reset state to look for next frame header
						rx_ctx.index = 0; // Reset index for next frame
						rx_ctx.timeout_counter = 0; // Reset timeout counter
					}
				}
				break;
			default:
			    rx_ctx.state = SYNC_0; // Should never reach here, reset state just in case
				break;
		}
	}

}






