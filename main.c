

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
static void getmeasurement(void);

/* Handles----------------------------------------------------------------------------------*/
PLL_Handle_t pllhandle;
UART_Handle_t uart2handle;

/* IMU Constants----------------------------------------------------------------------------------*/
const char gotomeasurement[] = {0xFA, 0xFF, 0x10, 0x00, 0xF1};
const char gotoconfig[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};

/* Private mti630 ahrs variables--------------------------------------------------------------------*/
MIT_data_t mtidata = {
	.head = 0,
	.tail = 0,
	.count = 0
};


int main(void)
{
    config();

	updateimu();

	while(true)
	{
	    getmeasurement();
	    PLL_API.delayMs(10);
	}
}


static void getmeasurement(void)
{
    if(mtidata.count < 90) return;

	uint16_t current = mtidata.tail;
	uint16_t next = (mtidata.tail + 1) & BITMASK;

	if(mtidata.buffer[current] == 0xFA && mtidata.buffer[next] == 0xFF)
	{
	    mtidata.bufferdummy[0] = mtidata.buffer[current];
	    mtidata.bufferdummy[1] = mtidata.buffer[next];
	    mtidata.taildummy = current;
		// Valid frame header found, process frame
		disablemtiinterrupt();
		mtidata.tail = (mtidata.tail + 90) & BITMASK; // Move tail to next position
		mtidata.count -= 90;
		enablemtiinterrupt();
	}
	else
	{
		disablemtiinterrupt();
		mtidata.tail = (mtidata.tail + 1) & BITMASK; // Move tail to next position
		mtidata.count--;
		enablemtiinterrupt();
	}
}

static void updateimu(void)
{
	UART_API.sendString(&uart2handle,gotoconfig,sizeof(gotoconfig));
	PLL_API.delayMs(1000);
	mtidata.count = 0;
	mtidata.head = 0;
	mtidata.tail = 0;
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

		// Store data in circular buffer
		uint16_t nextHead = (mtidata.head + 1) % (BITMASK);

		if(nextHead != mtidata.tail)
		{
			mtidata.buffer[mtidata.head] = data; // Store byte in buffer
			mtidata.head = nextHead; // Move head to next position
			mtidata.count++;
		}
		else
		{
			// Buffer is full, overwrite oldest data
			mtidata.buffer[mtidata.head] = data; // Store byte in buffer
			mtidata.head = nextHead; // Move head to next position
			mtidata.tail = (mtidata.tail + 1) & BITMASK; // Move tail to next position (overwrite oldest data)
		}
	}

}






