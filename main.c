/**
 * @file main.c
 * @author Alberto Vazquez
 * 
 * @brief source file main, C Interface for MTi-630 AHRS sensor, which provides functions
 * to configure the sensor, send commands to get measurements like:
 * - Sample Time Fine
 * - Orientacion (Quaternions)
 * - Angular velocity (Rate of turn)
 * - Acceleration
 * - Magnetic field
 * - Temperature
 * - State (Status Word)
 *
 * Bytes range:    		Hex value:			Function:
 * 0:cc                 0xFA				Preamble: message start byte
 * 1:                   0xFF        		BusID: direction of bus (master)
 * 2:                   0x36              	Message ID: message indentifier (0x36 = 54) MTData2
 * 3:                   0x55              	Frame type (85 bytes)
 * 4-88:                Data payload		Data of sensors
 * 89:                  Checksum			integrity verification byte
 *
 * @note This source file contains the main system initialization for Main Clock, UART modules for communication with
 * the MTi-630 AHRS sensor and the Master MCU.
 *
 *
 * * Alt+217 -> ┘    Alt+218 -> ┌    Alt+191 -> ┐    Alt+192-> └    Alt+196 -> ─    Alt+124 -> |    Alt+195-> ├    Alt+180 -> ┤
 *
 * ┌─────────────────────────────────┐
 * |        APLICATION (main.c)      |  <- Your api code
 * ├─────────────────────────────────┤
 * |      PUBLIC API (MTI630_API)    |  <- Simple and clean interface
 * ├─────────────────────────────────┤
 * |    IMPLEMENTATION (static       |
 * |   functions in mti630ahrs.c)    |  <- hidden inner logic
 * ├─────────────────────────────────┤
 * |        HARDWARE (TM4C123)       |  <- MCU's registers
 * └─────────────────────────────────┘
 * 
 * @version 1.00
 * @date 2026-05-29
 */
/* Includes-----------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pll.h"
#include "uart.h"
#include "mti630ahrs.h"
#include "algorithms.h"
#include "SysTick.h"

/* Private prototypes-------------------------------------------------------------------------------*/
static void config(void);

/* Handles----------------------------------------------------------------------------------*/
UART_Handle_t uart1handle;
UART_Handle_t uart2handle;
mti630_variables_t mti630_vars = {
    .checksumvalid = 0
};


/**
 * @brief 
 *
 *
 */
int main(void)
{
    config();


	while(true)
	{
		MTI630_API.get_measurements(&mti630_vars);
        SYSTICK_API.delay_ms(200);

        EnviarTramaCuaterniones(&uart1handle, mti630_vars.realquaternions[0], mti630_vars.realquaternions[1], mti630_vars.realquaternions[2], mti630_vars.realquaternions[3]);
	}
}


/**
 * @brief function for configuring the MTi-630 AHRS sensor.
 *
 * @note This function initializes the PLL to set the main clock to 80 MHz and configures UART2
 * for communication with the MTi-630 AHRS sensor.
 * It also enables UART2 interrupts for receiving data from the sensor.
 */
void config(void)
{
    PLL_Status_t statuspll = PLL_API.init(MHz80);

    // The PLL module initialized correctly
    if(statuspll != PLL_STATUS_SUCCESS) {

    }

    SYSTICK_Status_t statussystick = SYSTICK_API.init(); // The SysTick module configured correctly

   if(statussystick != SYSTICK_STATUS_SUCCESS) {

   }

   // MTI-630 AHRS Sensor Configuration
   mti630_config_t mti630config = {
       .mcu_uart_baudrate = UART_BAUD_115200
   };
   mti630_status_t mti630status = MTI630_API.init(&mti630config);
   if(mti630status != MTI630_STATUS_SUCCESS)
   {
       // Handle initialization error
   }

	/////////////////////UART1 Config
	UART_Config_t uart1config = {
		.module = UART_MODULE_1,
		.baudRate = UART_BAUD_115200,
		.clockFreqMHz = 80,
		.enableTx = true,
		.enableRx = true,
		.enableFIFO = false,
		.fifoLevel = UART_FIFO_LEVEL_1_8
	};
	UART_Status_t uart1status = UART_API.init(&uart1handle,
											&uart1config);
	switch (uart1status)
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
}









