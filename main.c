

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
#include "tm4c123gh6pm.h"

/* Private prototypes-------------------------------------------------------------------------------*/
static void config(void);
static void updateimu(void);
static void getmeasurements(void);


static void EnviarTramaCuaterniones(UART_Handle_t *handle, float q0, float q1, float q2, float q3);
static void UART_SendFloat(UART_Handle_t *handle, float f, uint32_t precision);


/* Handles----------------------------------------------------------------------------------*/
PLL_Handle_t pllhandle;
UART_Handle_t uart2handle;
UART_Handle_t uart1handle;

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

mti630_variables_t mti630_vars = {
	.checksumvalid = 0
};

ieee754_float_t float_variable = {
	.bytee = {0x00, 0x00, 0x00, 0x00},
	.float_value = 0.0f
};

/**
 * @brief 
 *
 *
 */
int main(void)
{
    config();

	updateimu();

	while(true)
	{
	    getmeasurements();
	    PLL_API.delayMs(10);
		EnviarTramaCuaterniones(&uart1handle, mti630_vars.realquaternions[0], mti630_vars.realquaternions[1], mti630_vars.realquaternions[2], mti630_vars.realquaternions[3]);
	}
}

/**
 * @brief 
 *
 *
 */
static void getmeasurements(void)
{
	int i = 0;
	int j = 0;
	mti630_vars.sumchecksum = 0;
	for(i = 1; i < MTI_FRAME_SIZE; i++)
	{
		mti630_vars.sumchecksum += frame_mgr.p_read->frame[i]; // Simple checksum validation, you should implement the actual checksum algorithm based on the sensor's datasheet
	}
	mti630_vars.checksumvalid = mti630_vars.sumchecksum;

	if(!(mti630_vars.checksumvalid == 0 && frame_mgr.p_read->frame[3] == 0x55)) //Checksum valid and correct frame type lenght(0x55 = 85 bytes, 84 payload bytes and 1 checksum byte)
	{
		return 0;
	}	

	// Sample Time Fine
	float_variable.bytee[0] = frame_mgr.p_read->frame[10];
	float_variable.bytee[1] = frame_mgr.p_read->frame[9];
	float_variable.bytee[2] = frame_mgr.p_read->frame[8];
	float_variable.bytee[3] = frame_mgr.p_read->frame[7];
	mti630_vars.sampletimefine2 = float_variable.uint32_value;
	mti630_vars.sampletimefine = (frame_mgr.p_read->frame[7] << 24) | (frame_mgr.p_read->frame[8] << 16) | (frame_mgr.p_read->frame[9] << 8) | frame_mgr.p_read->frame[10];
	mti630_vars.realsampletimefine = (float32_t)mti630_vars.sampletimefine/10000.0f;

	// Quaternions
	for(i=0;i<4;i++)	// Loop through the raw data of the 4 quaternions (w, x, y, z)
	{
		mti630_vars.quaternions[0][i] = frame_mgr.p_read->frame[14 + i];
		mti630_vars.quaternions[1][i] = frame_mgr.p_read->frame[18 + i];
		mti630_vars.quaternions[2][i] = frame_mgr.p_read->frame[22 + i];
		mti630_vars.quaternions[3][i] = frame_mgr.p_read->frame[26 + i];

		for(j=0;j<4;j++)
		{
			float_variable.bytee[j] = frame_mgr.p_read->frame[17 + i*4 - j]; // Copy raw bytes to union for conversion
		}
		mti630_vars.realquaternions[i] = float_variable.float_value; // Store converted quaternion value
	
	}	// float32 in unit quaternions (w,x,y,z)

	// Acceleration
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 acceleration components (x, y, z)
	{
		mti630_vars.acceleration[0][i] = frame_mgr.p_read->frame[33 + i];
		mti630_vars.acceleration[1][i] = frame_mgr.p_read->frame[37 + i];
		mti630_vars.acceleration[2][i] = frame_mgr.p_read->frame[41 + i];

		if(i<3)
		{
			for(j=0;j<4;j++)
			{
				float_variable.bytee[j] = frame_mgr.p_read->frame[36 + i*4 - j]; // Copy raw bytes to union for conversion
			}
			mti630_vars.realacceleration[i] = float_variable.float_value; // Store converted acceleration value
		}
	}	// float32 in m/s^2 (X,Y,Z)

	// Rate of turn
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 rate of turn components (x, y, z)
	{
		mti630_vars.rateofturn[0][i] = frame_mgr.p_read->frame[48 + i];
		mti630_vars.rateofturn[1][i] = frame_mgr.p_read->frame[52 + i];
		mti630_vars.rateofturn[2][i] = frame_mgr.p_read->frame[56 + i];

		if(i<3)
		{
			for(j=0;j<4;j++)
			{
				float_variable.bytee[j] = frame_mgr.p_read->frame[51 + i*4 - j]; // Copy raw bytes to union for conversion
			}
			mti630_vars.realrateofturn[i] = float_variable.float_value; // Store converted rate of turn value
		}
	} // float32 in rad/s (X,Y,Z)

	// Magnetic field
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 magnetic field components (x, y, z)
	{
		mti630_vars.magneticfield[0][i] = frame_mgr.p_read->frame[63 + i];
		mti630_vars.magneticfield[1][i] = frame_mgr.p_read->frame[67 + i];
		mti630_vars.magneticfield[2][i] = frame_mgr.p_read->frame[71 + i];

		if(i<3)
		{
			for(j=0;j<4;j++)
			{
				float_variable.bytee[j] = frame_mgr.p_read->frame[66 + i*4 - j]; // Copy raw bytes to union for conversion
			}
			mti630_vars.realmagneticfield[i] = float_variable.float_value; // Store converted magnetic field value
		}
	} // float32 in microtesla (X,Y,Z)


	// Temperature
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 temperature components (x, y, z)
	{
		mti630_vars.temperature[i] = frame_mgr.p_read->frame[78 + i];

		float_variable.bytee[i] = frame_mgr.p_read->frame[81 - i]; // Copy raw bytes to union for conversion
	}
	mti630_vars.realtemperature = float_variable.float_value; // Store converted temperature value


	// Status word
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 status word components (x, y, z)
	{
		mti630_vars.statusword[i] = frame_mgr.p_read->frame[85 + i];

		float_variable.bytee[i] = frame_mgr.p_read->frame[88 - i]; // Copy raw bytes to union for conversion
	}
	mti630_vars.realstatusword = float_variable.float_value; // Store converted status word value
}

/**
 * @brief function for set MTi-630 AHRS sensor to stop frames and get measurements.
 *
 *
 */
static void updateimu(void)
{
	UART_API.sendString(&uart2handle,gotoconfig,sizeof(gotoconfig));
	PLL_API.delayMs(1000);
	UART_API.sendString(&uart2handle,gotomeasurement,sizeof(gotomeasurement));
	PLL_API.delayMs(1000);

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

/**
 * @brief isr for UART2, which handles the reception of data from the MTi-630 AHRS sensor.
 *
 * @note This ISR reads the received data byte by byte and processes
 * it according to the current state of the reception using double buffering
 * to store the received frames. It also implements a timeout mechanism to
 * handle cases where the expected number of bytes is not received within a
 * certain time frame, which helps in error handling and synchronization of the data stream.
 *
 */
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
					rx_ctx.state = RX_PAYLOAD; // Move to payload receiving state
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
			case RX_PAYLOAD:
				frame_mgr.p_write->frame[rx_ctx.index++] = data; // Store received byte
				//rx_ctx.index++;
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







// FunciÃ³n auxiliar para convertir un float a texto y enviarlo
static void UART_SendFloat(UART_Handle_t *handle, float f, uint32_t precision) {
    // 1. Manejo del signo
    if (f < 0) {
        UART_API.sendByte(handle, '-');
        f = -f;
    }

    // 2. Parte entera
    uint32_t parte_entera = (uint32_t)f;
    char buffer_int[11]; // Suficiente para un uint32_t
    int i = 0;
    
    // Algoritmo simple para convertir entero a caracteres
    do {
        buffer_int[i++] = (parte_entera % 10) + '0';
        parte_entera /= 10;
    } while (parte_entera > 0);
    
    // Los dÃ­gitos estÃ¡n al revÃ©s, los enviamos correctamente
    while (i > 0) {
        UART_API.sendByte(handle, buffer_int[--i]);
    }

    // 3. Punto decimal
    UART_API.sendByte(handle, '.');

    // 4. Parte decimal
    float parte_decimal = f - (uint32_t)f;
	uint32_t p = 0;
    for (p=0; p < precision; p++) {
        parte_decimal *= 10;
        uint32_t digito = (uint32_t)parte_decimal;
        UART_API.sendByte(handle, digito + '0');
        parte_decimal -= digito;
    }
}


// FunciÃ³n principal para enviar tu trama de cuaterniones
static void EnviarTramaCuaterniones(UART_Handle_t *handle, float q0, float q1, float q2, float q3) {
    UART_SendFloat(handle, q0, 6); // Enviamos q0 con 6 decimales
    UART_API.sendByte(handle, ',');
    
    UART_SendFloat(handle, q1, 6);
    UART_API.sendByte(handle, ',');
    
    UART_SendFloat(handle, q2, 6);
    UART_API.sendByte(handle, ',');
    
    UART_SendFloat(handle, q3, 6);
    
    // 5. Fin de trama
    UART_API.sendByte(handle, '\n');
}

