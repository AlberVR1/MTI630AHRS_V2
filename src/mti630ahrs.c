/**
 * @file mti630ahrs.c
 * @author Alberto Vazquez
 * 
 * @brief source file for MTi-630 AHRS sensor, which provides functions
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
 * * Alt+217 -> â”˜    Alt+218 -> â”Œ    Alt+191 -> â”�    Alt+192-> â””    Alt+196 -> â”€    Alt+124 -> |    Alt+195-> â”œ    Alt+180 -> â”¤
 *
 * â”Œâ”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”�
 * |        APLICATION (main.c)      |  <- Your api code
 * â”œâ”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¤
 * |      PUBLIC API (MTI630_API)    |  <- Simple and clean interface
 * â”œâ”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¤
 * |    IMPLEMENTATION (static       |
 * |   functions in mti630ahrs.c)    |  <- hidden inner logic
 * â”œâ”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”¤
 * |        HARDWARE (TM4C123)       |  <- MCU's registers
 * â””â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”˜
 * 
 * @version 1.00
 * @date 2026-05-29
 */

#include "mti630ahrs.h"
#include "pll.h"
#include "tm4c123gh6pm.h"



void ahrs_callback(uint8_t data);

// 
static UART_Handle_t uart2handle;


/* IMU Constants----------------------------------------------------------------------------------*/
const char gotomeasurement[] = {0xFA, 0xFF, 0x10, 0x00, 0xF1};
const char gotoconfig[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
const char reset[] = {0xFA, 0xFF, 0x40, 0x00, 0xC1};

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



ieee754_float_t float_variable = {
	.bytee = {0x00, 0x00, 0x00, 0x00},
	.float_value = 0.0f
};


static uint32_t pll_get_frequency(void);
static UART_Status_t Configure_UART_2(const mti630_config_t *config);
static void updateimu(void);

mti630_status_t MTI630_init(mti630_config_t *config)
{
    if(config == NULL)
    {
        return MTI630_STATIS_INVALID_PARAM; // Invalid configuration pointer
    }

    UART_Status_t status_uart2 = Configure_UART_2(config);
    if(status_uart2 != UART_STATUS_SUCCESS)
    {
        return MTI630_STATIS_INVALID_PARAM; // UART initialization error
    }
    updateimu(); // Send initial commands to set sensor to measurement mode
    return MTI630_STATUS_SUCCESS;
}

mti630_status_t mti630_gotoconfig(void)
{
	UART_API.sendString(&uart2handle,gotoconfig,sizeof(gotoconfig));
	PLL_API.delayMs(1000);
    return MTI630_STATUS_SUCCESS;
}
mti630_status_t mti630_gotomeasurement(void)
{
	UART_API.sendString(&uart2handle,gotomeasurement,sizeof(gotomeasurement));
	PLL_API.delayMs(1000);
    return MTI630_STATUS_SUCCESS;
}

/**
 * @brief function for parsing the received frame from the MTi-630 AHRS sensor
 * and extracting the measurements.
 *
 *
 */
static mti630_status_t getmeasurements(mti630_variables_t *vars)
{
	int i = 0;
	int j = 0;
	vars->sumchecksum = 0;
	for(i = 1; i < MTI_FRAME_SIZE; i++)
	{
	    vars->sumchecksum += frame_mgr.p_read->frame[i]; // Simple checksum validation, you should implement the actual checksum algorithm based on the sensor's datasheet
	}
	vars->checksumvalid = vars->sumchecksum;

	if(!(vars->checksumvalid == 0 && frame_mgr.p_read->frame[3] == 0x55)) //Checksum valid and correct frame type lenght(0x55 = 85 bytes, 84 payload bytes and 1 checksum byte)
	{
		return MTI630_STATUS_FRAME_ERROR;
	}	

	// Sample Time Fine
	float_variable.bytee[0] = frame_mgr.p_read->frame[10];
	float_variable.bytee[1] = frame_mgr.p_read->frame[9];
	float_variable.bytee[2] = frame_mgr.p_read->frame[8];
	float_variable.bytee[3] = frame_mgr.p_read->frame[7];
	vars->sampletimefine2 = float_variable.uint32_value;
	vars->sampletimefine = (frame_mgr.p_read->frame[7] << 24) | (frame_mgr.p_read->frame[8] << 16) | (frame_mgr.p_read->frame[9] << 8) | frame_mgr.p_read->frame[10];
	vars->realsampletimefine = (float32_t)vars->sampletimefine/10000.0f;

	// Quaternions
	for(i=0;i<4;i++)	// Loop through the raw data of the 4 quaternions (w, x, y, z)
	{
	    vars->quaternions[0][i] = frame_mgr.p_read->frame[14 + i];
	    vars->quaternions[1][i] = frame_mgr.p_read->frame[18 + i];
	    vars->quaternions[2][i] = frame_mgr.p_read->frame[22 + i];
	    vars->quaternions[3][i] = frame_mgr.p_read->frame[26 + i];

		for(j=0;j<4;j++)
		{
			float_variable.bytee[j] = frame_mgr.p_read->frame[17 + i*4 - j]; // Copy raw bytes to union for conversion
		}
		vars->realquaternions[i] = float_variable.float_value; // Store converted quaternion value
	
	}	// float32 in unit quaternions (w,x,y,z)

	// Acceleration
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 acceleration components (x, y, z)
	{
		vars->acceleration[0][i] = frame_mgr.p_read->frame[33 + i];
		vars->acceleration[1][i] = frame_mgr.p_read->frame[37 + i];
		vars->acceleration[2][i] = frame_mgr.p_read->frame[41 + i];

		if(i<3)
		{
			for(j=0;j<4;j++)
			{
				float_variable.bytee[j] = frame_mgr.p_read->frame[36 + i*4 - j]; // Copy raw bytes to union for conversion
			}
			vars->realacceleration[i] = float_variable.float_value; // Store converted acceleration value
		}
	}	// float32 in m/s^2 (X,Y,Z)

	// Rate of turn
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 rate of turn components (x, y, z)
	{
		vars->rateofturn[0][i] = frame_mgr.p_read->frame[48 + i];
		vars->rateofturn[1][i] = frame_mgr.p_read->frame[52 + i];
		vars->rateofturn[2][i] = frame_mgr.p_read->frame[56 + i];

		if(i<3)
		{
			for(j=0;j<4;j++)
			{
				float_variable.bytee[j] = frame_mgr.p_read->frame[51 + i*4 - j]; // Copy raw bytes to union for conversion
			}
			vars->realrateofturn[i] = float_variable.float_value; // Store converted rate of turn value
		}
	} // float32 in rad/s (X,Y,Z)

	// Magnetic field
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 magnetic field components (x, y, z)
	{
		vars->magneticfield[0][i] = frame_mgr.p_read->frame[63 + i];
		vars->magneticfield[1][i] = frame_mgr.p_read->frame[67 + i];
		vars->magneticfield[2][i] = frame_mgr.p_read->frame[71 + i];

		if(i<3)
		{
			for(j=0;j<4;j++)
			{
				float_variable.bytee[j] = frame_mgr.p_read->frame[66 + i*4 - j]; // Copy raw bytes to union for conversion
			}
			vars->realmagneticfield[i] = float_variable.float_value; // Store converted magnetic field value
		}
	} // float32 in microtesla (X,Y,Z)


	// Temperature
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 temperature components (x, y, z)
	{
		vars->temperature[i] = frame_mgr.p_read->frame[78 + i];

		float_variable.bytee[i] = frame_mgr.p_read->frame[81 - i]; // Copy raw bytes to union for conversion
	}
	vars->realtemperature = float_variable.float_value; // Store converted temperature value


	// Status word
	for(i=0;i<4;i++)	// Loop through the raw data of the 3 status word components (x, y, z)
	{
		vars->statusword[i] = frame_mgr.p_read->frame[85 + i];

		float_variable.bytee[i] = frame_mgr.p_read->frame[88 - i]; // Copy raw bytes to union for conversion
	}
	vars->realstatusword = float_variable.uint32_value; // Store converted status word value

    return MTI630_STATUS_SUCCESS;
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

static UART_Status_t Configure_UART_2(const mti630_config_t *config)
{
    UART_Config_t uart2config = {
        .module = UART_MODULE_2,
        .baudRate = config->mcu_uart_baudrate,
        .clockFreqMHz = pll_get_frequency(),
        .enableTx = true,
        .enableRx = true,
        .enableFIFO = true,
        .fifoLevel = UART_FIFO_LEVEL_1_2
    };
    UART_Status_t uart2status = UART_API.init(&uart2handle, &uart2config);

    UART_API.enableInterrupt(&uart2handle, UART_FIFO_LEVEL_1_2, ahrs_callback);
    return uart2status;
}



static uint32_t pll_get_frequency(void)
{
    uint32_t frequency = 0;
    frequency = PLL_API.getPLLFrequency();
    return frequency;
}

const mti630_Interface_t MTI630_API = {
    .init = MTI630_init,
    .go_to_measurement = mti630_gotomeasurement,
    .go_to_config = mti630_gotoconfig,
    .get_measurements = getmeasurements
};



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
void ahrs_callback(uint8_t data)
{

    switch(rx_ctx.state)
        {
            case SYNC_0:
                if(data == 0xFA)
                {
                    frame_mgr.p_write->frame[0] = data; // Store first byte of header
                    rx_ctx.index = 1; // Move to next index for second byte
                    rx_ctx.state = SYNC_1;  // Move to next state
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
                        rx_ctx.state = SYNC_1;  // Move to next state
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
