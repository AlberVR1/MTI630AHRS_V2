/**
 * @file mti630ahrs.h
 * @author Alberto Vazquez
 * 
 * @brief header file for MTi-630 AHRS sensor, which provides the interface for configuring the sensor and retrieving measurements.
 *
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

#ifndef INC_MTI630AHRS_H_
#define INC_MTI630AHRS_H_

/* Includes-----------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "UART.h"

/* Private defines----------------------------------------------------------------------------------*/
#ifndef NULL
#define NULL          0
#endif

#define MTI_FRAME_SIZE 90
#define float32_t float

/* public unions------------------------------------------------------------------------------------*/
/** 
 * @brief union for converting 4 bytes to float, 4 bytes to uint32_t and vice versa
 *
**/
typedef union
{
    uint8_t bytee[4];
    float32_t float_value;
    uint32_t uint32_value;
} ieee754_float_t;

extern ieee754_float_t float_variable;

/* public enums-------------------------------------------------------------------------------------*/
/** 
 * @brief enumeration for the state machine of the UART reception,
 * used in the interrupt handler to parse the incoming data from
 * the MTi-630 AHRS sensor
 *
**/
typedef enum
{
    SYNC_0, //Waiting 0XFA
    SYNC_1, //Waiting 0xFF
    RX_PAYLOAD //Receiving datas
} rx_state_t;

/** 
 * @brief enumeration for the status of the MTi-630 AHRS sensor,
 * used for error handling and status reporting
 *
**/
typedef enum
{
    MTI630_STATUS_SUCCESS = 0,
    MTI630_STATUS_OK = 1,
    MTI630_STATUS_ERROR = 2,
    MTI630_STATIS_INVALID_PARAM = 3,
    MTI630_STATUS_FRAME_ERROR = 4
} mti630_status_t;

/* public structures--------------------------------------------------------------------------------*/
/** 
 * @brief structure for managing the reception of UART frames
 * from the MTi-630 AHRS sensor, using double buffering.
 *
**/
typedef struct
{
    uint8_t frame[MTI_FRAME_SIZE];
    volatile uint8_t complete;
} frame_buffer_t;

/** 
 * @brief structure for managing the reception of UART frames from
 * the MTi-630 AHRS sensor, using double buffering and state machine for parsing
 * the incoming data. It also includes counters for tracking the number of complete frames and errors.
 *
**/
typedef struct
{
    frame_buffer_t buffer_a;
    frame_buffer_t buffer_b;
    frame_buffer_t *p_write;    //ISR write here
    frame_buffer_t *p_read;     //Main loop read here
    volatile uint8_t frame_ready; //Flag set by ISR when a complete frame is received
    volatile uint32_t frame_errors; //Count of frames with errors
} UART_frame_manager_t;

//extern UART_frame_manager_t frame_mgr;

/** 
 * @brief structure for managing the state of the UART reception,
 * including the current state of the state machine,
 * the index for storing incoming bytes, and a timeout counter.
 *
**/
typedef struct
{
    rx_state_t state;
    uint16_t index;
    uint32_t timeout_counter;
} UART_rx_context_t;

//extern UART_rx_context_t rx_ctx;

/** 
 * @brief structure for storing the raw and converted measurements from the MTi-630 AHRS sensor, including
 * Sample Time Fine, Quaternions, Acceleration, Rate of Turn, Magnetic Field, Temperature and Status Word.
 *
**/
typedef struct
{
    uint8_t sumchecksum;
    uint8_t checksumvalid;
    uint32_t sampletimefine2; 
    uint32_t sampletimefine;        // High resolution sample time in 0.1ms units (e.g., 10000 = 1 second)
    float32_t realsampletimefine;   // Sample time in seconds (e.g., 1.0 for 1 second)
    uint8_t quaternions[4][4];         // Raw quaternion bytes from the sensor
    float32_t realquaternions[4];   // Converted quaternion values
    uint8_t acceleration[3][4];      // Raw acceleration bytes from the sensor
    float32_t realacceleration[3]; // Converted acceleration values
    uint8_t rateofturn[3][4];       // Raw rate of turn bytes from the sensor
    float32_t realrateofturn[3];   // Converted rate of turn values
    uint8_t magneticfield[3][4];      // Raw magnetic field bytes from the sensor
    float32_t realmagneticfield[3]; // Converted magnetic field values
    uint8_t temperature[4];         // Raw temperature bytes from the sensor
    float32_t realtemperature;      // Converted temperature value
    uint8_t statusword[4];         // Raw status word bytes from the sensor
    uint32_t realstatusword;       // Converted status word value
} mti630_variables_t;



/**
 * @brief structure for configuring the MCU and
 * UART modules for the MTi-630 AHRS sensor.
 *
**/
typedef struct
{
    UART_BaudRate_t mcu_uart_baudrate;
}mti630_config_t;

/* Public Function Pointers Structure---------------------------------------------------------------*/
/** 
 * @brief structure for defining the interface functions for initializing the MTi-630 AHRS sensor,
 * switching between measurement and configuration modes, and handling the UART communication.
 *
**/
typedef struct
{
    mti630_status_t (*init)(mti630_config_t *config);
    mti630_status_t (*go_to_measurement)(void);
    mti630_status_t (*go_to_config)(void);
    mti630_status_t (*get_measurements)(mti630_variables_t *vars);
}mti630_Interface_t;

/* Public API Interface-----------------------------------------------------------------------------*/
/** 
 * @brief structure for defining the public API interface for the MTi-630 AHRS sensor, which includes
 * functions for initializing the sensor, switching between measurement and configuration modes, and handling the UART communication.
 *
**/
extern const mti630_Interface_t MTI630_API;

#endif /* INC_MTI630AHRS_H_ */
