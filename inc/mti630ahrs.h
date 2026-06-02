/*
 * mti630ahrs.h
 *
 *  Created on: 27 may 2026
 *      Author: Control1
 */

#ifndef INC_MTI630AHRS_H_
#define INC_MTI630AHRS_H_

/* Includes-----------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Private defines----------------------------------------------------------------------------------*/
#ifndef NULL
#define NULL          0
#endif

#define MTI_FRAME_SIZE 90
#define float32_t float

// 
typedef union
{
    uint8_t bytee[4];
    float32_t float_value;
    uint32_t uint32_value;
} ieee754_float_t;

// Sync States
typedef enum
{
    SYNC_0, //Waiting 0XFA
    SYNC_1, //Waiting 0xFF
    RX_PAYLOAD //Receiving datas
} rx_state_t;


typedef struct
{
    uint8_t frame[MTI_FRAME_SIZE];
    volatile uint8_t complete;
} frame_buffer_t;


typedef struct
{
    frame_buffer_t buffer_a;
    frame_buffer_t buffer_b;
    frame_buffer_t *p_write;    //ISR write here
    frame_buffer_t *p_read;     //Main loop read here
    volatile uint8_t frame_ready; //Flag set by ISR when a complete frame is received
    volatile uint32_t frame_errors; //Count of frames with errors
} UART_frame_manager_t;


typedef struct
{
    rx_state_t state;
    uint16_t index;
    uint32_t timeout_counter;
} UART_rx_context_t;

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


void disablemtiinterrupt(void);
void enablemtiinterrupt(void);

#endif /* INC_MTI630AHRS_H_ */
