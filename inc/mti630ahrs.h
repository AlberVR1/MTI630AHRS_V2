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


// Sync States
typedef enum
{
    SYNC_0, //Waiting 0XFA
    SYNC_1, //Waiting 0xFF
    RX_PALOAD //Receiving datas
} rx_state_t;


typedef struct
{
    rx_state_t state;
    uint16_t index;
    uint32_t timeout_counter;
} UART_rx_context_t;




void disablemtiinterrupt(void);
void enablemtiinterrupt(void);

#endif /* INC_MTI630AHRS_H_ */
