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
#define RX_BUFFER_SIZE 256//181 // Power of 2 to performance optimization in circular buffer
#define BITMASK (RX_BUFFER_SIZE - 1) // Mask for circular buffer indexing
#define MTI_FRAME_SIZE 90
#define float32_t float

typedef struct
{
    uint8_t buffer[RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    uint8_t bufferdummy[2];
    uint16_t taildummy;
}MIT_data_t;

void disablemtiinterrupt(void);
void enablemtiinterrupt(void);

#endif /* INC_MTI630AHRS_H_ */
