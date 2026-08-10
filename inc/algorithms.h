/*
 * algorithms.h
 *
 *  Created on: 4 jun 2026
 *      Author: Control1
 */

#ifndef INC_ALGORITHMS_H_
#define INC_ALGORITHMS_H_

#include "stdint.h"
#include "uart.h"

void UART_SendFloat(UART_Handle_t *handle, float f, uint32_t precision);
void EnviarTramaCuaterniones(UART_Handle_t *handle, float q0, float q1, float q2, float q3);

#endif /* INC_ALGORITHMS_H_ */
