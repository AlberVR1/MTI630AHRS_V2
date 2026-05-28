/*
 * mti630ahrs.c
 *
 *  Created on: 27 may 2026
 *      Author: Control1
 */

#include "mti630ahrs.h"


// Definiciones de registros NVIC (suponiendo que no usas TivaWare)
#define NVIC_DIS1_R (*((volatile uint32_t *)0xE000E184))
#define NVIC_EN1_R  (*((volatile uint32_t *)0xE000E104))


void disablemtiinterrupt(void)
{
    NVIC_DIS1_R = (1 << 1); // Disable UART2 interrupt (IRQ 33)
}
void enablemtiinterrupt(void)
{
    NVIC_EN1_R = (1 << 1);
}
