/*
 * algorithms.c
 *
 *  Created on: 4 jun 2026
 *      Author: Control1
 */
#include "algorithms.h"





// Funcion auxiliar para convertir un float a texto y enviarlo
void UART_SendFloat(UART_Handle_t *handle, float f, uint32_t precision) {
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


// Funcion principal para enviar tu trama de cuaterniones
void EnviarTramaCuaterniones(UART_Handle_t *handle, float q0, float q1, float q2, float q3) {
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

