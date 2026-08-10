/**
 * @file pll.h
 * @author Alberto Vazquez
 * @brief source file for MCU Clock configuration
 *
 * This source file contains an API whitch includes and implementation functions
 * for managing TM4C123 clock,and implementations for delays in ms.
 *
 * @version 0.11
 * @date 2025-09-15
 */

#ifndef INCLUDE_PLL_H_
#define INCLUDE_PLL_H_

/**
 * @addtogroup MCU clock and delay
 * @{
 */

/* Includes ----------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Definitions -------------------------------------------------------------------------------------*/
#define float_t float

/* Public Enums ------------------------------------------------------------------------------------*/
typedef enum {
    MHz80 = 80,
    MHz50 = 50,
    MHz40 = 40,
    MHz25 = 25,
    MHz20 = 20,
    MHz16 = 16,
    MHz10 = 10,
    MHz8  = 8,
    MHz5  = 5,
    MHz4  = 4,
}PLL_Frequency_t;

typedef enum {
    PLL_STATUS_SUCCESS = 0,
    PLL_STATUS_ERROR,
    PLL_STATUS_INVALID_PARAM,
    PLL_STATUS_NOT_INITIALIZED
}PLL_Status_t;


/* Public Function Pointers Structure --------------------------------------------------------------*/
typedef struct {
    PLL_Status_t (*init)(PLL_Frequency_t frequency);
    uint32_t (*getPLLFrequency)(void);
    void (*delayMs)(uint32_t ms);
}PLL_Interface_t;

/* Public API Instance -----------------------------------------------------------------------------*/
extern const PLL_Interface_t PLL_API;



#endif /* INCLUDE_PLL_H_ */
