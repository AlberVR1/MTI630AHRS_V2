/**
 * @file SysTick.h
 * @author Alberto Vazquez
 *
 * @brief Header file for Systick module demonstration using BMP by an interface user.
 *
 * This header file contains the declarations for the SysTick module API and related structures.
 * Alt+217 -> ┘    Alt+218 -> ┌    Alt+191 -> ┐    Alt+192-> └    Alt+196 -> ─    Alt+124 -> |    Alt+195-> ├    Alt+180 -> ┤
 *
 * ┌─────────────────────────────────┐
 * |        APLICATION (main.c)      |  <- Your api code
 * ├─────────────────────────────────┤
 * |       PUBLIC API (SYSTICK_API)  |  <- Simple and clean interface
 * ├─────────────────────────────────┤
 * |    IMPLEMENTATION (static       |  <- hidden inner logic
 * |    functions in systick.c)      |
 * ├─────────────────────────────────┤
 * |        HARDWARE (TM4C123)       |  <- MCU's registers
 * └─────────────────────────────────┤
 *
 * main.c calls:
 * SYSTICK_API.Init()
 *
 * SYSTICK_Init() (funciÃ³n static)
 *
 * ┌─ SYSTICK_InitHardware()
 * |
 * |  ┌ SYSTICK_millis()
 * |  |
 * |  |  Configure SysTick timer
 * |  |
 * |  └ SYSTICK_Init()
 * |
 * |     Configure SysTick registers
 * |
 * └─ SYSTICK_millis()
 *
 *    Mark the SysTick timer as initialized
 *
 * @version 0.11
 * @date 2025-07-02
 */

/**
 * @addtogroup main
 * @{
 */

#ifndef INCLUDE_SYSTICK_H_
#define INCLUDE_SYSTICK_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief SysTick Status Codes enumeration
 */
typedef enum {
    SYSTICK_STATUS_SUCCESS = 0,
    SYSTICK_STATUS_HANDLER_NULL,
    SYSTICK_STATUS_ERROR
}SYSTICK_Status_t;

typedef enum {
    SYSTICK_EXIST,
    SYSTICK_NOT_EXIST
}SYSTICK_Exist_t;

/* Public Function Pointers Structure --------------------------------------------------------------*/
/**
 * @brief SysTick Interface structure containing all API functions
 */
typedef struct {
    SYSTICK_Status_t (*init)(void);
    SYSTICK_Status_t (*Stop_Count)(void);
    SYSTICK_Status_t (*Start_Count)(void);
    uint32_t (*milis)(void);
    void(*delay_ms)(uint32_t ms);
}SYSTICK_Interface_t;

/* Public API Instance -----------------------------------------------------------------------------*/
/**
 * @brief SYSTICK Public API Instance
 */
extern const SYSTICK_Interface_t SYSTICK_API;

#endif /* INCLUDE_SYSTICK_H_ */
