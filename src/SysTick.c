/**
 * @file SysTick.c
 * @author Alberto Vazquez
 *
 * @brief Source file for Systick module demonstration using BMP by an interface user.
 *
 * This source file contains the implementation for the SysTick module API and related functions.
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
/* Includes ---------------------------------------------------------------------------------------*/
#include "SysTick.h"
#include "pll.h"

/* Private Defines --------------------------------------------------------------------------------*/
#ifndef NULL
#define NULL 0
#endif

// Register databases for SysTick
#define NVIC_ST_CTRL_R      (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile uint32_t *)0xE000E018))
#define NVIC_SYS_PRI3_R     (*((volatile uint32_t *)0xE000ED20))

// SysTicks control bits
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source (1=system clock)
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter enable

/* Private Variables ------------------------------------------------------------------------------*/
static SYSTICK_Exist_t systick_exist = SYSTICK_NOT_EXIST;

/* Global Variables ------------------------------------------------------------------------------*/
volatile uint32_t msTicks = 0; // Counter for 1ms SysTicks

/*
 * @brief Configure SysTick
 *
 * This function configures SysTick to generate
 * an interrupt every 1ms
 *
 * @param handle Pointer to a SYSTICK_Handle_t structure that contains the callback function
 *
 * @return SYSTICK_Status_t Status of the initialization process
 *
 */
static SYSTICK_Status_t Init(void)
{
    uint32_t freq_mhz = PLL_API.getPLLFrequency();    //Get PLL frequency in MHz
    NVIC_ST_RELOAD_R = (freq_mhz * 1000)- 1; // Load frequency in Khz - 1 to generate an interrupt
    NVIC_ST_CURRENT_R = 0;              // Limpiar contador actual
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC;  // Usar reloj del sistema

    systick_exist = SYSTICK_EXIST; // Set flag to indicate that SysTick is configured

    return SYSTICK_STATUS_SUCCESS;
}

/*
 * @brief Stop SysTick counting and disable its interrupt
 *
 * This function stops the SysTick counter and disables its interrupt.
 * It also validates that the handle and callback are set before performing these actions.
 *
 */
static SYSTICK_Status_t Stop_Count(void)
{
    if(systick_exist == SYSTICK_NOT_EXIST) {
        return SYSTICK_STATUS_ERROR; // Validate that SysTick is configured
    }

    NVIC_ST_CTRL_R &= ~(NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE); // Deshabilitar interrupciÃ³n y contador
    NVIC_ST_CURRENT_R = 0;              // Limpiar contador actual

    msTicks = 0; // Reset SysTick counter
    return SYSTICK_STATUS_SUCCESS;
}

/*
 * @brief Start SysTick counting and enable its interrupt
 *
 * This function starts the SysTick counter and enables its interrupt.
 *
 */
static SYSTICK_Status_t Start_Count(void)
{
    if(systick_exist == SYSTICK_NOT_EXIST) {
        return SYSTICK_STATUS_ERROR; // Validate that SysTick is configured
    }

    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;    // Habilitar interrupciÃ³n y contador
    return SYSTICK_STATUS_SUCCESS;
}

/*
 * @brief Get the number of milliseconds since SysTick started
 *
 * This function returns the number of milliseconds that have elapsed since the SysTick counter was started.
 *
 * @return uint32_t Number of milliseconds since SysTick started
 *
 */
static uint32_t Get_Milis(void)
{
    return msTicks;
}

/**
 * @brief Set a delay
 *
 * This function create a loop in ms
 *
 * @param
 *
 */
void Delay_ms(uint32_t ms)
{

    Start_Count();  //Start Count

    while(msTicks < ms);

    Stop_Count();

}

/* Public API Instance ----------------------------------------------------------------------------*/
/*
 * @brief Public SYSTICK API Instance
 *
 */
const SYSTICK_Interface_t SYSTICK_API = {
    .init = Init,
    .Stop_Count = Stop_Count,
    .Start_Count = Start_Count,
    .milis = Get_Milis,
    .delay_ms = Delay_ms
};

/* Interrupt Service Routines ---------------------------------------------------------------------*/
/*
 * @brief SysTick Interrupt Handler
 *
 * This function is called every time the SysTick interrupt occurs (every 1ms).
 *
 */
void SysTick_Handler(void)
{
    msTicks++; // Increment the SysTick counter
}






