/**
 * @file UART.c
 * @author Daniel Mondragon Huerta (Refactored)
 * @brief Implementation file for MCU UART Module with Handle Pattern
 *
 * This implementation provides improved error handling, validation,
 * and follows embedded systems best practices.
 *
 * @version 2.0
 * @date 2025-12-01
 */

/* Includes --------------------------------------------------------------------------------*/
#include "UART.h"

/* Private Defines -------------------------------------------------------------------------*/
#ifndef NULL
#define NULL ((void*)0)
#endif

/* System Control Registers */
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_PRGPIO_R         (*((volatile uint32_t *)0x400FEA08))
#define SYSCTL_RCGCUART_R       (*((volatile uint32_t *)0x400FE618))
#define SYSCTL_PRUART_R         (*((volatile uint32_t *)0x400FEA18))

/* GPIO Register Offsets */
#define GPIO_OFFSET_AFSEL       0x420
#define GPIO_OFFSET_DEN         0x51C
#define GPIO_OFFSET_LOCK        0x520
#define GPIO_OFFSET_CR          0x524
#define GPIO_OFFSET_PCTL        0x52C
#define GPIO_OFFSET_AMSEL       0x528

/* UART Register Offsets */
#define UART_OFFSET_DR          0x000
#define UART_OFFSET_FR          0x018
#define UART_OFFSET_IBRD        0x024
#define UART_OFFSET_FBRD        0x028
#define UART_OFFSET_LCRH        0x02C
#define UART_OFFSET_CTL         0x030
#define UART_OFFSET_IFLS        0x034
#define UART_OFFSET_IM          0x038
#define UART_OFFSET_ICR         0x044
#define UART_OFFSET_CC          0xFC8

/* UART Control Register Bits */
#define UART_CTL_UARTEN         (1U << 0)
#define UART_CTL_TXE            (1U << 8)
#define UART_CTL_RXE            (1U << 9)

/* UART Flag Register Bits */
#define UART_FR_TXFF            (1U << 5)  // TX FIFO Full
#define UART_FR_RXFE            (1U << 4)  // RX FIFO Empty

/* UART Line Control Register */
#define UART_LCRH_WLEN_8        0x60       // 8-bit word length
#define UART_LCRH_FEN           (1U << 4)  // FIFO Enable

/* UART Interrupt Masks */
#define UART_IM_RXIM            (1U << 4)  // RX Interrupt Mask

/* NVIC Registers */
#define NVIC_EN0_R              (*((volatile uint32_t *)0xE000E100))
#define NVIC_EN1_R              (*((volatile uint32_t *)0xE000E104))
#define NVIC_DIS0_R             (*((volatile uint32_t *)0xE000E180))
#define NVIC_DIS1_R             (*((volatile uint32_t *)0xE000E184))

/* GPIO Unlock Key */
#define GPIO_LOCK_KEY           0x4C4F434B

/* Timeout value for waiting operations */
#define UART_TIMEOUT_CYCLES     100000U

/* Private Structures ----------------------------------------------------------------------*/

/**
 * @brief GPIO pin configuration for UART
 */
typedef struct {
    uint8_t port;       // GPIO port (0=A, 1=B, etc.)
    uint8_t rxPin;      // RX pin number
    uint8_t txPin;      // TX pin number
    uint32_t pctlMask;  // PCTL mask value
    uint32_t pctlValue; // PCTL configuration value
    bool needsUnlock;   // True if pins need unlock (PD7, PF0)
} UART_PinConfig_t;

/* Private Constants -----------------------------------------------------------------------*/

/**
 * @brief UART module base addresses
 */
static const uint32_t UART_BASE_ADDRESSES[] = {
    0x4000C000,  // UART0
    0x4000D000,  // UART1
    0x4000E000,  // UART2
    0x4000F000,  // UART3
    0x40010000,  // UART4
    0x40011000,  // UART5
    0x40012000,  // UART6
    0x40013000   // UART7
};

/**
 * @brief GPIO port base addresses
 */
static const uint32_t GPIO_BASE_ADDRESSES[] = {
    0x40004000,  // PORTA
    0x40005000,  // PORTB
    0x40006000,  // PORTC
    0x40007000,  // PORTD
    0x40024000,  // PORTE
    0x40025000   // PORTF
};

/**
 * @brief Pin configuration for each UART module
 */
static const UART_PinConfig_t UART_PIN_CONFIGS[] = {
    {0, 0, 1, 0x000000FF, 0x00000011, false},  // UART0: PA0/PA1
    {1, 0, 1, 0x000000FF, 0x00000011, false},  // UART1: PB0/PB1
    {3, 6, 7, 0xFF000000, 0x11000000, true},   // UART2: PD6/PD7 (needs unlock)
    {2, 6, 7, 0xFF000000, 0x11000000, false},  // UART3: PC6/PC7
    {2, 4, 5, 0x00FF0000, 0x00110000, false},  // UART4: PC4/PC5
    {4, 4, 5, 0x00FF0000, 0x00110000, false},  // UART5: PE4/PE5
    {3, 4, 5, 0x00FF0000, 0x00110000, false},  // UART6: PD4/PD5
    {4, 0, 1, 0x000000FF, 0x00000011, false}   // UART7: PE0/PE1
};

/**
 * @brief NVIC interrupt numbers for UART modules
 */
static const uint8_t UART_IRQ_NUMBERS[] = {
    5, 6, 33, 59, 60, 61, 62, 63
};

/* Private Function Prototypes -------------------------------------------------------------*/
static bool UART_IsValidHandle(const UART_Handle_t *handle);
static bool UART_IsValidConfig(const UART_Config_t *config);
static UART_Status_t UART_InitClock(UART_Module_t module);
static UART_Status_t UART_InitPins(UART_Module_t module);
static UART_Status_t UART_ConfigureModule(UART_Handle_t *handle, const UART_Config_t *config);
static uint32_t UART_GetBaseAddress(UART_Module_t module);
static uint32_t UART_GetGPIOBase(uint8_t port);
static void UART_EnableNVIC(UART_Module_t module);
static void UART_DisableNVIC(UART_Module_t module);

/* Private Implementation ------------------------------------------------------------------*/

/**
 * @brief Validate UART handle
 *
 * @param handle Pointer to UART handle
 * @return true if handle is valid, false otherwise
 */
static bool UART_IsValidHandle(const UART_Handle_t *handle)
{
    return (handle != NULL && handle->isInitialized);
}

/**
 * @brief Validate UART configuration
 *
 * @param config Pointer to UART configuration
 * @return true if configuration is valid, false otherwise
 */
static bool UART_IsValidConfig(const UART_Config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (config->module > UART_MODULE_7) {
        return false;
    }

    if (config->clockFreqMHz == 0 || config->clockFreqMHz > 80) {
        return false;
    }

    return true;
}

/**
 * @brief Get UART module base address
 *
 * @param module UART module number
 * @return Base address of the UART module
 */
static uint32_t UART_GetBaseAddress(UART_Module_t module)
{
    return UART_BASE_ADDRESSES[module];
}

/**
 * @brief Get GPIO port base address
 *
 * @param port GPIO port number (0=A, 1=B, etc.)
 * @return Base address of the GPIO port
 */
static uint32_t UART_GetGPIOBase(uint8_t port)
{
    return GPIO_BASE_ADDRESSES[port];
}

/**
 * @brief Initialize clock for UART module and associated GPIO
 *
 * @param module UART module to initialize
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_InitClock(UART_Module_t module)
{
    const UART_PinConfig_t *pinConfig = &UART_PIN_CONFIGS[module];
    uint32_t timeout = UART_TIMEOUT_CYCLES;

    // Enable GPIO clock
    SYSCTL_RCGCGPIO_R |= (1U << pinConfig->port);

    // Wait for GPIO to be ready
    while ((SYSCTL_PRGPIO_R & (1U << pinConfig->port)) == 0) {
        if (--timeout == 0) {
            return UART_STATUS_TIMEOUT;
        }
    }

    // Enable UART clock
    timeout = UART_TIMEOUT_CYCLES;
    SYSCTL_RCGCUART_R |= (1U << module);

    // Wait for UART to be ready
    while ((SYSCTL_PRUART_R & (1U << module)) == 0) {
        if (--timeout == 0) {
            return UART_STATUS_TIMEOUT;
        }
    }

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Initialize GPIO pins for UART
 *
 * @param module UART module
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_InitPins(UART_Module_t module)
{
    const UART_PinConfig_t *pinConfig = &UART_PIN_CONFIGS[module];
    uint32_t gpioBase = UART_GetGPIOBase(pinConfig->port);

    volatile uint32_t *gpioLock   = (volatile uint32_t *)(gpioBase + GPIO_OFFSET_LOCK);
    volatile uint32_t *gpioCr     = (volatile uint32_t *)(gpioBase + GPIO_OFFSET_CR);
    volatile uint32_t *gpioPctl   = (volatile uint32_t *)(gpioBase + GPIO_OFFSET_PCTL);
    volatile uint32_t *gpioAfsel  = (volatile uint32_t *)(gpioBase + GPIO_OFFSET_AFSEL);
    volatile uint32_t *gpioDen    = (volatile uint32_t *)(gpioBase + GPIO_OFFSET_DEN);
    volatile uint32_t *gpioAmsel  = (volatile uint32_t *)(gpioBase + GPIO_OFFSET_AMSEL);

    // Unlock GPIO if needed (for PD7, PF0)
    if (pinConfig->needsUnlock) {
        *gpioLock = GPIO_LOCK_KEY;
        *gpioCr |= (1U << pinConfig->txPin);  // Commit TX pin
    }

    // Configure PCTL for UART function
    *gpioPctl &= ~pinConfig->pctlMask;
    *gpioPctl |= pinConfig->pctlValue;

    // Enable alternate function
    *gpioAfsel |= (1U << pinConfig->rxPin) | (1U << pinConfig->txPin);

    // Enable digital function
    *gpioDen |= (1U << pinConfig->rxPin) | (1U << pinConfig->txPin);

    // Disable analog mode
    *gpioAmsel &= ~((1U << pinConfig->rxPin) | (1U << pinConfig->txPin));

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Configure UART module with baud rate and settings
 *
 * @param handle Pointer to UART handle
 * @param config Pointer to UART configuration
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_ConfigureModule(UART_Handle_t *handle, const UART_Config_t *config)
{
    uint32_t uartBase = UART_GetBaseAddress(config->module);

    volatile uint32_t *uartCtl  = (volatile uint32_t *)(uartBase + UART_OFFSET_CTL);
    volatile uint32_t *uartIbrd = (volatile uint32_t *)(uartBase + UART_OFFSET_IBRD);
    volatile uint32_t *uartFbrd = (volatile uint32_t *)(uartBase + UART_OFFSET_FBRD);
    volatile uint32_t *uartLcrh = (volatile uint32_t *)(uartBase + UART_OFFSET_LCRH);
    volatile uint32_t *uartCc   = (volatile uint32_t *)(uartBase + UART_OFFSET_CC);

    // Disable UART during configuration
    *uartCtl &= ~UART_CTL_UARTEN;

    // Calculate baud rate divisors
    // BRD = BRDI + BRDF = UARTSysClk / (16 * Baud Rate)
    uint32_t clockHz = config->clockFreqMHz * 1000000U;
    float brdFloat = (float)clockHz / (16.0f * (float)config->baudRate);
    uint32_t brdi = (uint32_t)brdFloat;
    uint32_t brdf = (uint32_t)((brdFloat - (float)brdi) * 64.0f + 0.5f);

    // Set baud rate divisors
    *uartIbrd = brdi;
    *uartFbrd = brdf;

    // Configure line control: 8-bit, no parity, 1 stop bit
    *uartLcrh = UART_LCRH_WLEN_8;
    if (config->enableFIFO) {
        *uartLcrh |= UART_LCRH_FEN;
    }

    // Use system clock
    *uartCc = 0x0;

    // Enable UART and configure TX/RX
    uint32_t ctlValue = UART_CTL_UARTEN;
    if (config->enableTx) {
        ctlValue |= UART_CTL_TXE;
    }
    if (config->enableRx) {
        ctlValue |= UART_CTL_RXE;
    }
    *uartCtl = ctlValue;

    // Update handle state
    handle->module = config->module;
    handle->baudRate = config->baudRate;
    handle->txEnabled = config->enableTx;
    handle->rxEnabled = config->enableRx;
    handle->fifoEnabled = config->enableFIFO;
    handle->status = UART_STATUS_SUCCESS;
    handle->isInitialized = true;

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Enable NVIC interrupt for UART module
 *
 * @param module UART module
 */
static void UART_EnableNVIC(UART_Module_t module)
{
    uint8_t irqNum = UART_IRQ_NUMBERS[module];

    if (irqNum < 32) {
        NVIC_EN0_R |= (1U << irqNum);
    } else {
        NVIC_EN1_R |= (1U << (irqNum - 32));
    }
}

/**
 * @brief Disable NVIC interrupt for UART module
 *
 * @param module UART module
 */
static void UART_DisableNVIC(UART_Module_t module)
{
    uint8_t irqNum = UART_IRQ_NUMBERS[module];

    if (irqNum < 32) {
        NVIC_DIS0_R |= (1U << irqNum);
    } else {
        NVIC_DIS1_R |= (1U << (irqNum - 32));
    }
}

/* Public API Implementation ---------------------------------------------------------------*/

/**
 * @brief Initialize UART module with configuration
 *
 * @param handle Pointer to UART handle
 * @param config Pointer to UART configuration
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_Init(UART_Handle_t *handle, const UART_Config_t *config)
{
    if (handle == NULL || config == NULL) {
        return UART_STATUS_INVALID_PARAM;
    }

    if (!UART_IsValidConfig(config)) {
        return UART_STATUS_INVALID_PARAM;
    }

    // Initialize handle
    handle->isInitialized = false;
    handle->interruptEnabled = false;

    // Initialize clock
    UART_Status_t status = UART_InitClock(config->module);
    if (status != UART_STATUS_SUCCESS) {
        return status;
    }

    // Initialize pins
    status = UART_InitPins(config->module);
    if (status != UART_STATUS_SUCCESS) {
        return status;
    }

    // Configure UART module
    status = UART_ConfigureModule(handle, config);

    return status;
}

/**
 * @brief Deinitialize UART module
 *
 * @param handle Pointer to UART handle
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_Deinit(UART_Handle_t *handle)
{
    if (!UART_IsValidHandle(handle)) {
        return UART_STATUS_INVALID_PARAM;
    }

    uint32_t uartBase = UART_GetBaseAddress(handle->module);
    volatile uint32_t *uartCtl = (volatile uint32_t *)(uartBase + UART_OFFSET_CTL);

    // Disable UART
    *uartCtl &= ~UART_CTL_UARTEN;

    // Disable interrupts if enabled
    if (handle->interruptEnabled) {
        UART_DisableNVIC(handle->module);
    }

    // Mark as uninitialized
    handle->isInitialized = false;
    handle->status = UART_STATUS_NOT_INITIALIZED;

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Send a single byte through UART
 *
 * @param handle Pointer to UART handle
 * @param data Byte to send
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_SendByte(UART_Handle_t *handle, uint8_t data)
{
    if (!UART_IsValidHandle(handle)) {
        return UART_STATUS_INVALID_PARAM;
    }

    if (!handle->txEnabled) {
        return UART_STATUS_ERROR;
    }

    uint32_t uartBase = UART_GetBaseAddress(handle->module);
    volatile uint32_t *uartFr = (volatile uint32_t *)(uartBase + UART_OFFSET_FR);
    volatile uint32_t *uartDr = (volatile uint32_t *)(uartBase + UART_OFFSET_DR);

    uint32_t timeout = UART_TIMEOUT_CYCLES;

    // Wait for TX FIFO not full
    while ((*uartFr & UART_FR_TXFF) != 0) {
        if (--timeout == 0) {
            return UART_STATUS_TIMEOUT;
        }
    }

    // Send data
    *uartDr = data;

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Send a string through UART
 *
 * @param handle Pointer to UART handle
 * @param string Pointer to string data
 * @param length Length of string (0 for null-terminated)
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_SendString(UART_Handle_t *handle, const char *string, uint32_t length)
{
    if (!UART_IsValidHandle(handle) || string == NULL) {
        return UART_STATUS_INVALID_PARAM;
    }

    if (!handle->txEnabled) {
        return UART_STATUS_ERROR;
    }

    // If length is 0, calculate it (null-terminated string)
    if (length == 0) {
        const char *ptr = string;
        while (*ptr != '\0') {
            ptr++;
            length++;
        }
    }

    // Send each character
    uint32_t i = 0;
    for (i = 0; i < length; i++) {
        UART_Status_t status = UART_SendByte(handle, (uint8_t)string[i]);
        if (status != UART_STATUS_SUCCESS) {
            return status;
        }
    }

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Receive a single byte from UART
 *
 * @param handle Pointer to UART handle
 * @param data Pointer to store received byte
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_ReceiveByte(UART_Handle_t *handle, uint8_t *data)
{
    if (!UART_IsValidHandle(handle) || data == NULL) {
        return UART_STATUS_INVALID_PARAM;
    }

    if (!handle->rxEnabled) {
        return UART_STATUS_ERROR;
    }

    uint32_t uartBase = UART_GetBaseAddress(handle->module);
    volatile uint32_t *uartFr = (volatile uint32_t *)(uartBase + UART_OFFSET_FR);
    volatile uint32_t *uartDr = (volatile uint32_t *)(uartBase + UART_OFFSET_DR);

    uint32_t timeout = UART_TIMEOUT_CYCLES;

    // Wait for data available
    while ((*uartFr & UART_FR_RXFE) != 0) {
        if (--timeout == 0) {
            return UART_STATUS_TIMEOUT;
        }
    }

    // Read data
    *data = (uint8_t)(*uartDr & 0xFF);

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Enable UART interrupt
 *
 * @param handle Pointer to UART handle
 * @param level FIFO interrupt level
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_EnableInterrupt(UART_Handle_t *handle, UART_FIFOLevel_t level)
{
    if (!UART_IsValidHandle(handle)) {
        return UART_STATUS_INVALID_PARAM;
    }

    uint32_t uartBase = UART_GetBaseAddress(handle->module);
    volatile uint32_t *uartIfls = (volatile uint32_t *)(uartBase + UART_OFFSET_IFLS);
    volatile uint32_t *uartIm   = (volatile uint32_t *)(uartBase + UART_OFFSET_IM);
    volatile uint32_t *uartIcr  = (volatile uint32_t *)(uartBase + UART_OFFSET_ICR);

    // Flush RX FIFO first
    UART_API.flush(handle);

    // Set FIFO level
    *uartIfls = (level << 3);

    // Enable RX interrupt
    *uartIm |= UART_IM_RXIM;

    // Clear any pending interrupts
    *uartIcr = 0xFFFF;

    // Enable NVIC
    UART_EnableNVIC(handle->module);

    handle->interruptEnabled = true;

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Disable UART interrupt
 *
 * @param handle Pointer to UART handle
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_DisableInterrupt(UART_Handle_t *handle)
{
    if (!UART_IsValidHandle(handle)) {
        return UART_STATUS_INVALID_PARAM;
    }

    uint32_t uartBase = UART_GetBaseAddress(handle->module);
    volatile uint32_t *uartIm = (volatile uint32_t *)(uartBase + UART_OFFSET_IM);

    // Disable RX interrupt
    *uartIm &= ~UART_IM_RXIM;

    // Disable NVIC
    UART_DisableNVIC(handle->module);

    handle->interruptEnabled = false;

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Flush UART RX FIFO
 *
 * @param handle Pointer to UART handle
 * @return UART_STATUS_SUCCESS on success, error code otherwise
 */
static UART_Status_t UART_Flush(UART_Handle_t *handle)
{
    if (!UART_IsValidHandle(handle)) {
        return UART_STATUS_INVALID_PARAM;
    }

    uint32_t uartBase = UART_GetBaseAddress(handle->module);
    volatile uint32_t *uartFr = (volatile uint32_t *)(uartBase + UART_OFFSET_FR);
    volatile uint32_t *uartDr = (volatile uint32_t *)(uartBase + UART_OFFSET_DR);

    // Read and discard all data in FIFO
    while ((*uartFr & UART_FR_RXFE) == 0) {
        (void)*uartDr;  // Read and discard
    }

    return UART_STATUS_SUCCESS;
}

/**
 * @brief Check if TX is ready (FIFO not full)
 *
 * @param handle Pointer to UART handle
 * @return true if ready, false otherwise
 */
static bool UART_IsTxReady(UART_Handle_t *handle)
{
    if (!UART_IsValidHandle(handle)) {
        return false;
    }

    uint32_t uartBase = UART_GetBaseAddress(handle->module);
    volatile uint32_t *uartFr = (volatile uint32_t *)(uartBase + UART_OFFSET_FR);

    return ((*uartFr & UART_FR_TXFF) == 0);
}

/**
 * @brief Check if RX has data available
 *
 * @param handle Pointer to UART handle
 * @return true if data available, false otherwise
 */
static bool UART_IsRxReady(UART_Handle_t *handle)
{
    if (!UART_IsValidHandle(handle)) {
        return false;
    }

    uint32_t uartBase = UART_GetBaseAddress(handle->module);
    volatile uint32_t *uartFr = (volatile uint32_t *)(uartBase + UART_OFFSET_FR);

    return ((*uartFr & UART_FR_RXFE) == 0);
}

/* Public API Instance ---------------------------------------------------------------------*/
const UART_Interface_t UART_API = {
    .init = UART_Init,
    .deinit = UART_Deinit,
    .sendByte = UART_SendByte,
    .sendString = UART_SendString,
    .receiveByte = UART_ReceiveByte,
    .enableInterrupt = UART_EnableInterrupt,
    .disableInterrupt = UART_DisableInterrupt,
    .flush = UART_Flush,
    .isTxReady = UART_IsTxReady,
    .isRxReady = UART_IsRxReady
};
