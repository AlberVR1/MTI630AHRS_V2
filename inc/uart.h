/**
 * @file UART.h
 * @author Daniel Mondragon Huerta (Refactored)
 * @brief Header file for MCU UART Module with Handle Pattern
 *
 * This header provides a clean HAL-style interface for UART modules
 * using handles, enums, and proper validation.
 *
 * @version 2.0
 * @date 2025-12-01
 */

#ifndef INCLUDE_UART_H_
#define INCLUDE_UART_H_

/**
 * @addtogroup UART Driver
 * @{
 */

/* Includes --------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Public Enums ----------------------------------------------------------------------------*/

/**
 * @brief UART module selection
 */
typedef enum {
    UART_MODULE_0 = 0,  ///< UART0 - Port A (PA0/PA1)
    UART_MODULE_1 = 1,  ///< UART1 - Port B (PB0/PB1)
    UART_MODULE_2 = 2,  ///< UART2 - Port D (PD6/PD7)
    UART_MODULE_3 = 3,  ///< UART3 - Port C (PC6/PC7)
    UART_MODULE_4 = 4,  ///< UART4 - Port C (PC4/PC5)
    UART_MODULE_5 = 5,  ///< UART5 - Port E (PE4/PE5)
    UART_MODULE_6 = 6,  ///< UART6 - Port D (PD4/PD5)
    UART_MODULE_7 = 7   ///< UART7 - Port E (PE0/PE1)
} UART_Module_t;

/**
 * @brief UART baud rate options
 */
typedef enum {
    UART_BAUD_4800   = 4800,
    UART_BAUD_9600   = 9600,
    UART_BAUD_14400  = 14400,
    UART_BAUD_19200  = 19200,
    UART_BAUD_38400  = 38400,
    UART_BAUD_57600  = 57600,
    UART_BAUD_115200 = 115200
} UART_BaudRate_t;

/**
 * @brief UART status codes
 */
typedef enum {
    UART_STATUS_SUCCESS = 0,
    UART_STATUS_ERROR,
    UART_STATUS_INVALID_PARAM,
    UART_STATUS_NOT_INITIALIZED,
    UART_STATUS_BUSY,
    UART_STATUS_TIMEOUT
} UART_Status_t;

/**
 * @brief UART FIFO level configuration
 */
typedef enum {
    UART_FIFO_LEVEL_1_8 = 0,  ///< Interrupt at 1/8 full    2 bytes
    UART_FIFO_LEVEL_1_4 = 1,  ///< Interrupt at 1/4 full    4 bytes
    UART_FIFO_LEVEL_1_2 = 2,  ///< Interrupt at 1/2 full (default)  8 bytes
    UART_FIFO_LEVEL_3_4 = 3,  ///< Interrupt at 3/4 full    12 bytes
    UART_FIFO_LEVEL_7_8 = 4   ///< Interrupt at 7/8 full    14 bytes
} UART_FIFOLevel_t;

/* Public Structures -----------------------------------------------------------------------*/

/**
 * @brief UART configuration structure
 */
typedef struct {
    UART_Module_t module;           ///< UART module to use
    UART_BaudRate_t baudRate;       ///< Baud rate
    uint32_t clockFreqMHz;          ///< System clock frequency in MHz
    bool enableTx;                  ///< Enable transmission
    bool enableRx;                  ///< Enable reception
    bool enableFIFO;                ///< Enable FIFO buffers
    UART_FIFOLevel_t fifoLevel;     ///< FIFO interrupt level
} UART_Config_t;

/**
 * @brief UART handle structure - Contains module state
 */
typedef struct {
    UART_Module_t module;           ///< Configured UART module
    UART_BaudRate_t baudRate;       ///< Current baud rate
    UART_Status_t status;           ///< Current status
    bool isInitialized;             ///< Initialization flag
    bool txEnabled;                 ///< TX enabled flag
    bool rxEnabled;                 ///< RX enabled flag
    bool fifoEnabled;               ///< FIFO enabled flag
    bool interruptEnabled;          ///< Interrupt enabled flag
} UART_Handle_t;

/* Public API Interface --------------------------------------------------------------------*/

/**
 * @brief UART API function pointers
 */
typedef struct {
    UART_Status_t (*init)(UART_Handle_t *handle, const UART_Config_t *config);
    UART_Status_t (*deinit)(UART_Handle_t *handle);
    UART_Status_t (*sendByte)(UART_Handle_t *handle, uint8_t data);
    UART_Status_t (*sendString)(UART_Handle_t *handle, const char *string, uint32_t length);
    UART_Status_t (*receiveByte)(UART_Handle_t *handle, uint8_t *data);
    UART_Status_t (*enableInterrupt)(UART_Handle_t *handle, UART_FIFOLevel_t level);
    UART_Status_t (*disableInterrupt)(UART_Handle_t *handle);
    UART_Status_t (*flush)(UART_Handle_t *handle);
    bool (*isTxReady)(UART_Handle_t *handle);
    bool (*isRxReady)(UART_Handle_t *handle);
} UART_Interface_t;

/* Public API Instance ---------------------------------------------------------------------*/
extern const UART_Interface_t UART_API;

/**
 * @}
 */

#endif /* INCLUDE_UART_H_ */
