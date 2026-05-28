/**
 * @file pll.c
 * @author Alberto Vazquez
 * @brief source file for MCU PWM Module x Configuration
 *
 *
 * @version 0.11
 * @date 2025-09-15
 */

/* Includes ----------------------------------------------------------------------------------------*/
#include "pll.h"

/* Private Defines ---------------------------------------------------------------------------------*/
#ifndef NULL
#define NULL 0
#endif

#define DEFAULT_SYSTEM_CLOCK_MHZ 16U  // Default system clock frequency in Hz
#define DYTU_CYCLE_TO_DECIMAL(x) ((float_t)(x) / 100.0f)  // Convert duty cycle percentage to decimal
//TM4C123GH6PM REGISTERS
#define SYSCTL_RCC2_R           (*((volatile uint32_t *)0x400FE070))
#define SYSCTL_RCC_R            (*((volatile uint32_t *)0x400FE060))
#define SYSCTL_RIS_R            (*((volatile uint32_t *)0x400FE050))

//TM4C123GH6PM FIELDS in the SYSCTL_RCC2 register
#define SYSCTL_RCC2_USERCC2     0x80000000  // Use RCC2
#define SYSCTL_RCC2_BYPASS2     0x00000800  // PLL Bypass 2
#define SYSCTL_RCC2_OSCSRC2_M   0x00000070  // Oscillator Source 2
#define SYSCTL_RCC2_OSCSRC2_MO  0x00000000  // MOSC
#define SYSCTL_RCC2_PWRDN2      0x00002000  // Power-Down PLL 2
#define SYSCTL_RCC2_DIV400      0x40000000  // Divide PLL as 400 MHz vs. 200
                                            // MHz
//TM4C123GH6PM FIELDS in the SYSCTL_RCC register
#define SYSCTL_RCC_XTAL_M       0x000007C0  // Crystal Value
#define SYSCTL_RCC_XTAL_16MHZ   0x00000540  // 16 MHz
//TM4C123GH6PM FIELDS in the SYSCTL_RIS register
#define SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status

/* Private Function Prototypes ---------------------------------------------------------------------*/
static PLL_Status_t PLL_InitHardware(PLL_Handle_t *handle);
static uint32_t PLL_CalculateFrequency(uint32_t frequency_MHz);
static bool PLL_IsValidHandle(const PLL_Handle_t *handle);
static bool PLL_IsValidFrequency(uint32_t frequency_MHz);

/* Private Implementation --------------------------------------------------------------------------*/
/**
 * @brief Initialize PLL module and channel
 *
 * @param handle Pointer to PWM handle
 * @param frequency Desired frequency in MHz
 * @return PLL_Status_t Status of the initialization
 */
PLL_Status_t PLL_Init(PLL_Handle_t *handle, PLL_Frequency_t frequency)
{
    if(!handle && !frequency) {
        return PLL_STATUS_INVALID_PARAM;
    }

    if(!PLL_IsValidFrequency(frequency)) {
        return PLL_STATUS_INVALID_PARAM;
    }

    // Initialize handle
    handle->frequency = frequency;
    handle->usePLL = false;

    // Initialize hardware
    PLL_Status_t status = PLL_InitHardware(handle);
    if(status == PLL_STATUS_SUCCESS) {
        handle->usePLL = true;
    }
    return status;
}

/**
 * @brief Initialize PLL hardware
 *
 * @param handle Pointer to PLL handle
 * @return PLL_Status_t Status of the initialization
 *
 */
static PLL_Status_t PLL_InitHardware(PLL_Handle_t *handle)
{
    if(!PLL_IsValidHandle(handle)) {
        return PLL_STATUS_INVALID_PARAM;
    }
    // 0: Configure the system to use RCC2 for advanced features
    //   such as 400 MHz PLL and non-integer system clock dividers.
    SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;
    // 1: Bypass the PLL while initializing.
    SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;
    // 2: Select the crystal value and oscillator source.
    SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M; // Clear XTAL field
    SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ; // Configure for 16 MHz crystal
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M; // Clear oscillator source field
    SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO; // Configure for main oscillator source
    // 3: Activate the PLL by clearing PWRDN.
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
    // 4: Set the desired system divider and the system clock to use the PLL.
    uint32_t pll_sysdiv2 = PLL_CalculateFrequency(handle->frequency);
    SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400; // Use 400 MHz PLL
    SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~0X1FC00000)   //Clear system clock divider field
                    + (pll_sysdiv2 << 22);          // Configure for desired system clock
    // 5: Wait for the PLL to lock by polling the PLLLRIS bit in the Raw Interrupt Status register.
    while((SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS) == 0) {};
    // 6: Enable use of PLL by clearing BYPASS.
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
    return PLL_STATUS_SUCCESS;
}

/**
 * @brief Validate PLL handle
 *
 * @param handle Pointer to PLL handle to validate
 * @return true if handle is valid, false otherwise
 *
 */
static bool PLL_IsValidHandle(const PLL_Handle_t *handle)
{
    return (handle != NULL && handle->usePLL) ? false : true;
}

/**
 * @brief Validate PLL frequency
 *
 * @param frequency_MHz Frequency in MHz to validate
 * @return true if frequency is valid, false otherwise
 *
 */
static bool PLL_IsValidFrequency(uint32_t frequency_MHz)
{
    switch(frequency_MHz) {
        case MHz80:
        case MHz50:
        case MHz40:
        case MHz25:
        case MHz20:
        case MHz16:
        case MHz10:
        case MHz8:
        case MHz5:
        case MHz4:
            return true;
        default:
            return false;
    }
}

/**
 * @brief Calculate PLL frequency for setting
 *
 * @param frequency_MHz Desired frequency in MHz
 * @return uint32_t Calculated frequency setting
 *
 */
static uint32_t PLL_CalculateFrequency(uint32_t frequency_MHz)
{

    if(frequency_MHz == 0) {
        frequency_MHz = DEFAULT_SYSTEM_CLOCK_MHZ;
    }
    return (uint32_t)((400U/frequency_MHz) - 1U);
}

/**
 * @brief Calculate PLL frequency and check if matches the requested frequency
 *
 * @return uint32_t Current PLL frequency in MHz
 */
uint32_t PLL_GetFrequency(void)
{
    //Get the current value of the SYSDIV2 field
    uint32_t sysdiv2 = (SYSCTL_RCC2_R & 0x1FC00000) >> 22;
    //Calculate the current frequency of the PLL
    return (uint32_t)(400U / (sysdiv2 + 1U));
}

/**
 * @brief Provide a blocking delay in milliseconds
 *
 * @param ms Number of milliseconds to delay
 * 
 */
 void delay_ms(uint32_t ms)
 {
     uint32_t i = 0, j = 0;
     uint32_t iterations = 0;
    // Get current value of the sysdiv2 field
    uint32_t sysdiv2 = PLL_GetFrequency();
    // Select correct counts by frequency selected
    switch(sysdiv2) {
    case 80: iterations = 6620; break;
    case 50: iterations = 4090; break;
    case 40: iterations = 3950; break;
    case 25: iterations = 2450; break;
    case 20: iterations = 1965; break;
    case 16: iterations = 1570; break;
    case 10: iterations = 978; break;
    case 8: iterations = 785; break;
    case 5: iterations = 480; break;
    case 4: iterations = 385; break;
    }
    //Method 1
    // Calculate total delay time by iterations and ms parameter
    //uint32_t total_iterations = iterations * ms;
    // Perform the delay
    //for(i = 0; i < total_iterations; i++) {};
    //Method 2
    //NOTE: to get mor delay use for two for loops one with ms and other with iterations
    //This will increase the precision of the delay
    for(i=0; i < ms; i++) {
        for(j=0; j < iterations; j++);
    }
 }

/* Public API Instance -----------------------------------------------------------------------------*/
const PLL_Interface_t PLL_API = {
    .init = PLL_Init,
    .getPLLFrequency = PLL_GetFrequency,
    .delayMs = delay_ms
};

