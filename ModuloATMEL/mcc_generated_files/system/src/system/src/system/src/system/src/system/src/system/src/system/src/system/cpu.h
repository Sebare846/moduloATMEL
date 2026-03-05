/**
 * CPU Generated Driver API Header File
 *
 * @file cpu.h
 *
 * @defgroup cpu CPU
 *
 * @brief This file contains the API Prototypes for the CPU driver.
 *
 * @version CPU Driver Version 1.0.2
 *
 * @version Package Version 1.0.2
*/
/*
© [2026] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/


#ifndef CPU_H
#define CPU_H

#include <stdint.h>

/**
@ingroup cpu
@brief Defines the system clock frequency.
*/
#ifndef F_CPU
/**
    @ingroup cpu 
    @def system frequency
    @misradeviation{@required, 21.1} Defining the system frequency using the F_CPU macro is required by the XC8 compiler for the built-in delay functions.
*/
/* cppcheck-suppress misra-c2012-21.1 */
#define F_CPU 16000000
#endif


/**
 * @ingroup cpu
 * @brief Initializes the CPU module.
 * @param None.
 * @return None.
 */
void CPU_Initialize(void);

/**
 * @ingroup cpu
 * @brief Enables Sleep mode for power management.
 * @param None. 
 * @return None.
 */
 void CPU_EnableSleep(void);

/**
 * @ingroup cpu
 * @brief Disables Sleep mode.
 * @param None. 
 * @return None.
 */
void CPU_DisableSleep(void);

/**
 * @ingroup cpu
 * @brief Sets Sleep mode.
 * @param uint8_t mode - Sleep mode for CPU. 
 * @return None.
 */
void CPU_SetSleepMode(uint8_t mode);

/**
 * @ingroup cpu
 * @brief Enables Low-Power and disables all the peripherals. The driver will enable the peripherals when needed.
 * @param None. 
 * @return None.
 */
void CPU_EnableLowPower(void);

#endif // CPU_H