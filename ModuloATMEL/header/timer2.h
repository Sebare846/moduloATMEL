/**
 * TC2 Generated Driver API Header File
 *
 * @file tc2.h
 *
 * @defgroup tc2 TC2
 *
 * @brief This file contains the API prototypes and other data types for the Timer Counter 2 (TC2) driver.
 *
 * @version TC2 Driver Version 1.0.0
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
  
#ifndef TC2_H
#define TC2_H

#include "../mcc_generated_files/system/utils/compiler.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @ingroup tc2
 * @brief Initializes the TC2 module.
 * @param None
 * @retval 0 - The TC2 initialization is successful.
 * @retval 1 - The TC2 initialization is unsuccessful.
 */
int8_t TC2_Initialize(void);

/**
 * @ingroup tc2
 * @brief Checks the overflow Interrupt flag status of the TC2 module.
 * @param None
 * @retval True  - Timer Overflow is detected.
 * @retval False - Timer Overflow is not detected.
 */
bool TC2_Get_Overflow_InterruptFlagStatus(void);

/**
 * @ingroup tc2
 * @brief Checks the Compare A interrupt flag status of the TC2 module.
 * @param None
 * @retval True  - The counter value matches the Output Compare Register A.
 * @retval False - The counter value does not match the Output Compare Register A.
 */
bool TC2_Get_COM2A_InterruptFlagStatus(void);

/**
 * @ingroup tc2
 * @brief Checks the Compare B interrupt flag status of the TC2 module.
 * @param None
 * @retval True  - The counter value matches the Output Compare Register B.
 * @retval False - The counter value does not match the Output Compare Register B.
 */
bool TC2_Get_COM2B_InterruptFlagStatus(void);

/**
 * @ingroup tc2
 * @brief Enables the overflow interrupt of the TC2 module.
 * @param None.
 * @return None.
 */
void TC2_Enable_Overflow_Interrupt(void);

/**
 * @ingroup tc2
 * @brief Disables the overflow interrupt of the TC2 module.
 * @param None.
 * @return None.
 */
void TC2_Disable_Overflow_Interrupt(void);

/**
 * @ingroup tc2
 * @brief Enables the Compare A interrupt of the TC2 module.
 * @param None.
 * @return None.
 */
void TC2_Enable_COM2A_Interrupt(void);

/**
 * @ingroup tc2
 * @brief Disables the Compare A interrupt of the TC2 module.
 * @param None
 * @return None.
 */
void TC2_Disable_COM2A_Interrupt(void);

/**
 * @ingroup tc2
 * @brief Enables the Compare B interrupt of the TC2 module.
 * @param None
 * @return None.
 */
void TC2_Enable_COM2B_Interrupt(void);

/**
 * @ingroup tc2
 * @brief Disables the Compare B interrupt of the TC2 module.
 * @param None
 * @return None.
 */
void TC2_Disable_COM2B_Interrupt(void);

/**
 * @ingroup tc2
 * @brief Writes a value to the TC2 timer register
 * @param timerValue - Value to be written to the timer register.
 * @return None.
 */
void TC2_WriteTimer(uint8_t timerValue);

/**
 * @ingroup tc2
 * @brief Reads the timer value of the TC2 module.
 * @param None
 * @retval uint8_t  - Timer register value.
 */
uint8_t TC2_ReadTimer(void);

void TMR2_SetHandler(void (*interruptHandler)(void));


#endif    //TC2_H
