/*
 * timer0.h
 *
 * Created: 9/2/2026 10:14:01
 *  Author: Seba
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

#ifndef TIMER0_H_
#define TIMER0_H_

#include "../mcc_generated_files/system/utils/compiler.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @ingroup tc0
 * @brief Initializes the TC0 module.
 * @param None.
 * @return 0 - The TC0 initialization was successful.
 * @return 1 - The TC0 initialization was not successful.
 */
int8_t TC0_Initialize(void);

/**
 * @ingroup tc0
 * @brief Checks the Overflow Interrupt flag status of the TC0 module.
 * @param None.
 * @return True  - Timer Overflow is detected.
 * @return False - Timer Overflow is not detected.
 */
bool TC0_Get_Overflow_InterruptFlagStatus(void);

/**
 * @ingroup tc0
 * @brief Checks the Compare A flag status of the TC0 module.
 * @param None.
 * @return True  - The counter value matches the Output Compare Register A.
 * @return False - The counter value does not match the Output Compare Register A.
 */
bool TC0_Get_CompareA_InterruptFlagStatus(void);

/**
 * @ingroup tc0
 * @brief Checks the Compare B flag status of the TC0 module.
 * @param None.
 * @return True  - The counter value matches the Output Compare Register B.
 * @return False - The counter value does not match the Output Compare Register B.
 */

bool TC0_Get_CompareB_InterruptFlagStatus(void);

/**
 * @ingroup tc0
 * @brief Enables the Overflow interrupt of the TC0 module.
 * @param None.
 * @return None.
 */
void TC0_Enable_Overflow_Interrupt(void);

/**
 * @ingroup tc0
 * @brief Disables the Overflow interrupt of the TC0 module.
 * @param None.
 * @return None.
 */
void TC0_Disable_Overflow_Interrupt(void);

/**
 * @ingroup tc0
 * @brief Enables the Compare A interrupt of the TC0 module.
 * @param None.
 * @return None.
 */
void TC0_Enable_CompareA_Interrupt(void);

/**
 * @ingroup tc0
 * @brief Disables the Compare A interrupt of the TC0 module
 * @param None.
 * @return None.
 */
void TC0_Disable_CompareA_Interrupt(void);

/**
 * @ingroup tc0
 * @brief Enables the Compare B interrupt of the TC0 module
 * @param None.
 * @return None.
 */
void TC0_Enable_CompareB_Interrupt(void);

/**
 * @ingroup tc0
 * @brief Disables the Compare B interrupt of the TC0 module
 * @param None.
 * @return None.
 */
void TC0_Disable_CompareB_Interrupt(void);

/**
 * @ingroup tc0
 * @brief Writes the timer value of the TC0 module
 * @param uint8_t TimerValue - Value to be written to the timer register.
 * @return None.
 */
void TC0_WriteTimer(uint8_t timerValue);

/**
 * @ingroup tc0
 * @brief Reads the timer value of the TC0 module.
 * @param None.
 * @return uint8_t - Timer register value
 */
uint8_t TC0_ReadTimer(void);


void TMR0_SetHandler(void (*interruptHandler)(void));



#endif /* TIMER0_H_ */