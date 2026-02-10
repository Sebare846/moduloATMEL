
/*
 * timer1.h
 *
 * Created: 2/9/2026 11:19:38 AM
 *  Author: sofia
 */ 
#ifndef TIMER1_H_
#define TIMER1_H_

#include "../mcc_generated_files/system/utils/compiler.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @ingroup tc1
 * @brief Initializes the TC1 module
 * @param None.
 * @return 0 - The TC1 initialization was successful.
 * @return 1 - The TC1 initialization was not successful.
 */
int8_t TC1_Initialize(void);

/**
 * @ingroup tc1
 * @brief Checks the Overflow Interrupt flag status of the TC1 module
 * @param None.
 * @return True  - Overflow interrupt was detected
 * @return False - No overflow interrupt was detected
 */
bool TC1_Get_Overflow_InterruptFlagStatus(void);

/**
 * @ingroup tc1
 * @brief Checks the Compare A flag status of the TC1 module
 * @param None.
 * @return True  - The counter value matches the Output Compare Register A.
 * @return False - The counter value not matches the Output Compare Register A.
 */
bool TC1_Get_CompareA_InterruptFlagStatus(void);

/**
 * @ingroup tc1
 * @brief Checks the Compare B flag status of the TC1 module
 * @param None.
 * @return True  - The counter value matches the Output Compare Register B.
 * @return False - The counter value not matches the Output Compare Register B.
 */

bool TC1_Get_CompareB_InterruptFlagStatus(void);

/**
 * @ingroup tc1
 * @brief Enables the Overflow interrupt of the TC1 module
 * @param None.
 * @return None.
 */
void TC1_Enable_Overflow_Interrupt(void);

/**
 * @ingroup tc1
 * @brief Disables the Overflow interrupt of the TC1 module
 * @param None.
 * @return None.
 */
void TC1_Disable_Overflow_Interrupt(void);

/**
 * @ingroup tc1
 * @brief Enables the Compare A interrupt of the TC1 module
 * @param None.
 * @return None.
 */
void TC1_Enable_CompareA_Interrupt(void);

/**
 * @ingroup tc1
 * @brief Disables the Compare A interrupt of the TC1 module
 * @param None.
 * @return None.
 */
void TC1_Disable_CompareA_Interrupt(void);

/**
 * @ingroup tc1
 * @brief Enables the Compare B interrupt of the TC1 module
 * @param None.
 * @return None.
 */
void TC1_Enable_CompareB_Interrupt(void);

/**
 * @ingroup tc1
 * @brief Disables the Compare B interrupt of the TC1 module
 * @param None.
 * @return None.
 */
void TC1_Disable_CompareB_Interrupt(void);

/**
 * @ingroup tc1
 * @brief Writes the timer value of the TC1 module
 * @param uint16_t - TimerValue
 * @return None.
 */
void TC1_WriteTimer(uint16_t timerValue);

/**
 * @ingroup tc1
 * @brief Reads the timer value of the TC1 module
 * @param None.
 * @return uint16_t - Timer register value
 */
uint16_t TC1_ReadTimer(void);

void TMR1_SetHandler(void (*interruptHandler)(void));

#endif    //TIMER1_H_
