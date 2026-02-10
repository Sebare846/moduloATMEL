/*
 * timer1.c
 *
 * Created: 2/9/2026 11:21:16 AM
 *  Author: sofia
 */ 

/**
  * TC1 Generated Driver File
  *
  * @file tc1.c
  *
  * @ingroup tc1
  *
  * @brief This file contains the API implementations for the TC1 module driver.
  *
  * @version TC1 Driver Version 1.0.0
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
#include <avr/io.h>
#include "../header/timer1.h"

void (*TMR1Handler)(void) = NULL;

int8_t TC1_Initialize(void) 
{
    //Compare A
    OCR1A = 0x9F;

    //Compare B
    OCR1B = 0x1;

    //Input Capture
    ICR1 = 0x0;

    //Count
    TCNT1 = 0x0;

    //PSRSYNC disabled; TSM disabled; 
    GTCCR = 0x0;

    //COM1A 0; COM1B 0; WGM1 0; 
    TCCR1A = 0x0;

    //CS1 RUNNING_NO_PRESCALING; ICES1 disabled; ICNC1 disabled; WGM1 1; 
    TCCR1B = 0x9;

    //FOC1A disabled; FOC1B disabled; 
    TCCR1C = 0x0;

    //ICIE1 disabled; OCIE1A enabled; OCIE1B disabled; TOIE1 disabled; 
    TIMSK1 = 0x2;

    return 0;
}

bool TC1_Get_Overflow_InterruptFlagStatus(void)
{
    if (TIFR1  & TOV1) {
        return true;
    } else {
        return false;
    }
}

bool TC1_Get_CompareA_InterruptFlagStatus(void)
{
    if (TIFR1  & OCF1A) {
        return true;
    } else {
        return false;
    }
}

bool TC1_Get_CompareB_InterruptFlagStatus(void)
{
    if (TIFR1  & OCF1B) {
        return true;
    } else {
        return false;
    }
}

void TC1_Enable_Overflow_Interrupt(void)
{
    TIMSK1 |= (1 << TOIE1);
}

void TC1_Disable_Overflow_Interrupt(void)
{
    TIMSK1 &= ~(1 << TOIE1);
}

void TC1_Enable_CompareA_Interrupt(void)
{
    TIMSK1 |= (1 << OCIE1A);
}

void TC1_Disable_CompareA_Interrupt(void)
{
    TIMSK1 &= ~(1 << OCIE1A);
}

void TC1_Enable_CompareB_Interrupt(void)
{
    TIMSK1 |= (1 << OCIE1B);
}

void TC1_Disable_CompareB_Interrupt(void)
{
    TIMSK1 &= ~(1 << OCIE1B);
}

void TC1_WriteTimer(uint16_t timerValue)
{
    TCNT1 = timerValue;
}

uint16_t TC1_ReadTimer(void)
{
    return TCNT1;
}


ISR(TIMER1_COMPB_vect)
{
    //clears the Compare B interrupt flag
    TIFR1 = OCF1B;

    /* Insert your Compare B interrupt handling code here */
    
}

ISR(TIMER1_OVF_vect)
{
    //clears the Overflow interrupt flag
    TIFR1 = TOV1;

    /* Insert your Overflow interrupt handling code here */
    
}

ISR(TIMER1_CAPT_vect)
{
    //clears the Overflow interrupt flag
    TIFR1 = ICF1;

    /* Insert your Overflow interrupt handling code here */
    
}


void TMR1_SetHandler(void (*interruptHandler)(void)){
	TMR1Handler = interruptHandler;
}

ISR(TIMER1_COMPA_vect)
{
	//clears the Compare A interrupt flag
	TIFR1 = OCF1A;
	
	// Si el puntero no es nulo, ejecuta la función asignada
	if(TMR1Handler != NULL){
		TMR1Handler();
	}
	/* Insert your Compare A interrupt handling code here */
	
}