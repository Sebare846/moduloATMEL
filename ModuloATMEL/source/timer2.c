/**
  * TC2 Generated Driver File
  *
  * @file tc2.c
  *
  * @ingroup tc2
  *
  * @brief This file contains the API implementation for the Timer Counter 2 (TC2) module driver.
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
#include <avr/io.h>
#include "../header/timer2.h"


void (*TMR2Handler)(void) = NULL;

int8_t TC2_Initialize(void) 
{
    //Compare A
  //  OCR2A = 0xCF; //9615 BR
	OCR2A = 0x67;

    //Compare B
    OCR2B = 0x0;

    //Count
    TCNT2 = 0x0;

    //PSRASY disabled; TSM disabled; 
    GTCCR = 0x0;

    //COM2A 0; COM2B 0; WGM2 2; 
    TCCR2A = 0x2;

    //CS2 RUNNING_CLK_8; FOC2A disabled; FOC2B disabled; WGM22 0; 
    TCCR2B = 0x2;

    //OCIE2A enabled; OCIE2B disabled; TOIE2 disabled; 
    TIMSK2 = 0x2;

    //AS2 disabled; EXCLK disabled; OCR2AUB disabled; OCR2BUB disabled; TCN2UB disabled; TCR2AUB disabled; TCR2BUB disabled; 
    ASSR = 0x0;

    return 0;
}

bool TC2_Get_Overflow_InterruptFlagStatus(void)
{
    if (TIFR2  & TOV2) {
        return true;
    } else {
        return false;
    }
}

bool TC2_Get_COM2A_InterruptFlagStatus(void)
{
    if (TIFR2  & OCF2A) {
        return true;
    } else {
        return false;
    }
}

bool TC2_Get_COM2B_InterruptFlagStatus(void)
{
    if (TIFR2  & OCF2B) {
        return true;
    } else {
        return false;
    }
}

void TC2_Enable_Overflow_Interrupt(void)
{
    TIMSK2 |= (1 << TOIE2);
}

void TC2_Disable_Overflow_Interrupt(void)
{
    TIMSK2 &= (1 << TOIE2);
}

void TC2_Enable_COM2A_Interrupt(void)
{
    TIMSK2 |= (1 << OCIE2A);
}

void TC2_Disable_COM2A_Interrupt(void)
{
    TIMSK2 &= (1 << OCIE2A);
}

void TC2_Enable_COM2B_Interrupt(void)
{
    TIMSK2 |= (1 << OCIE2B); 
}

void TC2_Disable_COM2B_Interrupt(void)
{
    TIMSK2 &= (1 << OCIE2B);
}

void TC2_WriteTimer(uint8_t timerValue)
{
    TCNT2 = timerValue;
}

uint8_t TC2_ReadTimer(void)
{
    return TCNT2;
}

void TMR2_SetHandler(void (*interruptHandler)(void)){
	TMR2Handler = interruptHandler;
}

ISR(TIMER2_COMPA_vect)
{
    //clears the Compare A interrupt flag
    TIFR2 = OCF2A;

	if(TMR2Handler != NULL){
		TMR2Handler();
	}

    /* Insert your Compare A interrupt handling code here */
    
}

