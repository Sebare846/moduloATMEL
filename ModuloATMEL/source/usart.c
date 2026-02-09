/*
 * usart.c
 *
 * Created: 9/2/2026 10:49:35
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

/**
  Section: Included Files
*/

#include "../header/usart.h"

/**
  Section: Macro Declarations
*/



/**
  Section: Driver Interface
 */

const uart_drv_interface_t UART0 = {
    .Initialize = &USART0_Initialize,
    .Deinitialize = &USART0_Deinitialize,
    .Read = &USART0_Read,
    .Write = &USART0_Write,
    .IsRxReady = &USART0_IsRxReady,
    .IsTxReady = &USART0_IsTxReady,
    .IsTxDone = &USART0_IsTxDone,
    .TransmitEnable = &USART0_TransmitEnable,
    .TransmitDisable = &USART0_TransmitDisable,
    .AutoBaudSet = NULL,
    .AutoBaudQuery = NULL,
    .BRGCountSet = NULL,
    .BRGCountGet = NULL,
    .BaudRateSet = NULL,
    .BaudRateGet = NULL,
    .AutoBaudEventEnableGet = NULL,
    .ErrorGet = &USART0_ErrorGet,
    .TxCompleteCallbackRegister = NULL,
    .RxCompleteCallbackRegister = NULL,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = &USART0_FramingErrorCallbackRegister,
    .OverrunErrorCallbackRegister = &USART0_OverrunErrorCallbackRegister,
    .ParityErrorCallbackRegister = &USART0_ParityErrorCallbackRegister,
    .EventCallbackRegister = NULL,
};

/**
  Section: USART0 variables
*/
/* cppcheck-suppress misra-c2012-19.2 */
static volatile usart0_status_t usart0RxLastError;

/**
  Section: USART0 APIs
*/
static void (*USART0_FramingErrorHandler)(void);
static void (*USART0_OverrunErrorHandler)(void);
static void (*USART0_ParityErrorHandler)(void);

static void USART0_DefaultFramingErrorCallback(void);
static void USART0_DefaultOverrunErrorCallback(void);
static void USART0_DefaultParityErrorCallback(void);



/**
  Section: USART0  APIs
*/

void USART0_Initialize(void)
{
    // Set the USART0 module to the options selected in the user interface.

    //UBRR0 207; 
    UBRR0 = 0xCF;

    //MPCM0 disabled; TXC0 disabled; U2X0 enabled; 
    UCSR0A = 0x2;

    //RXB80 disabled; RXCIE0 disabled; RXEN0 enabled; TXB80 disabled; TXCIE0 disabled; TXEN0 enabled; UCSZ02 disabled; UDRIE0 disabled; 
    UCSR0B = 0x18;

    //UCPOL0 disabled; UCSZ0 3; UMSEL0 Asynchronous Mode; UPM0 Disabled; USBS0 1-bit; 
    UCSR0C = 0x6;


    /* cppcheck-suppress misra-c2012-8.7 */
    USART0_FramingErrorCallbackRegister(USART0_DefaultFramingErrorCallback);
    /* cppcheck-suppress misra-c2012-8.7 */
    USART0_OverrunErrorCallbackRegister(USART0_DefaultOverrunErrorCallback);
    /* cppcheck-suppress misra-c2012-8.7 */
    USART0_ParityErrorCallbackRegister(USART0_DefaultParityErrorCallback);
    usart0RxLastError.status = 0;  
}

void USART0_Deinitialize(void)
{

    //UBRR0 207; 
    UBRR0 = 0x00;

    //MPCM0 disabled; TXC0 disabled; U2X0 enabled; 
    UCSR0A = 0x00;

    //RXB80 disabled; RXCIE0 disabled; RXEN0 enabled; TXB80 disabled; TXCIE0 disabled; TXEN0 enabled; UCSZ02 disabled; UDRIE0 disabled; 
    UCSR0B = 0x00;

    //UCPOL0 disabled; UCSZ0 3; UMSEL0 Asynchronous Mode; UPM0 Disabled; USBS0 1-bit; 
    UCSR0C = 0x00;

}

void USART0_Enable(void)
{
    UCSR0B |= ( 1 << RXEN0 ) | ( 1 << TXEN0 );
}

void USART0_Disable(void)
{
    UCSR0B &= ~(( 1 << RXEN0 ) | ( 1 << TXEN0 ));
}

void USART0_TransmitEnable(void)
{
    UCSR0B |= ( 1 << TXEN0 );
}

void USART0_TransmitDisable(void)
{
    UCSR0B &= ~( 1 << TXEN0 );
}

void USART0_ReceiveEnable(void)
{
    UCSR0B |= ( 1 << RXEN0 );
}

void USART0_ReceiveDisable(void)
{
    UCSR0B &= ~( 1 << RXEN0 );
}

uint8_t USART0_GetData(void)
{
    return UDR0;
}

bool USART0_IsRxReady(void)
{
    return (bool)(UCSR0A & ( 1 << RXC0 ));
}

bool USART0_IsTxReady(void)
{
    return (bool)(UCSR0A & ( 1 << UDRE0 ));
}

static bool UART0_IsTxBusy(void)
{
    return (!(UCSR0A & ( 1 << TXC0 )));
}

bool USART0_IsTxDone(void)
{
    return (bool)(UCSR0A & ( 1U << TXC0 ));
}

size_t USART0_ErrorGet(void)
{
    usart0RxLastError.status = 0;
    
    if(true == (bool)(UCSR0A & (1U << FE0)))
    {
        usart0RxLastError.ferr = 1;
        if(NULL != USART0_FramingErrorHandler)
        {
            USART0_FramingErrorHandler();
        }  
    }
    if(true == (bool)(UCSR0A & (1U << UPE0)))
    {
        usart0RxLastError.perr = 1;
        if(NULL != USART0_ParityErrorHandler)
        {
            USART0_ParityErrorHandler();
        }  
    }
    if(true == (bool)(UCSR0A & (1U << DOR0)))
    {
        usart0RxLastError.oerr = 1;
        if(NULL != USART0_OverrunErrorHandler)
        {
            USART0_OverrunErrorHandler();
        }   
    }
    return usart0RxLastError.status;
}

uint8_t USART0_Read(void)
{
    return UDR0;
}


void USART0_Write(uint8_t txData)
{
    UDR0 = txData;    // Write the data byte to the USART.
}
static void USART0_DefaultFramingErrorCallback(void)
{
    
}

static void USART0_DefaultOverrunErrorCallback(void)
{
    
}

static void USART0_DefaultParityErrorCallback(void)
{
    
}

/* cppcheck-suppress misra-c2012-8.7 */
void USART0_FramingErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART0_FramingErrorHandler = callbackHandler;
    }
}

/* cppcheck-suppress misra-c2012-8.7 */
void USART0_OverrunErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART0_OverrunErrorHandler = callbackHandler;
    }    
}

/* cppcheck-suppress misra-c2012-8.7 */
void USART0_ParityErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART0_ParityErrorHandler = callbackHandler;
    } 
}
