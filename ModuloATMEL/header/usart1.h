
/*
 * USART1.h
 *
 * Created: 2/26/2026 5:12:45 PM
 *  Author: sofia
 */ 

#ifndef USART1_H_
#define USART1_H_

/**
  Section: Included Files
 */

#include <stdbool.h>
#include <stdint.h>
#include "../mcc_generated_files/system/system.h"

#ifdef __cplusplus  // Provide C++ Compatibility
	extern "C" {
#endif

/*Banderas de control de USART*/
typedef union{
	struct{
		uint8_t TXEnable : 1;
		uint8_t RXEnable : 1;
		uint8_t RXready : 1; // 1 data receive 0 dato leido
		uint8_t TXready : 1; //si es 0 es busy
		uint8_t TXdone : 1;
		uint8_t RXdone : 1;
		uint8_t TXcomplete : 1;
		uint8_t RXcomplete : 1;
	} iFlags;
	uint8_t aFlags_USART1;
}_uFlags_USART1;
extern _uFlags_USART1 USART1;
extern uint8_t tikRX;

#define VRX	0x06 //PIND RX
#define VTX	0x07 //PORTD TX
#define TX_HIGH() (PORTD |=  (1<<VTX))
#define TX_LOW()  (PORTD &= ~(1<<VTX))
#define	ENABLE_RXPIN_ISR() (PCMSK2 |= (1 << PCINT22)); //habilita
#define	DISABLE_RXPIN_ISR() (PCMSK2 &= ~(1 << PCINT22));   // deshabilita PCINT en PD6
#define CLEAR_RXPIN_F()     (PCIFR  |=  (1 << PCIF2))     // limpia flag del grupo PCINT2 (PORTD)
#define PERIOD05BR		1
#define PERIODBR		2
#define USART1_TXENABLE		USART1.iFlags.TXEnable
#define USART1_RXENABLE		USART1.iFlags.RXEnable
#define USART1_RX_READY		USART1.iFlags.RXready
#define USART1_TX_READY		USART1.iFlags.TXready
#define USART1_RX_DONE		USART1.iFlags.RXdone
#define USART1_TX_DONE		USART1.iFlags.TXdone
#define USART1_TX_COMPLETE	USART1.iFlags.TXcomplete
#define USART1_RX_COMPLETE	USART1.iFlags.RXcomplete
//#define ISR_ENABLE  //Comentar esta linea si no se quiere interrupciones





typedef enum{
	USARTIDLE,
	START,
	USARTDATA,
	STOP,
}_eUsartTXStates;



void USART1_Initialize();

void USART1_Start_bit_detect_ISR();

void USART1_Start_bit_detect();

void USART1_TRANSMIT();

void USART1_RECEIVE();

void USART1_SendByte();

void USART1_receiveByte();

void USART1_Write(uint8_t b);

uint8_t USART1_Read();




#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif



#endif /* USART1_H_ */