
/*
 * usart1.c
 *
 * Created: 2/26/2026 5:35:37 PM
 *  Author: sofia
 */ 


/**
  Section: Included Files
*/

#include "../header/usart1.h"


_uFlags_USART1 USART1;
_eUsartTXStates UsartTXstates, UsartRXstates;

uint8_t tx_idx = 0, rx_idx=0;
uint8_t TXdata=0 ,TXreg=0, RXreg=0, RXdata=0;
uint8_t tikRX;


/**
  Section: USART0  APIs
*/
void USART1_Initialize(){
	//Inicializacion puertos RX TX
	DDRD |= (1<<7); //PIN TX PUERTO D7 COMO SALIDA
	DDRD &= ~(1<<6);
	TX_HIGH();
	/* INTERRUPCION DE PIN*/
	PORTD |=  (1 << VRX); // Pull-up RX
	
	
	//-----INICIALIZACION REGISTRO BANDERAS USART1------
	USART1.aFlags_USART1=0;
	USART1.iFlags.TXEnable=1;
	USART1.iFlags.RXEnable=1;
	USART1.iFlags.TXready = 1;
	USART1.iFlags.RXdone=1;
	USART1.iFlags.RXready=0;
	USART1.iFlags.RXcomplete=0;
	
	//-- HABILITAR INTERPCION DE FLANCO RX
	// Grupo PCINT2 corresponde a PORTD
	PCICR  |= (1 << PCIE2);                // habilita grupo PCINT2
	#ifdef ISR_ENABLE
		CLEAR_RXPIN_F();
		ENABLE_RXPIN_ISR();
	#endif
	
	
	UsartTXstates = USARTIDLE;
	UsartRXstates = USARTIDLE;
	
	
}


void USART1_Start_bit_detect_ISR(){
	if(!USART1.iFlags.RXcomplete){ //no hay nada para leer
		if(USART1.iFlags.RXdone){
			UsartRXstates = START;
			tikRX = PERIOD05BR;
			USART1.iFlags.RXdone=0; //BUSY
		}
	}
}

//copiar funcion de arriba 
void USART1_Start_bit_detect(){
	if(USART1.iFlags.RXdone){
	//	if(PCIFR  &  (1 << PCIF2)){
		//	CLEAR_RXPIN_F();
			if(!(PIND & (1<<VRX))){
				if(!USART1.iFlags.RXcomplete){ //no hay nada para leer
					UsartRXstates = START;
					tikRX = PERIOD05BR;
					USART1.iFlags.RXdone=0; //BUSY
					}
				}
			//}
		}
}

void USART1_TRANSMIT(){
	switch(UsartTXstates){
		case USARTIDLE:
			USART1.iFlags.TXdone = 1;
			TX_HIGH();
		break;
		case START:
			TX_LOW();
			UsartTXstates = USARTDATA;
		break;
		case USARTDATA:
			if(TXreg & (1<<tx_idx)){
				TX_HIGH();
				}else{
				TX_LOW();
			}
			tx_idx++;
			if(tx_idx>7){
				UsartTXstates = STOP;
			}
		break;
		case STOP:
			TX_HIGH();
			UsartTXstates = USARTIDLE;
		break;
		default:
			UsartTXstates = USARTIDLE;
		break;
	}
}

void USART1_RECEIVE(){
		switch(UsartRXstates){
			case USARTIDLE:
				USART1.iFlags.RXdone = 1;
			
			break;
			case START:
				if(!(PIND & (1<<VRX))){ //si es 0 es bit start
					UsartRXstates = USARTDATA;
					rx_idx  = 0;
					}else{
					UsartRXstates = USARTIDLE; //falso bit start
					USART1.iFlags.RXdone = 1;
					#ifdef ISR_ENABLE
						CLEAR_RXPIN_F();
						ENABLE_RXPIN_ISR();
					#endif
				}

			break;
			case USARTDATA:
				if(PIND & (1<<VRX)){  //HIGH
					RXreg |= (1 << rx_idx);
					}else{				//LOW
					RXreg &= ~(1 << rx_idx);;
				}
				rx_idx++;
				if(rx_idx>7){
					UsartRXstates = STOP;
				}
			break;
			case STOP:
				if(PIND & (1<<VRX)){ //stop
					USART1.iFlags.RXcomplete = 1;
					if(!USART1.iFlags.TXready){
						RXdata  = RXreg;
						RXreg = 0;
						USART1.iFlags.RXready = 1;
						USART1.iFlags.RXcomplete = 0;
					}
				}else{
					//no se recibio nada descarto dato
					USART1.iFlags.RXcomplete = 0;
					RXreg = 0;
				}
				USART1.iFlags.RXdone=1;
				UsartRXstates = USARTIDLE;
				#ifdef ISR_ENABLE
					CLEAR_RXPIN_F();
					ENABLE_RXPIN_ISR();
				#endif
			break;
			default:
			
			break;
		}
}

void USART1_SendByte(){
	TXreg  = TXdata;
	TXdata = 0;
	tx_idx  = 0;
	USART1.iFlags.TXready = 1;
	USART1.iFlags.TXdone = 0;
	UsartTXstates = START;
}

void USART1_receiveByte(){
	RXdata  = RXreg;
	RXreg = 0;
	USART1.iFlags.RXready = 1;
	USART1.iFlags.RXcomplete = 0;
}

void USART1_Write(uint8_t b){
	TXdata = b;
	USART1.iFlags.TXready=0;
}

uint8_t USART1_Read(){
	USART1.iFlags.RXready = 0;      // DATO LEIDO
	return RXdata;
}