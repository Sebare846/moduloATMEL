 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.2
 *
 * @version Package Version: 3.1.2
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
#include "mcc_generated_files/system/system.h"
#include "header/timer0.h"
#include "header/timer1.h"
#include "header/usart.h"
#include "header/modbus.h"
#include <stdint.h>
#include <util/delay.h>

//----------------------------- Definiciones tiempos - base 10ms -----------------------------
#define PERIOD100MS		10  
#define PERIOD250MS		25
#define PERIOD500MS		50
#define PERIOD1000MS	100
#define PERIOD10S		1000
#define PERIOD5S		500
//----------------------------------------- Mascaras -----------------------------------------
#define MASK_PORTC	0x0F
#define MASK_PORTB	0x1E
#define MASK_IN1	0x01
#define MASK_IN2	0x02
#define MASK_IN3	0x04
#define MASK_IN4	0x08
//---------------------------------------- Direcciones ----------------------------------------
#define START_SLAVE_ADDRESS		0x3FE
#define SLAVE_ADDRESS			0x3FF
#define START_CONFIG_ADDRESS	0x400
#define END_CONFIG_ADDRESS_R	0x411
#define END_CONFIG_ADDRESS_W	0x40D
#define START_EVENT_ADDRESS		0x772
#define END_EVENT_ADDRESS		0x89E
//----------------------------------------- Tamanios ------------------------------------------
#define BUFFER_LEN  256
#define REGISTERS   12
#define MAXEVENTS	50

/*
	Union para transformacion de tipos de datos
*/
typedef union{
	uint32_t u32;
	uint16_t ui16[2];
	uint8_t ui8[4];
}_uWord;

_uWord myWord;

/*
	Mapa de banderas para manejo de lectura de entradas digitales
*/
typedef union{
	struct{
		uint8_t isEvent1 : 1;
		uint8_t isEvent2 : 1;
		uint8_t isEvent3 : 1;
		uint8_t isEvent4 : 1;
		uint8_t RESERVED : 4;
	} iFlags;
	uint8_t aFlags_input;
}_uFlags_input;

_uFlags_input inputFlags;

/*
	Estructura de Evento
	* ID			: Numero de entrada
	* stateLastEvent: Estado de la entrada leida
	* timeLastEvent	: Tiempo del evento en formato unix
*/
typedef struct {
	uint8_t  ID;
	uint8_t  stateLastEvent;
	uint32_t timeLastEvent;
}_sEvent __attribute__((aligned(1)));

_sEvent myEvents[MAXEVENTS] __attribute__((section(".fixed_events"))); //772 espacio 12C

//-------------------------------------- Variables MODBUS -------------------------------------
_eDecodeStates decodeState;
_sModbusFrame myModbusFrame;
_uCommFlags commFlags ; //banderas programa USART
uint16_t  t15Tik=0, t35Tik=0;
uint8_t auxbrConfig=0, auxParity=0;
//---------------------------------- Variables Comunicacion -----------------------------------
uart_drv_interface_t myUSARTHandler;
volatile uint8_t bufferRx[BUFFER_LEN];
uint8_t bufferTx[BUFFER_LEN];
uint8_t indexRxR,indexRxW,indexTxR,indexTxW, trashByte;
//------------------------------------------ Tickers ------------------------------------------
uint8_t hbTime=0,unixTik=0;
uint16_t auxTik=0;
//------------------------------- Variables captura de eventos --------------------------------
uint16_t tikBetweenTime[4];
uint8_t	tikDebounce[4];
uint8_t oldValueA=0, newValueA=0, outValues=0, lecture=0;
//------------------------ Banco de Registros (con direcciones fijas) -------------------------
uint16_t EventIndexW=0,EventIndexR=0;
uint8_t is500ms=0,isDecodeData=0; //banderas sacadas de input flags

uint8_t slaveAddress __attribute__((section(".mySlaveAddress"))); //0x3FF
typedef struct {
	uint32_t unixTime; //400 401 402 403
	uint8_t brConfig; //404
	uint8_t parity; //405
	uint16_t timeBtw[4];//tiempos configurables //406 407 408 409 40A 40B 40C 40D
	uint16_t diagnosticsRegister; //40E 40F
	uint16_t newEventCounter; //410 411
} _sModbusReg;

// Colocar en sección específica
_sModbusReg modbusRegister __attribute__((section(".modbus_r"))); //a partir de 0x400 por los common

typedef enum{
	SLAVE_EEPROM_ADDRESS = 1,
	BRCONFIG_EEPROM_ADDRESS,
	PARITY_EEPROM_ADDRESS,
	TIMEBTW0_LOW_EEPROM_ADDRESS,
	TIMEBTW0_HIGH_EEPROM_ADDRESS,
	TIMEBTW1_LOW_EEPROM_ADDRESS,
	TIMEBTW1_HIGH_EEPROM_ADDRESS,
	TIMEBTW2_LOW_EEPROM_ADDRESS,
	TIMEBTW2_HIGH_EEPROM_ADDRESS,
	TIMEBTW3_LOW_EEPROM_ADDRESS,
	TIMEBTW3_HIGH_EEPROM_ADDRESS,
	}_eEEPROM_Address;



// ---------------------- Prototipos de Funciones ----------------------
/*
	brief EEPROM_Write	: 
	param [in] address	: 
	param [in] data		:
	return				:
*/
int8_t EEPROM_Write(uint16_t address, uint8_t data);
/*
	brief EEPROM_Read	:
	param [in] address	:
	return				:
*/
int16_t EEPROM_Read(uint16_t address);
/*
	brief EEPROM_Erase	:
	param [in] address	:
	return				:
*/
int8_t EEPROM_Erase(uint16_t address);
/*
	brief UARTsendByte :
*/
void UARTsendByte();
/*
	brief MODBUS_DecodeFrame	:
	param [in] data				:
*/
void MODBUS_DecodeFrame(uint8_t data);
/*
	brief MODBUS_ProcessFunction :
*/
void MODBUS_ProcessFunction();
/*
	brief MODBUS_SendExceptionCode :

*/
void MODBUS_SendExceptionCode();
/*
	brief MODBUS_CalculateCRC	:
	param [in] data				:
*/
void MODBUS_CalculateCRC(uint8_t data);
/*
	brief do10us :
*/
void do10us();
/*
	brief do10ms :
*/
void do10ms();
/*
	brief RestartTikValues : 

*/
void RestartTikValues();

/*
	brief RegInput :
*/
void RegInput();
/*
	brief UpdateState	:
	param [in] lecture	:
	param [in] mask		:
	param [in] indice	:
*/
void UpdateState(uint8_t lecture, uint8_t mask, uint8_t indice);

void SentEvents(uint16_t count);
void InitializeEvents();


// ---------------------- Implementacion de ISR ----------------------
ISR(USART_RX_vect){
	
	RestartTikValues(); 
	MODBUS_DecodeFrame(UDR0);
}

void UARTsendByte(){
	while(indexTxR != indexTxW){
		if (USART0_IsTxReady()) {
			USART0_Write(bufferTx[indexTxR++]);
		}
	}
}

void USART0_ReceiveInterruptEnable(void){
	UCSR0B |= (1 << (RXCIE0));
}

void USART0_ReceiveInterruptDisable(void){
	UCSR0B &= ~(1 << (RXCIE0));
}

int8_t EEPROM_Write(uint16_t address, uint8_t data) {
	//Validar direccion o da la vuelta
	if(address < 1 || address > 1023){
		return -1;
	}	
	while(EECR & (1<<EEPE)); //Este bit deben estar en 0.	
	cli();
	//Direccion del registro
	EEARL = address;	
	//Dato a guardar
	EEDR = data;
	//Configurar a escritura seteando EEMPE
	EECR |= (1<<EEMPE);
	//Iniciar la escritura seteando EEPE 
	EECR |= (1<<EEPE);
	//Restaurar interrupciones
	sei();
	
	return 1;
}

int16_t EEPROM_Read(uint16_t address) {
	//Validar direccion o da la vuelta
	if(address < 1 || address > 1023){
		return -1;
	}
	while(EECR & (1<<EEPE)); //Estos bits deben estar en 0.
	//Direccion del registro
	EEAR = address;
	//Leer datos
	EECR |= (1 << EERE);
	//devolver dato leido
	return EEDR;
}

void RestartTikValues(){
	
	if(auxbrConfig){
		if(auxParity){
			t15Tik = T15_BR11;
			t35Tik = T35_BR11;
			}else{
			t15Tik = T15_BR10;
			t35Tik = T35_BR10;
		}
	}else{
		if(auxParity){
			t15Tik = T15_BR01;
			t35Tik = T35_BR01;
			}else{
			t15Tik = T15_BR00;
			t35Tik = T35_BR00;
		}
	}
}

void do10us(){
	if(t15Tik){
		t15Tik--;
	}else if((decodeState != IDLE) && (decodeState != DISCARD)){
		decodeState = DISCARD;
	}
	if(t35Tik){
		t35Tik--;
	}else{
		decodeState = IDLE;
		indexRxR = indexRxW;
	}
}

void do10ms(){
	hbTime--;
	commFlags.iFlags.isEvent1 = 1;
	unixTik--;
	if(!unixTik){ //esto no se si aca o en el main idk
		if(is500ms){
			modbusRegister.unixTime++ ;				
			is500ms = 0;
		}else{
			is500ms=1;
		}
		unixTik = PERIOD250MS;
	}
}

void InitializeEvents(){
	 for(int i = 0; i < MAXEVENTS; i++) {
		  myEvents[i].ID = 0;
		  myEvents[i].stateLastEvent = 0;
		  myEvents[i].timeLastEvent = 0;
	 }
}

void RegInput(){
		
	    newValueA = ~(PINC & MASK_PORTC);
	    inputFlags.aFlags_input = (newValueA ^ oldValueA);
	    
	    if(tikBetweenTime[0]){
		    tikBetweenTime[0]--;
		}else{
		    if(inputFlags.iFlags.isEvent1){
			    tikDebounce[0] = PERIOD100MS;
			    lecture = (lecture & ~(MASK_IN1)) | (newValueA & MASK_IN1);
			    }else{
			    if(tikDebounce[0]){
				    tikDebounce[0]--;
				    if(tikDebounce[0] == 0){
					    UpdateState(lecture, MASK_IN1, 0);
				    }
			    }
		    }
	    }
	    
	    if(tikBetweenTime[1]){
		    tikBetweenTime[1]--;
		    }else{
		    if(inputFlags.iFlags.isEvent2){
			    tikDebounce[1] = PERIOD100MS;
			    lecture = (lecture & ~(MASK_IN2)) | (newValueA & MASK_IN2);
			    }else{
			    if(tikDebounce[1]){
				    tikDebounce[1]--;
				    if(tikDebounce[1] == 0){
					    UpdateState(lecture, MASK_IN2, 1);
				    }
			    }
		    }
	    }
	    
	    if(tikBetweenTime[2]){
		    tikBetweenTime[2]--;
		    }else{
		    if(inputFlags.iFlags.isEvent3){
			    tikDebounce[2] = PERIOD100MS;
			    lecture = (lecture & ~(MASK_IN3)) | (newValueA & MASK_IN3);
			}else{
			    if(tikDebounce[2]){
				    tikDebounce[2]--;
				    if(tikDebounce[2] == 0){
					    UpdateState(lecture, MASK_IN3, 2);
				    }
			    }
		    }
	    }
	    
	    if(tikBetweenTime[3]){
		    tikBetweenTime[3]--;
		    }else{
		    if(inputFlags.iFlags.isEvent4){
			    tikDebounce[3] = PERIOD100MS;
			    lecture = (lecture & ~(MASK_IN4)) | (newValueA & MASK_IN4);
			    }else{
			    if(tikDebounce[3]){
				    tikDebounce[3]--;
				    if(tikDebounce[3] == 0){
					    UpdateState(lecture, MASK_IN4, 3);
				    }
			    }
		    }
	    }
	    
	    oldValueA = newValueA;
	    
	    TC0_Enable_CompareA_Interrupt();
	    
	    commFlags.iFlags.isEvent1 = 0;
    }

 void UpdateState(uint8_t lect, uint8_t mask, uint8_t indice){
	
	  if(((~PINC) & mask) == (lect & mask)){ //Probar asi y sino ~(PORTA & mask)
		  if(lect & mask){
			  outValues |= mask; 
			  myEvents[EventIndexW].stateLastEvent = 1;
		   }else{
			  if(outValues & mask){
				  tikBetweenTime[indice] = modbusRegister.timeBtw[indice];
			  }
			 outValues &= ~mask;
			 myEvents[EventIndexW].stateLastEvent = 0;
  
		    }
	    }
		//new event y index cosas separadas, new events se debe reiniciar cada que se envian esa cantidad de datos
		myEvents[EventIndexW].ID = indice;
	   // myEvents[EventIndexW].stateLastEvent = outValues & mask;
	    myEvents[EventIndexW].timeLastEvent = modbusRegister.unixTime;
		EventIndexW++;
	    modbusRegister.newEventCounter++;
	    if(EventIndexW == MAXEVENTS){
		    EventIndexW = 0;
	    }
    
}

/*--------------------------------*/
void SentEvents(uint16_t count){//LoadEvents
	for (uint8_t i = 0; i<count; i++ ){
		if(EventIndexR == MAXEVENTS){
			EventIndexR=0;
		}
		bufferTx[indexTxW++] = myEvents[EventIndexR].ID;
		bufferTx[indexTxW++] = myEvents[EventIndexR].stateLastEvent;
		//SEPARAR 
		myWord.u32 = myEvents[EventIndexR].timeLastEvent;
		bufferTx[indexTxW++] = myWord.ui8[0];
		bufferTx[indexTxW++] = myWord.ui8[1];
		bufferTx[indexTxW++] = myWord.ui8[2];
		bufferTx[indexTxW++] = myWord.ui8[3];
		
		EventIndexR++;
	}
}

void MODBUS_CalculateCRC(uint8_t data){//, uint16_t crc){
	myModbusFrame.crcSlave ^= (uint16_t)data;
	for (uint8_t i = 0; i < 8; i++)
	{
		if (myModbusFrame.crcSlave & 0x0001)
		myModbusFrame.crcSlave = (myModbusFrame.crcSlave >> 1) ^ 0xA001;
		else
		myModbusFrame.crcSlave >>= 1;
	}
}

// Funcion para recibir un byte a traves de UART leido
void MODBUS_DecodeFrame(uint8_t data){
	switch(decodeState){
		case IDLE:
			if(data == MODBUS_BROADCAST_ADDRESS || data == MODBUS_SLAVE_ADDRESS){
				if(data == MODBUS_BROADCAST_ADDRESS) commFlags.iFlags.isBroadcast = 1;
				myModbusFrame.nBytes = 5;
				myModbusFrame.crcSlave = 0xFFFF;
				MODBUS_CalculateCRC(data);
				decodeState = FUNCTION;			
			}else{
				decodeState = DISCARD;
			}
		break;
		case FUNCTION:
			myModbusFrame.function = data;
			MODBUS_CalculateCRC(data);
			myModbusFrame.nBytes--;
			if(data == WRITE_MULTIPLE_REGISTERS) myModbusFrame.nBytes++; 
			decodeState = DATA;
		break;
		case DATA:
			bufferRx[indexRxW++] = data;
			MODBUS_CalculateCRC(data);
			myModbusFrame.nBytes--;
			if(myModbusFrame.nBytes == 0){
				if(myModbusFrame.function == WRITE_MULTIPLE_REGISTERS){
					myModbusFrame.nBytes = data;
					decodeState = EXTRADATA;
				}else{
					decodeState = CRCL;
				}
			}
			
		break;
		case EXTRADATA:
			bufferRx[indexRxW++] = data;
			MODBUS_CalculateCRC(data);
			myModbusFrame.nBytes--;
			if(myModbusFrame.nBytes == 0){
				decodeState = CRCL;
			}
		break;
		case CRCL:
			myModbusFrame.crcMaster = data;
			decodeState = CRCH;
		break;
		case CRCH:
			myModbusFrame.crcMaster |= (uint16_t)(data << 8);
			if(myModbusFrame.crcSlave == myModbusFrame.crcMaster){
				commFlags.iFlags.dataReady = 1;
			}else{
				indexRxR = indexRxW;
			}
			decodeState = IDLE;
		break;
		case DISCARD: 
			trashByte = data;
		break;
		default:
			decodeState = IDLE;
			indexRxR = indexRxW;
		break;
	}

}

void MODBUS_ProcessFunction(){
	uint8_t countIndex, byteCount = 0;
	uint16_t regAddress, regValue, regCant, subfunction;
	myModbusFrame.crcSlave = 0xFFFF;
	switch(myModbusFrame.function){
		case READ_HOLDING_REGISTERS: //Ver si conviene copiar valores en RAM o dejarlos solo en EEPROM
			myModbusFrame.errorCode = 0x83;
			if(commFlags.iFlags.isBroadcast){
				commFlags.iFlags.isBroadcast = 0;
				return;
			}
			//Direccion inicial
			myWord.ui8[1] = bufferRx[indexRxR++];
			myWord.ui8[0] = bufferRx[indexRxR++];
			regAddress = myWord.ui16[0];
			//Cantidad de registros
			myWord.ui8[1] = bufferRx[indexRxR++];
			myWord.ui8[0] = bufferRx[indexRxR++];
			regCant = myWord.ui16[0];
			
			//CONTROLAR CANTIDAD 
			if(regCant<1 || regCant > 0x07B){ //error cantidad
				myModbusFrame.excepCode = INVALID_DATA_VALUE;
			    commFlags.iFlags.isExceptionCode = 1;
				return;
			}
			//Controlar validez direcciones
			if((regAddress < START_CONFIG_ADDRESS) || (regAddress>END_CONFIG_ADDRESS_R && regAddress <START_EVENT_ADDRESS) || regAddress > END_EVENT_ADDRESS){
				myModbusFrame.excepCode = INVALID_DATA_ADDRESS;
				commFlags.iFlags.isExceptionCode = 1;
				return;
			}			
			if(regAddress>=START_CONFIG_ADDRESS && regAddress <= END_CONFIG_ADDRESS_R){
				if((regAddress+((regCant -1)*2) > END_CONFIG_ADDRESS_R)){
					//error address
					myModbusFrame.excepCode = INVALID_DATA_ADDRESS;
					commFlags.iFlags.isExceptionCode = 1;
					return;
				}

			}
			if(regAddress>=START_EVENT_ADDRESS && regAddress <= END_EVENT_ADDRESS){
				if((regAddress + (regCant -1)*2) > END_EVENT_ADDRESS){
					myModbusFrame.excepCode = INVALID_DATA_ADDRESS;
					commFlags.iFlags.isExceptionCode = 1;
					return;
				}
			}
			
			// Cargar mensaje de respuesta
			bufferTx[indexTxW++] = slaveAddress;
			bufferTx[indexTxW++] = READ_HOLDING_REGISTERS;
			//////if para load eventos
			countIndex = indexTxW;
			bufferTx[indexTxW++] = 0x00; // Almacenar en la pocision de cantidad de bytes
			for(uint8_t i=0 ; i<(regCant*2) ; i++){
				bufferTx[indexTxW++] = *((uint8_t *)(regAddress + i)); //volatil
				byteCount++;
			}
			bufferTx[countIndex] = byteCount;
			for(uint8_t i = indexTxR; i < indexTxW + 3;i++){
				MODBUS_CalculateCRC(bufferTx[i]);
			}
			myWord.ui16[0] = myModbusFrame.crcSlave;
			bufferTx[indexTxW++] = myWord.ui8[0];
			bufferTx[indexTxW++] = myWord.ui8[1];
		break;
		case WRITE_SINGLE_REGISTER:
			myModbusFrame.errorCode = 0x86;
			
			//Direccion inicial
			myWord.ui8[1] = bufferRx[indexRxR++];
			myWord.ui8[0] = bufferRx[indexRxR++];
			regAddress = myWord.ui16[0];
			//Cantidad de registros
			myWord.ui8[1] = bufferRx[indexRxR++];
			myWord.ui8[0] = bufferRx[indexRxR++];
			regValue = myWord.ui16[0];
			
			//controlar direccion
			if((regAddress < START_SLAVE_ADDRESS) || (regAddress == SLAVE_ADDRESS) || (regAddress>END_CONFIG_ADDRESS_W && regAddress <START_EVENT_ADDRESS) || regAddress > END_EVENT_ADDRESS){
				myModbusFrame.excepCode = INVALID_DATA_ADDRESS;
				commFlags.iFlags.isExceptionCode = 1;
				return;
			}
			
			*((uint8_t *)(regAddress)) = myWord.ui8[1];
			*((uint8_t *)(regAddress+1)) = myWord.ui8[0];
		
			if(commFlags.iFlags.isBroadcast){
				commFlags.iFlags.isBroadcast = 0;
				return;
			}
		
			bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
			bufferTx[indexTxW++] = WRITE_SINGLE_REGISTER;
			myWord.ui16[0] = regAddress;
			bufferTx[indexTxW++] = myWord.ui8[1];
			bufferTx[indexTxW++] = myWord.ui8[0];
			bufferTx[indexTxW++] = *((uint8_t *)(regAddress));
			bufferTx[indexTxW++] = *((uint8_t *)(regAddress+1));
		
			for(uint8_t i = indexTxR; i < indexTxW;i++){
				MODBUS_CalculateCRC(bufferTx[i]);
			}
			myWord.ui16[0] = myModbusFrame.crcSlave;
			bufferTx[indexTxW++] = myWord.ui8[0];
			bufferTx[indexTxW++] = myWord.ui8[1];
		break;
		case DIAGNOSTICS:
			myModbusFrame.errorCode = 0x88;
			if(commFlags.iFlags.isBroadcast){
				commFlags.iFlags.isBroadcast = 0;
				return;
			}
			//Subfuncion
			myWord.ui8[1] = bufferRx[indexRxR++];
			myWord.ui8[0] = bufferRx[indexRxR++];
			subfunction = myWord.ui16[0];
			myWord.ui8[1] = bufferRx[indexRxR++];
			myWord.ui8[0] = bufferRx[indexRxR++];
			regValue = myWord.ui16[0];
			
			switch(subfunction){
				case RETURN_QUERY_DATA:
				//Eco
				break;
				case RESTART_COMMUNICATIONS:
					if(regValue == 0x0000 || regValue == 0xFF00){
						//Eco
						commFlags.iFlags.restartComms = 1; ////
						}else{
						myModbusFrame.excepCode = INVALID_DATA_VALUE;
						commFlags.iFlags.isExceptionCode = 1;
						return;
					}
				break;
				case RETURN_DIAGNOSTICS_REGISTER:
					if(regValue == 0x0000){
						//Registro de diagnostico
						//regValue  = diagnosticsRegister;
						// regValue |= diagnosticsRegister;
						}else{
						myModbusFrame.excepCode = INVALID_DATA_VALUE;
						commFlags.iFlags.isExceptionCode = 1;
						return;
					}
				break;
				default:
					myModbusFrame.excepCode = INVALID_FUNCTION;
					commFlags.iFlags.isExceptionCode = 1;
					indexRxR = indexRxW;
				return;
			}
				bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
				bufferTx[indexTxW++] = DIAGNOSTICS;
				myWord.ui16[0] = subfunction;
				bufferTx[indexTxW++] = myWord.ui8[1];
				bufferTx[indexTxW++] = myWord.ui8[0];
				myWord.ui16[0] = regValue;
				bufferTx[indexTxW++] = myWord.ui8[1];
				bufferTx[indexTxW++] = myWord.ui8[0];
				for(uint8_t i=indexTxR; i < indexTxW; i++){
					MODBUS_CalculateCRC(bufferTx[i]);
				}
				myWord.ui16[0] = myModbusFrame.crcSlave;
				bufferTx[indexTxW++] = myWord.ui8[0];
				bufferTx[indexTxW++] = myWord.ui8[1];
				break;
		case WRITE_MULTIPLE_REGISTERS: //Incompleto
			myModbusFrame.errorCode = 0x90;
			//Direccion inicial
			myWord.ui8[1] = bufferRx[indexRxR++];
			myWord.ui8[0] = bufferRx[indexRxR++];
			regAddress = myWord.ui16[0];
			//Cantidad de registros
			myWord.ui8[1] = bufferRx[indexRxR++];
			myWord.ui8[0] = bufferRx[indexRxR++];
			regCant = myWord.ui16[0];
			//Cantidad de bytes
			byteCount = bufferRx[indexRxR++];
		
			//Controlar cantidad de registros
			if((regCant < 1 || regCant > 0x007B) || (byteCount != (regCant*2))){
				myModbusFrame.excepCode = INVALID_DATA_VALUE;
				commFlags.iFlags.isExceptionCode = 1;
				return;
			}

			//Controlar validez direcciones
			if((regAddress < START_CONFIG_ADDRESS) || (regAddress>END_CONFIG_ADDRESS_W)){
				myModbusFrame.excepCode = INVALID_DATA_ADDRESS;
				commFlags.iFlags.isExceptionCode = 1;
				return;
			}else{
				if(regAddress+((regCant -1)*2) >=END_CONFIG_ADDRESS_W){
					//error address
					myModbusFrame.excepCode = INVALID_DATA_ADDRESS;
					commFlags.iFlags.isExceptionCode = 1;
					return;
				}
			}

			for(uint8_t i=0; i<(regCant*2);i++){
				*((uint8_t *)(regAddress+i)) = bufferRx[indexRxR++];
			}
		
			if(commFlags.iFlags.isBroadcast){
				commFlags.iFlags.isBroadcast = 0;
				return;
			}
			bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
			bufferTx[indexTxW++] = WRITE_MULTIPLE_REGISTERS;
			myWord.ui16[0] = regAddress;
			bufferTx[indexTxW++] = myWord.ui8[1];
			bufferTx[indexTxW++] = myWord.ui8[0];
			myWord.ui16[0] = regCant;
			bufferTx[indexTxW++] = myWord.ui8[1];
			bufferTx[indexTxW++] = myWord.ui8[0];
			for(uint8_t i = indexTxR; i < indexTxW;i++){
				MODBUS_CalculateCRC(bufferTx[i]);
			}
			myWord.ui16[0] = myModbusFrame.crcSlave;
			bufferTx[indexTxW++] = myWord.ui8[0];
			bufferTx[indexTxW++] = myWord.ui8[1];
		break;
		default:
			if(commFlags.iFlags.isBroadcast){
				return;
			}
		myModbusFrame.excepCode = INVALID_FUNCTION;
		commFlags.iFlags.isExceptionCode = 1;
		indexRxR = indexRxW;
		break;
	}
	 
}

void MODBUS_SendExceptionCode(){
	commFlags.iFlags.isExceptionCode = 0;
	bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
	bufferTx[indexTxW++] = myModbusFrame.errorCode;
	bufferTx[indexTxW++] = myModbusFrame.excepCode;
	for(uint8_t i = indexTxR; i < indexTxW;i++){
		MODBUS_CalculateCRC(bufferTx[i]);
	}
	myWord.ui16[0] = myModbusFrame.crcSlave;
	bufferTx[indexTxW++] = myWord.ui8[0];
	bufferTx[indexTxW++] = myWord.ui8[1];
}

/*
    Main application
*/

int main(void){
	
	SYSTEM_Initialize();
	
	uint8_t auxSlaveAddress, hbState = 0; 
	uint16_t auxtimeBtw[4];
	
	//LEER DE EEPROM
	slaveAddress = EEPROM_Read(SLAVE_EEPROM_ADDRESS);
	modbusRegister.brConfig = EEPROM_Read(BRCONFIG_EEPROM_ADDRESS);
	modbusRegister.parity = EEPROM_Read(PARITY_EEPROM_ADDRESS);

	myWord.ui8[0] = EEPROM_Read(TIMEBTW0_LOW_EEPROM_ADDRESS);
	myWord.ui8[1] = EEPROM_Read(TIMEBTW0_HIGH_EEPROM_ADDRESS);
	modbusRegister.timeBtw[0]= myWord.ui16[0];
	myWord.ui8[1] = EEPROM_Read(TIMEBTW1_HIGH_EEPROM_ADDRESS);
	myWord.ui8[0] = EEPROM_Read(TIMEBTW1_LOW_EEPROM_ADDRESS);
	modbusRegister.timeBtw[1]= myWord.ui16[0];
	myWord.ui8[1] = EEPROM_Read(TIMEBTW2_HIGH_EEPROM_ADDRESS);
	myWord.ui8[0] = EEPROM_Read(TIMEBTW2_LOW_EEPROM_ADDRESS);
	modbusRegister.timeBtw[2]= myWord.ui16[0];
	myWord.ui8[1] = EEPROM_Read(TIMEBTW3_HIGH_EEPROM_ADDRESS);
	myWord.ui8[0] = EEPROM_Read(TIMEBTW3_LOW_EEPROM_ADDRESS);
	modbusRegister.timeBtw[3]= myWord.ui16[0];

	auxSlaveAddress = slaveAddress;
	auxbrConfig = modbusRegister.brConfig;
	auxParity = modbusRegister.parity;
	auxtimeBtw[0] = modbusRegister.timeBtw[0];	
	auxtimeBtw[1] = modbusRegister.timeBtw[1];
	auxtimeBtw[2] = modbusRegister.timeBtw[2];
	auxtimeBtw[3] = modbusRegister.timeBtw[3];
	
	if(slaveAddress == 0xFF){
		slaveAddress = 0x02;
		auxSlaveAddress = slaveAddress;
	}
	if(modbusRegister.brConfig == 0xFF){
		modbusRegister.brConfig = 0x00;
		auxbrConfig = modbusRegister.brConfig;
	}
	if(modbusRegister.parity == 0xFF){
		modbusRegister.parity = 0x01;
		auxParity = modbusRegister.parity;
	}
	if(modbusRegister.timeBtw[0] == 0xFFFF){
		modbusRegister.timeBtw[0] = 0x00;
		auxtimeBtw[0] = modbusRegister.timeBtw[0];
	}
	if(modbusRegister.timeBtw[1] == 0xFFFF){
		modbusRegister.timeBtw[1] = 0x00;
		auxtimeBtw[1] = modbusRegister.timeBtw[1];
	}
	if(modbusRegister.timeBtw[2] == 0xFFFF){
		modbusRegister.timeBtw[2] = 0x00;
		auxtimeBtw[2] = modbusRegister.timeBtw[2];
	}
	if(modbusRegister.timeBtw[3] == 0xFFFF){
		modbusRegister.timeBtw[3] = 0x00;
		auxtimeBtw[3] = modbusRegister.timeBtw[3];
	}
	modbusRegister.unixTime				= 1770766800;
	modbusRegister.diagnosticsRegister	= 0x3333; //Ver que valor darle que nos sirva
	modbusRegister.newEventCounter		= 0x0000;

	
	
	decodeState = IDLE;
	hbTime = PERIOD250MS;
	auxTik = 25000;
	commFlags.aFlags = 0;
	commFlags.iFlags.silenceTime = 1;
	indexTxW = 0;
	indexTxR = 0;
	indexRxW = 0;
	indexRxR = 0;
		
	unixTik = PERIOD500MS;

	/*-----LECTURA ENTRADAS-----------------------*/
	newValueA = ~(PORTC & MASK_PORTC);
	outValues = newValueA;
	inputFlags.aFlags_input = 0;
	commFlags.aFlags = 0;
	is500ms=0;

	tikBetweenTime[0] = 0;
	tikBetweenTime[1] = 0;
	tikBetweenTime[2] = 0;
	tikBetweenTime[3] = 0;
		

	USART0_Initialize(modbusRegister.brConfig, modbusRegister.parity);
	TC0_Initialize();
	TC1_Initialize();
	InitializeEvents();
	
	sei();
	
	TMR0_SetHandler(do10ms);
	TMR1_SetHandler(do10us);
	USART0_ReceiveInterruptEnable();

	
	while(1){
		
			if(!hbTime){ //Heartbeat
			//	PORTB ^= (1<<5);
				hbState ^= (1<<5);
				hbTime = PERIOD250MS;
			}
					 
			if(commFlags.iFlags.dataReady){ //DECODIFICA SI EL FRAME SE RECIBIO CORRECTAMENTE
				commFlags.iFlags.dataReady = 0;
				MODBUS_ProcessFunction();
			}

			if(indexTxR != indexTxW){ //Si hay datos para transmitir, transmite todo lo que tiene
				USART0_ReceiveInterruptDisable();
				PORTD |= 0x4; //MAX HIGH
				UARTsendByte();
			}
			if(indexTxR == indexTxW){ //Si no hay datos para transmitir vuelve a la interrupcion
				if(USART0_IsTxDone()){ //IS TRANSMIT COMPLETE
					UCSR0A |= (1<<TXC0);           // limpiar TXC0
					USART0_ReceiveInterruptEnable();
					PORTD &= ~ 0x4; //MAX_SetLow();
				}
			}

			if(commFlags.iFlags.isExceptionCode){ 
				MODBUS_SendExceptionCode();
			}
 
			if(commFlags.iFlags.restartComms){ //resetear usart segun baudrate y paridad
				//GUARDAR TODOS LOS REGISTROS EN EEPROM Y RESETEAR USART
				//modbusRegister.brConfig = 0x00;
				//modbusRegister.parity = 0x00;
						if(auxSlaveAddress != slaveAddress){
							//cargar eeprom
							EEPROM_Write(SLAVE_EEPROM_ADDRESS, slaveAddress);
							auxSlaveAddress = slaveAddress;
						}
						
						if(auxbrConfig != modbusRegister.brConfig){
							//cargar eeprom
							EEPROM_Write(BRCONFIG_EEPROM_ADDRESS,modbusRegister.brConfig);
							auxbrConfig = modbusRegister.brConfig;
						}
						if(auxParity != modbusRegister.parity){
							//cargar eeprom
							EEPROM_Write(PARITY_EEPROM_ADDRESS,modbusRegister.parity);
							auxParity = modbusRegister.parity;
						}
						if(auxtimeBtw[0] != modbusRegister.timeBtw[0]){
							//cargar eeprom
							myWord.ui16[0]=modbusRegister.timeBtw[0];
							EEPROM_Write(TIMEBTW0_HIGH_EEPROM_ADDRESS,myWord.ui8[1]);
							EEPROM_Write(TIMEBTW0_LOW_EEPROM_ADDRESS,myWord.ui8[0]);
							auxtimeBtw[0] = modbusRegister.timeBtw[0];
						}
						
						if(auxtimeBtw[1] != modbusRegister.timeBtw[1]){
							//cargar eeprom
							myWord.ui16[0]=modbusRegister.timeBtw[1];
							EEPROM_Write(TIMEBTW1_HIGH_EEPROM_ADDRESS,myWord.ui8[1]);
							EEPROM_Write(TIMEBTW1_LOW_EEPROM_ADDRESS,myWord.ui8[0]);
							auxtimeBtw[1] = modbusRegister.timeBtw[1];
						}
						if(auxtimeBtw[2] != modbusRegister.timeBtw[2]){
							//cargar eeprom
							myWord.ui16[0]=modbusRegister.timeBtw[2];
							EEPROM_Write(TIMEBTW2_HIGH_EEPROM_ADDRESS,myWord.ui8[1]);
							EEPROM_Write(TIMEBTW2_LOW_EEPROM_ADDRESS,myWord.ui8[0]);
							auxtimeBtw[2] = modbusRegister.timeBtw[2];
						}
						if(auxtimeBtw[3] != modbusRegister.timeBtw[3]){
							//cargar eeprom
							myWord.ui16[0]=modbusRegister.timeBtw[3];
							EEPROM_Write(TIMEBTW3_HIGH_EEPROM_ADDRESS,myWord.ui8[1]);
							EEPROM_Write(TIMEBTW3_LOW_EEPROM_ADDRESS,myWord.ui8[0]);
							auxtimeBtw[3] = modbusRegister.timeBtw[3];
						}
				USART0_Deinitialize();
				USART0_Initialize(modbusRegister.brConfig,modbusRegister.parity);
				commFlags.iFlags.restartComms = 0;
			}
			
	//---------------------------- PROGRAMA LEDS---------------------------------------------
		
		if(commFlags.iFlags.isEvent1){
			//deshabilitar interrupcion
			TC0_Disable_CompareA_Interrupt();
			RegInput();
		}
	
	//-----------------------DATOS EEPROM------------------------------------------------------------
		//1ra vez leer variables de eprom y cargar modbus registers y aux value tmb
		//Faltaria declarar las variables en la eeprom
		

		
		PORTB = ((outValues << 1) & MASK_PORTB) | hbState ; 

	}
}