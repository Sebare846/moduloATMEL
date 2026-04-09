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

/* Include -------------------------------------------------------------------*/
#include "mcc_generated_files/system/system.h"
#include "header/timer0.h"
#include "header/timer1.h"
#include "header/timer2.h"
#include "header/usart.h"
#include "header/usart1.h"
#include "header/modbus.h"
#include <stdint.h>
/* END Include ---------------------------------------------------------------*/

/* typedef -------------------------------------------------------------------*/
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
	Mapa de banderas para el manejo del baudrate USART1 
	
*/ 
typedef union{
	struct{
		uint8_t BR9600 : 1;
		uint8_t RESERVED : 7;
	} iFlags;
	uint8_t aFlags_TIM2;
}_uFlags_TIM2;
_uFlags_TIM2 TIM2_BR;

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

typedef struct {
	uint32_t unixTime;				//0x400 0x401 0x402 0x403
	uint8_t brConfig;				//0x404
	uint8_t parity;					//0x405
	uint16_t timeBtw[4];			//0x406 0x407 x0408 0x409 0x40A x40B x40C x40D 
	uint16_t idCounter[4];			//0x40E 0x41F x0410 0x411 0x412 x413 x414 x415 
	uint16_t diagnosticsRegister;	//0x416 x417
	uint16_t newEventCounter;		//0x418 x419
	uint16_t eventIndexR;			//0x41A x41B
} _sModbusReg;

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

/*
* Start Address:	0x01
* End Address:		0x0B
* Total Bytes:		0x0B
*/
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
/* END typedef ---------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
//----------------------------- Definiciones tiempos - base 10ms -----------------------------
#define PERIOD100MS		10
#define PERIOD250MS		25
#define PERIOD500MS		50
#define PERIOD1000MS	100
#define PERIOD10S		1000
#define PERIOD5S		500
//----------------------------- Definiciones tiempos - base 10us -----------------------------
#define PERIOD5MS		500
#define PERIOD1S_TIM2	9615
#define PERIOD05BR		1
#define PERIODBR		2
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
#define END_CONFIG_ADDRESS_R	0x41B
#define END_CONFIG_ADDRESS_W	0x415
#define START_EVENT_ADDRESS		0x772
#define END_EVENT_ADDRESS		0x89E
//----------------------------------------- Tamanios ------------------------------------------
#define BUFFER_LEN  256
#define REGISTERS   14
#define MAXEVENTS	50
/* END Define ----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
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
	brief ISRVirtualUsart	:
*/
void ISRVirtualUsart();
/*
	brief USART0_ReceiveInterruptEnable :
*/
static inline void USART0_ReceiveInterruptEnable();
/*
	brief USART0_ReceiveInterruptDisable :
*/
static inline void USART0_ReceiveInterruptDisable();
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
/*
	brief LoadEvents	:
	param [in] count	:
*/
void LoadEvents(uint16_t count);
/*
	brief InitializeEvents :
*/
void InitializeEvents();
/* END Function prototypes ---------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
// -------------------------------------- Variables MODBUS -------------------------------------
_eDecodeStates decodeState;
_sModbusFrame myModbusFrame;
_uCommFlags commFlags;
uint16_t  t15Tik=0, t35Tik=0;
uint8_t auxbrConfig=0, auxParity=0;
//------------------------------- Variables Comunicacion USART0 -------------------------------
volatile uint8_t bufferRx[BUFFER_LEN];
uint8_t bufferTx[BUFFER_LEN];
uint8_t indexRxR, indexRxW, indexTxR, indexTxW, trashByte;
usart0_status_t usartFlag;
uart_drv_interface_t myUSARTHandler;
//-------------------------------- Variables USART1 -------------------------------------------
uint8_t indexRx = 0, indexTx=0;
//uint8_t bufferRx_U1[256], bufferTx_U1[256];
//------------------------------------------ Tickers ------------------------------------------
uint16_t auxTik=0, tim2Tik=0, tikResponse;
uint8_t hbTime=0, unixTik=0;
//------------------------------- Variables captura de eventos --------------------------------
uint16_t tikBetweenTime[4], eventIndexW;
uint8_t	tikDebounce[4];
uint8_t oldValueA, newValueA, outValues, lecture, is500ms;
//--------------------------- Variables con direccion determinada -----------------------------
uint8_t slaveAddress __attribute__((section(".mySlaveAddress"))); //0x3FF
/*
* Start Address:	0x400
* End Address:		0x41E
* Total Bytes:		0x0C
*/
_sModbusReg modbusRegister __attribute__((section(".modbus_r")));
/*
* Start Address:	0x772
* End Address:		0x89E
* Total Bytes:		0x12C
*/
_sEvent myEvents[MAXEVENTS] __attribute__((section(".fixed_events")));
/* END Global variables ------------------------------------------------------*/

/* Function prototypes user code ---------------------------------------------*/
ISR(USART_RX_vect){
	RestartTikValues(); 
	MODBUS_DecodeFrame(UDR0);
}

int8_t EEPROM_Write(uint16_t address, uint8_t data){
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

int16_t EEPROM_Read(uint16_t address){
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

static inline void USART0_ReceiveInterruptEnable(){
	UCSR0B |= (1 << (RXCIE0));
}

static inline void USART0_ReceiveInterruptDisable(){
	UCSR0B &= ~(1 << (RXCIE0));
}

void UARTsendByte(){
		if (UCSR0A & (1 << UDRE0)) {
			USART0_Write(bufferTx[indexTxR++]);
		}
}

void ISRVirtualUsart(){
	tim2Tik--;
	tikRX--;
	if(!tikRX){
		//FUNCION LECTURA
		tikRX = PERIODBR;
		if(USART1_RXENABLE){
			USART1_RECEIVE();
		}//else inicializar variables
	}
	
	if(USART1_RX_COMPLETE == 1 && USART1_RX_READY == 0){
		USART1_receiveByte();
	}
	
	if(TIM2_BR.iFlags.BR9600){
		TIM2_BR.iFlags.BR9600 = 0;
		
		if(USART1_TX_DONE == 1 && USART1_TX_READY == 0){ //hay dato para guardar en rx data
			USART1_SendByte();
		}
		
		if(USART1_TXENABLE)//else inicializar variables
		USART1_TRANSMIT();
		}else{
		TIM2_BR.iFlags.BR9600 = 1;
	}
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
	if(tikResponse){
		tikResponse--;
	}
}

void do10ms(){
	hbTime--;
	unixTik--;
	if(!unixTik){
		if(is500ms){
			modbusRegister.unixTime++;				
			is500ms = 0;
		}else{
			is500ms = 1;
		}
		unixTik = PERIOD500MS;
	}
	commFlags.iFlags.isEvent1 = 1;
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
	    TIMSK0 |= (1 << OCIE0A);
	    commFlags.iFlags.isEvent1 = 0;
    }

 void UpdateState(uint8_t lect, uint8_t mask, uint8_t indice){
	  if(((~PINC) & mask) == (lect & mask)){
		  if(lect & mask){
			  outValues |= mask; 
			  modbusRegister.idCounter[indice]++;
			  myEvents[eventIndexW].stateLastEvent = 1;
		   }else{
			  if(outValues & mask){
				  tikBetweenTime[indice] = modbusRegister.timeBtw[indice];
			  }
			 outValues &= ~mask;
			 myEvents[eventIndexW].stateLastEvent = 0;
		    }
	    }
		myEvents[eventIndexW].ID = indice;
	    myEvents[eventIndexW].timeLastEvent = modbusRegister.unixTime;
		eventIndexW++;
	    modbusRegister.newEventCounter++;
	    if(eventIndexW == MAXEVENTS){
		    eventIndexW = 0;
	    }
}

void InitializeEvents(){
	for(uint8_t i = 0; i < MAXEVENTS; i++) {
		myEvents[i].ID = 0;
		myEvents[i].stateLastEvent = 0;
		myEvents[i].timeLastEvent = 0;
	}
}

void LoadEvents(uint16_t count){
	for (uint8_t i = 0; i<count; i++ ){
		if(modbusRegister.eventIndexR == MAXEVENTS){
			modbusRegister.eventIndexR=0;
		}
		bufferTx[indexTxW++] = myEvents[modbusRegister.eventIndexR].ID;
		bufferTx[indexTxW++] = myEvents[modbusRegister.eventIndexR].stateLastEvent;
		myWord.u32 = myEvents[modbusRegister.eventIndexR].timeLastEvent;
		bufferTx[indexTxW++] = myWord.ui8[0];
		bufferTx[indexTxW++] = myWord.ui8[1];
		bufferTx[indexTxW++] = myWord.ui8[2];
		bufferTx[indexTxW++] = myWord.ui8[3];	
		modbusRegister.eventIndexR++;
		if(modbusRegister.newEventCounter)
			modbusRegister.newEventCounter--;
	}
}

void MODBUS_CalculateCRC(uint8_t data){
	myModbusFrame.crcSlave ^= (uint16_t)data;
	for (uint8_t i = 0; i < 8; i++){
		if (myModbusFrame.crcSlave & 0x0001)
			myModbusFrame.crcSlave = (myModbusFrame.crcSlave >> 1) ^ 0xA001;
		else
			myModbusFrame.crcSlave >>= 1;
	}
}

void MODBUS_DecodeFrame(uint8_t data){
	switch(decodeState){
		case IDLE:
			commFlags.iFlags.isBroadcast = 0;
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
	uint8_t auxIndex, countIndex, byteCount = 0, auxLoadCant = 0;
	uint16_t regAddress, regValue, regCant, subfunction;
	myModbusFrame.crcSlave = 0xFFFF;
	switch(myModbusFrame.function){
		case READ_HOLDING_REGISTERS:
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
			//Controlar cantidad 
			if(regCant<1 || regCant > 0x07B){ //error cantidad
				myModbusFrame.excepCode = INVALID_DATA_VALUE;
			    commFlags.iFlags.isExceptionCode = 1;
				return;
			}
			//Controlar validez direcciones
			if((regAddress <= 0) || ((regAddress > MAXEVENTS) && (regAddress < START_CONFIG_ADDRESS)) || (regAddress>END_CONFIG_ADDRESS_R && regAddress <START_EVENT_ADDRESS) || regAddress > END_EVENT_ADDRESS){
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
			countIndex = indexTxW;
			bufferTx[indexTxW++] = 0x00; 
			if(regAddress > 0 && regAddress < MAXEVENTS){
				auxLoadCant = (regCant/3);
				if(auxLoadCant > modbusRegister.newEventCounter)
					auxLoadCant = modbusRegister.newEventCounter;
				LoadEvents(auxLoadCant);				
				byteCount = regCant * 2;
			}else{
				for(uint8_t i=0 ; i<(regCant*2) ; i++){
					bufferTx[indexTxW++] = *((uint8_t *)(regAddress + i)); 
					byteCount++;
				}
			}
			bufferTx[countIndex] = byteCount;
			
			auxIndex = indexTxR;
			while(auxIndex != indexTxW){
				MODBUS_CalculateCRC(bufferTx[auxIndex++]);
			}
			
			myWord.ui16[0] = myModbusFrame.crcSlave;
			bufferTx[indexTxW++] = myWord.ui8[0];
			bufferTx[indexTxW++] = myWord.ui8[1];
			
			tikResponse = PERIOD5MS;		
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
		
			auxIndex = indexTxR;
			while(auxIndex != indexTxW){
				MODBUS_CalculateCRC(bufferTx[auxIndex++]);
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
						commFlags.iFlags.restartComms = 1; 
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
				
				auxIndex = indexTxR;
				while(auxIndex != indexTxW){
					MODBUS_CalculateCRC(bufferTx[auxIndex++]);
				}
				myWord.ui16[0] = myModbusFrame.crcSlave;
				bufferTx[indexTxW++] = myWord.ui8[0];
				bufferTx[indexTxW++] = myWord.ui8[1];
				break;
		case WRITE_MULTIPLE_REGISTERS:
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
			
			auxIndex = indexTxR;
			while(auxIndex != indexTxW){
				MODBUS_CalculateCRC(bufferTx[auxIndex++]);
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
	uint8_t auxIndex;
	commFlags.iFlags.isExceptionCode = 0;
	bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
	bufferTx[indexTxW++] = myModbusFrame.errorCode;
	bufferTx[indexTxW++] = myModbusFrame.excepCode;
	
	auxIndex = indexTxR;
	while(auxIndex != indexTxW){
		MODBUS_CalculateCRC(bufferTx[auxIndex++]);
	}

	myWord.ui16[0] = myModbusFrame.crcSlave;
	bufferTx[indexTxW++] = myWord.ui8[0];
	bufferTx[indexTxW++] = myWord.ui8[1];
}
/* END Function prototypes user code -----------------------------------------*/

/*
    Main application
*/
int main(void){
/* Local variables -----------------------------------------------------------*/
	uint8_t auxSlaveAddress, hbState = 0, byte=0;
	uint16_t auxtimeBtw[4];
/* END Local variables -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/
	SYSTEM_Initialize();
	
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
	modbusRegister.eventIndexR = 0;
	modbusRegister.unixTime				= 1770766800;
	modbusRegister.diagnosticsRegister	= 0x3333; //Ver que valor darle que nos sirva
	modbusRegister.newEventCounter		= 0x0000;

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
	
	decodeState = IDLE;
	hbTime = PERIOD250MS;
	auxTik = 25000;
	commFlags.aFlags = 0;
	commFlags.iFlags.silenceTime = 1;
	usartFlag.isTrasmitting = 0;
	indexTxW = 0;
	indexTxR = 0;
	indexRxW = 0;
	indexRxR = 0;
		
	unixTik = PERIOD500MS;
	tikRX	= PERIODBR;
	tim2Tik = PERIOD1S_TIM2;
	
	newValueA = ~(PINC & MASK_PORTC);
	outValues = newValueA;
	inputFlags.aFlags_input = 0;
	commFlags.aFlags = 0;
	is500ms = 0;

	tikBetweenTime[0] = 0;
	tikBetweenTime[1] = 0;
	tikBetweenTime[2] = 0;
	tikBetweenTime[3] = 0;
	
	modbusRegister.idCounter[0] = 0;
	modbusRegister.idCounter[1] = 0;
	modbusRegister.idCounter[2] = 0;
	modbusRegister.idCounter[3] = 0;
		
	USART0_Initialize(modbusRegister.brConfig, modbusRegister.parity);
	TC0_Initialize();
	TC1_Initialize();
	TC2_Initialize();
	USART1_Initialize();
	InitializeEvents();
	
	sei();
	
	TMR0_SetHandler(do10ms);
	TMR1_SetHandler(do10us);
	TMR2_SetHandler(ISRVirtualUsart);
	USART0_ReceiveInterruptEnable();

	while(1){
			if(!hbTime){
				hbState ^= (1<<5);
				hbTime = PERIOD250MS;
			}

			USART1_Start_bit_detect();
			if(USART1_RX_READY){
				byte = USART1_Read();
				USART1_Write(byte);
			}		
	 
			if(commFlags.iFlags.dataReady){
				commFlags.iFlags.dataReady = 0;
				MODBUS_ProcessFunction();
			}

			if((indexTxR != indexTxW) && tikResponse == 0){
				if(usartFlag.isTrasmitting){
					UARTsendByte();	
				}else{
					PORTD |= (1<<2);
					usartFlag.isTrasmitting = 1;	
				}
			}
			
			if(indexTxR==indexTxW && usartFlag.isTrasmitting){
				if(UCSR0A & (1<<TXC0)){
					PORTD &= ~(1<<2);
					usartFlag.isTrasmitting = 0;
					UCSR0A &= ~(1<<TXC0);      
				}
			}
			
			if(commFlags.iFlags.isExceptionCode){ 
				MODBUS_SendExceptionCode();
			}
 
			if(commFlags.iFlags.restartComms){
				//GUARDAR TODOS LOS REGISTROS EN EEPROM Y RESETEAR USART
						if(auxSlaveAddress != slaveAddress){
							EEPROM_Write(SLAVE_EEPROM_ADDRESS, slaveAddress);
							auxSlaveAddress = slaveAddress;
						}
						if(auxbrConfig != modbusRegister.brConfig){
							EEPROM_Write(BRCONFIG_EEPROM_ADDRESS,modbusRegister.brConfig);
							auxbrConfig = modbusRegister.brConfig;
						}
						if(auxParity != modbusRegister.parity){
							EEPROM_Write(PARITY_EEPROM_ADDRESS,modbusRegister.parity);
							auxParity = modbusRegister.parity;
						}
						if(auxtimeBtw[0] != modbusRegister.timeBtw[0]){
							myWord.ui16[0]=modbusRegister.timeBtw[0];
							EEPROM_Write(TIMEBTW0_HIGH_EEPROM_ADDRESS,myWord.ui8[1]);
							EEPROM_Write(TIMEBTW0_LOW_EEPROM_ADDRESS,myWord.ui8[0]);
							auxtimeBtw[0] = modbusRegister.timeBtw[0];
						}
						if(auxtimeBtw[1] != modbusRegister.timeBtw[1]){
							myWord.ui16[0]=modbusRegister.timeBtw[1];
							EEPROM_Write(TIMEBTW1_HIGH_EEPROM_ADDRESS,myWord.ui8[1]);
							EEPROM_Write(TIMEBTW1_LOW_EEPROM_ADDRESS,myWord.ui8[0]);
							auxtimeBtw[1] = modbusRegister.timeBtw[1];
						}
						if(auxtimeBtw[2] != modbusRegister.timeBtw[2]){
							myWord.ui16[0]=modbusRegister.timeBtw[2];
							EEPROM_Write(TIMEBTW2_HIGH_EEPROM_ADDRESS,myWord.ui8[1]);
							EEPROM_Write(TIMEBTW2_LOW_EEPROM_ADDRESS,myWord.ui8[0]);
							auxtimeBtw[2] = modbusRegister.timeBtw[2];
						}
						if(auxtimeBtw[3] != modbusRegister.timeBtw[3]){
							myWord.ui16[0]=modbusRegister.timeBtw[3];
							EEPROM_Write(TIMEBTW3_HIGH_EEPROM_ADDRESS,myWord.ui8[1]);
							EEPROM_Write(TIMEBTW3_LOW_EEPROM_ADDRESS,myWord.ui8[0]);
							auxtimeBtw[3] = modbusRegister.timeBtw[3];
						}
				USART0_Deinitialize();
				USART0_Initialize(modbusRegister.brConfig,modbusRegister.parity);
				USART0_ReceiveInterruptEnable();
				commFlags.iFlags.restartComms = 0;
			}
			
		if(commFlags.iFlags.isEvent1){
			TIMSK0 &= ~(1 << OCIE0A);
			RegInput();
		}
			
		PORTB = ((outValues << 1) & MASK_PORTB) | hbState ; 
	}
	/* END User code -------------------------------------------------------------*/
	return 0;
}

