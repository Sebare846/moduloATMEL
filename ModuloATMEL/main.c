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
#include "header/usart.h"
#include <stdint.h>
/******Definiciones MODBUS, hacer libreria***************************************************************************/
#define MODBUS_SLAVE_ADDRESS		slaveAddress
#define MODBUS_BROADCAST_ADDRESS	0x00
#define BUFFER_LEN  256
#define REGISTERS   12
#define READ_HOLDING_REGISTERS 0x03
#define WRITE_SINGLE_REGISTER 0x06
#define DIAGNOSTICS 0x08
#define WRITE_MULTIPLE_REGISTERS 0x10
/******Definiciones lectura de entradas***************************************************************************/
#define MASKC 0x0F
#define MASKB 0x1E
#define IN1 0x01
#define IN2 0x02
#define IN3 0x04
#define IN4 0x08
#define MAXREGISTERS 16
/******Definiciones tiempos***************************************************************************/
#define PERIOD100MS 10  //Valor del periodo de 100ms con base 10ms
#define PERIOD250MS 25
#define PERIOD500MS 50
#define PERIOD1000MS 100
#define PERIOD10S   1000
#define PERIOD5S    500
#define T15_BR00 156
#define T15_BR10 172
#define T15_BR11 75
#define T35_BR00 365
#define T35_BR10 401
#define T35_BR11 175

typedef union{
	uint16_t ui16;
	uint8_t ui8[2];
}_uHalfWord;

_uHalfWord myHalfWord;

typedef union {
	uint32_t ui32;
	struct {
		uint16_t ui16L;   
		uint16_t ui16H;   
	} half;
	uint8_t ui8[4];
} _uWord32;

_uWord32 myWord32;


typedef union{
	struct{
		uint8_t isEvent1            : 1;
		uint8_t dataReady           : 1;
		uint8_t silenceTime         : 1;//ver si es necesario
		uint8_t isExceptionCode     : 1;
		uint8_t isBroadcast         : 1;
		uint8_t updateParameters    : 1;
		uint8_t restartComms        : 1;
		uint8_t btwFrame            : 1;
	} iFlags;
	uint8_t aFlags;
}_uFlags;

_uFlags intFlags; //banderas programa USART
/*LECTURA ENTRADAS------------------------------------------------------------*/
typedef union{
	struct{
		uint8_t isEvent1 : 1;
		uint8_t isEvent2 : 1;
		uint8_t isEvent3 : 1;
		uint8_t isEvent4 : 1;
		uint8_t DecodeData : 1;
		uint8_t is500ms : 1;
		uint8_t RESERVED : 2;
	} iFlags;
	uint8_t aFlags_input;
}_uFlags_input;

_uFlags_input inputFlags;

typedef struct {
	uint8_t  ID;
	uint8_t  stateLastEvent;
	uint32_t timeLastEvent;
}_sEvent __attribute__((aligned(1)));


_sEvent myEvents[MAXREGISTERS];
/*---------------------------------------------------------*/
typedef enum{
	IDLE,
	FUNCTION,
	DATA,
	EXTRADATA,
	CRCL,
	CRCH
}_eDecodeStates;

_eDecodeStates decodeState;

typedef enum{
	INVALID_FUNCTION = 0x01,
	INVALID_DATA_ADDRESS,
	INVALID_DATA_VALUE,
	EXCEPTION_CODE_N
}_eExceptionCodes;

_eExceptionCodes exceptionCodes;

typedef enum{
	RETURN_QUERY_DATA = 0x00,
	RESTART_COMMUNICATIONS,
	RETURN_DIAGNOSTICS_REGISTER
}_eDiagnosticsSubfunctions;

_eDiagnosticsSubfunctions diagnosticsSubfunction;

// Heartbeat
uint8_t hbTime, silenceTik, BRconfig, t15Tik, t35Tik, unixTik;

uint16_t reg[3] = {0x02F0 , 0x00A1, 0x1000};

// Comunicacion
volatile uint8_t bufferRx[BUFFER_LEN];
uint8_t bufferTx[BUFFER_LEN];
uint8_t indexRxR = 0, indexRxW=0 ,indexTxR = 0,indexTxW = 0;

// Trama Modbus
uint8_t function, nBytes, byteCount, errorCode, excepCode;
uint16_t crcLocal, crcMsg;



//-----------------------------------------------------------------------
// ------------------- "Banco" lógico de registros (sin direcciones fijas) -------------------
uint16_t slaveAddress;
uint16_t unixTimeH;//hacer una union capaz
uint16_t unixTimeL;
uint16_t registro;
uint16_t deadTime1;
uint16_t deadTime2;
uint16_t deadTime3;
uint16_t deadTime4;
uint16_t dbTime1;
uint16_t dbTime2;
uint16_t dbTime3;
uint16_t dbTime4;
uint16_t diagnosticsRegister;

uint16_t * const registerAddresses[REGISTERS] = {
	&unixTimeH,             // 1
	&unixTimeL,             // 2
	&registro,              // 3
	&deadTime1,             // 4
	&deadTime2,             // 5
	&deadTime3,             // 6
	&deadTime4,             // 7
	&dbTime1,               // 8
	&dbTime2,               // 9
	&dbTime3,               // 10
	&dbTime4,                // 11
	&diagnosticsRegister     //12
};


uart_drv_interface_t myUSARTHandler;

// ---------------------- Prototipos de Funciones ----------------------

void UARTsendByte();
void UARTrcvByte(uint8_t data);
void DecodeData();
void SendData();
void ReadData();
void Init_UART();
void initTimer1();
void do10us();
void do10ms();
void CrcCalculation(uint8_t data);
void SendExceptionCode();
void RestartTikValues();

//-----------------Manejo EEPROM--------------------
int8_t EEPROM_Write(uint16_t address, uint8_t data);
int16_t EEPROM_Read(uint16_t address);
int8_t EEPROM_Erase(uint16_t address);

//-----------------Lectura entradas--------------------

void RegInput();
void UpdateState(uint8_t lecture, uint8_t mask, uint8_t indice);

uint16_t TIMEBEETWEN[4] = {0,PERIOD5S,PERIOD10S,0}; //Tiempos configurables, mandar a eeprom
uint16_t tikBetweenTime[4];
uint8_t tim1L, tim1H, hbTime, tikDebounce[4];
//uint8_t time500ms;
uint8_t oldValueA, newValueA, outValues, lecture; // podria escribirse en la alta la old value y en la baja la nueva
uint8_t eventCounter;
uint32_t dateTime;



// ---------------------- Implementacion de ISR ----------------------
ISR(USART_RX_vect){
	if(intFlags.iFlags.btwFrame){
		if(!t15Tik){
			intFlags.iFlags.btwFrame=0; //vuelvo a esperar new frame
			decodeState=IDLE; 
			inputFlags.iFlags.DecodeData=0;//descarto mensaje
		}else{
			inputFlags.iFlags.DecodeData=1;	//deco
		}
	}else{
		if(!t35Tik){ //NewFrame
			intFlags.iFlags.btwFrame=1;
			inputFlags.iFlags.DecodeData=1; //deco
		}else{
			inputFlags.iFlags.DecodeData=0;//descarto
		}	
	}
	RestartTikValues(); 
	if(inputFlags.iFlags.DecodeData)
		UARTrcvByte(UDR0);
	
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
		switch(BRconfig){
		case 0x00:
			t15Tik = T15_BR00;
			t35Tik = T35_BR00;
		break;
		case 0x10:
			t15Tik = T15_BR10;
			t35Tik = T35_BR10;
		break;
		default: 
			t15Tik = T15_BR11;
			t35Tik = T15_BR11;
		break;
		
	} 
}

void do10us(){
	
	if(!t15Tik){
		t15Tik=0;
	}else{
		t15Tik--;
	}
	if(!t35Tik){
		t35Tik=0;
	}else{
		t35Tik--;
	}
}

void do10ms(){
	hbTime--;
	silenceTik--;
	intFlags.iFlags.isEvent1 = 1;
	unixTik--;
	if(!unixTik){ //esto no se si aca o en el main idk
		if(inputFlags.iFlags.is500ms){
			inputFlags.iFlags.is500ms = 0;
			//unix++;
			unixTimeL++;
			if(!unixTimeL) unixTimeH++;
				
		}else
			inputFlags.iFlags.is500ms=1;
	}
	
}

void RegInput(){
		
	    newValueA = ~(PINC & MASKC);
	    //PORTB |= ((newValueA << 1) & MASKB);
	    inputFlags.aFlags_input = (newValueA ^ oldValueA);
	    
	    if(tikBetweenTime[0]){
		    tikBetweenTime[0]--;
		}else{
		    if(inputFlags.iFlags.isEvent1){
			    tikDebounce[0] = PERIOD100MS;
			    lecture = (lecture & ~(IN1)) | (newValueA & IN1);
			    }else{
			    if(tikDebounce[0]){
				    tikDebounce[0]--;
				    if(tikDebounce[0] == 0){
					    UpdateState(lecture, IN1, 0);
				    }
			    }
		    }
	    }
	    
	    if(tikBetweenTime[1]){
		    tikBetweenTime[1]--;
		    }else{
		    if(inputFlags.iFlags.isEvent2){
			    tikDebounce[1] = PERIOD100MS;
			    lecture = (lecture & ~(IN2)) | (newValueA & IN2);
			    }else{
			    if(tikDebounce[1]){
				    tikDebounce[1]--;
				    if(tikDebounce[1] == 0){
					    UpdateState(lecture, IN2, 1);
				    }
			    }
		    }
	    }
	    
	    if(tikBetweenTime[2]){
		    tikBetweenTime[2]--;
		    }else{
		    if(inputFlags.iFlags.isEvent3){
			    tikDebounce[2] = PERIOD100MS;
			    lecture = (lecture & ~(IN3)) | (newValueA & IN3);
			}else{
			    if(tikDebounce[2]){
				    tikDebounce[2]--;
				    if(tikDebounce[2] == 0){
					    UpdateState(lecture, IN3, 2);
				    }
			    }
		    }
	    }
	    
	    if(tikBetweenTime[3]){
		    tikBetweenTime[3]--;
		    }else{
		    if(inputFlags.iFlags.isEvent4){
			    tikDebounce[3] = PERIOD100MS;
			    lecture = (lecture & ~(IN4)) | (newValueA & IN4);
			    }else{
			    if(tikDebounce[3]){
				    tikDebounce[3]--;
				    if(tikDebounce[3] == 0){
					    UpdateState(lecture, IN4, 3);
				    }
			    }
		    }
	    }
	    
	    oldValueA = newValueA;
	    
		
	    TC0_Enable_CompareA_Interrupt();
	    
	    intFlags.iFlags.isEvent1 = 0;
    }

 void UpdateState(uint8_t lect, uint8_t mask, uint8_t indice){
	
	  if(((~PINC) & mask) == (lect & mask)){ //Probar asi y sino ~(PORTA & mask)
		  if(lect & mask){
			  outValues |= mask; // CORRI UNO PORQUE SE DESPLAZA 
		   }else{
			  if(outValues & mask){
				  tikBetweenTime[indice] = TIMEBEETWEN[indice];
			  }
			 outValues &= ~mask;
  
		    }
	    }
	    myEvents[eventCounter].ID = indice;
	    myEvents[eventCounter].stateLastEvent = outValues & mask;
	    myEvents[eventCounter].timeLastEvent = dateTime;
	    eventCounter++;
	    if(eventCounter == MAXREGISTERS){
		    eventCounter = 0;
	    }
    

}

/*--------------------------------*/


void CrcCalculation(uint8_t data){//, uint16_t crc){
	crcLocal ^= data;
	for (uint8_t i = 0; i < 8; i++)
	{
		if (crcLocal & 0x0001)
		crcLocal = (crcLocal >> 1) ^ 0xA001;
		else
		crcLocal >>= 1;
	}
}

void UARTsendByte(){  
	if (USART0_IsTxReady()) { 
		USART0_Write(bufferTx[indexTxR]);     
		indexTxR = (indexTxR + 1) & (BUFFER_LEN - 1);
	}
}

// Funcion para recibir un byte a traves de UART leido
void UARTrcvByte(uint8_t data){
	switch(decodeState){
		case IDLE:
			if(data == MODBUS_BROADCAST_ADDRESS || data == MODBUS_SLAVE_ADDRESS){
				if(data == MODBUS_BROADCAST_ADDRESS) intFlags.iFlags.isBroadcast = 1;
				nBytes = 5;
				crcLocal = 0xFFFF;
				CrcCalculation(data);
				decodeState = FUNCTION;
				PORTB ^= 0b00000010;
				
			}
		break;
		case FUNCTION:
			function = data;
			CrcCalculation(data);
			nBytes--;
			if(data == 0x10) nBytes++;
			decodeState = DATA;
		break;
		case DATA:
			bufferRx[indexRxW] = data;
			indexRxW =(indexRxW + 1) & (BUFFER_LEN-1);
			CrcCalculation(data);
			nBytes--;
			if(nBytes <= 0){
				if(function == 0x10){
					nBytes = data;
					decodeState = EXTRADATA;
					}else{
					decodeState = CRCL;
				}
			}
		break;
		case EXTRADATA:
			bufferRx[indexRxW] = data;
			indexRxW =(indexRxW + 1) & (BUFFER_LEN-1);
			CrcCalculation(data);
			nBytes--;
			if(nBytes <= 0){
				decodeState = CRCL;
			}
		break;
		case CRCL:
			crcMsg = (uint16_t)data;
			decodeState = CRCH;
		break;
		case CRCH:
			PORTB ^= 0b00000100;
			crcMsg |= (uint16_t)(data << 8);
			if(crcLocal == crcMsg){
				PORTB ^= 0b00001000;
				intFlags.iFlags.dataReady = 1;
				}else{
				//error de CRC
				indexRxR = indexRxW;
			}
			decodeState = IDLE;
			intFlags.iFlags.btwFrame=0;
			RestartTikValues();
		break;
		default:
			decodeState = IDLE;
			indexRxR = indexRxW;
			intFlags.iFlags.btwFrame=0;
			RestartTikValues();
		break;
	}
}

void DecodeData(){
	uint8_t countIndex, byteCount=0;
	uint16_t regAddress, regValue, regCant, subfunction;
	indexTxW = 0;
	indexTxR = 0;
	crcLocal = 0xFFFF;
	PORTB ^= 0b00010000;
	switch(function){
		case READ_HOLDING_REGISTERS: //Ver si conviene copiar valores en RAM o dejarlos solo en EEPROM
			errorCode = 0x83;
			if(intFlags.iFlags.isBroadcast){
				intFlags.iFlags.isBroadcast = 0;
				return;
			}
			//Direccion inicial
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			regAddress = myHalfWord.ui16;
			//Cantidad de registros
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			regCant = myHalfWord.ui16;
			//Controlar cantidad de registros
			if(regCant < 1 || regCant > 0x07D0){
				excepCode = INVALID_DATA_VALUE;
				return;
			}
			//Controlar validez de direccion
			if((regAddress < 1) || (regAddress > REGISTERS) || ((regCant-1) > (REGISTERS - regAddress))){
				//PORTBbits.RB7 ^= 1;
				excepCode = INVALID_DATA_ADDRESS;
				intFlags.iFlags.isExceptionCode = 1;
				return;
			}
			// Cargar mensaje de respuesta
			bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
			bufferTx[indexTxW++] = READ_HOLDING_REGISTERS;
			countIndex = indexTxW;
			bufferTx[indexTxW++] = 0x00; // Almacenar en la pocision de cantidad de bytes
			for(uint8_t i=0 ; i<regCant ; i++){
				bufferTx[indexTxW++] = *((uint8_t *)registerAddresses[regAddress-1+i]);
				bufferTx[indexTxW++] = *((uint8_t *)(registerAddresses[regAddress-1+i])+1);
				byteCount += 2;
			}
			bufferTx[countIndex] = byteCount;
			//SaveData(bufferTx, &countIndex, byteCount);
			for(uint8_t i = 0; i < byteCount + 3;i++){
				CrcCalculation(bufferTx[i]);
			}
			myHalfWord.ui16 = crcLocal;
			bufferTx[indexTxW++] = myHalfWord.ui8[0];
			bufferTx[indexTxW++] = myHalfWord.ui8[1];
		break;
		case WRITE_SINGLE_REGISTER:
			errorCode = 0x86;
			
			//Direccion inicial
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			regAddress = myHalfWord.ui16;
			//Cantidad de registros
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			regValue = myHalfWord.ui16;
			//Controlar cantidad de registros
			if(regValue < 0 || regValue > 0xFFFF){
				excepCode = INVALID_DATA_VALUE;
				return;
			}
			//Controlar validez de direccion
			if(regAddress > REGISTERS){
				//PORTBbits.RB7 ^= 1;
				excepCode = INVALID_DATA_ADDRESS;
				intFlags.iFlags.isExceptionCode = 1;
				return;
			}
				
			*((uint8_t *)registerAddresses[regAddress-1]) = myHalfWord.ui8[0];
			*((uint8_t *)(registerAddresses[regAddress-1]+1)) = myHalfWord.ui8[1];
		
			if(intFlags.iFlags.isBroadcast){
				intFlags.iFlags.isBroadcast = 0;
				return;
			}
		
			bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
			bufferTx[indexTxW++] = WRITE_SINGLE_REGISTER;
			myHalfWord.ui16 = regAddress;
			bufferTx[indexTxW++] = myHalfWord.ui8[0];
			bufferTx[indexTxW++] = myHalfWord.ui8[1];
			bufferTx[indexTxW++] = *((uint8_t *)registerAddresses[regAddress-1]);
			bufferTx[indexTxW++] = *((uint8_t *)(registerAddresses[regAddress-1])+1);
		
			for(uint8_t i = 0; i < 6;i++){
				CrcCalculation(bufferTx[i]);
			}
			myHalfWord.ui16 = crcLocal;
			bufferTx[indexTxW++] = myHalfWord.ui8[0];
			bufferTx[indexTxW++] = myHalfWord.ui8[1];
		break;
		case DIAGNOSTICS:
			errorCode = 0x88;
			if(intFlags.iFlags.isBroadcast){
				intFlags.iFlags.isBroadcast = 0;
				return;
			}
			//Subfuncion
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			subfunction = myHalfWord.ui16;
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			regValue = myHalfWord.ui16;
			
			switch(subfunction){
				case RETURN_QUERY_DATA:
				//Eco
				break;
				case RESTART_COMMUNICATIONS:
					if(regValue == 0x0000 || regValue == 0xFF00){
						//Eco
						intFlags.iFlags.restartComms = 1; ////
						}else{
						excepCode = INVALID_DATA_VALUE;
						intFlags.iFlags.isExceptionCode = 1;
						return;
					}
				break;
				case RETURN_DIAGNOSTICS_REGISTER:
					if(regValue == 0x0000){
						//Registro de diagnostico
						//regValue  = diagnosticsRegister;
						// regValue |= diagnosticsRegister;
						}else{
						excepCode = INVALID_DATA_VALUE;
						intFlags.iFlags.isExceptionCode = 1;
						return;
					}
				break;
				default:
					excepCode = INVALID_FUNCTION;
					intFlags.iFlags.isExceptionCode = 1;
					indexRxR = indexRxW;
				return;
		}
		bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
		bufferTx[indexTxW++] = DIAGNOSTICS;
		myHalfWord.ui16 = subfunction;
		bufferTx[indexTxW++] = myHalfWord.ui8[0];
		bufferTx[indexTxW++] = myHalfWord.ui8[1];
		myHalfWord.ui16 = regValue;
		bufferTx[indexTxW++] = myHalfWord.ui8[0];
		bufferTx[indexTxW++] = myHalfWord.ui8[1];
		for(uint8_t i=0; i < 6; i++){
			CrcCalculation(bufferTx[i]);
		}
		myHalfWord.ui16 = crcLocal;
		bufferTx[indexTxW++] = myHalfWord.ui8[0];
		bufferTx[indexTxW++] = myHalfWord.ui8[1];
		break;
		case WRITE_MULTIPLE_REGISTERS: //Incompleto
			errorCode = 0x90;
			//Direccion inicial
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			regAddress = myHalfWord.ui16;
			//Cantidad de registros
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			regCant = myHalfWord.ui16;
			//Cantidad de bytes
			byteCount = bufferRx[indexRxR++];
		
			//Controlar cantidad de registros
			if((regCant < 1 || regCant > 0x007B) || (byteCount != (regCant*2))){
				excepCode = INVALID_DATA_VALUE;
				intFlags.iFlags.isExceptionCode = 1;
				return;
			}
			//Controlar validez de direccion
			if((regAddress < 1) || (regAddress > REGISTERS) || ((regCant-1) > (REGISTERS - regAddress))){
				excepCode = INVALID_DATA_ADDRESS;
				intFlags.iFlags.isExceptionCode = 1;
				return;
			}
			for(uint8_t i=0; i<regCant;i++){
			myHalfWord.ui8[0] = bufferRx[indexRxR++];
			myHalfWord.ui8[1] = bufferRx[indexRxR++];
			regValue = myHalfWord.ui16;
				*((uint8_t *)registerAddresses[regAddress-1+i]) = myHalfWord.ui8[0];
				*((uint8_t *)(registerAddresses[regAddress-1+i]+1)) = myHalfWord.ui8[1];
			}
		
			if(intFlags.iFlags.isBroadcast){
				intFlags.iFlags.isBroadcast = 0;
				return;
			}
			bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
			bufferTx[indexTxW++] = WRITE_MULTIPLE_REGISTERS;
			myHalfWord.ui16 = regAddress;
			bufferTx[indexTxW++] = myHalfWord.ui8[0];
			bufferTx[indexTxW++] = myHalfWord.ui8[1];
			myHalfWord.ui16 = regCant;
			bufferTx[indexTxW++] = myHalfWord.ui8[0];
			bufferTx[indexTxW++] = myHalfWord.ui8[1];
			for(uint8_t i = 0; i < 6;i++){
				CrcCalculation(bufferTx[i]);
			}
			myHalfWord.ui16 = crcLocal;
			bufferTx[indexTxW++] = myHalfWord.ui8[0];
			bufferTx[indexTxW++] = myHalfWord.ui8[1];
		break;
		default:
			if(intFlags.iFlags.isBroadcast){
				return;
			}
		excepCode = INVALID_FUNCTION;
		intFlags.iFlags.isExceptionCode = 1;
		indexRxR = indexRxW;
		break;
	}
	indexRxR = indexRxW;
	
}

void SendExceptionCode(){
	intFlags.iFlags.isExceptionCode = 0;
	bufferTx[indexTxW++] = (uint8_t)MODBUS_SLAVE_ADDRESS;
	bufferTx[indexTxW++] = errorCode;
	bufferTx[indexTxW++] = excepCode;
	for(uint8_t i = 0; i < 3;i++){
		CrcCalculation(bufferTx[i]);
	}
	myHalfWord.ui16 = crcLocal;
	bufferTx[indexTxW++] = myHalfWord.ui8[1];
	bufferTx[indexTxW++] = myHalfWord.ui8[1];
}

void USART0_ReceiveInterruptEnable(void){
	UCSR0B |= (1 << (RXCIE0));
}
void USART0_ReceiveInterruptDisable(void){
	UCSR0B &= ~(1 << (RXCIE0));
}

/*
    Main application
*/

int main(void){
	
    SYSTEM_Initialize();
	USART0_Initialize();
	TC0_Initialize();
	
	sei();
	
	TMR0_SetHandler(do10ms);
	TMR1_SetHandler(do10us);
	//myUSARTHandler.RxCompleteCallbackRegister(ReadData);
	//sei();
	USART0_ReceiveInterruptEnable();

	uint8_t hbState = 0;

  decodeState = IDLE;
  hbTime = PERIOD250MS;
  intFlags.aFlags = 0;
  intFlags.iFlags.silenceTime = 1;
  silenceTik = 6;
  BRconfig = 0;
  unixTik = PERIOD500MS;
  
  unixTimeH               = 0x1111;
  unixTimeL               = 0x2222;
  registro                = 0x3333;
  deadTime1               = 0x4444;
  deadTime2               = 0x5555;
  deadTime3               = 0x6666;
  deadTime4               = 0x7777;
  dbTime1                 = 0x8888;
  dbTime2                 = 0x9999;
  dbTime3                 = 0xAAAA;
  dbTime4                 = 0xBBBB;
  diagnosticsRegister     = 0xCDEF;
  
  /*-----LECTURA ENTRADAS-----------------------*/
  newValueA = ~(PORTC & MASKC);
  outValues = newValueA;
  inputFlags.aFlags_input = 0;
  intFlags.aFlags = 0;

  tikBetweenTime[0] = 0;
  tikBetweenTime[1] = 0;
  tikBetweenTime[2] = 0;
  tikBetweenTime[3] = 0;

  
  
    while(1){
		
		 if(!hbTime){ //Heartbeat
			 hbState ^= (1<<5);
			 hbTime = PERIOD1000MS;
		 }
		 

		 if(intFlags.iFlags.dataReady){ //DECODIFICA SI EL BYTE SE RECIBIO CORRECTAMENTE
			 intFlags.iFlags.dataReady = 0;
			 DecodeData();
		 }else{
			if(silenceTik < 2 && indexTxR != indexTxW){ //Si hay datos para mandar y transcurre un periodo t habilita el max para transmitir
				intFlags.iFlags.silenceTime = 0;
				USART0_ReceiveInterruptDisable();
				PORTD |= 0x4; //MAX HIGH
			}
		 }
		 if(indexTxR != indexTxW && intFlags.iFlags.silenceTime == 0){ //Si hay datos para transmitir, transmite
			 UARTsendByte();
		 }
		 if(indexTxR == indexTxW && intFlags.iFlags.silenceTime == 0){ //Si no hay datos para transmitir vuelve a la interrupcion
			 if(USART0_IsTxDone()){ //IS TRANSMIT COMPLETE
				 UCSR0A |= (1<<TXC0);           // limpiar TXC0
				 intFlags.iFlags.silenceTime = 1;
				 silenceTik = 4;
				 PORTD &= ~ 0x4; //MAX_SetLow();
				 USART0_ReceiveInterruptEnable();
			 }
		 }


		 if(intFlags.iFlags.isExceptionCode){ //leido
			 SendExceptionCode();
		 }
 
		 if(intFlags.iFlags.restartComms){ //leido
			 intFlags.iFlags.restartComms = 0;
		 }
 //---------------------------- PROGRAMA LEDS---------------------------------------------
		

		if(intFlags.iFlags.isEvent1){
			//deshabilitar interrupcion
			TC0_Disable_CompareA_Interrupt();
			RegInput();
		}
	
		PORTB = ((outValues << 1) & MASKB) | hbState ; 

//---------------------------------------

	}
}