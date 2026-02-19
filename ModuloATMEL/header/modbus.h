/*
 * modbus.h
 *
 * Created: 10/2/2026 09:50:32
 *  Author: Seba
 */ 


#ifndef MODBUS_H_
#define MODBUS_H_

extern uint8_t slaveAddress;

#define MODBUS_SLAVE_ADDRESS		slaveAddress
#define MODBUS_BROADCAST_ADDRESS	0x00

#define READ_HOLDING_REGISTERS		0x03
#define WRITE_SINGLE_REGISTER		0x06
#define DIAGNOSTICS					0x08
#define WRITE_MULTIPLE_REGISTERS	0x10

#define T15_BR00 156	//9600 SIN PARIDAD
#define T15_BR10 172	// 9600 CON PARIDAD
#define T15_BR11 75		//19200
#define T35_BR00 365
#define T35_BR10 401
#define T35_BR11 175

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
}_uCommFlags;

typedef enum{
	IDLE,
	FUNCTION,
	DATA,
	EXTRADATA,
	CRCH,
	CRCL,
	DISCARD
}_eDecodeStates;

typedef enum{
	INVALID_FUNCTION = 0x01,
	INVALID_DATA_ADDRESS,
	INVALID_DATA_VALUE,
	EXCEPTION_CODE_N
}_eExceptionCodes;

typedef enum{
	RETURN_QUERY_DATA = 0x00,
	RESTART_COMMUNICATIONS,
	RETURN_DIAGNOSTICS_REGISTER
}_eDiagnosticsSubfunctions;

typedef struct{
	uint8_t function;
	uint8_t nBytes;
	uint8_t errorCode;
	uint8_t excepCode;
	uint16_t crcSlave;
	uint16_t crcMaster;
	}_sModbusFrame;
#endif /* MODBUS_H_ */