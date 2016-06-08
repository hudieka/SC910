#ifndef __MODBUS_H__
#define __MODBUS_H__


#define true    1
#define false   0

#define HMI_SLAVE_ADDRESS					11

#define HMI_READ_DIGITAL_OUTPUT				0x01
#define HMI_READ_DIGITAL_INPUT				0x02
#define HMI_READ_ANOLOG_OUTPUT				0x03
#define HMI_READ_ANOLOG_INPUT				0x04
#define HMI_FORCE_SINGLE_DIGITAL_OUTPUT		0x05
#define HMI_FORCE_SINGLE_ANOLOG_OUTPUT		0x06
#define HMI_FORCE_MUL_DIGITAL_OUTPUT		0x0F		
#define HMI_FORCE_MUL_ANOLOG_OUTPUT			0x10


#define MODBUS_SEND      0xaa
#define MODBUS_CLOSE     0

typedef struct tagHMIHeader
{
    unsigned char	bSlaveAdd;
    unsigned char	bFuncCode;
    unsigned char	StartAddHigh;
    unsigned char	StartAddLow;/*Motorola Format*/
}HMIHeader;

#define MODBUS_MAX_BUFFER  	             200
#define HMI_ACCESS_HEADER(header)	((HMIHeader*)(header))

typedef struct tagRxBuffer
{ 
	unsigned short	uCount;
	unsigned char	byData[MODBUS_MAX_BUFFER];  
}RxBuffer;  // Receive

typedef struct tagTxBuffer
{ 
	unsigned short	uCount;
	unsigned short	uSend;
	unsigned char	byData[MODBUS_MAX_BUFFER];  
}TxBuffer;  // Transmit


#define MAKEWORD(high,low)		((high*256)+low)


unsigned char ModbusReceive(unsigned char *pbyData, unsigned short uCount,unsigned char port);
void SendAckToMaster(unsigned int port);
unsigned char CstProcessHMIService();
unsigned int GenCrcCode(unsigned char *datapt,unsigned char bytecount);
void Wait35Char(unsigned int port);

extern void CstModbusSendAck(void);
















#endif