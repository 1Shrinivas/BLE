#ifndef _COMM_MANAGER_H_
#define _COMM_MANAGER_H_

# include "stdint.h"
/*********************Enum Declaration ***************************************/
enum op_mode {close,idle,m_0,m_tx,m_rx,s_0,s_rx,s_tx};
/*********************Enum Declaration End***************************************/

#define ACK   		0x06
#define NACK  		0x15
#define HDR_SIZE 	0x07	

#pragma pack(1)
typedef struct _IPacket
{
	uint8_t header;
	uint8_t requested_param_id;
	uint16_t subid;
	uint16_t size;
	uint8_t footer;
	uint16_t pendRead;
	uint8_t buffer[648];

}IPacket;


#pragma pack(1)
typedef struct _OPacket
{
	uint8_t header;
	uint8_t requested_param_id;
	uint16_t subid;
	uint16_t size;
	uint8_t footer;
	uint8_t buffer[648];

}OPacket;



/*********************Function Declaration ***************************************/
void check_state(void);
void uart_trigger(void);


uint8_t  TX_Event(uint8_t ID, uint8_t Sub_Id, uint8_t *src, uint16_t Len);


/**********************Function Declaration End**************************************/

#endif
