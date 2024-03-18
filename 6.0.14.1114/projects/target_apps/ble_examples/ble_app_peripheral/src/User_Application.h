#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "stdint.h"
#include "macros.h"
#include "comm_manager.h"


#define SCHEDULER_ON 			    	0x01
#define ACK											0x06
#define NAK											0x15
#define MSB 										0x01
#define LSB 										0x00
#define PAYLOAD_OFFSET 					0x02
#define ERROR 									0xFF
	
#define IND_SUCESS 				  		0x0
#define MAX_BLE_NAME_LEN 				50
#define SOFT_REVISION_OFFSET 		30
#define BND_INFO_OFSET 					50
#define ALERTS_LENGTH           129

#define UNSERVICEABLE_FLAG   	  0x10
#define READING_IN_PROGRESS     0x07
#define SCHEDULER_RUNNING       0x08
#define SCHDILER_ORIGIN         0x01
#define EXTERNAL_ENTITY         0x02
#define ALGOHR_INDEX            0x07
#define HEADER_LENGTH 	 			  0x02

//void ECG_SAMPLES_Write_Handler(uint8_t *);
void Update_Connection_Status(uint8_t ID, uint8_t *);
//uint8_t notify_stm(uint8_t ID, uint8_t Sub_Id, uint8_t *Data, uint16_t Len);
void UpdateVitalsReq(uint8_t *Vital_Data);
void InitialiseScheduler(void);
void uart_reinit(void);
void ModifySchTime(uint32_t Delay);
void ResetCommLock(void);
char *Get_Software_Revision(void);
void Send_To_Gatt_Client(uint8_t *Param, uint8_t Service_Len, uint8_t Handle_Value);
void Update_Ind_srcid(uint16_t id);
void Get_Indication_src_id(uint16_t *);
void update_tx_rx_state(uint8_t update_state);
void snd_data_to_stm(OPacket *SendReq);
void sendOverBle(IPacket *Data_Request);
void Send_IndData_Overble(IPacket *Data_Request);
void UART_CLOSE_PORT_INIT(void);
void UART_OPEN_PORT_INIT(void);
void sendUserInfoAck(IPacket *Data_Request);
void sendUserInfoNAck(IPacket *Data_Request);
void PPG_SAMPLES_Write_Handler(uint8_t *PPg_Request);
void hdlHeartRate(IPacket *);
void hdlSpo2(IPacket *);
void hdlCbt(IPacket *);
void hdlBloodPressure(IPacket *);
void hdlAlgo_Hr(IPacket *);
void hdlBatPerct(IPacket *);
void hdlStpCount(IPacket *);
void hdlStpGoal(IPacket *);
void hdlsos(IPacket *);
void hdlVitalBusy(IPacket *);
void hdlVitalError(IPacket *);
void Stop_Scheduler(void);
void InitialiseScheduler(void);
void ModifySchTime(uint32_t );
void send_Ack(uint8_t );
void SendMacIdToStm(void);
void HdlCommand(IPacket *);
void vital_unserviceable(uint8_t id,uint8_t reason);
void hdlSkinTempature(IPacket *Data_Request);
void Update_disconnection_status(void);

#define DELAY_500_US()                                 \
	for (volatile int wait = 0; wait < 0x1500; wait++) \
	{                                                  \
	}



#endif
/*
@END
*/
