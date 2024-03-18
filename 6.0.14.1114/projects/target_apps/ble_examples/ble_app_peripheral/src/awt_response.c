/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "user_custs1_impl.h"
#include "user_custs1_def.h"
#include "User_Application.h"
#include "user_peripheral.h"
# include "uart.h"
//@END

/******************************************************************
* All global variables declaration.  
*******************************************************************/
extern uint8_t BLE_MAC_ID[6];
 uint8_t scheduler_running;

//@END


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

//@END

/***************************************************************************************
 * Incoming packet data segregation.
 ***************************************************************************************/
void sendOverBle(IPacket *Data_Request)
{
	
	switch (Data_Request->subid)
	{
		case comm_wHR:
		{
			hdlHeartRate(Data_Request);
			
		}break;
	  case comm_wSPO2:
		{
			hdlSpo2(Data_Request);
			
		}break;

	  case comm_wCBT:
		{
			hdlCbt(Data_Request);
			
		}break;
		
		case comm_wskintemp:
		{
			hdlSkinTempature(Data_Request);
		}break;
	  case comm_wBP:
		{
			hdlBloodPressure(Data_Request);
			hdlAlgo_Hr(Data_Request); 
		}break;
		case comm_wbatt:
		{
			hdlBatPerct(Data_Request);
			
		}break;
	
		case comm_wstepcount:
		{
			hdlStpCount(Data_Request);
		}break;
		
		case comm_wStepgoalSet:
		{
			hdlStpGoal(Data_Request);
		}break;
		
		case comm_wStepgoal:
				hdlStpGoal(Data_Request);
			break;
		
		case comm_wECG:
					Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, SVC1_ECG_1_VAL);	//sending over network
		break;
		
		case comm_wsos:
				hdlsos(Data_Request);
		break;
		case comm_wUconfig:
			Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, USR_NAME_1_VAL);	//sending over network
		break;
		
		case comm_wVitalBusy:
			hdlVitalBusy(Data_Request);
		break;
		
		case comm_wUpdate:
				Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);	//sending over network
		break;
		
		case comm_wOtaReset:
			Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);	//sending over network
			break;
		
		case comm_wUcConigwrite:
			Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);
			break;
		
		case comm_wFactory_Rst:
			Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);
			break;
		
		case comm_wStepnctRst:
			Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);
			break;
		
		case comm_wfall:
			Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);
			break;
		
//		case comm_wSchedlrControl:
//		{
//			(Data_Request->buffer[0])? InitialiseScheduler() : Stop_Scheduler();
//			 ModifySchTime((Data_Request->buffer[1] << 8) | (Data_Request->buffer[2] & 0xff));			
//		}break;
		
		case comm_wChargerConnected:
			Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);	
		break;
		
		case comm_wBatterylow:
			 Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);	
		break;
		
		case comm_wVitalAlertRange:
				Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);	
		break;
		
		case comm_wChargerRemoved:
				Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);	
		break;
		
		case comm_wblemacid:
				Send_To_Gatt_Client(Data_Request->buffer, Data_Request->size, VITAL_VAL);	
			break;
		
		case 0xff: //vital error msg
				hdlVitalError(Data_Request);
		break;

		
		default :
			return;
		
		
	}
	
}

/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlHeartRate(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= HEART_RATE;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
		
//		if((scheduler_running == true) && (Data_Request->requested_param_id == INDICATION))
//		{
//			vitals_pkt[ORIGIN_TYPE] 	= SCHDILER_ORIGIN;
//		}
//		else if(Data_Request->requested_param_id == RESPONSE)
//		{
//			scheduler_running = 0;  
//		}				
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}
/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlBatPerct(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= BATTERY_PERCENTAGE;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}
/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlSpo2(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= BLOOD_OXYGEN;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
		
//		if((scheduler_running == true) && (Data_Request->requested_param_id == INDICATION))
//		{
//			scheduler_running = false;
//			vitals_pkt[ORIGIN_TYPE] 	= SCHDILER_ORIGIN;
//		}
//		else if(Data_Request->requested_param_id == RESPONSE)
//		{
//			scheduler_running = 0;  
//		}			
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}
/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlCbt(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= BODY_TEMPERATURE;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];		
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}

/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlSkinTempature(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= SKIN_TEMPERATURE;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
		
//		if((scheduler_running == true) && (Data_Request->requested_param_id == INDICATION))
//		{
//			vitals_pkt[ORIGIN_TYPE] 	= SCHDILER_ORIGIN;
//		}
//		else if(Data_Request->requested_param_id == RESPONSE)
//		{
//			scheduler_running = 0;  
//		}			
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}
/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlBloodPressure(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= BLOOD_PRESSURE;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
	
}
/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlStpCount(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= PEDOMETER_SENSOR;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}
/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlStpGoal(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= STEP_GOAL;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}

/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlAlgo_Hr(IPacket *Data_Request)
{
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = ALGOHR_INDEX; 
		vitals_pkt[PARAM_ID] 	     	= ALGO_HR;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}
/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlsos(IPacket *Data_Request)
{	
		uint8_t vitals_pkt[8] = {0};
		uint8_t index = 0;
		vitals_pkt[PARAM_ID] 	     	= SOS_IND;
		vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
		vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
		vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
		vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];		
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);
}
/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlVitalBusy(IPacket *Data_Request)
{
			uint8_t vital_id = 0;
			uint8_t vitals_pkt[8] = {0};
			uint8_t index = 1;
			switch(Data_Request->buffer[0])
			{
					
				case comm_wHR:
					vital_id = HEART_RATE;
					if((scheduler_running == true) && (Data_Request->requested_param_id == INDICATION))
					{
						vitals_pkt[ORIGIN_TYPE] 	= SCHDILER_ORIGIN;
					}
					else
					{	
						vitals_pkt[ORIGIN_TYPE] 	= Data_Request->buffer[6];
					}
					break;
				case comm_wSPO2:
					vital_id = BLOOD_OXYGEN;
					if((scheduler_running == true) && (Data_Request->requested_param_id == INDICATION))
					{
						scheduler_running = 0;
						vitals_pkt[ORIGIN_TYPE] 	= SCHDILER_ORIGIN;
					}
					else
					{	
						vitals_pkt[ORIGIN_TYPE] 	= Data_Request->buffer[6];
					}				
					break;
				case comm_wECG:
					vital_id = EKG;
					break;
				case comm_wskintemp:
					vital_id = SKIN_TEMPERATURE;
					if((scheduler_running == true) && (Data_Request->requested_param_id == INDICATION))
					{
						vitals_pkt[ORIGIN_TYPE] 	= SCHDILER_ORIGIN;
					}
					else
					{	
						vitals_pkt[ORIGIN_TYPE] 	= Data_Request->buffer[6];
					}					
					break;
				case comm_wBP:
					vital_id = BLOOD_PRESSURE;
					vitals_pkt[ORIGIN_TYPE] 	= Data_Request->buffer[6];
					break;				
				
			}

			vitals_pkt[PARAM_ID] 	     	= VITAL_BUSY;
			vitals_pkt[PARAM_VAL_MSB] 	= vital_id;
			vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
			vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
			vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
			vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
			vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
			Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}

/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void vital_unserviceable(uint8_t id,uint8_t reason)
{	
		uint8_t vital_id = 0;
		switch(id)
		{
			case comm_wHR:
				vital_id = HEART_RATE;
				break;
			case comm_wSPO2:
				vital_id = BLOOD_OXYGEN;
				break;
			case comm_wECG:
				vital_id = EKG;
				break;
			case comm_wskintemp:
				vital_id = SKIN_TEMPERATURE;
				break;
			case comm_wBP:
				vital_id = BLOOD_PRESSURE;
				break;
			case comm_wbatt:
				vital_id = BATTERY_PERCENTAGE;
				break;					                   
			case comm_wstepcount:
				vital_id = PEDOMETER_SENSOR;
				break;	
			case comm_wStepgoal:
				vital_id = STEP_GOAL;
				break;	
			case comm_wtimesync:
				vital_id = TIME_Sync;
				break;		
			case comm_wsos:
				vital_id = SOS;
				break;
			case comm_wSwitchtoWifi:
				vital_id = SWITCH_TO_WIFI;
				break;
			case comm_wValidBuild:
				vital_id = UPDATE_HANDLER_OTA;
				break;				
				
		}
		uint8_t vitals_pkt[8] = {0};
		vitals_pkt[PARAM_ID] 	     	= VITAL_BUSY;
		vitals_pkt[PARAM_VAL_MSB] 	= vital_id;
		vitals_pkt[PARAM_VAL_LSB] 	= reason;
		vitals_pkt[TIMESTAMP_3] 	  = 0;
		vitals_pkt[TIMESTAMP_2]   	= 0;
		vitals_pkt[TIMESTAMP_1]   	= 0;
		vitals_pkt[TIMESTAMP_0] 	  = 0;
		vitals_pkt[ORIGIN_TYPE] 	  = EXTERNAL_ENTITY;
		Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network
}

/***************************************************************************************
 * Creates an 8 byte packet structure to be sent over the network.
 ****************************************************************************************/
void hdlVitalError(IPacket *Data_Request)
{
			uint8_t vital_id = 0;
			switch(Data_Request->buffer[0])
			{
				case comm_wHR:
					vital_id = HEART_RATE;
					break;
				case comm_wSPO2:
					vital_id = BLOOD_OXYGEN;
					break;
				case comm_wECG:
					vital_id = EKG;
					break;
				case comm_wskintemp:
					vital_id = SKIN_TEMPERATURE;
					break;
				case comm_wBP:
					vital_id = BLOOD_PRESSURE;
					break;
				case comm_wStepgoalSet:
					vital_id = STEP_GOAL;
					break;
				case comm_wtimesync:
					vital_id = TIME_Sync;
					break;
				
			}
			uint8_t vitals_pkt[8] = {0};
			uint8_t index = 1;
			vitals_pkt[PARAM_ID] 	     	= vital_id;
			vitals_pkt[PARAM_VAL_MSB] 	= Data_Request->buffer[index++];
			vitals_pkt[PARAM_VAL_LSB] 	= Data_Request->buffer[index++];
			vitals_pkt[TIMESTAMP_3] 	  = Data_Request->buffer[index++];
			vitals_pkt[TIMESTAMP_2]   	= Data_Request->buffer[index++];
			vitals_pkt[TIMESTAMP_1]   	= Data_Request->buffer[index++];
			vitals_pkt[TIMESTAMP_0] 	  = Data_Request->buffer[index++];
			vitals_pkt[ORIGIN_TYPE] 	  = Data_Request->buffer[index++];
			Send_To_Gatt_Client(vitals_pkt, VITAL_CHAR_LEN, VITAL_VAL);	//sending over network	
}

/***************************************************************************************
 * Send userinfo Ack confirmation to app
 ****************************************************************************************/
void sendUserInfoAck(IPacket *Data_Request)
{
				uint8_t ack = Data_Request->subid;
				Send_To_Gatt_Client(&ack, sizeof(ack), USR_NAME_1_VAL);
}

/***************************************************************************************
 * Send userinfo Nack failure to app
 ****************************************************************************************/
void sendUserInfoNAck(IPacket *Data_Request)
{
				uint8_t ack = Data_Request->subid;
				Send_To_Gatt_Client(&ack, sizeof(ack), USR_NAME_1_VAL);
}

/***************************************************************************************
 * Send data to app over the network.
 ****************************************************************************************/
void Send_To_Gatt_Client(uint8_t *Param, uint8_t Service_Len, uint8_t Handle_Value)
{
	uint16_t src_id;
	if (ke_state_get(TASK_APP) == APP_CONNECTED)
	{
		struct custs1_val_ind_req* req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_IND_REQ,
                                                          prf_get_task_from_id(TASK_ID_CUSTS1),
                                                          TASK_APP,
                                                          custs1_val_ind_req,
                                                          Service_Len);
		//get src ID
		Get_Indication_src_id(&src_id);
		uint8_t conidx = KE_IDX_GET(src_id);
		
    req->conidx = app_env[conidx].conidx;
		req->handle = Handle_Value;
    req->length = Service_Len;
		memcpy(req->value, Param, Service_Len);

    ke_msg_send(req);
		
	}
}


/******************************************************************
* Function to send the MACID to ST chip  
*******************************************************************/
void SendMacIdToStm(void)
{
	OPacket opacket;
	opacket.header = 0xBE;
  opacket.requested_param_id = RESPONSE;
	opacket.subid = comm_wblemacid;
	opacket.size = 0x06;
	opacket.footer = 0xFE;
	memcpy(&opacket.buffer,BLE_MAC_ID,0x06);
	uart_send(UART1, (uint8_t *)&opacket, (HDR_SIZE + opacket.size), UART_OP_INTR);

}

/******************************************************************
* Handle command request send from the ST for MACID.
*******************************************************************/
void HdlCommand(IPacket *Data_Request)
{
	switch(Data_Request->subid)
	{
			case comm_wUpdate:
//			Data_Request->buffer[0] = 18;
//			Send_To_Gatt_Client(Data_Request->buffer, 8, VITAL_VAL);
			break;
			case comm_wblemacid:
					SendMacIdToStm();
			break;
	}
}


