/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "user_custs1_impl.h"
#include "user_custs1_def.h"
#include "User_Application.h"
# include "uart.h"
//@END

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
void USER_INFO_SENDING_Function(struct custs1_val_write_ind const *);
void update_alert_data(struct custs1_val_write_ind const *ptr);
void handle_Ind_Cfm( struct custs1_val_ind_cfm const *msg_param);
void send_totalpress_count_to_app();
//@END

/******************************************************************
* All global variables declaration.  
*******************************************************************/
uint8_t BLE_MAC_ID[6];
uint16_t Indication_Src_ID = 0x00;
__attribute__((section("text")))
char device_config[700] = "BME_680"; //don't change sub string name

//@END

/******************************************************************
*
* All incoming messages requesting from the network must be processed immediately.	  
*
*******************************************************************/
void user_catch_rest_hndl(ke_msg_id_t const msgid,
						  void const *param,
						  ke_task_id_t const dest_id,
						  ke_task_id_t const src_id)
{
	Update_Ind_srcid(src_id);

	switch (msgid)
	{
	case CUSTS1_VAL_WRITE_IND:
	{
		struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)(param);
		// update src_id
		Update_Ind_srcid(src_id);

		switch (msg_param->handle)
		{

		case VITAL_VAL:
			UpdateVitalsReq((uint8_t *)msg_param->value);
			break;

		
		
/*		
		case SVC1_ECG_1_VAL: /// SVC1_PPG_1_VAL
			ECG_SAMPLES_Write_Handler((uint8_t *)msg_param->value);
				
			break;

		case SVC1_PPG_1_VAL: /// SVC1_PPG_1_VAL
			PPG_SAMPLES_Write_Handler((uint8_t *)msg_param->value);
			break;

		case USR_NAME_1_VAL:
			USER_INFO_SENDING_Function(msg_param);
			break;

		case ALRT_TTL_VAL:
			update_alert_data(msg_param);
			break;

*/
		default:
			break;
		}
	}
	break;

	case CUSTS1_VAL_NTF_CFM:
	{
	}
	break;

	case CUSTS1_VAL_IND_CFM:
	{
		struct custs1_val_ind_cfm const *msg_param = (struct custs1_val_ind_cfm const *)(param);
		handle_Ind_Cfm(msg_param);
	}
	break;

	case GAPC_PARAM_UPDATED_IND:
	{
		// Cast the "param" pointer to the appropriate message structure
		struct gapc_param_updated_ind const *msg_param = (struct gapc_param_updated_ind const *)(param);

		// Check if updated Conn Params filled to preferred ones
		if ((msg_param->con_interval >= user_connection_param_conf.intv_min) &&
			(msg_param->con_interval <= user_connection_param_conf.intv_max) &&
			(msg_param->con_latency == user_connection_param_conf.latency) &&
			(msg_param->sup_to == user_connection_param_conf.time_out))
		{
		}
	}
	break;

	case CUSTS1_VALUE_REQ_IND:
	{
		struct custs1_value_req_ind const *msg_param = (struct custs1_value_req_ind const *)param;

		switch (msg_param->att_idx)
		{

		default:
		{
			// Send Error message
			struct custs1_value_req_rsp *rsp = KE_MSG_ALLOC(CUSTS1_VALUE_REQ_RSP,
															src_id,
															dest_id,
															custs1_value_req_rsp);

			// Provide the connection index.
			rsp->conidx = app_env[msg_param->conidx].conidx;
			// Provide the attribute index.
			rsp->att_idx = msg_param->att_idx;
			// Force current length to zero.
			rsp->length = 0;
			// Set Error status
			rsp->status = ATT_ERR_APP_ERROR;
			// Send message
			ke_msg_send(rsp);
		}
		break;
		}
	}
	break;

	default:
		break;
	}
}

 /* FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/******************************************************************
* All vital requests Handling.	  
*******************************************************************/
//uint16_t get_adcfun(void);

//void Send_adc_to_app()
//{

//		
//	uint8_t val[8] = {0};
//	//uint16_t adc = get_adcfun();
//	val[0]=  (adc >> 8 & 0xff);
//	val[1]=  adc  & 0xff;
//	Send_To_Gatt_Client(val, 8, VITAL_VAL);	//sending over network
//	
//	//for(uint32_t i=0;i<100000;i++);

//	

//}


void UpdateVitalsReq(uint8_t *Vital_Data)
{
	
	if(*Vital_Data == 0x01) // temperature
	{
		send_temp_to_app();
	}	
	if(*Vital_Data == 0x02) // Humidity 
	{
		send_Humidity_to_app();
	}
	if(*Vital_Data == 0x03) // Pressure
	{
		send_pressure_to_app();

	}
	if(*Vital_Data == 0x04) // gas
	{
		send_Gas_resi_to_app();
	}
	if(*Vital_Data == 0x05) // gas
	{
		send_Co2_equivalent_to_app();
	
	}
	if(*Vital_Data == 0x06) // gas
	{
		send_IAQ_equivalent_to_app();
	}

	
/*	
	return;
	uint8_t Return = 0x00;
	
	switch(*Vital_Data)
	{
		
		case HEART_RATE:
			Return = notify_stm(COMMAND, comm_wHR, NULL, NULL);
		break;
		
		case BLOOD_OXYGEN:		
				Return = notify_stm(COMMAND, comm_wSPO2, NULL, NULL);
		break;
		case BODY_TEMPERATURE:
				//Return = notify_stm(COMMAND, comm_wCBT, NULL, NULL); //CBT disabled
		break;
		
		case SKIN_TEMPERATURE:
			Return = notify_stm(COMMAND, comm_wskintemp, NULL, NULL);
		break;
		
		case BLOOD_PRESSURE:
			Return = notify_stm(COMMAND, comm_wBP, NULL, NULL); 
			break;
		
		case BATTERY_PERCENTAGE:
			Return = notify_stm(COMMAND, comm_wbatt, NULL, NULL);
			break;
		
		case TIME_Sync:
				 Return = notify_stm(INDICATION, comm_wtimesync, &Vital_Data[3], sizeof(uint32_t));
			break;

		case PEDOMETER_SENSOR:
				Return = notify_stm(COMMAND, comm_wstepcount, NULL, NULL); 		
			break;
		
		case STEP_GOAL:
				Return = notify_stm(COMMAND, comm_wStepgoalSet, &Vital_Data[1], sizeof(uint16_t));
			break;	

		case UPDATE_HANDLER_OTA:
				Return = notify_stm(INDICATION, comm_wValidBuild, &Vital_Data[2], sizeof(uint8_t));
			break;
		
		case SOS:
			Return = notify_stm(INDICATION, comm_wsos,&Vital_Data[2], sizeof(uint8_t) );
		break;
		
		case SETUP_CONFIG:
			Return = notify_stm(COMMAND, comm_wSetupWrComplete, NULL, NULL);
		break;
		
		case SWITCH_TO_WIFI:
			Return = notify_stm(INDICATION, comm_wSwitchtoWifi, NULL, NULL);
		break;
		
		case BT_MAC_ID:
			Return = notify_stm(COMMAND, comm_wblemacid, NULL, NULL);
		break;		
		
	*/	



}

/******************************************************************
* Checking for the TX event current state.
* In case of busy state, it sends a vital busy message over the network
*******************************************************************/
//uint8_t notify_stm(uint8_t ID, uint8_t Sub_Id, uint8_t *Data, uint16_t Len)
//{	
//	if(TX_Event(ID, Sub_Id, Data, Len)) //Handling Vitals unserviceable cases 
//	{ 
//			vital_unserviceable(Sub_Id,READING_IN_PROGRESS);
//			return 1;
//	}
//	else
//	{		
//		return 0;
//	}

//}

/******************************************************************
* Function to send ECG data request to ST	  
*******************************************************************/
//void ECG_SAMPLES_Write_Handler(uint8_t *Ecg_Request)
//{
//	if(*Ecg_Request == EKG)
//	{
//		notify_stm(COMMAND, comm_wECG, NULL, NULL);
//	}
//}

/******************************************************************
* Function to send PPG data request to ST	  
*******************************************************************/
//void PPG_SAMPLES_Write_Handler(uint8_t *PPg_Request)
//{
//	// notify_stm(PPG_SAMPLES_ID, *PPg_Request, NULL, NULL);
//}

/******************************************************************
* Function to send alert services data to ST	  
*******************************************************************/
void update_alert_data(struct custs1_val_write_ind const *ptr)
{
	uint8_t *alert_info = (uint8_t *)ptr->value;
	uint8_t size = ptr->length;
	
	if(size > ALERTS_LENGTH)
	{
		uint8_t error[8] = {0xff}; //not valid alert msg
		Send_To_Gatt_Client(error, VITAL_CHAR_LEN, VITAL_VAL);
	}
	else
	{
		//notify_stm(INDICATION, comm_wAlerts, alert_info, size);
	}	
}


/******************************************************************
* Function to send the User Info Data	to ST  
*******************************************************************/
void USER_INFO_SENDING_Function(struct custs1_val_write_ind const *ptr)
{
	DELAY_500_US();
	
	// Byte 0 to Byte 12 is the user info header.
	// Payload is of 128 bytes when the mode is BULK write and BULK read.
	switch(ptr->value[0])
	{
		case 1:
//			notify_stm(INDICATION, comm_wUconfig, (uint8_t *)ptr->value, ptr->length);	// BULK WRITE Scenario
			break;
		case 2:
//			notify_stm(COMMAND, comm_wUconfig, (uint8_t *)ptr->value, ptr->length);		// BULK read Scenario
			break;
		case 3:
			break;
		case 4:
			break;
		
	}	
	return;
}

/******************************************************************
* Function to update MACID global buffer.  
*******************************************************************/
void UpdateBleMacId(uint8_t *Mac_ID)
{
	for (uint8_t i = 0, j = 5; i < 6; i++)
	{
		BLE_MAC_ID[i] = Mac_ID[j--];
	}
}

void snd_data_to_stm(OPacket *SendReq)
{
	uart_send(UART1, (uint8_t *)SendReq, SendReq->size + HDR_SIZE, UART_OP_INTR);
}

void handle_Ind_Cfm( struct custs1_val_ind_cfm const *msg_param)
{	
	if(msg_param->handle == SVC1_PPG_1_VAL && msg_param->status == IND_SUCESS){
		//send_Ack(ACK);// send ack/nack
	}
	if(msg_param->handle == SVC1_ECG_1_VAL && msg_param->status == IND_SUCESS){
	//	send_Ack(ACK);// send ack/nack
	}
	if(msg_param->handle == VITAL_VAL && msg_param->status == IND_SUCESS){
		if(GPIO_GetPinStatus(GPIO_PORT_0, GPIO_PIN_4))
		{
			// code enters here only if the data was received as an indication.
			//send_Ack(ACK);
		}
	}
	if(msg_param->handle == ALRT_TTL_VAL && msg_param->status == IND_SUCESS){
		//send_Ack(ACK);// send ack/nack
	}
	if(msg_param->handle == USR_NAME_1_VAL && msg_param->status == IND_SUCESS){
	//	send_Ack(ACK);// send ack/nack
	}
	else if(msg_param->handle == SVC1_PPG_1_VAL && msg_param->status!=IND_SUCESS){
		// Send Nack
	}
	
}

void Update_Ind_srcid(uint16_t id)
{
	Indication_Src_ID = id;
}

void Get_Indication_src_id(uint16_t *ID)
{
		*ID = Indication_Src_ID;
}

char * Get_Software_Revision(void)
{
		return (char *)(&device_config[SOFT_REVISION_OFFSET]);
	
}

uint8_t* GetBondInfo(void)
{	
	return (uint8_t *)(&device_config[BND_INFO_OFSET]);
}
