//# include "gpio.h"
//# include "uart.h"
//# include "comm_manager.h"
//# include "User_Application.h"



//volatile enum op_mode mode = close;
//static uint8_t port_status =0;



//IPacket ipacket;
//OPacket opacket;


//uint16_t  ReadUart(uart_t *Uart_Instance, uint8_t *Dest, uint16_t Length);
//static int8_t Handle_Incomming(uart_t *Uart_Instance, IPacket *_ipacket);
//static int8_t Process_Incomming(IPacket *_ipacket);
//static int8_t Slave_Response(IPacket *_ipacket);
//void serviceCommand(IPacket *Data_Request);
////uint16_t UART_PEEK(uart_t* Uart_Instance);


//extern bool Comm_Lock;

//void send_Ack(uint8_t type)
//{
//	OPacket opacket;
//	
//	opacket.header = 0xBE;
//  opacket.requested_param_id = 0x03;
//	opacket.subid = type;
//	opacket.size = 0x00;
//	opacket.footer = 0xFE;
//	uart_send(UART1, (uint8_t *)&opacket, HDR_SIZE, UART_OP_BLOCKING);
//}
///******************************************************************
//*
//*	Open Port and signal ST with  P03
//* High  open port and signal ST
//* Low   close port and signal ST
//*
//*******************************************************************/
//void port_open(uint8_t state)
//{	
//	if(state)
//	{		
//		UART_OPEN_PORT_INIT();	
//		GPIO_SetActive(GPIO_PORT_0, GPIO_PIN_3);	
//  }
//  else 
//	{
//		UART_CLOSE_PORT_INIT();	
//		GPIO_SetInactive(GPIO_PORT_0, GPIO_PIN_3);	
//		memset(&ipacket, 0, sizeof(ipacket));
//		
//	}		
//}

///******************************************************************
//*	
//*
//*    Trigger from ST to open port  P04.
//* 
//*
//*
//********************************************************************/
//void uart_trigger(void)
//{		
//	NVIC_DisableIRQ(ASM_GPIO_IRQ);
//	
//	bool Edge = GPIO_GetPinStatus(GPIO_PORT_0, GPIO_PIN_4);
//	
//	
//	if(Edge)
//	{
//		// handler code 
//		
//		port_status =1;
//		
//		// end of handler code 
//		
//		
//		// interrupt handling 
//		GPIO_ResetIRQ(ASM_GPIO_IRQ);
//		NVIC_ClearPendingIRQ(ASM_GPIO_IRQ);
//		GPIO_EnableIRQ(GPIO_PORT_0, GPIO_PIN_4, ASM_GPIO_IRQ, true, true, 0);
//		
//	}
//	else
//	{
//		// handler code 
//		
//		port_status =0;
//		
//		// end of handler code 
//		
//		// interrupt handling
//		GPIO_ResetIRQ(ASM_GPIO_IRQ);
//		NVIC_ClearPendingIRQ(ASM_GPIO_IRQ);
//		GPIO_EnableIRQ(GPIO_PORT_0, GPIO_PIN_4, ASM_GPIO_IRQ, false, true, 0);
//		
//		
//	}
//		
//	NVIC_EnableIRQ(ASM_GPIO_IRQ);
//}



//void handle_close()
//{
//	if (port_status)
//	{
//		mode = s_0;		
//	}
//}

//void handle_idle()
//{
//  if(port_status ==0)
//	{
//	  port_open(0);
//		mode = close;
//		return;
//	}	
//	
////	int ret = ReadUart(UART1, (uint8_t *)&ipacket, HDR_SIZE);
////	if(ret >0)
////	{	  
////		mode=s_rx;
////	}

//}
//	
//void handle_slave_init()
//{
//	port_open(1);
//	mode = s_rx ;
//}

//void handle_slave_rx()
//{
//	if(port_status ==0)
//	{
//	  port_open(0);
//		mode = close;
//		return;
//	}
//	
//	int ret = Handle_Incomming(UART1, &ipacket);
//	if(ret ==1)
//	{
//		mode = s_tx;
//	}
//	else if(ret == -1)
//	{
//	  //read error
//		port_open(0);
//		
//	}
//	
//	return;
//  
//}
//	
//void handle_slave_tx()
//{
//	if(port_status ==0)
//	{
//	  port_open(0);
//		mode = close;
//		return;
//	}
//  int ret = Slave_Response(&ipacket);
//	if(ret ==1)
//	{
//	  mode = idle; 
//		memset(&ipacket, 0, sizeof(ipacket));
//	}
//	
//}
//	

//	
//void handle_master_init()
//{	
//	if(port_status)
//	{
//	  mode = m_tx;
//	}
//}

//void handle_master_tx()
//{
//	if(port_status ==0)
//	{
//	  port_open(0);
//		mode = close;
//		return;
//	}	
//	snd_data_to_stm(&opacket);
//	mode = m_rx;	
//	
//}
//	
//void handle_master_rx()
//{
//	
//	int ret = Handle_Incomming(UART1, &ipacket);
//	if(ret ==1)
//	{		
//		ret =Process_Incomming(&ipacket);
//		if(ret ==1)
//		{
//			mode = idle; 
//			memset(&ipacket, 0, sizeof(ipacket));
//		}
//		
//	}
//	else if(ret == -1)
//	{
//	  //read error
//		port_open(0);
//		
//	}
//}
//	



///******************************************************************
//* check_state
//********************************************************************/
//void check_state(void)
//{
//	// s_0,s_rx,s_tx
//	switch(mode)
//	{
//		case close:
//		{
//			handle_close();
//		}break;
//		case idle:
//		{
//			handle_idle();
//		}break;
//		case m_0:
//		{
//			handle_master_init();
//		}break;
//		case m_tx:
//		{
//			handle_master_tx();
//		}break; 
//		case m_rx:
//		{
//			handle_master_rx();
//		}break;
//		case s_0:
//		{	
//			handle_slave_init();
//		}break; 
//		case s_tx:
//		{
//			handle_slave_tx();	
//		}break; 
//		case s_rx:
//		{
//			handle_slave_rx();
//		}break;
//		
//		default:
//		{
//			
//		}break;		
//	}
//}


//# define Header() (_ipacket->pendRead < HDR_SIZE)
///****************************************************************************************
// * return 0 when request is being processed
// * return 1 when request has been  handled
// * 
// * 
// * 
//******************************************************************************************/

//static int8_t Handle_Incomming(uart_t *Uart_Instance, IPacket *_ipacket)
//{    
//	// Reads the complete header   
//	if( Header() )
//	{
//		_ipacket->pendRead = ReadUart(Uart_Instance, (uint8_t *)_ipacket, HDR_SIZE);		
//		if ((_ipacket->pendRead == HDR_SIZE) && ((_ipacket->header!= 0xBE) || (_ipacket->footer!= 0xFE)))
//		{
//				return -1;
//		}
//    else
//		    return 0;
//	}
//	// Reads the complete payload
//	else
//	{     
//		if((_ipacket->pendRead = ReadUart(Uart_Instance, (uint8_t *)_ipacket->buffer, _ipacket->size) + HDR_SIZE) == ( _ipacket->size + HDR_SIZE) )
//		{
//		    return 1;			
//		}
//		else return 0;
//	}
//}

//int8_t Slave_Response(IPacket *_ipacket)
//{
//	if (_ipacket->header != 0xBE)
//	{
//		return -1; // Invalid Header
//	}
//	switch (_ipacket->requested_param_id)
//	{
//			case INDICATION:
//			{
//				sendOverBle(_ipacket);
//				return 1;
//			}

//			case COMMAND:    // process command 
//			{
//				
//				HdlCommand(_ipacket);
//				return 1;
//			}
//			
//			case RESPONSE: //Data handling during measurement after disconnecting and reconnecting
//			{
//				sendOverBle(_ipacket);
//				return 1;
//			}
//			
//			default:
//			{
//				return 0;
//			}
//	}
//}

//int8_t Process_Incomming(IPacket *_ipacket)
//{
//	if (_ipacket->header != 0xBE)
//	{
//		return -1;
//	}
//	switch (_ipacket->requested_param_id)
//	{
//		case RESPONSE:
//		{
//			sendOverBle(_ipacket);
//			return 1;
//		}

//		case 0x03:
//		{
//			if((_ipacket->buffer[0] == comm_wUconfig) && (_ipacket->subid == 0x06))
//			{
//				sendUserInfoAck(_ipacket); //ACK
//			}
//			else if((_ipacket->buffer[0] == comm_wUconfig) && (_ipacket->subid == 0x15))
//			{
//				sendUserInfoNAck(_ipacket); //Nack
//			}
//			return 1;
//		}

//	}
//    return 0;
//}


///****************************************************************************************
// * 
// * return 1   host busy 
// * return 0   transmission success 
// * 
// * 
//******************************************************************************************/
//uint8_t	TX_Event(uint8_t ID, uint8_t Sub_Id, uint8_t *src, uint16_t Len)
//{

//	if( ( mode!=idle ) && (mode!=close) )
//	{
//		return 1;
//	}

//	memset(&opacket, 0, sizeof(OPacket)); // Clearing the entire structure. 
//	opacket.header = 0xBE;
//	opacket.requested_param_id = ID;
//	opacket.subid = Sub_Id;
//	opacket.size =  Len ;
//	opacket.footer = 0xFE;
//	
//	if (Len)
//	{
//		memcpy(opacket.buffer, src, Len); // check for passkey.
//	}

//	
//	if(mode == idle)
//	{
//		mode = m_tx;
//	}
//	else if (mode == close)
//	{
//		mode = m_0;
//		port_open(1);
//	}
//	
//	return 0;	

//}

///****************************************************************************************
// * Taking care of disconnection when the host's busy  
// * ***************************************************************************************/
//void Update_disconnection_status(void) 
//{
//	if( ( mode!=idle ) && (mode!=close) )
//	{
//		if(GPIO_GetPinStatus(GPIO_PORT_0, GPIO_PIN_3))
//		{
//			port_open(0);
//			DELAY_500_US(); //for debug
//			port_status = 0;			
//			mode = close;
//		}
//	}
//}
