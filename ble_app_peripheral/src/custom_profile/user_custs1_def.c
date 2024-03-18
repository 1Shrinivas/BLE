/**
 ****************************************************************************************
 *
 * @file user_custs1_def.c
 *
 * @brief Custom Server 1 (CUSTS1) profile database definitions.
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @defgroup USER_CONFIG
 * @ingroup USER
 * @brief Custom server 1 (CUSTS1) profile database definitions.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "co_utils.h"
#include "prf_types.h"
#include "attm_db_128.h"
#include "user_custs1_def.h"
#define USER_INFO_STRING 0

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// Service 1 of the custom server 1
static const att_svc_desc128_t Ecg_Service                      = ECG_SVC_UUID_128;

static const uint8_t ECG_SAMPLES_UUID[ATT_UUID_128_LEN]         = ECG_SAMPLES_UUID_128;

static const att_svc_desc128_t Dbg_Service 											 = DBG_SERVICE_UUID_128;
static const uint8_t DBG_WIRELESS_UUID[ATT_UUID_128_LEN]         = DBG_CHAR_UUID_128;


static const uint8_t PPG_SAMPLES_UUID[ATT_UUID_128_LEN]             = PPG_SAMPLES_UUID_128;



// Service 3 of the custom server 1
static const att_svc_desc128_t Vital_Service                  	 = VITAL_SERVICE_UUID_128;
	
static const uint8_t VITAL_UUID[ATT_UUID_128_LEN]      					 = 	VITAL_STATUS_UUID_128;

// TIME Service of the custom server 1



//ALERT SERVICE
static const att_svc_desc128_t  Alert_Service     							     = ALERT_SERVICE_UUID_128;
static const uint8_t ALERT_STATUS_UUID[ATT_UUID_128_LEN]              = ALERT_SERVICE_UUID;


//static const uint8_t ALERT_MSG_UUID[ATT_UUID_128_LEN]       		     = ALERT_MSG_UUID_128;
//static const uint8_t ALERT_TYPE_UUID[ATT_UUID_128_LEN]       		     = ALERT_TYPE_UUID_128;

//USER INFO Service  of the custom server 1

static const att_svc_desc128_t User_Info_Service                    					  = USER_INFO_SERVICE;

static const uint8_t USR_INFO_UUID_128[ATT_UUID_128_LEN]       								  =	NAME_SERVICE_CHAR;//CHAR 1 



// Attribute specifications
static const uint16_t att_decl_svc       = ATT_DECL_PRIMARY_SERVICE;
static const uint16_t att_decl_char      = ATT_DECL_CHARACTERISTIC;
static const uint16_t att_desc_cfg       = ATT_DESC_CLIENT_CHAR_CFG;
static const uint16_t att_desc_user_desc = ATT_DESC_CHAR_USER_DESCRIPTION;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

const uint8_t custs1_services[]  = {SVC1_ECG_SVC,VITALS_IDX_SVC,ALERT_SVC_IDX_SVC,USR_IDX_SVC, WIRELESSS_DBG_SVC, CUSTS1_IDX_NB};
const uint8_t custs1_services_size = ARRAY_LEN(custs1_services) - 1;
const uint16_t custs1_att_max_nb = CUSTS1_IDX_NB;

/// Full CUSTS1 Database Description - Used to add attributes into the database
const struct attm_desc_128 custs1_att_db[CUSTS1_IDX_NB] =
{
    /*************************
     * Service 1 configuration
     *************************
     */

//  ECG_SAMPLES_UUID
    // Service 1 Declaration
    [SVC1_ECG_SVC]                     = {(uint8_t*)&att_decl_svc, ATT_UUID_128_LEN, PERM(RD, ENABLE),
                                            sizeof(Ecg_Service), sizeof(Ecg_Service), (uint8_t*)&Ecg_Service},


    // ECG Value 1 Characteristic Declaration
    [SVC1_ECG_1_CHAR]          = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // ECG Value 1 Characteristic Value
    [SVC1_ECG_1_VAL]           = {ECG_SAMPLES_UUID, ATT_UUID_128_LEN, PERM(RD, ENABLE) | PERM(IND, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            ECG_SAMPLES_CHAR_LEN, 0, NULL},
		
																						
    // ECG Value 1 Client Characteristic Configuration Descriptor
    [SVC1_ECG_1_IND_CFG]       = {(uint8_t*)&att_desc_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            sizeof(uint16_t), 0, NULL},
		
		//Service Characteristics
		[SVC1_PPG_1_CHAR]               = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},
		//Service Write Propertie
		[SVC1_PPG_1_VAL]                = {PPG_SAMPLES_UUID, ATT_UUID_128_LEN, PERM(RD, ENABLE) | PERM(IND, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            PPG_SAMPLES_CHAR_LEN, 0, NULL},
		
		//Service Notify Propertie
		[SVC1_PPG_1_IND_CFG]						= {(uint8_t*)&att_desc_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                           sizeof(uint16_t), 0, NULL},																				
	
    // ECG Value 1 Characteristic User Description
    [SVC1_ECG_VAL_1_USER_DESC]     = {(uint8_t*)&att_desc_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(DEF_SVC1_ADC_VAL_1_USER_DESC) - 1, sizeof(DEF_SVC1_ADC_VAL_1_USER_DESC) - 1,
                                            (uint8_t *) DEF_SVC1_ADC_VAL_1_USER_DESC},
    /*************************
     * Vitals Service configuration
     *************************
     */
																						 

			[VITALS_IDX_SVC] = {(uint8_t*)&att_decl_svc, ATT_UUID_128_LEN, PERM(RD, ENABLE),
                         sizeof(Vital_Service), sizeof(Vital_Service), (uint8_t*)&Vital_Service},
			
		  // Vitals STATUS Characteristic
			
			[VITAL_CHAR] = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE), 0, 0, NULL},
			
			
			[VITAL_VAL] = {VITAL_UUID, ATT_UUID_128_LEN, PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE) |
                                          PERM(IND, ENABLE) , VITAL_CHAR_LEN, 0, NULL},
															
			
			[VITAL_IND] = {(uint8_t*)&att_desc_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),sizeof(uint16_t), 0, NULL},
				
				/*************************
     *  ALERT Service configuration
     *************************
     */
				
			[ALERT_SVC_IDX_SVC] = {(uint8_t*)&att_decl_svc, ATT_UUID_128_LEN, PERM(RD, ENABLE),
                         sizeof(Alert_Service), sizeof(Alert_Service), (uint8_t*)&Alert_Service},
	      
			//Characteristic												 
			[ALRT_TTL_CHAR] = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE), 0, 0, NULL},
			
			//Service Write Propertie
		  [ALRT_TTL_VAL] = {ALERT_STATUS_UUID, ATT_UUID_128_LEN, PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE)| PERM(IND, ENABLE) ,ALERT_INFO_SERVICE_LEN, 0, NULL}, 

			//Service Notify Propertie
			 [ALERT_TTL_IND] 	= {(uint8_t*)&att_desc_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),sizeof(uint16_t), 0, NULL},
				 
			
			/*************************
     *  USER INFO Service configuration
     *************************
     */
			//Service Creation
	[USR_IDX_SVC]                     = {(uint8_t*)&att_decl_svc, ATT_UUID_128_LEN, PERM(RD, ENABLE), sizeof(User_Info_Service), sizeof(User_Info_Service), (uint8_t*)&User_Info_Service},
	
	//Service Characteristics
  [USR_NAME_1_CHAR]          				= {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),0, 0, NULL},
  
	//Service Write Propertie
	[USR_NAME_1_VAL]                  = {USR_INFO_UUID_128, ATT_UUID_128_LEN, PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE) |
                                          PERM(IND, ENABLE) , USER_INFO_SERVICE_NAME_LEN, 0, NULL},
	//Service Notify Propertie
	[USR_NAME_1_IND] 									= {(uint8_t*)&att_desc_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),sizeof(uint16_t), 0, NULL},
		
		//Service Creation
	[WIRELESSS_DBG_SVC]                     = {(uint8_t*)&att_decl_svc, ATT_UUID_128_LEN, PERM(RD, ENABLE), sizeof(Dbg_Service), sizeof(Dbg_Service), (uint8_t*)&Dbg_Service},
	
	//Service Characteristics
  [WIRELESSS_DBG_1_CHAR]          				= {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),0, 0, NULL},
  
	//Service Write Propertie
	[WIRELESSS_DBG_1_VAL]                  = {DBG_WIRELESS_UUID, ATT_UUID_128_LEN, PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE) |
                                          PERM(IND, ENABLE) , DBG_SAMPLES_CHAR_LEN, 0, NULL},
	//Service Notify Propertie
	[WIRELESSS_DBG_1_IND_CFG] 									= {(uint8_t*)&att_desc_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),sizeof(uint16_t), 0, NULL},
		
		// Dbg_Service DBG_WIRELESS_UUID

	
};

