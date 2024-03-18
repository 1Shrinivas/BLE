/**
 ****************************************************************************************
 *
 * @file user_peripheral.c
 *
 * @brief Peripheral project source code.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h" // SW configuration
#include "gap.h"
#include "app_easy_timer.h"
#include "user_peripheral.h"
#include "user_custs1_impl.h"
#include "user_custs1_def.h"
#include "co_bt.h"
#include "uart.h"
#include "app.h"
#include "app_prf_perm_types.h"
#include "app_easy_security.h"
#include "app_security.h"
#include "app_task.h"
#include "app_utils.h"
#include "User_Application.h"

#if (WLAN_COEX_ENABLED)
#include "wlan_coex.h"
#include "lld.h"
#endif
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
#define _30SECONDS 3000
#define _1SECOND 100
uint16_t ADVERTISEMENT_DURATION = _1SECOND;
bool Comm_Lock = false;
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



// Manufacturer Specific Data ADV structure type
struct mnf_specific_data_ad_structure
{
	uint8_t ad_structure_size;
	uint8_t ad_structure_type;
	uint8_t company_id[APP_AD_MSD_COMPANY_ID_LEN];
	uint8_t proprietary_data[APP_AD_MSD_DATA_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

uint8_t app_connection_idx __SECTION_ZERO("retention_mem_area0");
timer_hnd app_adv_data_update_timer_used __SECTION_ZERO("retention_mem_area0");
timer_hnd app_param_update_request_timer_used __SECTION_ZERO("retention_mem_area0");

// Retained variables
struct mnf_specific_data_ad_structure mnf_data __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
// Index of manufacturer data in advertising data or scan response data (when MSB is 1)
uint8_t mnf_data_index __SECTION_ZERO("retention_mem_area0");						   //@RETENTION MEMORY
uint8_t stored_adv_data_len __SECTION_ZERO("retention_mem_area0");					   //@RETENTION MEMORY
uint8_t stored_scan_rsp_data_len __SECTION_ZERO("retention_mem_area0");				   //@RETENTION MEMORY
uint8_t stored_adv_data[ADV_DATA_LEN] __SECTION_ZERO("retention_mem_area0");		   //@RETENTION MEMORY
uint8_t stored_scan_rsp_data[SCAN_RSP_DATA_LEN] __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Manufacturer Specific Data
 ****************************************************************************************
 */
static void mnf_data_init()
{
	mnf_data.ad_structure_size = sizeof(struct mnf_specific_data_ad_structure) - sizeof(uint8_t); // minus the size of the ad_structure_size field
	mnf_data.ad_structure_type = GAP_AD_TYPE_MANU_SPECIFIC_DATA;
	mnf_data.company_id[0] = APP_AD_MSD_COMPANY_ID & 0xFF;		  // LSB
	mnf_data.company_id[1] = (APP_AD_MSD_COMPANY_ID >> 8) & 0xFF; // MSB
	mnf_data.proprietary_data[0] = 0;
	mnf_data.proprietary_data[1] = 0;
}

/**
 ****************************************************************************************
 * @brief Update Manufacturer Specific Data
 ****************************************************************************************
 */
static void mnf_data_update()
{
	uint16_t data;

	data = mnf_data.proprietary_data[0] | (mnf_data.proprietary_data[1] << 8);
	data += 1;
	mnf_data.proprietary_data[0] = data & 0xFF;
	mnf_data.proprietary_data[1] = (data >> 8) & 0xFF;

	if (data == 0xFFFF)
	{
		mnf_data.proprietary_data[0] = 0;
		mnf_data.proprietary_data[1] = 0;
	}
}

/**
 ****************************************************************************************
 * @brief Add an AD structure in the Advertising or Scan Response Data of the
 *        GAPM_START_ADVERTISE_CMD parameter struct.
 * @param[in] cmd               GAPM_START_ADVERTISE_CMD parameter struct
 * @param[in] ad_struct_data    AD structure buffer
 * @param[in] ad_struct_len     AD structure length
 * @param[in] adv_connectable   Connectable advertising event or not. It controls whether
 *                              the advertising data use the full 31 bytes length or only
 *                              28 bytes (Document CCSv6 - Part 1.3 Flags).
 ****************************************************************************************
 */
static void app_add_ad_struct(struct gapm_start_advertise_cmd *cmd, void *ad_struct_data, uint8_t ad_struct_len, uint8_t adv_connectable)
{
	uint8_t adv_data_max_size = (adv_connectable) ? (ADV_DATA_LEN - 3) : (ADV_DATA_LEN);

	if ((adv_data_max_size - cmd->info.host.adv_data_len) >= ad_struct_len)
	{
		// Append manufacturer data to advertising data
		memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len], ad_struct_data, ad_struct_len);

		// Update Advertising Data Length
		cmd->info.host.adv_data_len += ad_struct_len;

		// Store index of manufacturer data which are included in the advertising data
		mnf_data_index = cmd->info.host.adv_data_len - sizeof(struct mnf_specific_data_ad_structure);
	}
	else if ((SCAN_RSP_DATA_LEN - cmd->info.host.scan_rsp_data_len) >= ad_struct_len)
	{
		// Append manufacturer data to scan response data
		memcpy(&cmd->info.host.scan_rsp_data[cmd->info.host.scan_rsp_data_len], ad_struct_data, ad_struct_len);

		// Update Scan Response Data Length
		cmd->info.host.scan_rsp_data_len += ad_struct_len;

		// Store index of manufacturer data which are included in the scan response data
		mnf_data_index = cmd->info.host.scan_rsp_data_len - sizeof(struct mnf_specific_data_ad_structure);
		// Mark that manufacturer data is in scan response and not advertising data
		mnf_data_index |= 0x80;
	}
	else
	{
		// Manufacturer Specific Data do not fit in either Advertising Data or Scan Response Data
		ASSERT_WARNING(0);
	}
	// Store advertising data length
	stored_adv_data_len = cmd->info.host.adv_data_len;
	// Store advertising data
	memcpy(stored_adv_data, cmd->info.host.adv_data, stored_adv_data_len);
	// Store scan response data length
	stored_scan_rsp_data_len = cmd->info.host.scan_rsp_data_len;
	// Store scan_response data
	memcpy(stored_scan_rsp_data, cmd->info.host.scan_rsp_data, stored_scan_rsp_data_len);
}

/**
 ****************************************************************************************
 * @brief Advertisement data update timer callback function.
 ****************************************************************************************
 */
static void adv_data_update_timer_cb()
{
	// If mnd_data_index has MSB set, manufacturer data is stored in scan response
	uint8_t *mnf_data_storage = (mnf_data_index & 0x80) ? stored_scan_rsp_data : stored_adv_data;

	// Update manufacturer data
	mnf_data_update();

	// Update the selected fields of the advertising data (manufacturer data)
	memcpy(mnf_data_storage + (mnf_data_index & 0x7F), &mnf_data, sizeof(struct mnf_specific_data_ad_structure));

	// Update advertising data on the fly
	app_easy_gap_update_adv_data(stored_adv_data, stored_adv_data_len, stored_scan_rsp_data, stored_scan_rsp_data_len);

	// Restart timer for the next advertising update
	app_adv_data_update_timer_used = app_easy_timer(ADVERTISEMENT_DURATION, adv_data_update_timer_cb);
}

/**
 ****************************************************************************************
 * @brief Parameter update request timer callback function.
 ****************************************************************************************
 */
static void param_update_request_timer_cb()
{
	app_easy_gap_param_update_start(app_connection_idx);
	app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
}

void user_app_init(void)
{
	app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;

	// Initialize Manufacturer Specific Data
	mnf_data_init();

	// Initialize Advertising and Scan Response Data
	memcpy(stored_adv_data, USER_ADVERTISE_DATA, USER_ADVERTISE_DATA_LEN);
	stored_adv_data_len = USER_ADVERTISE_DATA_LEN;
	memcpy(stored_scan_rsp_data, USER_ADVERTISE_SCAN_RESPONSE_DATA, USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN);
	stored_scan_rsp_data_len = USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN;
#if (WLAN_COEX_ENABLED)
	wlan_coex_init();

	// Adds priority case for a specific connection
	// wlan_coex_prio_criteria_add(WLAN_COEX_BLE_PRIO_ADV, LLD_ADV_HDL, 0);
#endif

GPIO_EnableIRQ(GPIO_PORT_0, GPIO_PIN_4, ASM_GPIO_IRQ, false, true, 0);
//GPIO_RegisterCallback(ASM_GPIO_IRQ, uart_trigger);

	default_app_on_init();

#if (BLE_APP_SEC)
	// Set service security requirements
	app_set_prf_srv_perm(TASK_ID_CUSTS1, APP_CUSTS1_SEC_REQ); // // Sandeep
	// Fetch bond data from the external memory
	app_easy_security_bdb_init();
#endif
}



void user_app_on_pairing_succeeded(uint8_t conidx)
{
	if (app_sec_env[conidx].auth & GAP_AUTH_BOND)
	{
		
		app_easy_security_bdb_add_entry(&app_sec_env[conidx]);
	}
}

void user_app_adv_start(void)
{
	// Schedule the next advertising data update
	app_adv_data_update_timer_used = app_easy_timer(300, adv_data_update_timer_cb);

	struct gapm_start_advertise_cmd *cmd;
	cmd = app_easy_gap_undirected_advertise_get_active();

	// Add manufacturer data to initial advertising or scan response data, if there is enough space
	app_add_ad_struct(cmd, &mnf_data, sizeof(struct mnf_specific_data_ad_structure), 1);

	app_easy_gap_undirected_advertise_start();

#if SCHEDULER_ON
	//InitialiseScheduler();
#endif
}

void user_app_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
{
	

	if (app_env[connection_idx].conidx != GAP_INVALID_CONIDX)
	{
		app_connection_idx = connection_idx;

		// Stop the advertising data update timer
		app_easy_timer_cancel(app_adv_data_update_timer_used);

		// Check if the parameters of the established connection are the preferred ones.
		// If not then schedule a connection parameter update request.
		if ((param->con_interval < user_connection_param_conf.intv_min) ||
			(param->con_interval > user_connection_param_conf.intv_max) ||
			(param->con_latency != user_connection_param_conf.latency) ||
			(param->sup_to != user_connection_param_conf.time_out))
		{
			// Connection params are not these that we expect
			app_param_update_request_timer_used = app_easy_timer(APP_PARAM_UPDATE_REQUEST_TO, param_update_request_timer_cb);
		}
//		notify_stm(INDICATION, comm_wbleconn, NULL, NULL);
	}
	else
	{
		// No connection has been established, restart advertising
		user_app_adv_start();
	}

	default_app_on_connection(connection_idx, param);
}

void user_app_adv_undirect_complete(uint8_t status)
{
	// If advertising was canceled then update advertising data and start advertising again
	if (status == GAP_ERR_CANCELED)
	{
		user_app_adv_start();
	}
}

void user_app_disconnect(struct gapc_disconnect_ind const *param)
{
	
	// Cancel the parameter update request timer
	if (app_param_update_request_timer_used != EASY_TIMER_INVALID_TIMER)
	{
		app_easy_timer_cancel(app_param_update_request_timer_used);
		app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
	}
	// Update manufacturer data for the next advertsing event
	mnf_data_update();
	// Restart Advertising
	user_app_adv_start();

	/* Disconnect Code Goes Here */
	//Update_disconnection_status(); //disconnection status
//	notify_stm(INDICATION, comm_wblediscon, NULL, NULL);
}
#if (BLE_APP_SEC)
void user_app_on_tk_exch(uint8_t conidx,
						 struct gapc_bond_req_ind const *param)
{
	bool accept = true;

	if (param->data.tk_type == GAP_TK_OOB)
	{
		uint8_t oob_tk[KEY_LEN] = APP_SECURITY_OOB_TK_VAL;
		app_easy_security_tk_exch(conidx, (uint8_t *)oob_tk, KEY_LEN, accept);
	}
	else if (param->data.tk_type == GAP_TK_DISPLAY)
	{
		uint32_t KEY = 0;

		uint8_t buf[6] = {0};
		KEY = app_sec_gen_tk(); // For Generating a Passkey
		KEY = 123456;
		app_easy_security_tk_exch(conidx, (uint8_t *)&KEY, sizeof(KEY), accept); // Regestreing a passkey

		for (uint8_t i = 0; i < 6; i++)
		{
			buf[5 - i] = KEY % 10;
			KEY /= 10;
		}

//			notify_stm(INDICATION, comm_wblepasskey, buf, sizeof(buf));

		app_easy_security_tk_exch(conidx, (uint8_t *)&KEY, sizeof(KEY), accept);
	}
	else if (param->data.tk_type == GAP_TK_KEY_ENTRY)
	{

#if defined(CFG_PRINTF)
		arch_printf("\r\n Passkey Entered: %u", passkey);
#endif
	}
	else if (param->data.tk_type == GAP_TK_KEY_CONFIRM)
	{
#if defined(CFG_PRINTF)
		uint32_t passkey;
		// Print the 6 Least Significant Digits of Passkey
		char buf[6];
		passkey = (param->tk.key[0] << 0) | (param->tk.key[1] << 8) |
				  (param->tk.key[2] << 16) | (param->tk.key[3] << 24);
		arch_printf("\r\n Confirmation Value: ");
		for (uint8_t i = 0; i < 6; i++)
		{
			buf[5 - i] = passkey % 10;
			passkey /= 10;
		}
		for (uint8_t i = 0; i < 6; i++)
		{
			arch_printf("%u", buf[i]);
		}
#endif

		app_easy_security_tk_exch(conidx, (uint8_t *)&param->tk, sizeof(param->tk), accept);
	}
}
#endif // (BLE_APP_SEC)


void ResetCommLock_cb(void)
{
	if(Comm_Lock)
	{
		ResetCommLock() ;
		// Notify App
	}
}

void StartMeasuremet_Wait_Timer(void)
{
	app_easy_timer(40 * 100, ResetCommLock_cb);
}

void ResetCommLock(void)
{
	Comm_Lock = false ; 
}



/// @} APP
