#ifndef __MACROS
#define __MACROS

#include <stdint.h>

# define PARAM_ID			0x00
# define PARAM_VAL_MSB		0x01
# define PARAM_VAL_LSB		0x02
# define TIMESTAMP_3		0x03
# define TIMESTAMP_2		0x04
# define TIMESTAMP_1		0x05
# define TIMESTAMP_0		0x06
# define ORIGIN_TYPE		0x07
# define TYPE_OFFSET		0x02

enum _Exchange_Type
{
	COMMAND,
	RESPONSE,
	INDICATION
};


enum comm_subid
{
	// Id's from 0x0000 to 0x0028 are reserved for system related functions
	// VITALS START
	comm_NONE                  = 0x0028,
	comm_wHR                   = 0x0029,
	comm_wSPO2                 = 0x002A,
	comm_wCBT                  = 0x002B,
	comm_wskintemp             = 0x002C,
	comm_wBP                   = 0x002D,
	comm_wAlgoHr               = 0x002E,
	comm_wECG                  = 0x002F,
	comm_wrawData              = 0x0030,
	// enum from 8-20 are reserved for raw data

	// CONNECTION PARAMETERS START
	comm_wbleconn              = 0x003D,
	comm_wblediscon            = 0x003E,
	comm_wblepasskey           = 0x003F,
	comm_wblebondinfo          = 0x0040,
	comm_wblemacid             = 0x0041,
	comm_wSwitchtoWifi         = 0x0042,
	
	// enum from 25-40 is related for connection parameters.

	// CRITICAL PARAMETERS START
	comm_wsos                  = 0x0051,
	comm_wfall                 = 0x0052,

	// OTHER SYSTEM PARAMETERS START
	comm_wbatt                 = 0x0053,
	comm_wstepcount            = 0x0054,
	comm_wStepgoal             = 0x0055,
	comm_wtimesync             = 0x0056,
	comm_wUpdate               = 0x0057,
	comm_wValidBuild		   		 = 0x0058,
	comm_wOtaReset			       = 0x0059,
	comm_wVitalBusy            = 0x005A,
	comm_wScheduler			       = 0x005B,
	comm_wFactory_Rst          = 0x005C,
	comm_wStepnctRst           = 0x005D,
	comm_wUcConigwrite         = 0x005E,
	comm_wSetupWrComplete      = 0x005F,
	comm_wStepgoalSet          = 0x0060,
	comm_wSchedlrControl       = 0x0061,
	comm_wSchedulerTimeChange  = 0x0062,
	comm_wChargerConnected     = 0x0063,
	comm_wBatterylow           = 0x0064,
	comm_wVitalAlertRange      = 0x0065,
	comm_wChargerRemoved       = 0x0066,	

	// CONFIG PARAMETERS START
	comm_wAlerts               = 0x006F,
	// USER CoNFIG PARAMETERS START
	comm_wUconfig              = 0x0070,
	comm_wUcWeight             = 0x0071,
	comm_wUcHeight             = 0x0072,
	comm_wUcCountry            = 0x0073,
	comm_wUcPhNO               = 0x0074,
	comm_wUcAddr               = 0x0075,
	comm_wUcVitalsRng          = 0x0076,
	comm_wUcID                 = 0x0077,
	comm_wUcWatchId            = 0x0078,
	comm_wUcConfig             = 0x0079,
	comm_wUcWifiCred1          = 0x007A,
	comm_wUcWifiCred2          = 0x007B,
	comm_wUcWifiCred3          = 0x007C,
	comm_wUcWifiCred4          = 0x007D,
	comm_wUcWifiCred5          = 0x007E,
	comm_wUcWifiCred6          = 0x007F,
	comm_wUcWifiCred7          = 0x0080,
	comm_wUcWifiCred8          = 0x0081,
	comm_wUcWifiCred9          = 0x0082,
	comm_wUcWifiCred10         = 0x0083,
	comm_wUcSosAnalysis        = 0x0084,
	comm_wUcDiagData           = 0x0085,
	comm_wUcDevPref            = 0x0086,
	comm_wUcPasscode           = 0x0087,
	comm_wUcDevName            = 0x0088,
	comm_wUcTimeZone           = 0x0089,
	comm_wUcHrCoeff            = 0x008A,
	comm_wUcSpo2Coeff          = 0x008B,
	comm_wUcCbtCoeff           = 0x008C,
	comm_wUcSbpCoeff           = 0x008D,
	comm_wUcDbpCoeff           = 0x008E,
	comm_wUcEcgCoeff           = 0x008F,
	comm_wUcPpgCoeff           = 0x0090,
	comm_UcSchdlrTime          = 0x0091,
	comm_err
	// Id's From 0x007C to 0xFFFF are unused.
};




/*
* These enums are as per the ID assigned to the vitals mentioned in the Ble communication spec.
* These Id's are used when the GATT Client wants to make a request/enquiry on a particular feild
*/

typedef enum
{
 HEART_RATE 	    			=	0x01,
 BLOOD_OXYGEN 		    	=	0x02,
 BODY_TEMPERATURE 		  =	0x03,
 SKIN_TEMPERATURE 			= 0x04,
 BATTERY_PERCENTAGE 		=	0x05,
 BLOOD_PRESSURE 			  =	0x06,
 PEDOMETER_SENSOR 			= 0x07,
 STEP_GOAL 					    = 0x08,
 TIME_Sync 					    = 0x09,
 SOS 						        = 0x0A,
 BT_MAC_ID              = 0x0B,
 UPDATE_HANDLER_OTA 		=	0x0C,
 SETUP_CONFIG_WRITTEN   = 0x0D,
 VITAL_BUSY             = 0x10,		
 SETUP_CONFIG           = 0x0D,   
 EKG                    = 0xAA,
 ALGO_HR                = 0x16,
 SWITCH_TO_WIFI         = 0x0E,		
 
 _END
}write_to_watch_t;


# define SOS_IND       0x0C


/***************************************************************/
typedef enum STATE
{
	REQUEST_HR = 0,
	HR_WAIT_INTERVAL = 1,
	REQUEST_SPO2 = 2,
	SPO2_WAIT_INTERVAL = 3,
	REQUEST_TEMPERATURE = 4,
	TEMPERATURE_WAIT_INTERVAL = 5,
	START_WAIT_INTERVAL = 6,
} state_t;

typedef struct schedulerState
{
	state_t NextState;
	state_t currentState;
} schedulerState_t;

/***************************************************************/


#endif
