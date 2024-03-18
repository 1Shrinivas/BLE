/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDES
 ****************************************************************************************
 */

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>     // standard definitions
#include <stdbool.h>    // boolean definition
#include "arch.h"
#include "arch_api.h"
#include "boot.h"       // boot definition
#include "rwip.h"       // BLE initialization
#include "syscntl.h"    // System control initialization
#include "em_map_ble.h"
#include "ke_mem.h"
#include "ke_event.h"
#include "user_periph_setup.h"

#include "uart.h"   // UART initialization
#include "nvds.h"   // NVDS initialization
#include "rf.h"     // RF initialization
#include "app.h"    // application functions
#include "dbg.h"    // For dbg_warning function

#include "datasheet.h"

#include "em_map_ble.h"

#include "lld_sleep.h"
#include "rwble.h"
#include "rf_585.h"
#include "gpio.h"
#include "hw_otpc.h"

#include "lld_evt.h"
#include "arch_console.h"

#include "arch_system.h"

#include "system_library.h"

#include "arch_wdg.h"

#include "user_callback_config.h"


#include "ea.h"

#include "arch_ram.h"
#include "User_Application.h"

#include "comm_manager.h"
#include "user_custs1_def.h"

#include <stdio.h>
#include "string.h"
#include <stdint.h>
#include "arch_system.h"
#include "user_periph_setup.h"
#include "gpio.h"
#include "uart.h"
#include "uart_utils.h"
#include "i2c_eeprom.h"
#include "bme68x.h"
#include "bme68x_defs.h"




#if defined (__DA14531__)
#include "otp_cs.h"
#include "adc.h"
#endif

#if (USE_RANGE_EXT)
#include "range_ext_api.h"
#endif

#if (WLAN_COEX_ENABLED)
#include "wlan_coex.h"
#endif

/**
 * @addtogroup DRIVERS
 * @{
 */




/*
 * DEFINES
 ****************************************************************************************
 */
#define BUS_SLAVE_ADDRESS (0x76<<1)
#define BUFFER_SIZE             (256)

#define HEATR_DUR	2000
#define N_MEAS		6
#define LOW_TEMP	150
#define HIGH_TEMP 	350
#define SAMPLE_COUNT  UINT16_C(300)
#define WAIGHTTIME 1000
#define UART_TERMINAL  0
// Constants for conversion
#define ATMOSPHERIC_PRESSURE 1013.25 // in hPa (standard atmospheric pressure)
#define GAS_CONSTANT 8.314 // in J/(mol·K) (ideal gas constant)
#define MOL_WEIGHT_AIR 28.97 // in g/mol (molecular weight of dry air)

/*
 * VARIABLE DEFINITIONS
 ****************************************************************************************
 */

uint8_t wr_data[BUFFER_SIZE];
uint8_t rd_data[BUFFER_SIZE];



/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */
typedef struct vitals_ble
{
    uint8_t requested_parameter;
    uint8_t data[2];
    uint8_t origin_bit;
    uint32_t timestamp;
    uint8_t pad;

}vitals_ble_t;




/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


extern struct arch_sleep_env_tag sleep_env;
extern last_ble_evt arch_rwble_last_event;

#if defined (__DA14531__)
/// Structure containing info about debugger configuration and OTP CS registers
/// normally handled by the Booter - defined in otp_cs.c
extern otp_cs_booter_val_t booter_val;
#endif

static uint32_t code_size                      __SECTION_ZERO("retention_mem_area0");
static uint8_t ret_mode_for_non_ret_heap       __SECTION_ZERO("retention_mem_area0");
static uint8_t ret_mode                        __SECTION_ZERO("retention_mem_area0");
static uint8_t ret_mode_for_ret_data           __SECTION_ZERO("retention_mem_area0");
static uint8_t reinit_non_ret_heap             __SECTION_ZERO("retention_mem_area0");


volatile uint16_t TotalPressButton=0;

# define USER_DATA_AREA   	   0x10000
/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */
__STATIC_INLINE void otp_prepare(uint32_t code_size);
__STATIC_INLINE bool ble_is_powered(void);
__STATIC_INLINE void ble_turn_radio_off(void);
__STATIC_INLINE void schedule_while_ble_on(void);
__STATIC_INLINE void arch_turn_peripherals_off(sleep_mode_t current_sleep_mode);
__STATIC_INLINE void arch_goto_sleep(sleep_mode_t current_sleep_mode);
__STATIC_INLINE void arch_switch_clock_goto_sleep(sleep_mode_t current_sleep_mode);
__STATIC_INLINE void arch_resume_from_sleep(void);
__STATIC_INLINE sleep_mode_t rwip_power_down(void);
__STATIC_INLINE arch_main_loop_callback_ret_t app_asynch_trm(void);
__STATIC_INLINE arch_main_loop_callback_ret_t app_asynch_proc(void);
__STATIC_INLINE void app_asynch_sleep_proc(void);
__STATIC_INLINE void app_sleep_prepare_proc(sleep_mode_t *sleep_mode);
__STATIC_INLINE void app_sleep_exit_proc(void);
__STATIC_INLINE void app_sleep_entry_proc(sleep_mode_t sleep_mode);

__STATIC_INLINE void init_retention_mode(void);
#if !defined (__DA14531__)
__STATIC_INLINE void set_ldo_ret_trim(uint8_t retained_ram_blocks);
#endif
#if !defined (CFG_CUSTOM_SCATTER_FILE)
__STATIC_INLINE uint8_t get_retention_mode(void);
__STATIC_INLINE uint8_t get_retention_mode_data(void);
#endif
__STATIC_INLINE uint8_t get_retention_mode_non_ret_heap(void);
void i2c_test(void);
void delay_bMe(unsigned int i);
void delay_ms(uint32_t ms);
int32_t pressure_measurement(int64_t t_fine,float Pressure_value);
float Gasadcdata(uint8_t temp,float gas_res);
/*
 * MAIN FUNCTION
 ****************************************************************************************
 */
/********************************* BUS COMMUNICATION READ AND WRITE ********************************/

i2c_cfg_t  i2c1;


void ReadI2CBus(uint8_t address, uint8_t *data,uint16_t len)
{
	i2c_read(address,data,len);
}

void WriteI2CBus(uint8_t addres, const uint8_t data ,uint16_t len)
{
  i2c_write(addres,data,len);
}
void delay_ms(uint32_t ms) {
    // Calculate the number of clock cycles required for the delay
    // The actual value will depend on your system's clock frequency
    uint32_t cycles = ms * (32000000 / 1000U);//160000 1ms

    // Perform the delay using a loop
    for (uint32_t i = 0; i < cycles; ++i) {
        __NOP(); // No operation (adjust based on your compiler)
    }
}

#define HIGH 1


volatile int msTicks = 0;
void SysTick_Handler(void)
  {                               /* SysTick interrupt Handler. */
    msTicks++;
		
		
	}
void SystickEnable_timer(void)
{
	uint32_t returnCode;
	
	//returnCode = SysTick_Config(SystemCoreClock / 1000);
		returnCode = SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);
}
void SysTickDisable(void)
{
    NVIC_DisableIRQ(SysTick_IRQn);
}
/*************** SYSTICK TIMER Stop ***************************/
/*************** PUSH BUTTON CallBack *************************/
void delay_m(int n)
{
   int i;
   for (i= 0; i<16000; i++)
 __nop();
}


void delay_bMe(unsigned int i)
{
unsigned int j;
	for(j=0;j<32000;j++);
}
	
float temperature;

/*! Pressure in Pascal */
float pressure;

/*! Humidity in % relative humidity x1000 */
float humidity;

/*! Gas resistance in Ohms */
float gas_Resist=0;
/************************************* Temperature Variables ****************************************/



uint32_t tempAdc = 0;
uint8_t conadc = 0;

int64_t var1,var2,var3;
uint16_t temp_comp = 0;

float temp_value;
double t_fine =0;



uint8_t calc_res(float temp,struct bme68x_dev *device){

	float var1;
	float var2;
	float var3;
	float var4;
	float var5;
	float amb_temp = 25;
	uint8_t res_heat;

	if (temp > 400) /* Cap temperature */
	{
		temp = 400;
	}

	var1 = (((float)device->calib.par_gh1 / (16.0f)) + 49.0f);
	var2 = ((((float)device->calib.par_gh2 / (32768.0f)) * (0.0005f)) + 0.00235f);
	var3 = ((float)device->calib.par_gh3 / (1024.0f));
	var4 = (var1 * (1.0f + (var2 * (float)temp)));
	var5 = (var4 + (var3 * (float)amb_temp));
	res_heat =
			(uint8_t)(3.4f *
					((var5 * (4 / (4 + (float)device->calib.res_heat_range)) *
							(1 / (1 + ((float)device->calib.res_heat_val * 0.002f)))) -
							25));

	return res_heat;
}
uint8_t calc_gas_wait(uint16_t dur)
{
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xfc0)
	{
		durval = 0xff; /* Max duration*/
	}
	else
	{
		while (dur > 0x3F)
		{
			dur = dur / 4;
			factor += 1;
		}

		durval = (uint8_t)(dur + (factor * 64));
	}

	return durval;
}
struct bme68x_dev *device;





/**************************************** END ***************************************************/

///********************************* Gas Variable *******************************************/

//uint16_t par_g1 = 0 ;

//uint8_t par_g2_lsb = 0;
//uint8_t par_g2_msb = 0;

//uint16_t par_g2 = 0;

//uint8_t par_g3 = 0;

//uint16_t heat_range = 0;
//uint16_t Res_heat_range = 0;
//uint16_t Res_heat_val = 0;
//uint16_t heat_value = 0;


//uint8_t gas_adc_lsb = 0;
//uint8_t gas_adc_msb = 0;
//uint8_t conc=0;

//uint16_t gas_adc = 0;

//uint8_t gas_range = 0;

//uint8_t range_switching_error = 0;
//uint8_t range = 0;

//uint8_t switch_range =0;


///********************************** END ****************************************************/
float temp=0;

struct Result {
	float Temp_bme680;
	double t_fine_bme680;
};


/************************************* Temperature Measurement Function *******************/
struct Result Temperature_measurement()
{
	uint8_t tempbuf[3];

	uint8_t par_t1_lsb = 0;
	uint8_t par_t1_msb = 0;
	uint16_t par_t1 = 0;

	uint8_t par_t2_lsb = 0;
	uint8_t par_t2_msb = 0;
	int16_t par_t2 = 0;

	int8_t par_t3 = 0;

	struct Result result;




	i2c_write(0x74,0x8D,1);//0x61
	i2c_write(0x75,0x08,1); //IIR Filter

	i2c_read(0xE9,&par_t1_lsb,1);  // PART1
	i2c_read(0xEA,&par_t1_msb,1);
	par_t1 = (par_t1_msb<<8)| (par_t1_lsb);

	i2c_read(0x8A,&par_t2_lsb,1); // PART2
	i2c_read(0x8B,&par_t2_msb,1);
	par_t2 = (par_t2_msb<<8)| (par_t2_lsb);

	i2c_read(0x8C,&par_t3,1);  // PART3

	memset(tempbuf,0, sizeof(tempbuf));
	i2c_read(0x22, &tempbuf[0],1);
	i2c_read(0x23, &tempbuf[1],1);
	i2c_read(0x24, &tempbuf[2],1);


	conadc = (tempbuf[2] >> 4);
	tempAdc = (tempbuf[0] << 12) | (tempbuf[1] << 4) | (conadc);


	double var1;
	double var2;
	double calc_temp;

	/* calculate var1 data */
	var1 = ((((double)tempAdc / 16384.0) - ((double)par_t1/ 1024.0)) * ((double)par_t2));

	/* calculate var2 data */
	var2 =
			(((((double)tempAdc / 131072.0) - ((double)par_t1 / 8192.0)) *
					(((double)tempAdc / 131072.0) - ((double)par_t1 / 8192.0))) * ((double)par_t3 * 16.0));

	/* t_fine value*/
	t_fine = (var1 + var2);

	/* compensated temperature data*/
	calc_temp = ((t_fine) / 5120.0f);
	result.t_fine_bme680 = t_fine; // store the t_fine
	temp_value = calc_temp;

	result.Temp_bme680 = temp_value; // store the temp value
	return result;


}

/********************************* Pressure Measurement Function **************************/
float pressure = 0;
int32_t pressure_measurement(int64_t t_fine,float Pressure_value)
{
	uint8_t par_p1_lsb = 0;  // Par_p1
	uint8_t par_p1_msb = 0;
	uint16_t par_p1;

	uint8_t par_p2_lsb = 0;  //par_p2
	uint8_t par_p2_msb = 0;
	int16_t par_p2;

	int8_t par_p3;     //par_p3

	uint8_t par_p4_lsb = 0;  //par_p4
	uint8_t par_p4_msb = 0;
	int16_t par_p4;

	uint8_t par_p5_lsb = 0;  //par_p5
	uint8_t par_p5_msb = 0;
	int16_t par_p5;

	int8_t par_p6;    //par_p6

	int8_t par_p7;    //par_p7

	uint8_t par_p8_lsb = 0;  //par_p8
	uint8_t par_p8_msb = 0;
	int16_t par_p8;

	uint8_t par_p9_lsb = 0;  //par_p9
	uint8_t par_p9_msb = 0;
	int16_t par_p9;

	uint8_t par_p10;      //par_p10

	uint8_t pres_buf[3];
	uint32_t pres_adc = 0;
	uint8_t Pres_conadc = 0;

	i2c_write(0x74,0x8D,1);//0x61

	i2c_write(0x75,0x08,1); //IIR Filter

	i2c_read(0x8E,&par_p1_lsb,1);                  // read par_p1
	i2c_read(0x8F, &par_p1_msb,1);
	par_p1 = (par_p1_msb<<8)| (par_p1_lsb);

	i2c_read(0x90,&par_p2_lsb,1);                  //read par_p2
	i2c_read(0x91, &par_p2_msb,1);
	par_p2 = (par_p2_msb<<8)| (par_p2_lsb);


	i2c_read(0x92, &par_p3,1);                      //read par_p3

	i2c_read(0x94,&par_p4_lsb,1);                   //re ad par_p4
	i2c_read(0x95, &par_p4_msb,1);
	par_p4 = (par_p4_msb<<8)| (par_p4_lsb);

	i2c_read(0x96,&par_p5_lsb,1);                    //read par_p5
	i2c_read(0x97, &par_p5_msb,1);
	par_p5 = (par_p5_msb<<8)| (par_p5_lsb);

	i2c_read(0x99, &par_p6,1);                        //read par_p6

	i2c_read(0x98, &par_p7,1);                        //read par_p7

	i2c_read(0x9C,&par_p8_lsb,1);                     //read par_p8
	i2c_read(0x9D, &par_p8_msb,1);
	par_p8 = (par_p8_msb<<8)| (par_p8_lsb);

	i2c_read(0x9E,&par_p9_lsb,1);                     //read par_p9
	i2c_read(0x9F, &par_p9_msb,1);
	par_p9 = (par_p9_msb<<8)| (par_p9_lsb);

	i2c_read(0xA0, &par_p10,1);                        //read par_p10

	memset(pres_buf,0, sizeof(pres_buf));
	//		i2c_read_Mul(0x1F, &pres_buf, 3);   // read the pressure adc

	i2c_read(0x1F, &pres_buf[0], 1);
	i2c_read(0x20, &pres_buf[1], 1);
	i2c_read(0x21, &pres_buf[2], 1);


	Pres_conadc = (pres_buf[2] >> 4);
	pres_adc = (pres_buf[0] << 12) | (pres_buf[1]<< 4) | (Pres_conadc);

	uint8_t data;
	i2c_read(0x1D,&data,1);

	/* This value is used to check precedence to multiplication or division
	 * in the pressure compensation equation to achieve least loss of precision and
	 * avoiding overflows.
	 * i.e Comparing value, pres_ovf_check = (1 << 31) >> 1
	 */
	double var1;
	double var2;
	double var3;
	double press_comp;

	var1 = ((double)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)par_p6 / 131072.0);
	var2 = var2 + (var1 * (double)par_p5 * 2.0);
	var2 = (var2 / 4.0) + ((double)par_p4 * 65536.0); var1 = ((((double)par_p3 * var1 * var1) / 16384.0) +
			((double)par_p2 * var1)) / 524288.0; var1 = (1.0 + (var1 / 32768.0)) * (double)par_p1; press_comp = 1048576.0 - (double)pres_adc;
	press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1; var1 = ((double)par_p9 * press_comp * press_comp) / 2147483648.0; var2 = press_comp * ((double)par_p8 / 32768.0);
	var3 = (press_comp / 256.0) * (press_comp / 256.0) * (press_comp / 256.0) * (par_p10 / 131072.0);
	press_comp = press_comp + (var1 + var2 + var3 + ((double)par_p7 * 128.0)) / 16.0;
	Pressure_value = press_comp/100.0;

	return Pressure_value;

}
/********************************* Pressure Function END ***********************************/
int32_t var1_hum;
int32_t var2_hum;
int32_t var3_hum;
int32_t var4_hum;
int32_t var5_hum;
int32_t var6_hum;
int32_t temp_scaled;
int32_t calc_hum;
/********************************* Hummunity Measurement Function **************************/
float hum = 0;
uint16_t Humidity_measurement(double t_fine,float hum)
{


	uint8_t par_h1_lsb ;
	uint8_t par_h1_msb ;

	uint16_t par_h1;

	uint8_t par_h1_low ;

	uint8_t par_h2_lsb ;
	uint8_t par_h2_msb ;
	uint16_t par_h2;

	uint8_t par_h2_low ;

	int8_t par_h3;

	int8_t par_h4;

	int8_t par_h5;


	uint8_t par_h6;

	int8_t par_h7;

	uint8_t hum_buf[2];

	uint8_t hum_Adc_lsb ;
	uint8_t hum_Adc_msb;
	int16_t hum_adc;


	i2c_write(0x72,0x02,1); //oversampling *2
	i2c_write(0x74,0x01,1); // Foreced mode


	i2c_read(0xE2,&par_h1_lsb,1);
	i2c_read(0xE3, &par_h1_msb,1);

	par_h1 = ((par_h1_msb << 4) | (par_h1_lsb & 0x0F));    // par_h1

	i2c_read(0xE2,&par_h2_lsb,1);
	i2c_read(0xE1, &par_h2_msb,1);


	par_h2 = ((par_h2_msb << 4) | (par_h2_lsb >> 4));       //par_h2


	i2c_read(0xE4,&par_h3,1);                     //par_h3

	i2c_read(0xE5, &par_h4,1);                     //par_h4

	i2c_read(0xE6, &par_h5,1);                     //par_h5

	i2c_read(0xE7, &par_h6,1);						//par_h6

	i2c_read(0xE8, &par_h7,1);                     //par_h7

	//	i2c_read_Mul(0x25, &hum_buf, 2);
	i2c_read(0x26,&hum_Adc_lsb,1);
	i2c_read(0x25, &hum_Adc_msb,1);

	hum_adc = (hum_Adc_msb << 8 )| (hum_Adc_lsb);  // hum_Adc

	uint8_t data;
	i2c_read(0x1D,&data,1);

	double calc_hum = 0;
	double var1 = 0;
	double var2 = 0;
	double var3 = 0;
	double var4 = 0;
	double temp_comp =0;

	/* compensated temperature data*/
	temp_comp = ((t_fine) / 5120.0);
	var1 = hum_adc - (((double)par_h1 * 16.0) + (((double)par_h3 / 2.0) * temp_comp));
	var2 = var1 * (((double)par_h2 / 262144.0) * (1.0 + (((double)par_h4 / 16384.0) *
			temp_comp) + (((double)par_h5 / 1048576.0) * temp_comp * temp_comp)));
	var3 = (double)par_h6 / 16384.0;
	var4 = (double)par_h7 / 2097152.0;
	calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

	hum = calc_hum;
	return hum;


}
/************************************* END **********************************************/

void InitGas(void)
{
	i2c_write(0xE0,0xB6,1);//reset
	i2c_write(0x64,0x11,1); //Res_heat_range
	i2c_write(0x6D,0x11,1); //Res_heat_range
	//i2c_write(0x70,0x04,1);
	
	
}
//bool flag = 0;
void Gasforcemode(void)
{
	  i2c_write(0x74,0x01,1);// Forced mode
	  i2c_write(0x71,0x10,1); //gas run enable
}

	
uint8_t gas_adc_msb = 0;
uint8_t gas_adc_lsb = 0;
uint16_t GasAdcData_gas = 0;


	uint16_t heat_range = 0;
	uint16_t heat_value = 0;

	uint8_t par_g1 = 0 ;

	uint8_t par_g2_lsb = 0;
	uint8_t par_g2_msb = 0;

	uint16_t par_g2 = 0;

	uint8_t par_g3 = 0;

	uint8_t Res_heat_range = 0;
	uint8_t Res_heat_val = 0;



	uint8_t gas_range = 4;

	uint8_t range_switching_error = 0;
	uint8_t range = 0;
	uint8_t switch_range =0;

float varA_GAS = 0;
float gas_res = 0;
float resistance;
int gasResistance; 
float Humidity;

char buffer[100];


float gas=0;
struct bme68x_heatr_conf Heat;
struct bme68x_dev dev;
float varg1 = 0;
float calc_gas_res;



//float temp =0;
float GasAdcData(int8_t amb)
{
	int8_t rslt;
	uint8_t *nb_conv = 0;
	uint8_t hctrl, run_gas = 0;
	uint8_t ctrl_gas_data[2]={0};
	uint8_t ctrl_gas_addr[2] = {0x70, 0x71 };

	i2c_write(0x74,0x01,1);// Forced mode


	i2c_read(0x70, &ctrl_gas_data[0], 2);
	i2c_read(0x71, &ctrl_gas_data[1], 2);


	uint8_t i;
	uint8_t shared_dur;
	uint8_t write_len = 0;
	uint8_t heater_dur_shared_addr = BME68X_REG_SHD_HEATR_DUR;
	uint8_t rh_reg_addr[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t rh_reg_data[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t gw_reg_addr[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t gw_reg_data[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	rh_reg_addr[0] = BME68X_REG_RES_HEAT0;
	rh_reg_data[0] = calc_res(Heat.heatr_temp, device);
	gw_reg_addr[0] = BME68X_REG_GAS_WAIT0;
	gw_reg_data[0] = calc_gas_wait(Heat.heatr_dur);
	(*nb_conv) = 0;
	write_len = 1;


	i2c_write(0x5A,0x6f,1); // Res_heat value

	i2c_write(0x64,0x65,1); //Res_heat_range



	i2c_read(0xED,&par_g1,1);        // par_g1



	i2c_read(0xEB,&par_g2_lsb,1);
	i2c_read(0xEC, &par_g2_msb,1);

	par_g2 = (par_g2_msb<<8) | (par_g2_lsb);  // par_g2


	i2c_read(0xEE,&par_g3,1);   // par_g3


	i2c_read(0x02,&Res_heat_range,1); // Res_heat_range

	uint8_t mask = 0x30;
	heat_range = (Res_heat_range & mask);


	i2c_read(0x00,&Res_heat_val,1);    //Res_heat_val
	heat_value = Res_heat_val;

	i2c_read(0x2A, &gas_adc_msb,1);
	i2c_read(0x2B, &gas_adc_lsb,1);

	GasAdcData_gas = (gas_adc_msb<<2)|(gas_adc_lsb);

	i2c_read(0x2B, &gas_range,1);

	range = (gas_range & 0x0F);    	// gas_range


	i2c_read(0x04,&range_switching_error,1);    // range_switching_error
	switch_range = ((int8_t) range_switching_error & (int8_t) 0xF0) / 16;


	// Check the 'heat_stab_r' status bit (bit 4).
	uint8_t heaterStability = (gas_adc_lsb >> 4) & 1;

	if (heaterStability) {
		// Heater temperature is stable.
	} else {
		// Heater temperature is not stable.
	}

	uint8_t res_heat_[10]={0};
	uint8_t res_heat_data[10] = {0};

	// Set the I2C address of the BME680 sensor.
	uint8_t sensorAddress = 0x5A; // Replace with the actual sensor address.

	// Define the target heater resistance value you want to set.
	uint8_t targetResistance = 0x7F; // Replace with your desired target resistance value.

	// Determine the register address based on 'x' (in this case, 'x = 3').
	uint8_t registerAddress = 0x60 + 3;

	// Create a data buffer with the register address and target resistance value.
	uint8_t data[2] = {registerAddress, targetResistance};

	// Use the I2C write function to send the data to the sensor.
	uint8_t datas;
	i2c_write(0x5A, datas,1);

	uint8_t _gas_data;
	i2c_read(0x1D,&_gas_data,1);  // Data checking

	
	
	//i2c_read(0x2B,&_gas_data,1);
	//i2c_read(0x2A,&_gas_data1,1);
	for(int i=0; i<9 ;i++){

		res_heat_[i] = 0x63 - i ;
		i2c_write(res_heat_[i], res_heat_data[i],1);

		i2c_read(res_heat_[i], &res_heat_data[i],1);

	}
	uint16_t target_temp = 320;
	int32_t amb_temp_1 = 25;
	//	amb_temp_2 = bme680.Temp_bme680;

	struct Result bme680 = Temperature_measurement(); // temperature function

	t_fine = bme680.t_fine_bme680;
	temp = bme680.Temp_bme680;


	//int32_t amb_temp_1 = 25;
	//	amb_temp_2 = bme680.Temp_bme680;

			t_fine = bme680.t_fine_bme680;
			temp = bme680.Temp_bme680;
      amb =temp;
	
	double var1_gas;
double var2_gas;
double var3_gas;
double var4_gas;
double var5_gas;
double res_heat_x100;
double res_heat_x;

	
	var1_gas = ((double)par_g1 / 16.0) + 49.0;
	var2_gas = (((double)par_g2 / 32768.0) * 0.0005) + 0.00235;
	var3_gas = (double)par_g3 / 1024.0;
	var4_gas = var1_gas * (1.0 + (var2_gas * (double) target_temp));
	var5_gas = var4_gas + (var3_gas * (double)amb);
	res_heat_x = (uint8_t)(3.4 * ((var5_gas * (4.0 / (4.0 + (double)heat_range)) * (1.0/(1.0 + ((double)heat_value * 0.002)))) - 25));


	float lookupTable1[16] = {1,1,1,1,1,0.99,1,0.992,1,1,0.998,0.995,1,0.99,1,1 };
	float lookupTable2[16] = {8000000,4000000,2000000,1000000,499500.4995,248262.1648,125000,63004.03226,31281.28128,15625,7812.5,3906.25,1953.125,976.5625,488.28125,244.140625};


	varA_GAS = (1340.0 + 5.0 * range_switching_error) * lookupTable1[gas_range];
	calc_gas_res = varA_GAS * lookupTable2[gas_range] / (GasAdcData_gas - 512.0 + varA_GAS);

	
  return calc_gas_res;//gas_res;

}

	double var1_gas  =0;
	double var2_gas = 0;
	double var3_gas = 0;
	double var4_gas =0;
	double var5_gas =0;
//	double res_heat_x100;
	double res_heat_x = 0;
float index_percentage;

float Gasadcdata(uint8_t temp,float gas_res)
{

	i2c_write(0x5A,0x6f,1); // Res_heat value

	i2c_write(0x64,0x65,1); //Res_heat_range



	i2c_read(0xED,&par_g1,1);        // par_g1



	i2c_read(0xEB,&par_g2_lsb,1);
	i2c_read(0xEC, &par_g2_msb,1);

	par_g2 = (par_g2_msb<<8) | (par_g2_lsb);  // par_g2


	i2c_read(0xEE,&par_g3,1);   // par_g3


	i2c_read(0x02,&Res_heat_range,1); // Res_heat_range

	uint8_t mask = 0x30;
	heat_range = (Res_heat_range & mask);


	i2c_read(0x00,&Res_heat_val,1);    //Res_heat_val
	heat_value = Res_heat_val;

	i2c_read(0x2A, &gas_adc_msb,1);
	i2c_read(0x2B, &gas_adc_lsb,1);


	GasAdcData_gas = (gas_adc_msb<<2)|(gas_adc_lsb);

	i2c_read(0x2B, &gas_range,1);

	range = (gas_range & 0x0F);    	// gas_range


	i2c_read(0x04,&range_switching_error,1);    // range_switching_error
	switch_range = ((int8_t) range_switching_error & (int8_t) 0xF0) / 16;

//uint8_t datas;
//	i2c_write(0x5A, datas,1);

//	uint8_t _gas_data;
//	i2c_read(0x1D,&_gas_data,1);  // Data checking

	uint16_t target_temp = 320;
	//int32_t amb_temp_1 = 25;
	//	amb_temp_2 = bme680.Temp_bme680;


	var1_gas = ((double)par_g1 / 16.0) + 49.0;
	var2_gas = (((double)par_g2 / 32768.0) * 0.0005) + 0.00235;
	var3_gas = (double)par_g3 / 1024.0;
	var4_gas = var1_gas * (1.0 + (var2_gas * (double) target_temp));
	var5_gas = var4_gas + (var3_gas * (double)temp);
	res_heat_x = (uint8_t)(3.4 * ((var5_gas * (4.0 / (4.0 + (double)heat_range)) * (1.0/(1.0 + ((double)heat_value * 0.002)))) - 25));


	float lookupTable1[16] = {1,1,1,1,1,0.99,1,0.992,1,1,0.998,0.995,1,0.99,1,1 };
	float lookupTable2[16] = {8000000,4000000,2000000,1000000,499500.4995,248262.1648,125000,63004.03226,31281.28128,15625,7812.5,3906.25,1953.125,976.5625,488.28125,244.140625};


	varA_GAS = (1340.0 + 5.0 * range_switching_error) * lookupTable1[range];
	gas_res = varA_GAS * lookupTable2[range] / (GasAdcData_gas - 512.0 + varA_GAS);


	return gas_res/1000.0f;


}
const float oGasResistanceBaseLine =8000000.0f;//149598.0f;
  float hum_baseline = 50.0f;
  float hum_weighting = 0.25f;
  float gas_offset = 0.0f;
  float hum_offset = 0.0f;
  float hum_score = 0.0f;
  float gas_score = 0.0f;
float humn;
float co2_equivalent = 0;
float fCalulate_IAQ_Index( int gasResistance, float Humidity)
{

  gas_offset = oGasResistanceBaseLine - gas_Resist;
  hum_offset =humn - hum_baseline;
  // calculate hum_score as distance from hum_baseline
  if ( hum_offset > 0.0f )
  {
    hum_score = 100.0f - hum_baseline - hum_offset;
    hum_score /= ( 100.0f - hum_baseline );
    hum_score *= ( hum_weighting * 100.0f );
  } else {
    hum_score = hum_baseline + hum_offset;
    hum_score /= hum_baseline;
    hum_score *= ( 100.0f - (hum_weighting * 100.0f) );
  }
  //calculate gas score as distance from baseline
  if ( gas_offset > 0.0f )
  {
    gas_score = gas_Resist/ oGasResistanceBaseLine;
    gas_score *= ( 100.0f - (hum_weighting * 100.0f ) );
  } else {
    gas_score = 100.0f - ( hum_weighting * 100.0f );
  }
	 co2_equivalent = (0.6 * hum_score + 0.4 * gas_score)*10.0f;

  return ( hum_score + gas_score );
}

/************************************* END **********************************************/
uint8_t id,id1,id2,id3,id4;
//#define chip 0x0D
/********************************* Pressure Function END ***********************************/

float temp_bme680 = 0.0f;
int32_t calc_hum;
float pressure_value;
float hum;
float pressure;
float amb = 0;
uint16_t Res_gas;
uint8_t pData;
//float temp = 0.0f;
int8_t amb_temp = 0;

/**
 ****************************************************************************************
 * @brief BLE main function.
 *        This function is called right after the booting process has completed.
 *        It contains the main function loop.
 ****************************************************************************************
 */
int main(void);
void uart_send_cb(uint16_t length);
void uart_receive_cb(uint16_t length);
void uart_error_cb(uart_t * Uart_Instance, uint8_t err_no);


void uart_receive_cb(uint16_t length)
{
	
}
void uart_error_cb(uart_t * Uart_Instance, uint8_t err_no)
{
 
}



float adc_volts = 0;
static uint16_t TotalPress = 0;
bool StatusFlag = 0;
//static uint16_t Press_switch = 0;
void adc_val(void)
{
	 
	//adc_volts = get_adcfun();


		if((adc_volts > 1660) &&(StatusFlag == 0))
			{
				
        StatusFlag = 1;
			  TotalPressButton++;
			
			//"TotalPress = spi_store(&TotalPressButton);
				
			for(long i=0;i<800000;i++)
     {
      __NOP();
     }
				uint8_t val[8] = {0};
	val[0]=  (TotalPress >> 8 & 0xff);
	val[1]=   TotalPress  & 0xff;
	Send_To_Gatt_Client(val, 8, VITAL_VAL);	//sending over network

		
		  } 
			 else if ((adc_volts < 1660) &&(StatusFlag == 1))
			{
				StatusFlag = 0;
			
		
      } 
			

}

void detect_decrease_threshold(int arr[], int size, int threshold) {
    int start_index = -1;
    int end_index = -1;

    for (int i = 0; i < size - 1; i++) {
        if (arr[i] > arr[i + 1] && arr[i] >= threshold && arr[i + 1] < threshold) {
            if (start_index == -1) {
                start_index = i;
            }
            end_index = i + 1;
        }
    }

    if (start_index != -1) {
        printf("Decrease below threshold %d detected from index %d to index %d\n", threshold, start_index, end_index);
    } else {
        printf("No decrease below threshold %d detected in the array\n", threshold);
    }
}

void send_temp_to_app()
{
	char R_buff[50] = {0};
	struct Result bme680 =  Temperature_measurement();
	uint16_t temp1 = bme680.Temp_bme680;
	uint8_t val_temp[128] = {0};
	val_temp[0]=  (temp1 >> 8 & 0xff);
	val_temp[1]=   temp1  & 0xff;
	Send_To_Gatt_Client(val_temp, 128, VITAL_VAL);	//sending over network
	
	#ifdef UART_TERMINAL
	delay_m(1000);
	sprintf(R_buff,"\n\r Temprature =%.2f°C",bme680.Temp_bme680);
  printf_string(UART,R_buff);
	#endif
}

uint8_t i=0;
void send_pressure_to_app()
{
	char p_buff[50] = {0};
	pressure = pressure_measurement(t_fine,pressure_value);
	uint16_t pres = pressure;
	
	pressure = 913;	
	if(i==1)
		pressure = 914;
else if(i==2)
		pressure = 912;
	i++;
	uint8_t val_pressure[128] = {0};
	val_pressure[0]=  (pres >> 8 & 0xff);
	val_pressure[1]=   pres  & 0xff;
	Send_To_Gatt_Client(val_pressure, 128, VITAL_VAL);
	#ifdef UART_TERMINAL
	delay_m(1000);
	sprintf(p_buff,"\n\r Pressure =%.2fhPa",pressure);
  printf_string(UART,p_buff);
	#endif
}

void send_Humidity_to_app()
{
	char H_buff[50] ={0};
  humn = Humidity_measurement(t_fine,hum);  // humidity Function

	uint16_t Humidity = humn;
	uint8_t val_Humidity[128] = {0};
	val_Humidity[0]=  (Humidity >> 8 & 0xff);
	val_Humidity[1]=   Humidity  & 0xff;
	Send_To_Gatt_Client(val_Humidity, 128, VITAL_VAL);
	
		#ifdef UART_TERMINAL
	delay_m(1000);
	sprintf(H_buff,"\n\r Humidity =%.2f%%",humn);
  printf_string(UART,H_buff);
	#endif

}
	

void send_Gas_resi_to_app()
{
	char G_buff[50] ={0};
	gas_Resist = Gasadcdata(temp,gas_res);
	
	uint16_t gas_res_b = gas_Resist;
	uint8_t val_gas_res[128] = {0};
	val_gas_res[0]=  (gas_res_b >> 8 & 0xff);
	val_gas_res[1]=   gas_res_b  & 0xff;
	Send_To_Gatt_Client(val_gas_res, 128, VITAL_VAL);
	
	#ifdef UART_TERMINAL
	delay_m(1000);
	sprintf(G_buff,"\n\r Gas =%.3fKohm",gas_Resist);
  printf_string(UART,G_buff);
	#endif
}
  
void send_Co2_equivalent_to_app()
{
	char c_buff[50] ={0};
	co2_equivalent = (0.6 * hum_score + 0.4 * gas_score)*10.0f;
	uint16_t co2_e= co2_equivalent;
	uint8_t val_co2_e[128] = {0};
	val_co2_e[0]=  (co2_e >> 8 & 0xff);
	val_co2_e[1]=   co2_e  & 0xff;
	Send_To_Gatt_Client(val_co2_e, 128, VITAL_VAL);
	
	#ifdef UART_TERMINAL
	delay_m(1000);
	sprintf(c_buff,"\n\r Co2_eq =%.2fppm",co2_equivalent);
  printf_string(UART,c_buff);
	#endif
}
void send_IAQ_equivalent_to_app()
{
	char I_buff[50] ={0};
	index_percentage = fCalulate_IAQ_Index(gasResistance,Humidity);

	uint16_t IAq_e= index_percentage;
	uint8_t val_IAq_e[128] = {0};
	val_IAq_e[0]=  (IAq_e >> 8 & 0xff);
	val_IAq_e[1]=   (IAq_e  & 0xff);
	Send_To_Gatt_Client(val_IAq_e, 128, VITAL_VAL);
	
	#ifdef UART_TERMINAL
	delay_m(1000);
	sprintf(I_buff,"\n\r AQI =%.2f %%",index_percentage);
  printf_string(UART,I_buff);
	#endif
}
int detect_decrease_threshold_of_Gas_Value(float arr[], int size, int threshold);
int detect_decrease_threshold_of_Gas_Value(float arr[], int size, int threshold) {
    int start_index = -1;
    int end_index = -1;

    for (int i = 0; i < size - 1; i++) {
        if (arr[i] > arr[i + 1] && arr[i] >= threshold && arr[i + 1] < threshold) {
            if (start_index == -1) {
                start_index = i;
            }
            end_index = i + 1;
        }
    }

    if (start_index != -1) 
	{
		return 1;
        //printf("Decrease below threshold %d detected from index %d to index %d\n", threshold, start_index, end_index);
    } else {
		return 0;
        //printf("No decrease below threshold %d detected in the array\n", threshold);
    }
}



/***********************************************************************************/
/* Main code start from here */
/* All the System configuration has done inside system_init() function */ 
/**********************************************************************************/
int temp_arr[8]=0;
int hum_arr[8]=0;
int press_arr[8]=0;
int gas_arr1[10]=0;
int gas_arr[200]=0;

int32_t calc_hum;
float pressure_value;
float hum;
float pressure;
float humn;
uint8_t pData;
double tfine_bme680 = 0;

uint8_t pData;

struct bme68x_dev dev;
struct bme68x_data data1;
uint8_t n_meass;
uint16_t gas_res_adc =0;
uint8_t gas_range; 
char R_buff[50];
/*******************************************Global data for Gas value potential decrease***********************************************/
float Gas_Array[200];
int Number_of_Gas_samples = 200,Gas_i = 0,Gas_Freq = 1;
float Gas_Threshould = 150.0f;
/*******************************************Global data for Gas value potential decrease***********************************************/

int main(void)
{
//		FILE *FP=fopen("Data.txt","w+");
//	fprintf(FP,"%lf",10);
//	fclose(FP);
    sleep_mode_t sleep_mode;
		// initialize retention mode
    init_retention_mode();
    //global initialise
    system_init();
	i2c_test();
	i2c_bme_initialize();
	InitGas();
	i2c_read(0xD0,&id,1);
//	i2c_write(0x74,0x01,1);
//		i2c_read(0x74,&id,1);



	#if defined (CFG_COEX)
	  //SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0);            // disabled JTAG for Wifi Co-ex
	#endif
/*
 ************************************************************************************
 * Platform initialization
 ************************************************************************************
 */
 
	while(1)
	{
		
		//uint8_t i=0;
		//delay_m(1000);
	
	
	struct Result bme680 = Temperature_measurement();

			t_fine = bme680.t_fine_bme680;
			temp= bme680.Temp_bme680;
			//	delay_m(100);
		

		pressure = 915;//pressure_measurement(t_fine,pressure_value);
		humn= Humidity_measurement(t_fine,hum);  // humidity Function
		
 /*****************************************************Checking Exponential Curve************************/
Gasforcemode();
			 delay_m(1000);

		 gas_Resist = Gasadcdata(temp,gas_res);//Calling Gas value funtion


			 // gasPercentage = convertToPercentage(resistance, baselineResistance);
				index_percentage = fCalulate_IAQ_Index(gasResistance,Humidity);
				 co2_equivalent = (0.6 * hum_score + 0.4 * gas_score)*10.0f;

//		Gas_Array[Gas_i]=gas_Resist;
//		//Gas_i=(Gas_i + 1) % Number_of_Gas_samples;
//		Gas_i++;
//		if(Gas_i == 200) 
//		{
//   

//		if(detect_decrease_threshold_of_Gas_Value(Gas_Array,Number_of_Gas_samples,Gas_Threshould))
//			{
//				printf("Gas value is decreased\n");
//				memset(Gas_Array,0,sizeof(Gas_Array));
//				Gas_i=0;
//			}
//Gas_i = 0;
//		}
		/*****************************************************Checking Exponential Curve************************/
//     sprintf(R_buff, "\n\rTemprature = %.2f°C\n\rPressure = %.2fhPa\n\rHumidity = %.2f%%\n\rAQI = %.2f%%\n\rCO2 Equivalent = %.2fppm\n\r", temp, pressure, humn, index_percentage, co2_equivalent);
//     printf_string(UART,R_buff);
			 delay_m(20000);
		//adc_val();
	
			do
			{
				// schedule all pending events
				schedule_while_ble_on();
			}while (app_asynch_proc() != GOTO_SLEEP);  
			//grant control to an application, try to go to power down																								
			//if the application returns GOTO_SLEEP

			//wait for interrupt and go to sleep if this is allowed
			if (((!BLE_APP_PRESENT) && (check_gtl_state())) || (BLE_APP_PRESENT))
			{
					//Disable the interrupts
					GLOBAL_INT_STOP();

					app_asynch_sleep_proc();

					// get the allowed sleep mode
					// time from rwip_power_down() to __WFI() must be kept as short as possible!!
					sleep_mode = rwip_power_down();

					if ((sleep_mode == mode_ext_sleep) || (sleep_mode == mode_ext_sleep_otp_copy))
					{
							//power down the radio and whatever is allowed
							arch_goto_sleep(sleep_mode);

							// In extended sleep mode the watchdog timer is disabled
							// (power domain PD_SYS is automatically OFF). However, if the debugger
							// is attached the watchdog timer remains enabled and must be explicitly
							// disabled.
							if ((GetWord16(SYS_STAT_REG) & DBG_IS_UP) == DBG_IS_UP)
							{
									wdg_freeze();    // Stop watchdog timer
							}

							//wait for an interrupt to resume operation
							__WFI();

							if ((GetWord16(SYS_STAT_REG) & DBG_IS_UP) == DBG_IS_UP)
							{
									wdg_resume();    // Resume watchdog timer
							}

							//resume operation
							arch_resume_from_sleep();
					}
					else if (sleep_mode == mode_idle)
					{
							if (((!BLE_APP_PRESENT) && check_gtl_state()) || (BLE_APP_PRESENT))
							{
									//wait for an interrupt to resume operation
									__WFI();
							}
					}
					// restore interrupts
					GLOBAL_INT_START();
			}
			wdg_reload(WATCHDOG_DEFAULT_PERIOD);
			
		//	spi_store();
					
		}
	}

/**
 ****************************************************************************************
 * @brief Power down the BLE Radio and whatever is allowed according to the sleep mode and
 *        the state of the system and application
 * @param[in] current_sleep_mode The current sleep mode proposed by the application.
 ****************************************************************************************
 */
__STATIC_INLINE void arch_goto_sleep(sleep_mode_t current_sleep_mode)
{
    sleep_mode_t sleep_mode = current_sleep_mode;
#if (USE_RANGE_EXT)
    // Disable range extender
    range_ext.disable(NULL);
#endif

#if (WLAN_COEX_ENABLED)
    // Drive to inactive state the pin used for the BLE event in progress signal
    wlan_coex_set_ble_eip_pin_inactive();
#endif

    // Turn the radio off
    ble_turn_radio_off();

    // Grant access to the application to check if we can go to sleep
    app_sleep_prepare_proc(&sleep_mode); // SDK Improvements for uniformity this one should be changed?

    // Turn the peripherals off according to the current sleep mode
    arch_turn_peripherals_off(sleep_mode);

    // Hook for app specific tasks just before sleeping
    app_sleep_entry_proc(sleep_mode);

#if defined (__DA14531__)
    // Buck mode: switch on LDO_LOW (DC/DC will be automatically disabled in sleep)
    if (!GetBits16(ANA_STATUS_REG, BOOST_SELECTED))
    {
        /*
            In buck mode make sure that LDO_LOW is on before going to sleep.
            Set the LDO_LOW in low current mode.
        */
        SetBits16(POWER_CTRL_REG, LDO_LOW_CTRL_REG, 2);
    }

    // Set for proper RCX operation
    SetBits16(GP_DATA_REG, 0x60, 2);
#endif

    // Do the last house keeping of the clocks and go to sleep
    arch_switch_clock_goto_sleep(sleep_mode);
}

/**
 ****************************************************************************************
 * @brief Manage the clocks and go to sleep.
 * @param[in] current_sleep_mode The current sleep mode proposed by the system so far
 ****************************************************************************************
 */
__STATIC_INLINE void arch_switch_clock_goto_sleep(sleep_mode_t current_sleep_mode)
{
    if ( (current_sleep_mode == mode_ext_sleep) || (current_sleep_mode == mode_ext_sleep_otp_copy) )
    {
#if !defined (__DA14531__)
        SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_ENABLE, 0);      // Set BIAS to '0' if sleep has been decided
#endif

#if (USE_POWER_OPTIMIZATIONS)
#if defined (__DA14531__)
    SetBits16(CLK_RC32M_REG, RC32M_DISABLE, 0);             // Enable RC32M
#else
    SetBits16(CLK_16M_REG, RC16M_ENABLE, 1);                // Enable RC16M
#endif

    for (volatile int i = 0; i < 20; i++);

    SetBits16(CLK_CTRL_REG, SYS_CLK_SEL, 1);                // Switch to RC16M (DA14585/586) or RC32M (DA14531)

#if defined (__DA14531__)
    while( (GetWord16(CLK_CTRL_REG) & RUNNING_AT_RC32M) == 0 );
#else
    while( (GetWord16(CLK_CTRL_REG) & RUNNING_AT_RC16M) == 0 );
#endif
    SetWord16(CLK_FREQ_TRIM_REG, 0);                        // Set zero value to CLK_FREQ_TRIM_REG
    // Do not disable the XTAL16M (DA14585/586) or XTAL32M (DA14531).
    // It will be disabled when we sleep...
#endif // (USE_POWER_OPTIMIZATIONS)
    }
}

/**
 ****************************************************************************************
 * @brief  An interrupt came, resume from sleep.
 ****************************************************************************************
 */
__STATIC_INLINE void arch_resume_from_sleep(void)
{
#if defined (__DA14531__)
    if (arch_get_sleep_mode() == ARCH_EXT_SLEEP_OTP_COPY_ON)
    {
        // When waking up from extended sleep with OTP copy, structure content
        // shall be restored.
        booter_val.dbg_cfg = GetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE);
        booter_val.bandgap_reg = GetWord16(BANDGAP_REG);
        booter_val.clk_rc32m_reg = GetWord16(CLK_RC32M_REG);
        booter_val.clk_rc32k_reg = GetWord16(CLK_RC32K_REG);
    }

    if (!GetBits16(ANA_STATUS_REG, BOOST_SELECTED) && GetBits16(DCDC_CTRL_REG, DCDC_ENABLE))
    {
        /* In buck mode turn off LDO_LOW if DC/DC converter is enabled to allow the
           DC/DC converter to power VBAT_LOW rail. */
        SetBits16(POWER_CTRL_REG, LDO_LOW_CTRL_REG, 1);
    }

    // Set for proper RCX operation
    SetBits16(GP_DATA_REG, 0x60, 1);
#endif

    // Check if non retained heap should be re-initialized
    if (reinit_non_ret_heap)
    {
        ke_mem_init(KE_MEM_NON_RETENTION, (uint8_t*)(rom_cfg_table[rwip_heap_non_ret_pos]), rom_cfg_table[rwip_heap_non_ret_size]);
    }

#if defined (__DA14531__)
    // Re-initialize peripherals and pads
    periph_init();
#endif

    // Hook for app specific tasks just after waking up
    app_sleep_exit_proc( );

#if ((EXTERNAL_WAKEUP) && (!BLE_APP_PRESENT)) // external wake up, only in external processor designs
    ext_wakeup_resume_from_sleep();
#endif

    // restore ARM Sleep mode
    // reset SCR[2]=SLEEPDEEP bit else the mode=idle __WFI() will cause a deep sleep
    // instead of a processor halt
    SCB->SCR &= ~(1<<2);
}

/**
 ****************************************************************************************
 * @brief Check if the BLE module is powered on.
 ****************************************************************************************
 */
__STATIC_INLINE bool ble_is_powered()
{
    return ((GetBits16(CLK_RADIO_REG, BLE_ENABLE) == 1) &&
            (GetBits32(BLE_DEEPSLCNTL_REG, DEEP_SLEEP_STAT) == 0) &&
            !(rwip_prevent_sleep_get() & RW_WAKE_UP_ONGOING));
}

/**
 ****************************************************************************************
 * @brief Call the scheduler if the BLE module is powered on.
 ****************************************************************************************
 */
__STATIC_INLINE void schedule_while_ble_on(void)
{
    // BLE clock is enabled
    while (ble_is_powered())
    {
        //execute messages and events
        rwip_schedule();
#if defined (__DA14531__)
        rcx20_read_freq(false);
#endif

#if !defined(__FPGA__)
        if (arch_rwble_last_event == BLE_EVT_END)
        {
#if defined (__DA14531__)
            rcx20_read_freq(true);
#else
            rcx20_read_freq();
#endif

            uint32_t sleep_duration = 0;
            //if you have enough time run a temperature calibration of the radio
            if (ea_sleep_check(&sleep_duration, 4)) //6 slots -> 3.750 ms
            {
                // check time and temperature to run radio calibrations.
                conditionally_run_radio_cals();
            }
        }
#endif

        //grant control to the application, try to go to sleep
        //if the application returns GOTO_SLEEP
        if (app_asynch_trm() != GOTO_SLEEP)
        {
            continue; // so that rwip_schedule() is called again
        }
        else
        {
            arch_printf_process();
            break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Power down the ble ip if possible.
 * @return sleep_mode_t return the current sleep mode
 ****************************************************************************************
 */
__STATIC_INLINE sleep_mode_t rwip_power_down(void)
{
    sleep_mode_t sleep_mode;

#if ((EXTERNAL_WAKEUP) && (!BLE_APP_PRESENT)) // External wake up, only in external processor designs
    ext_wakeup_enable(EXTERNAL_WAKEUP_GPIO_PORT, EXTERNAL_WAKEUP_GPIO_PIN, EXTERNAL_WAKEUP_GPIO_POLARITY);
#endif

    // if app has turned sleep off, rwip_sleep() will act accordingly
    // time from rwip_sleep() to __WFI() must be kept as short as possible!
    sleep_mode = rwip_sleep();

    // BLE is sleeping ==> app defines the mode
    if (sleep_mode == mode_sleeping)
    {
        if (sleep_env.slp_state == ARCH_EXT_SLEEP_ON)
        {
            sleep_mode = mode_ext_sleep;
        }
        else
        {
            sleep_mode = mode_ext_sleep_otp_copy;
        }
    }
#if ((EXTERNAL_WAKEUP) && (!BLE_APP_PRESENT))
    else
    {
        // Disable external wake up
        ext_wakeup_disable();
    }
#endif

    return (sleep_mode);
}

/**
 ****************************************************************************************
 * @brief Turn the radio off.
 ****************************************************************************************
 */
__STATIC_INLINE void ble_turn_radio_off(void)
{
    SetBits16(PMU_CTRL_REG, RADIO_SLEEP, 1); // turn off radio
}

/**
 ****************************************************************************************
 * @brief Initialize retention mode.
 ****************************************************************************************
 */
__STATIC_INLINE void init_retention_mode(void)
{
#if defined (CFG_CUSTOM_SCATTER_FILE)
    // User selects which RAM blocks to retain (code and retention data or only retention data)
    ret_mode = ret_mode_for_ret_data = ALL_RAM_BLOCKS;

#else
    // RAM retention mode for retention data only
    ret_mode_for_ret_data = get_retention_mode_data();

    // RAM retention mode for code and data
#if defined (__DA14531__)
    ret_mode = get_retention_mode() & ret_mode_for_ret_data;
#else
    ret_mode = get_retention_mode() | ret_mode_for_ret_data;
#endif

#endif
    // RAM retention mode for the non retainable heap data only
    ret_mode_for_non_ret_heap = get_retention_mode_non_ret_heap();
}

/**
 ****************************************************************************************
 * @brief Define which RAM blocks will be retained based on the base address of the
 *        retention data which touch the aforementioned RAM blocks.
 *        The last RAM block is always retained.
 * @return the retention mode (the RAM blocks to be retained)
 * @note This function is strongly related to the default SDK scatter file structure.
 ****************************************************************************************
 */
#if !defined (CFG_CUSTOM_SCATTER_FILE)
__STATIC_INLINE uint8_t get_retention_mode_data(void)
{
    uint32_t ret_mem_base_addr = RET_MEM_BASE;

#if defined (__DA14531__)
    // Check the base address of the retention data
    if (ret_mem_base_addr >= RAM_3_BASE_ADDR)
    {
        // Retain RAM_3 block
        return (RAM_3_BLOCK);
    }
    else if (ret_mem_base_addr >= RAM_2_BASE_ADDR)
    {
        // Retain RAM_2 and RAM_3 block
        return (RAM_2_BLOCK & RAM_3_BLOCK);
    }
#else
    // Check the base address of the retention data
    if (ret_mem_base_addr >= RAM_4_BASE_ADDR)
    {
        // Retain RAM_4 block
        return (RAM_4_BLOCK);
    }
    else if (ret_mem_base_addr >= RAM_3_BASE_ADDR)
    {
        // Retain RAM_3 and RAM_4 block
        return (RAM_3_BLOCK | RAM_4_BLOCK);
    }
    else if (ret_mem_base_addr >= RAM_2_BASE_ADDR)
    {
        // Retain RAM_2, RAM_3 and RAM_4 block
        return (RAM_2_BLOCK | RAM_3_BLOCK | RAM_4_BLOCK);
    }
#endif
    // Retain all RAM blocks
    return ALL_RAM_BLOCKS;
}
//void i2c_test(void)
//{

//RESERVE_GPIO(SCL, GPIO_PORT_0, GPIO_PIN_8, PID_I2C_SCL);
//    RESERVE_GPIO(SDA, GPIO_PORT_0, GPIO_PIN_11, PID_I2C_SDA);
//	    RESERVE_GPIO(OUTPUT, GPIO_PORT_0, GPIO_PIN_5, FUNC_GPIO);
//	
//  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_8, INPUT, PID_I2C_SCL, true);
//  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_11, OUTPUT, PID_I2C_SDA, true);
//	
//	 GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_5, OUTPUT, FUNC_GPIO, true);
//	
//	
//}
void i2c_test(void)
{

//	  RESERVE_GPIO(SDA, GPIO_PORT_0, GPIO_PIN_11, PID_I2C_SDA);
//    RESERVE_GPIO(SCL, GPIO_PORT_0, GPIO_PIN_8, PID_I2C_SCL);
//	 

//  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_11, INPUT, PID_I2C_SDA, true);
//  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_8, OUTPUT, PID_I2C_SCL, true);
	
	
		  RESERVE_GPIO(SCL, GPIO_PORT_0, GPIO_PIN_11, PID_I2C_SCL);
    RESERVE_GPIO(SDA, GPIO_PORT_0, GPIO_PIN_8, PID_I2C_SDA);
	 

  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_11, INPUT, PID_I2C_SCL, true);
  GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_8, OUTPUT, PID_I2C_SDA, true);
	
	
}

/**
 ****************************************************************************************
 * @brief Define which RAM blocks will be retained based on the code and the retention
 *        data size.
 *        The last RAM block is always retained.
 * @return the retention mode (the RAM blocks to be retained)
 * @note This function is strongly related to the default SDK scatter file.
 ****************************************************************************************
 */
__STATIC_INLINE uint8_t get_retention_mode(void)
{
    // The following equation calculates the code size and is strongly related to the
    // default SDK scatter file structure. Count vector table and rom patches as well.
    code_size = (CODE_AREA_BASE - RAM_1_BASE_ADDR) + CODE_AREA_LENGTH;

#if defined (__DA14531__)
    // Check the code size
    if (code_size <= (RAM_2_BASE_ADDR - RAM_1_BASE_ADDR))
    {
        // Code fits in RAM_1 block
        return (RAM_1_BLOCK);
    }
#else
    // Check the code size
    if (code_size <= (RAM_2_BASE_ADDR - RAM_1_BASE_ADDR))
    {
        // Code fits in RAM_1 block
        return (RAM_1_BLOCK);
    }
    else if (code_size <= (RAM_3_BASE_ADDR - RAM_1_BASE_ADDR))
    {
        // Code fits in RAM_1 block and RAM_2 block
        return (RAM_1_BLOCK | RAM_2_BLOCK);
    }
#endif
    // Retain all RAM blocks
    return ALL_RAM_BLOCKS;
}
#endif // CFG_CUSTOM_SCATTER_FILE

/**
 ****************************************************************************************
 * @brief Calculate the RAM blocks which will be retained depending on the non retained
 *        heap base address and size.
 * @return the retention mode (the RAM blocks to be retained)
 ****************************************************************************************
 */
__STATIC_INLINE uint8_t get_retention_mode_non_ret_heap()
{
    uint32_t non_ret_heap_base = NON_RET_HEAP_BASE;
    uint32_t non_ret_heap_length = NON_RET_HEAP_LENGTH;

#if defined (__DA14531__)
    if (non_ret_heap_base >= RAM_3_BASE_ADDR)
    {
        return (RAM_3_BLOCK);
    }
    else if (non_ret_heap_base >= RAM_2_BASE_ADDR)
    {
        if (non_ret_heap_base + non_ret_heap_length < RAM_3_BASE_ADDR)
        {
            return (RAM_2_BLOCK);
        }
        else
        {
            return (RAM_2_BLOCK & RAM_3_BLOCK);
        }
    }
    else
    {
        if (non_ret_heap_base + non_ret_heap_length < RAM_2_BASE_ADDR)
        {
            return (RAM_1_BLOCK);
        }
        else if (non_ret_heap_base + non_ret_heap_length < RAM_3_BASE_ADDR)
        {
            return (RAM_1_BLOCK & RAM_2_BLOCK);
        }
        else
        {
            return ALL_RAM_BLOCKS;
        }
    }
#else
    if (non_ret_heap_base >= RAM_4_BASE_ADDR)
    {
        return (RAM_4_BLOCK);
    }
    else if (non_ret_heap_base >= RAM_3_BASE_ADDR)
    {
        if (non_ret_heap_base + non_ret_heap_length < RAM_4_BASE_ADDR)
        {
            return (RAM_3_BLOCK);
        }
        else
        {
            return (RAM_3_BLOCK | RAM_4_BLOCK);
        }
    }
    else if (non_ret_heap_base >= RAM_2_BASE_ADDR)
    {
        if (non_ret_heap_base + non_ret_heap_length < RAM_3_BASE_ADDR)
        {
            return (RAM_2_BLOCK);
        }
        else if (non_ret_heap_base + non_ret_heap_length < RAM_4_BASE_ADDR)
        {
            return (RAM_2_BLOCK | RAM_3_BLOCK);
        }
        else
        {
            return (RAM_2_BLOCK | RAM_3_BLOCK | RAM_4_BLOCK);
        }
    }
    else
    {
        if (non_ret_heap_base + non_ret_heap_length < RAM_2_BASE_ADDR)
        {
            return (RAM_1_BLOCK);
        }
        else if (non_ret_heap_base + non_ret_heap_length < RAM_3_BASE_ADDR)
        {
            return (RAM_1_BLOCK | RAM_2_BLOCK);
        }
        else if (non_ret_heap_base + non_ret_heap_length < RAM_4_BASE_ADDR)
        {
            return (RAM_1_BLOCK | RAM_2_BLOCK | RAM_3_BLOCK);
        }
        else
        {
            return ALL_RAM_BLOCKS;
        }
    }
#endif
}

#if !defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief Set required LDO_RET_TRIM value in BANDGAP_REG. This value depends on the
 *        size of retained RAM and the temperature range device operation.
 * @param[in] retained_ram_blocks RAM blocks to be retained.
 * @note By default the [BANDGAP_REG.LDO_RET_TRIM] is loaded with the respective OTP
 *       header value (done by ROM booter).
 ****************************************************************************************
 */
__STATIC_INLINE void set_ldo_ret_trim(uint8_t retained_ram_blocks)
{
#if (USE_MID_TEMPERATURE)
    SetBits16(BANDGAP_REG, LDO_RET_TRIM, DEFAULT_LDO_SET);
#elif (USE_HIGH_TEMPERATURE || USE_EXT_TEMPERATURE)
    if ((retained_ram_blocks == RAM_SIZE_80KB_OPT1) || (retained_ram_blocks == RAM_SIZE_80KB_OPT2) || (retained_ram_blocks == RAM_SIZE_96KB_OPT1))
    {
            SetBits16(BANDGAP_REG, LDO_RET_TRIM, DEFAULT_LDO_SET_96K);  // LDO trim value up to 96KB retained RAM
    }
    else
    {
            SetBits16(BANDGAP_REG, LDO_RET_TRIM, DEFAULT_LDO_SET_64K);  // LDO trim value up to 64KB retained RAM
    }
#endif
}
#endif

/**
 ****************************************************************************************
 * @brief  Turn the peripherals off according to the current sleep mode.
 * @param[in] current_sleep_mode The current sleep mode proposed by the system so far
 ****************************************************************************************
 */
__STATIC_INLINE void arch_turn_peripherals_off(sleep_mode_t current_sleep_mode)
{
    if (current_sleep_mode == mode_ext_sleep || current_sleep_mode == mode_ext_sleep_otp_copy)
    {
        uint8_t retained_ram_blocks = 0;
        SCB->SCR |= 1<<2; // enable deep sleep  mode bit in System Control Register (SCR[2]=SLEEPDEEP)

#if defined (__DA14531__)
        SetBits16(PAD_LATCH_REG, PAD_LATCH_EN, 0);
#else
        SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 0);          // activate PAD latches

        SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 1);          // turn off peripheral power domain
#endif
        if (current_sleep_mode == mode_ext_sleep)
        {
            /*
             * Sleep mode: EXTENDED - image kept at external resource
             *
             * The RAM blocks that hold the code and the retention data
             * must be retained.
             */

            // OTP copy and OTP copy emulation will be disabled
            SetBits16(SYS_CTRL_REG, OTP_COPY, 0);
            retained_ram_blocks = ret_mode;
        }
        else
        {
            /*
             * Sleep mode: EXTENDED - image kept at OTP (OTP mirroring takes place on wake-up)
             *
             * The RAM blocks that hold the retention data must be retained.
             */

#if (DEVELOPMENT_DEBUG)

            // enable OTP copy emulation
            SetBits16(SYS_CTRL_REG, OTP_COPY, 1);
            SetBits16(SYS_CTRL_REG, DEV_PHASE, 1);
            retained_ram_blocks = ret_mode;
#else
            // enable OTP copy
            SetBits16(SYS_CTRL_REG, DEV_PHASE, 0);
            retained_ram_blocks = ret_mode_for_ret_data;
#endif
            otp_prepare((code_size + 3) >> 2);       // convert function argument from bytes to 32-bit words
        }

        // manage the non-retained heap
        // Note: non-retained heap is either empty or not. If it is non empty it must be retained.
        if (!ke_mem_is_empty(KE_MEM_NON_RETENTION))
        {
            reinit_non_ret_heap = 0;
#if defined (__DA14531__)
            retained_ram_blocks &= ret_mode_for_non_ret_heap;
#else
            retained_ram_blocks |= ret_mode_for_non_ret_heap;
#endif
        }
        else
        {
            reinit_non_ret_heap = 1;
        }

        // dynamically select the retained RAM blocks
        arch_ram_set_retention_mode(retained_ram_blocks);

#if !defined (__DA14531__)
        set_ldo_ret_trim(retained_ram_blocks);
#endif
    }
}

/**
 ****************************************************************************************
 * @brief Prepare OTP Controller in order to be able to reload SysRAM at the next power-up.
 ****************************************************************************************
 */
__STATIC_INLINE void otp_prepare(uint32_t code_size)
{
    // Initialize OTP controller
    hw_otpc_init();

    SetBits16(SYS_CTRL_REG, OTP_COPY, 1);

    // Copy the size of software from the first word of the retention mem.
    SetWord32 (OTPC_NWORDS_REG, code_size - 1);

    // Close OTP controller
    hw_otpc_close();

#if defined (__DA14531__) && (!USE_POWER_MODE_BYPASS)
    // In Boost mode enable the DCDC converter to supply VBAT_HIGH for OTP when the system will
    // wake up.
    if (GetBits16(ANA_STATUS_REG, BOOST_SELECTED))
    {
        // Clear reservation status of DCDC converter
        hw_otpc_clear_dcdc_reserved();

        // Set DCDC converter voltage level for OTP read (required for OTP copy to RAM at wake up)
        syscntl_dcdc_set_level(SYSCNTL_DCDC_LEVEL_1V8);

        // Enable the DCDC converter (required for OTP copy to RAM at wake up)
        SetBits16(DCDC_CTRL_REG, DCDC_ENABLE, 1);
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Used for sending messages to kernel tasks generated from
 *        asynchronous events that have been processed in app_asynch_proc.
 * @return KEEP_POWERED to force calling of schedule_while_ble_on(), else GOTO_SLEEP
 ****************************************************************************************
 */
__STATIC_INLINE arch_main_loop_callback_ret_t app_asynch_trm(void)
{
    if (user_app_main_loop_callbacks.app_on_ble_powered != NULL)
    {
        return user_app_main_loop_callbacks.app_on_ble_powered();
    }
    else
    {
        return GOTO_SLEEP;
    }
}

/**
 ****************************************************************************************
 * @brief Used for processing of asynchronous events at ?user? level. The
 *        corresponding ISRs should be kept as short as possible and the
 *        remaining processing should be done at this point.
 * @return KEEP_POWERED to force calling of schedule_while_ble_on(), else GOTO_SLEEP
 ****************************************************************************************
 */
__STATIC_INLINE arch_main_loop_callback_ret_t app_asynch_proc(void)
{
    if (user_app_main_loop_callbacks.app_on_system_powered != NULL)
    {
        return user_app_main_loop_callbacks.app_on_system_powered();
    }
    else
    {
        return GOTO_SLEEP;
    }
}

/**
 ****************************************************************************************
 * @brief Used for updating the state of the application just before sleep checking starts.
 ****************************************************************************************
 */
__STATIC_INLINE void app_asynch_sleep_proc(void)
{
    if (user_app_main_loop_callbacks.app_before_sleep != NULL)
        user_app_main_loop_callbacks.app_before_sleep();
}

/**
 ****************************************************************************************
 * @brief Used to override the slected extended sleep mode , based on the current
 *        application state.
 *        BLE and Radio are still powered off.
 * @param[in] sleep_mode     Sleep Mode
 ****************************************************************************************
 */
__STATIC_INLINE void app_sleep_prepare_proc(sleep_mode_t *sleep_mode)
{
    if (user_app_main_loop_callbacks.app_validate_sleep != NULL)
        (*sleep_mode) = user_app_main_loop_callbacks.app_validate_sleep(*sleep_mode);
}

/**
 ****************************************************************************************
 * @brief Used for application specific tasks just before entering the low power mode.
 * @param[in] sleep_mode     Sleep Mode
 ****************************************************************************************
 */
__STATIC_INLINE void app_sleep_entry_proc(sleep_mode_t sleep_mode)
{
    if (user_app_main_loop_callbacks.app_going_to_sleep != NULL)
        user_app_main_loop_callbacks.app_going_to_sleep(sleep_mode);
}

/**
 ****************************************************************************************
 * @brief Used for application specific tasks immediately after exiting the low power mode.
 * @param[in] sleep_mode     Sleep Mode
 ****************************************************************************************
 */
__STATIC_INLINE void app_sleep_exit_proc(void)
{
    if (user_app_main_loop_callbacks.app_resume_from_sleep != NULL)
        user_app_main_loop_callbacks.app_resume_from_sleep();
}

#if defined (__GNUC__)
// Defining these functions forces the linker to ignore unwind functions
void __aeabi_unwind_cpp_pr0(void) {};
void __aeabi_unwind_cpp_pr1(void) {};
void __aeabi_unwind_cpp_pr2(void) {};
#endif








/// @} DRIVERS
