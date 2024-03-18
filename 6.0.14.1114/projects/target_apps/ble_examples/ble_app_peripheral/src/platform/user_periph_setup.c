/**
 ****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief Peripherals setup and initialization.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */



/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 #if (WLAN_COEX_ENABLED)
#include "wlan_coex.h"
#endif

#include "user_periph_setup.h"
#include "datasheet.h"
#include "system_library.h"
#include "rwip_config.h"
#include "gpio.h"
#include "uart.h"
#include "syscntl.h"

# include "time.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#define DEBUG_UART2 1
/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 ****************************************************************************************
 */
#if (WLAN_COEX_ENABLED)
// Configuration struct for WLAN coexistence
const wlan_coex_cfg_t wlan_coex_cfg = {
 .ext_24g_eip_port = WLAN_COEX_24G_EIP_PORT,
 .ext_24g_eip_pin = WLAN_COEX_24G_EIP_PIN,
 .ble_eip_port = WLAN_COEX_BLE_EIP_PORT,
 .ble_eip_pin = WLAN_COEX_BLE_EIP_PIN,
   .ble_prio_port = WLAN_COEX_BLE_PRIO_PORT,
   .ble_prio_pin = WLAN_COEX_BLE_PRIO_PIN,
#if defined (CFG_WLAN_COEX_DEBUG)
   .debug_a_port = WLAN_COEX_DEBUG_A_PORT,
   .debug_a_pin = WLAN_COEX_DEBUG_A_PIN,
   .debug_b_port = WLAN_COEX_DEBUG_B_PORT,
   .debug_b_pin = WLAN_COEX_DEBUG_B_PIN,
#endif
   .irq = WLAN_COEX_IRQ,
};
#endif






#if DEVELOPMENT_DEBUG

void uart_reinit(void);

void uart_send_cb(uint16_t length);
void uart_error_cb(uart_t * Uart_Instance, uint8_t err_no);
void GPIO_reservations(void)
{
/*
	
	
    i.e. to reserve P0_1 as Generic Purpose I/O:
    RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);
*/

	RESERVE_GPIO(SDA, GPIO_PORT_0, SPI_DO_PIN, PID_SPI_DO);
  RESERVE_GPIO(SCL, GPIO_PORT_0, SPI_DI_PIN, PID_SPI_DI);
  RESERVE_GPIO(SCL, GPIO_PORT_0, SPI_CLK_PIN, PID_SPI_CLK);
  RESERVE_GPIO(SCL, GPIO_PORT_0, SPI_EN_PIN, PID_SPI_EN);
//	//RESERVE_GPIO(UART2_TX, UART2_TX_PORT, UART2_TX_PIN, PID_UART2_TX);
//#if defined (CFG_PRINTF_UART2)
//    //RESERVE_GPIO(UART2_TX, UART2_TX_PORT, UART2_TX_PIN, PID_UART2_TX);
//#endif
//	GPIO_Disable_HW_Reset();

////   RESERVE_GPIO(UART1_TX, UART1_TX_PORT, UART1_TX_PIN, PID_UART1_TX);
////	 RESERVE_GPIO(UART1_RX, UART1_RX_PORT, UART1_RX_PIN, PID_UART1_RX);
//	 RESERVE_GPIO(UART_DATA,GPIO_PORT_0, GPIO_PIN_4,PID_GPIO);
//	 
////	#if DEBUG_UART2
////	 RESERVE_GPIO(UART2_TX, UART2_TX_PORT, UART2_TX_PIN, PID_UART2_TX); //debug uart2
////	#else
////	RESERVE_GPIO(ECG_DATA,GPIO_PORT_0, GPIO_PIN_3,PID_GPIO);
////	#endif
//	
//#if !defined (__DA14586__)
//    //RESERVE_GPIO(SPI_EN, SPI_EN_PORT, SPI_EN_PIN, PID_SPI_EN);
//#endif

RESERVE_GPIO(UART2_TX, UART2_TX_PORT, UART2_TX_PIN, PID_UART2_TX);
}

#endif

void set_pad_functions(void)
{
/*
    i.e. to set P0_1 as Generic purpose Output:
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_1, OUTPUT, PID_GPIO, false);
*/

#if defined (__DA14586__)
    // Disallow spontaneous DA14586 SPI Flash wake-up
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_3, OUTPUT, PID_GPIO, true);
#else
    // Disallow spontaneous SPI Flash wake-up
   // GPIO_ConfigurePin(SPI_EN_PORT, SPI_EN_PIN, OUTPUT, PID_SPI_EN, true);
	  GPIO_ConfigurePin(SPI_EN_PORT, SPI_EN_PIN, OUTPUT, PID_SPI_EN, true);
    GPIO_ConfigurePin(SPI_CLK_PORT, SPI_CLK_PIN, OUTPUT, PID_SPI_CLK, false);
    GPIO_ConfigurePin(SPI_DO_PORT, SPI_DO_PIN, OUTPUT, PID_SPI_DO, false);
    GPIO_ConfigurePin(SPI_DI_PORT, SPI_DI_PIN, INPUT, PID_SPI_DI, false);
#endif
GPIO_Disable_HW_Reset();
#if defined (CFG_PRINTF_UART2)
    // Configure UART2 TX Pad
    GPIO_ConfigurePin(UART2_TX_PORT, UART2_TX_PIN, OUTPUT, PID_UART2_TX, false);
#endif
	

	#if (WLAN_COEX_ENABLED)
   wlan_coex_gpio_cfg();
#endif

	 //GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_4, INPUT_PULLDOWN, PID_GPIO, false); //PB14 on stm
	 
#if DEBUG_UART2
	GPIO_ConfigurePin(UART2_TX_PORT, UART2_TX_PIN, OUTPUT, PID_UART2_TX, false); //debug uart2
#else
	//GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_3, OUTPUT, PID_GPIO, false);// PB13 
#endif
}



static uart_cfg_t uart_cfg = {
    .baud_rate = UART_BAUDRATE_230400,
    .data_bits = UART2_DATABITS,
    .parity = UART_PARITY_NONE,
    .stop_bits = UART2_STOPBITS,
    .auto_flow_control = UART2_AFCE,
    .use_fifo = UART2_FIFO,
    .tx_fifo_tr_lvl = UART2_TX_FIFO_LEVEL,
    .rx_fifo_tr_lvl = UART2_RX_FIFO_LEVEL,
    .intr_priority = 2,
};

static const spi_cfg_t spi_cfg = {
    .spi_ms = SPI_MS_MODE,
    .spi_cp = SPI_CP_MODE,
    .spi_speed = SPI_SPEED_MODE,
    .spi_wsz = SPI_WSZ,
    .spi_cs = SPI_CS,
    .cs_pad.port = SPI_EN_PORT,
    .cs_pad.pin = SPI_EN_PIN,
#if defined (__DA14531__)
    .spi_capture = SPI_EDGE_CAPTURE,
#endif
#if defined (CFG_SPI_DMA_SUPPORT)
    .spi_dma_channel = SPI_DMA_CHANNEL_01,
    .spi_dma_priority = DMA_PRIO_0,
#endif
};

// Configuration struct for SPI FLASH
static const spi_flash_cfg_t spi_flash_cfg = {
    .chip_size = SPI_FLASH_DEV_SIZE,
};

void periph_init(void)
{
#if defined (__DA14531__)
	
	GPIO_Disable_HW_Reset();
    // In Boost mode enable the DCDC converter to supply VBAT_HIGH for the used GPIOs
	
    syscntl_dcdc_turn_on_in_boost(SYSCNTL_DCDC_LEVEL_3V0);
#else
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));
    SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_ENABLE, 1);
#endif

    // ROM patch
    patch_func();
	
#if DEBUG_UART2
	   uart_initialize(UART2, &uart_cfg); //debug uart2
#endif		 

	//uart_initialize(UART2, &uart_cfg); //debug uart2
     spi_flash_configure_env(&spi_flash_cfg);

    // Initialize SPI
//	i2c_bme_initialize();


    set_pad_functions();

    // Enable the pads
    GPIO_set_pad_latch_en(true);
		
}


void uart_reinit(void)
{
	
	Close_Uart(UART1);
	
	uart_disable(UART1);
	uart_cfg.baud_rate = UART_BAUDRATE_230400;
	uart_initialize(UART1, &uart_cfg);
	
	Open_Uart(UART1);
		
}

void uart_send_cb(uint16_t len)
{

}

void UART_OPEN_PORT_INIT(void)
{	
	  uart_initialize(UART1, &uart_cfg);
		uart_register_tx_cb(UART1, uart_send_cb);
//		uart_register_err_cb(UART1, uart_error_cb);
		Open_Uart(UART1);
	
	  GPIO_ConfigurePin(UART1_TX_PORT, UART1_TX_PIN, OUTPUT, PID_UART1_TX, false);
    GPIO_ConfigurePin(UART1_RX_PORT, UART1_RX_PIN, INPUT, PID_UART1_RX, false);
    GPIO_set_pad_latch_en(true);
	
}

void UART_CLOSE_PORT_INIT(void)
{
		Close_Uart(UART1);
		uart_disable(UART1);
	
}



