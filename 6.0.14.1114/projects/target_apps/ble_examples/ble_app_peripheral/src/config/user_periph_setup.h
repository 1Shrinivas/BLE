/**
 ****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "spi_flash.h"
#include "i2c.h"
#include "i2c_eeprom.h"



/*
 * DEFINES
 ****************************************************************************************
 */
#define DEBUG_UART_2  1
#define DEBUGGER_UART1_PINCONFIG  0x00

/****************************************************************************************/
/* UART2 configuration                                                                  */
/****************************************************************************************/
#define    UART                        UART2

// Define UART2 Tx Pad
#if defined (__DA14531__)

#if DEBUGGER_UART1_PINCONFIG
		#define UART1_TX_PORT           GPIO_PORT_0
    #define UART1_TX_PIN            GPIO_PIN_1
    #define UART1_RX_PORT           GPIO_PORT_0
    #define UART1_RX_PIN            GPIO_PIN_9
#else 	
		#define UART1_TX_PORT           GPIO_PORT_0
    #define UART1_TX_PIN            GPIO_PIN_0
    #define UART1_RX_PORT           GPIO_PORT_0
    #define UART1_RX_PIN            GPIO_PIN_1
#endif

#else
    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_6
#endif



#if DEBUG_UART_2 
    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_6
#endif

// Define UART2 Settings
#define UART2_BAUDRATE              UART_BAUDRATE_115200//UART_BAUDRATE_1000000//UART_BAUDRATE_500000
#define UART2_DATABITS              UART_DATABITS_8
#define UART2_PARITY                UART_PARITY_EVEN
#define UART2_STOPBITS              UART_STOPBITS_1
#define UART2_AFCE                  UART_AFCE_DIS
#define UART2_FIFO                  UART_FIFO_EN
#define UART2_TX_FIFO_LEVEL         UART_TX_FIFO_LEVEL_0
#define UART2_RX_FIFO_LEVEL         UART_RX_FIFO_LEVEL_0


/****************************************************************************************/
/* LED configuration                                                                    */
/****************************************************************************************/
#if defined (__DA14531__)
    #define GPIO_LED_PORT           GPIO_PORT_0
    #define GPIO_LED_PIN            GPIO_PIN_9
#else
    #define GPIO_LED_PORT           GPIO_PORT_1
    #define GPIO_LED_PIN            GPIO_PIN_0
#endif

/****************************************************************************************/
/* SPI configuration                                                                    */
/****************************************************************************************/
// Define SPI Pads
#if defined (__DA14531__)
    #define SPI_EN_PORT             GPIO_PORT_0
    #define SPI_EN_PIN              GPIO_PIN_1

    #define SPI_CLK_PORT            GPIO_PORT_0
    #define SPI_CLK_PIN             GPIO_PIN_4

    #define SPI_DO_PORT             GPIO_PORT_0
    #define SPI_DO_PIN              GPIO_PIN_0

    #define SPI_DI_PORT             GPIO_PORT_0
    #define SPI_DI_PIN              GPIO_PIN_3

#elif !defined (__DA14586__)
    #define SPI_EN_PORT             GPIO_PORT_0
    #define SPI_EN_PIN              GPIO_PIN_3

    #define SPI_CLK_PORT            GPIO_PORT_0
    #define SPI_CLK_PIN             GPIO_PIN_0

    #define SPI_DO_PORT             GPIO_PORT_0
    #define SPI_DO_PIN              GPIO_PIN_6

    #define SPI_DI_PORT             GPIO_PORT_0
    #define SPI_DI_PIN              GPIO_PIN_5
#endif
/****************************************************************************************/
/* I2C configuration                                                                    */
/****************************************************************************************/
// Define I2C Pins
#if defined (__DA14531__)
    #define I2C_SCL_PORT           // GPIO_PORT_0
    #define I2C_SCL_PIN             //GPIO_PIN_11
    #define I2C_SDA_PORT            //GPIO_PORT_0
    #define I2C_SDA_PIN            // GPIO_PIN_8
#else
    #define I2C_SCL_PORT            GPIO_PORT_2
    #define I2C_SCL_PIN             GPIO_PIN_3
    #define I2C_SDA_PORT            GPIO_PORT_2
    #define I2C_SDA_PIN             GPIO_PIN_1
#endif

// Select EEPROM characteristics
#define I2C_EEPROM_DEV_SIZE         0x20000               // EEPROM size in bytes
#define I2C_EEPROM_PAGE_SIZE        256                   // EEPROM page size in bytes
#define I2C_SLAVE_ADDRESS           0x76                  // Set slave device address
#define I2C_SPEED_MODE              I2C_SPEED_FAST        // Speed mode: I2C_SPEED_STANDARD (100 kbits/s), I2C_SPEED_FAST (400 kbits/s)
#define I2C_ADDRESS_MODE            I2C_ADDRESSING_7B     // Addressing mode: {I2C_ADDRESSING_7B, I2C_ADDRESSING_10B}
#define I2C_ADDRESS_SIZE            I2C_2BYTES_ADDR       // Address width: {I2C_1BYTE_ADDR, I2C_2BYTES_ADDR, I2C_3BYTES_ADDR}


/***************************************************************************************/
/* Production debug output configuration                                               */
/***************************************************************************************/
#if PRODUCTION_DEBUG_OUTPUT
#if defined (__DA14531__)
   // #define PRODUCTION_DEBUG_PORT   GPIO_PORT_0
    //#define PRODUCTION_DEBUG_PIN    GPIO_PIN_11
#else
    #define PRODUCTION_DEBUG_PORT   GPIO_PORT_2
    #define PRODUCTION_DEBUG_PIN    GPIO_PIN_5
#endif
#endif

/****************************************************************************************/
/* WLAN COEX pin configuration                                                          */
/****************************************************************************************/

#if (WLAN_COEX_ENABLED)
#if defined (__DA14531__)

   /// Input signal to device: 2.4GHz external device event in progress indication.
   #define WLAN_COEX_24G_EIP_PORT      GPIO_PORT_0
   #define WLAN_COEX_24G_EIP_PIN       GPIO_PIN_2

   /// Output signal from device: BLE event in progress indication.
 #define WLAN_COEX_BLE_EIP_PORT      GPIO_PORT_0
 //#define WLAN_COEX_BLE_EIP_PIN       GPIO_PIN_5

   /// Output signal from device: BLE priority indication.
   #define WLAN_COEX_BLE_PRIO_PORT     GPIO_PORT_0
   #define WLAN_COEX_BLE_PRIO_PIN      GPIO_PIN_4

#if defined (CFG_WLAN_COEX_DEBUG)
   /// BLE radio overruled signal pin definition.
   /// This signal goes high when the BLE radio is forced to be off due to external 2.4GHz device activity.
   #define WLAN_COEX_DEBUG_A_PORT      GPIO_PORT_0
   #define WLAN_COEX_DEBUG_A_PIN       GPIO_PIN_9

   /// External 2.4GHz device EIP handler signal pin definition.
   /// This signal indicates when an external 2.4GHz device wants to start or stop sending data.
   #define WLAN_COEX_DEBUG_B_PORT      GPIO_PORT_0
  // #define WLAN_COEX_DEBUG_B_PIN       GPIO_PIN_8
#endif

#else

   /// Input signal to device: 2.4GHz external device event in progress indication.
   #define WLAN_COEX_24G_EIP_PORT      GPIO_PORT_0
   #define WLAN_COEX_24G_EIP_PIN       GPIO_PIN_0

   /// Output signal from device: BLE event in progress indication.
   #define WLAN_COEX_BLE_EIP_PORT      GPIO_PORT_0
   #define WLAN_COEX_BLE_EIP_PIN       GPIO_PIN_3

   /// Output signal from device: BLE priority indication.
   #define WLAN_COEX_BLE_PRIO_PORT     GPIO_PORT_0
   #define WLAN_COEX_BLE_PRIO_PIN      GPIO_PIN_2

#if defined (CFG_WLAN_COEX_DEBUG)
   /// BLE radio overruled signal pin definition.
   //  This signal goes high when the BLE radio is forced to be off due to external 2.4GHz device activity.
   #define WLAN_COEX_DEBUG_A_PORT      GPIO_PORT_0
   #define WLAN_COEX_DEBUG_A_PIN       GPIO_PIN_1

   /// External 2.4GHz device EIP handler signal pin definition.
   /// This signal indicates when an external 2.4GHz device wants to start or stop sending data.
   #define WLAN_COEX_DEBUG_B_PORT      GPIO_PORT_1
   #define WLAN_COEX_DEBUG_B_PIN       GPIO_PIN_3
#endif

#endif

// GPIO IRQ number. Interrupt is fired when 2.4GHz external device event in progress signal is activated.
#define WLAN_COEX_IRQ            4

#endif // WLAN_COEX_ENABLED
/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
 // Define SPI Configuration
    #define SPI_MS_MODE             SPI_MS_MODE_MASTER
    #define SPI_CP_MODE             SPI_CP_MODE_0
    #define SPI_WSZ                 SPI_MODE_8BIT
    #define SPI_CS                  SPI_CS_0

#if defined(__DA14531__)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
    #define SPI_EDGE_CAPTURE        SPI_MASTER_EDGE_CAPTURE
#else // (DA14585, DA14586)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
#endif


/****************************************************************************************/
/* SPI Flash configuration                                                              */
/****************************************************************************************/
#if !defined (__DA14586__)
#define SPI_FLASH_DEV_SIZE          (256 * 1024)
#endif


#if DEVELOPMENT_DEBUG
/**
 ****************************************************************************************
 * @brief   Reserves application's specific GPIOs
 * @details Used only in Development mode (#if DEVELOPMENT_DEBUG)
 *          i.e. to reserve P0_1 as Generic Purpose I/O:
 *          RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);
 ****************************************************************************************
 */
void GPIO_reservations(void);
#endif

/**
 ****************************************************************************************
 * @brief   Sets the functionality of application pads
 * @details i.e. to set P0_1 as Generic purpose Output:
 *          GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_1, OUTPUT, PID_GPIO, false);
 ****************************************************************************************
 */
void set_pad_functions(void);

/**
 ****************************************************************************************
 * @brief   Initializes application's peripherals and pins
 ****************************************************************************************
 */
void periph_init(void);


#endif // _USER_PERIPH_SETUP_H_
