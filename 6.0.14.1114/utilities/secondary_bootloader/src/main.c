/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief Secondary Bootloader application
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include "arch_system.h"
#include "datasheet.h"
#include "syscntl.h"
#include "user_periph_setup.h"
#include "uart_booter.h"
#include "i2c_eeprom.h"
#include "spi_flash.h"
#include "bootloader.h"
#include "gpio.h"
#include "uart.h"
#include "hw_otpc.h"

#if defined (__DA14531__)
#include "adc.h"
#endif

#if defined (__DA14531__)
/**
 ****************************************************************************************
 * @brief Enable DA14531 HW reset functionality.
 ****************************************************************************************
 */
static void enable_hw_reset(void)
{
    GPIO_DisablePorPin();
    GPIO_SetPinFunction(GPIO_PORT_0, GPIO_PIN_0, INPUT_PULLDOWN, PID_GPIO);
    // Wait 10us for the line to be discharged
    for (int i = 0; i < 4 * 10; i++);
    GPIO_Enable_HW_Reset();
}
#endif

/**
 ****************************************************************************************
 * @brief Run user application after receiving a binany from uart or reading the
 *        binary from a non-volatile bootable memory
 ****************************************************************************************
 */
static void Start_run_user_application(void)
{
    volatile unsigned short tmp;

    if (*(volatile unsigned*)0x07FC0000 & 0x07FC0000)
    {
        tmp = GetWord16(SYS_CTRL_REG);
        //Remap to SysRAM and trigger  SW Reset
        tmp = (tmp & ~REMAP_ADR0) | 0x2;
        tmp |= SW_RESET;
        SetWord16(SYS_CTRL_REG, tmp);
    }
    tmp = GetWord16(SYS_CTRL_REG);
    //  tmp&=~0x0003;
    tmp |= 0x8000;
    SetWord16(SYS_CTRL_REG,tmp);
}

extern int FwDownload(void);
extern void uart_receive_callback(uint16_t length);
extern void uart_error_callback(uart_t *uart, uint8_t uart_err_status);

#ifdef UART_SUPPORTED
static const uart_cfg_t uart_cfg =
            {
                .baud_rate = UART_FRAC_BAUDRATE,
                .data_bits = UART_DATABITS_8,
                .parity = UART_PARITY_NONE,
                .stop_bits = UART_STOPBITS_1,
                .auto_flow_control = UART_AFCE_DIS,
                .use_fifo = UART_FIFO_EN,
                .tx_fifo_tr_lvl = UART_TX_FIFO_LEVEL_0,
                .rx_fifo_tr_lvl = UART_RX_FIFO_LEVEL_0,
                .intr_priority = 2,
                .uart_rx_cb = uart_receive_callback,
                .uart_err_cb = uart_error_callback,
            };
#endif

int main(void)
{
#if defined(SPI_FLASH_SUPPORTED ) || defined(I2C_EEPROM_SUPPORTED)
    int ret = -1;
#endif

#ifdef UART_SUPPORTED
    int fw_size;
    unsigned int i;
    char *from ;
    char *to;
#endif

    /*
     ************************************************************************************
     * Watchdog start (maximum value) - PLEASE DISABLE WATCHDOG WHILE TESTING!
     ************************************************************************************
     */
    SetBits(WATCHDOG_CTRL_REG, NMI_RST, 1);    // WDOG will generate HW reset upon expiration
    SetWord16(WATCHDOG_REG, 0xFF);             // Reset WDOG
    SetWord16(RESET_FREEZE_REG, FRZ_WDOG);     // Start WDOG

    system_init();

    while (1)
    {
#ifdef UART_SUPPORTED
        if (GPIO_GetPinStatus(UART_GPIO_PORT, UART_RX_PIN))
        {
            uart_initialize(UART1, &uart_cfg);

            // Initialize UART pins
            GPIO_ConfigurePin(UART_GPIO_PORT, UART_TX_PIN, OUTPUT, PID_UART1_TX, false);
            GPIO_ConfigurePin(UART_GPIO_PORT, UART_RX_PIN, INPUT, PID_UART1_RX, false);

            // Download FW
            fw_size = FwDownload();

            uart_disable(UART1);

            if (fw_size > 0)
            {
                from = (char*) (SYSRAM_COPY_BASE_ADDRESS);
                to = (char*) (SYSRAM_BASE_ADDRESS);
                for (i = 0; i < fw_size; i++)
                {
                    to[i] = from[i];
                }

                SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
                SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG

                // Reset UART pins
                GPIO_ConfigurePin(UART_GPIO_PORT, UART_TX_PIN, INPUT, PID_GPIO, false);
                GPIO_ConfigurePin(UART_GPIO_PORT, UART_RX_PIN, INPUT, PID_GPIO, false);

                #if defined (__DA14531__)
                    enable_hw_reset();
                #endif

                Start_run_user_application();
            }
        }
#endif

#ifdef SPI_FLASH_SUPPORTED
        ret = spi_loadActiveImage();
        if (!ret)
        {
            SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
            SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG

            spi_release();

            // reset SPI GPIOs
            GPIO_ConfigurePin(SPI_EN_PORT, SPI_EN_PIN, INPUT_PULLUP, PID_GPIO, true);
            GPIO_ConfigurePin(SPI_CLK_PORT, SPI_CLK_PIN, INPUT_PULLDOWN, PID_GPIO, false);
            GPIO_ConfigurePin(SPI_DO_PORT, SPI_DO_PIN, INPUT_PULLDOWN, PID_GPIO, false);
            GPIO_ConfigurePin(SPI_DI_PORT, SPI_DI_PIN, INPUT_PULLDOWN, PID_GPIO, false);

            #if defined (__DA14531__)
                enable_hw_reset();
            #endif

            Start_run_user_application();
        }
#endif

#ifdef I2C_EEPROM_SUPPORTED
        ret = eeprom_loadActiveImage();
        if (!ret)
        {
            SetWord16(WATCHDOG_REG, 0xC8);          // 200 * 10.24ms active time for initialization!
            SetWord16(RESET_FREEZE_REG, FRZ_WDOG);  // Start WDOG

            i2c_eeprom_release();

            // reset I2C GPIOs
            GPIO_ConfigurePin(I2C_SCL_PORT, I2C_SCL_PIN, INPUT_PULLDOWN, PID_GPIO, false);
            GPIO_ConfigurePin(I2C_SDA_PORT, I2C_SDA_PIN, INPUT_PULLDOWN, PID_GPIO, false);

            #if defined (__DA14531__)
                enable_hw_reset();
            #endif

            Start_run_user_application();
        }
#endif

    } // while (1)
}

