/**
 ****************************************************************************************
 *
 * @file da1458x_config_basic.h
 *
 * @brief Basic compile configuration file.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _DA1458X_CONFIG_BASIC_H_
#define _DA1458X_CONFIG_BASIC_H_

#if defined (__DA14531__)

/****************************************************************************************************************/
/* Notify the SDK about the fixed power mode (currently used only for Bypass):                                  */
/*     - CFG_POWER_MODE_BYPASS = Bypass mode                                                                    */
/****************************************************************************************************************/
#undef CFG_POWER_MODE_BYPASS

/****************************************************************************************************************/
/* Max buffer size used for the communication with the external tool over UART                                  */
/****************************************************************************************************************/
#define ALLOWED_DATA_UART       (0x08000)

/****************************************************************************************************************/
/* Max buffer size used for the communication with the external tool over SWD                                   */
/****************************************************************************************************************/
#define ALLOWED_DATA_SWD        (0x04000)

#else

/****************************************************************************************************************/
/* Max buffer size used for the communication with the external tool over UART                                  */
/****************************************************************************************************************/
#define ALLOWED_DATA_UART       (0x0FFFF)

/****************************************************************************************************************/
/* Max buffer size used for the communication with the external tool over SWD                                   */
/****************************************************************************************************************/
#define ALLOWED_DATA_SWD        (0x10000)

#endif

#endif // _DA1458X_CONFIG_BASIC_H_
