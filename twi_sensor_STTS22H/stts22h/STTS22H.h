/**
 * Copyright (c) 2023
 * All rights reserved.
 *
 * Author - Shubham Mane
 * Date Created - 20 August 2023
 * Last date Modified - 20 August 2023
 * 
 * @brief - Contains the registers address for STTS22H
 * @file - STTS22H.h
 */
 
#ifndef __STTS22H_H__
#define __STTS22H_H__
 
/**
 * @brief - TWI Slave address
 */

#define STTS22H_TWI_ADDRESS             0x3CU

/**
 * @brief - Internal register Address
 */

#define STTS22H_WHOAMI_ADDRESS          0x01U

#define STTS22H_TEMP_H_LIMIT_ADDRESS    0x02U

#define STTS22H_TEMP_L_LIMIT_ADDRESS    0x03U

#define STTS22H_CTRL_ADDRESS            0x04U

#define STTS22H_STATUS_ADDRESS          0x05U

#define STTS22H_TEMP_L_OUT_ADDRESS      0x06U

#define STTS22H_TEMP_H_OUT_ADDRESS      0x07U
 
#endif // __STTS22H_H__
 
/* End of Code */
