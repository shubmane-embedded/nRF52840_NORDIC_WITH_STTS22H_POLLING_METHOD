/**
 * Copyright (c) 2023
 * All rights reserved.
 *
 * Author - Shubham Mane
 * Date Created - 20 August 2023
 * Last date Modified - 20 August 2023
 * 
 * @brief - example of stts22h on nRF52840
 * @file - main.c
 */

/**
 * @brief - basic header files
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/**
 * @brief - nRF header files
 */
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

/**
 * @brief - nRF log header files
 */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**
 * @brief - Sensor header files
 */
#include "STTS22H.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Count TX transaction on TWI */
static uint64_t TWI_TX_COUNT = 0;

/* Count RX transaction on TWI */
static uint64_t TWI_RX_COUNT = 0;

/* Error codes */
typedef enum
{
  SUCCESS = 0,
  FAILURE = 1
}error_code; 

/* Store the status of function */
error_code f_ret_status = FAILURE;

/* Variable to store the data in WHO_AM_I register */
uint8_t WHO_AM_I = 0;

/* Variables to store the temprature */
uint8_t temp_low = 0;
uint8_t temp_high = 0;
float total = 0;

/* Upper and lower thershold temprature */
uint8_t upper_thershold_temprature = 0;
uint8_t lower_thershold_temprature = 0;

/* Control configuration */
typedef enum
{
    one_shot = 0,
    low_odr  = 1,
    freerun_25_hz   = 2,
    freerun_50_hz   = 3,
    freerun_100_hz  = 4,
    freerun_200_hz  = 5
} control_reg_config;

/* Control configuration variable */
control_reg_config config;

/**
 * @brief TWI events handler./**
 * @brief - nRF log Initialization
 */
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
            {
                TWI_TX_COUNT++;
            }
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                TWI_RX_COUNT++;
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI Configuration 
 */
error_code twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true,
       .hold_bus_uninit    = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

    return (!err_code) ? SUCCESS : FAILURE;
}

/**
 * @brief - Write a byte over TWI
 */
error_code twi_slave_write_byte(uint8_t slave_address, uint8_t register_address, uint8_t data)
{
    ret_code_t err_code;
    uint8_t total[2] = {register_address, data};
    m_xfer_done = false;

    err_code = nrf_drv_twi_tx(&m_twi, slave_address, total, sizeof(total), false);
    APP_ERROR_CHECK(err_code);

    while(m_xfer_done == false) // Waiting for the TWI transaction to get complete
    {
        __WFE();        
    }

    return (!err_code) ? SUCCESS : FAILURE;
}

/**
 * @brief - Read a byte over TWI
 */
error_code twi_slave_read_byte(uint8_t slave_address, uint8_t register_address, uint8_t *data)
{
    ret_code_t err_code;
    m_xfer_done = false;

    err_code = nrf_drv_twi_tx(&m_twi, slave_address, &register_address, sizeof(register_address), false);
    APP_ERROR_CHECK(err_code);

    while(m_xfer_done == false) // Waiting for the TWI transaction to get complete
    {
        __WFE();        
    }

    if(err_code != NRF_SUCCESS) // If transmission was not successful, exit from the function with false as return value
        return FAILURE;

    m_xfer_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, slave_address, data, sizeof(uint8_t));
    APP_ERROR_CHECK(err_code);

    while(m_xfer_done == false) // Waiting for the TWI transaction to get complete
    {
        __WFE();        
    }

    return (!err_code) ? SUCCESS : FAILURE;
}

/**
 * @brief - Detection of the sensor
 */
error_code WHO_AM_I_STTS22H (void)
{
    if (twi_slave_read_byte(STTS22H_TWI_ADDRESS, STTS22H_WHOAMI_ADDRESS, &WHO_AM_I) == SUCCESS && WHO_AM_I == 0xA0)
    {
        NRF_LOG_INFO("TWI Slave - STTS22H - Detected successfully");
        return SUCCESS;
    }
    else
    {
        NRF_LOG_INFO("TWI Slave - STTS22H - Detection failure");
        return FAILURE;
    }
}

/**
 * @brief - Reading the upper thershold temprature
 */
error_code get_upper_thershold_temprature (uint8_t *data)
{
    uint8_t upper_temp_register;
    float temp;

    f_ret_status = twi_slave_read_byte(STTS22H_TWI_ADDRESS, STTS22H_TEMP_H_LIMIT_ADDRESS, &upper_temp_register);
    
    temp = (upper_temp_register - 63) * 0.64;
    *data = round(temp);

    return (*data != NULL) ? SUCCESS : FAILURE;
}

/**
 * @brief - Reading the upper thershold temprature
 */
error_code get_lower_thershold_temprature (uint8_t *data)
{
    uint8_t lower_temp_register;
    float temp;

    f_ret_status = twi_slave_read_byte(STTS22H_TWI_ADDRESS, STTS22H_TEMP_L_LIMIT_ADDRESS, &lower_temp_register);
    
    temp = (lower_temp_register - 63) * 0.64;
    *data = round(temp);

    return (*data != NULL) ? SUCCESS : FAILURE;
}

/**
 * @brief - Setting upper thershold for temprature
 */
void set_upper_thershold_temprature (uint8_t thershold)
{
    uint8_t temp_h_limit;

    temp_h_limit = round((thershold / 0.64) + (63));
    f_ret_status = twi_slave_write_byte(STTS22H_TWI_ADDRESS, STTS22H_TEMP_H_LIMIT_ADDRESS, temp_h_limit);
}

/**
 * @brief - Setting lower thershold for temprature
 */
void set_lower_thershold_temprature (uint8_t thershold)
{
    uint8_t temp_l_limit;

    temp_l_limit = round((thershold / 0.64) + (63));
    f_ret_status = twi_slave_write_byte(STTS22H_TWI_ADDRESS, STTS22H_TEMP_L_LIMIT_ADDRESS, temp_l_limit);
}

/**
 * @brief - Read the temprature from the sensor STTS22H
 */
void get_temprature_stts22h (void)
{
     f_ret_status = twi_slave_read_byte(STTS22H_TWI_ADDRESS, STTS22H_TEMP_L_OUT_ADDRESS, &temp_low);
     f_ret_status = twi_slave_read_byte(STTS22H_TWI_ADDRESS, STTS22H_TEMP_H_OUT_ADDRESS, &temp_high);

     total = ((temp_high << 8) | temp_low);
     NRF_LOG_INFO("Temprature = " NRF_LOG_FLOAT_MARKER " Celcius", NRF_LOG_FLOAT(total/100));
}

/**
 * @brief - nRF log Initialization
 */
void log_init (void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * @brief - main function
 */
int main(void)
{
    /* nRF log Initialization */
    log_init ();
    NRF_LOG_INFO("----Demo of STTS22H on nRF52840----");

    /* TWI Master Initialization */
    NRF_LOG_INFO("TWI Master Initialization...");
    nrf_delay_ms(2000);
    f_ret_status = twi_init();

    if (f_ret_status == SUCCESS)
    {
        NRF_LOG_INFO("TWI Master Initialization successful");
    }
    else
    {
        NRF_LOG_INFO("TWI Master Initialization Failure, Exiting...");
        exit(EXIT_FAILURE);
    }

    /* TWI Slave Detection */
    NRF_LOG_INFO("TWI Slave Detection...");
    nrf_delay_ms(2000);
    f_ret_status = WHO_AM_I_STTS22H();

    /* STTS22H sensor configuration */
    /* Step 1 - Setting the upper thershold */
    
    NRF_LOG_INFO("----Setting Upper and Lower thershold temprature for STTS22H----");

    /* Setting the thershold */
    set_upper_thershold_temprature (80); // upper_thershold
    set_lower_thershold_temprature (20); // lower_thershold

    /* Reading and printing the lower and upper thershold temparture */
    f_ret_status = get_upper_thershold_temprature (&upper_thershold_temprature);
    f_ret_status = get_lower_thershold_temprature (&lower_thershold_temprature);

    NRF_LOG_INFO("upper_thershold_temprature = %d", upper_thershold_temprature);
    NRF_LOG_INFO("lower_thershold_temprature = %d", lower_thershold_temprature);

    /* STTS22H configuration */

    /* Setting variable to respective configuration */
    config = freerun_200_hz;

    switch(config)
    {
        case one_shot:
            f_ret_status = twi_slave_write_byte(STTS22H_TWI_ADDRESS, STTS22H_CTRL_ADDRESS, 0x01);
            NRF_LOG_INFO("One shot mode is enabled");
            break; 
        case low_odr:
            f_ret_status = twi_slave_write_byte(STTS22H_TWI_ADDRESS, STTS22H_CTRL_ADDRESS, 0x80);
            NRF_LOG_INFO("Low ODR is set");
            break;
        case freerun_25_hz:
            f_ret_status = twi_slave_write_byte(STTS22H_TWI_ADDRESS, STTS22H_CTRL_ADDRESS, 0x04);
            NRF_LOG_INFO("ODR is set is 25 HZ");
            break;
        case freerun_50_hz:
            f_ret_status = twi_slave_write_byte(STTS22H_TWI_ADDRESS, STTS22H_CTRL_ADDRESS, 0x14);
            NRF_LOG_INFO("ODR is set is 50 HZ");
            break;
        case freerun_100_hz:
            f_ret_status = twi_slave_write_byte(STTS22H_TWI_ADDRESS, STTS22H_CTRL_ADDRESS, 0x24);
            NRF_LOG_INFO("ODR is set is 100 HZ");
            break;
        case freerun_200_hz:
            f_ret_status = twi_slave_write_byte(STTS22H_TWI_ADDRESS, STTS22H_CTRL_ADDRESS, 0x34);
            NRF_LOG_INFO("ODR is set is 200 HZ");
            break;
        default:/**
 * @brief - nRF log Initialization
 */
            break;
    }
    
    while(true)
    {
        get_temprature_stts22h();
        nrf_delay_ms(2000);
    }

    return 0;
}

/* End of Code */
