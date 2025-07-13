/*
 * Copyright 2025 NXP
 * 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <FreeRTOS.h>
#include <task.h>

#include "common/code_utils.hpp"

#include <openthread/cli.h>

#include "addons_cli.h"
#include "br_rtos_manager.h"

#include "pin_mux.h"
#include "fsl_common.h"
#include "fsl_io_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_i2c.h"
#include "PERI_I2C.h"

#include "fsl_p3t1755.h"

#include "Temp_sensor.h"

#define EXAMPLE_I2C_MASTER_BASE		I2C2
#define I2C_MASTER_CLOCK_FREQUENCY CLOCK_GetFlexCommClkFreq(2U)
#define I2C_TIME_OUT_INDEX 100000000U
#define SENSOR_SLAVE_ADDR          0x48U
#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)

#define CCC_RSTDAA  0x06U
#define CCC_SETDASA 0x87U

#define I2C_BAUDRATE 100000U

#define I2C_MASTER_SLAVE_ADDR 	0x7EU
#define I2C_DATA_LENGTH       	32U

volatile status_t g_completionStatus;
volatile bool g_masterCompletionFlag = false;
i2c_master_handle_t g_m_handle;
p3t1755_handle_t p3t1755Handle;

double temperature;

void I2C2_InitPins(void)
{
   /* Initialize FC2_I2C_16_17 functionality on pin GPIO_16, GPIO_17 (pin C6_E7) */
   IO_MUX_SetPinMux(IO_MUX_FC2_I2C_16_17);
}


static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_masterCompletionFlag = true;
    }
}


void Temp_Sensor_task(void *arg)
{
	
	status_t result = kStatus_Success;

	while (1)
	{
		/* Read Temperature */
		result = P3T1755_ReadTemperature(&p3t1755Handle, &temperature);
		if (result != kStatus_Success)
		{
			otCliOutputFormat("\r\nP3T1755 read temperature failed.\r\n");
		}
		else
		{
			//otCliOutputFormat("Temperature: %f \r", temperature);
		}

		vTaskDelay(1000);
	}
}

double Get_Temperature (void)
{
    return temperature;
}


uint32_t I2C2_GetFreq(void)
{
    return CLOCK_GetFlexCommClkFreq(2U);
}


void mem_cpy(uint8_t *source, uint8_t *dest, uint16_t len)
{
	uint32_t index;
	for(index = 0; index < len; index ++)
	{
		dest[index] = source[index];
	}
}

status_t I2C_WriteSensor(uint8_t deviceAddress, uint32_t regAddress, uint8_t *regData, size_t dataSize)
{
    status_t result                  = kStatus_Success;
    i2c_master_transfer_t masterXfer = {0};
    uint32_t timeout                 = 0U;

    masterXfer.slaveAddress   = deviceAddress;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = regAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = regData;
    masterXfer.dataSize       = dataSize;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    g_masterCompletionFlag = false;
    g_completionStatus     = kStatus_Success;
    result                 = I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }

    while (!g_masterCompletionFlag)
    {
        timeout++;
        if ((g_completionStatus != kStatus_Success) || (timeout > I2C_TIME_OUT_INDEX))
        {
            break;
        }
    }

    if (timeout == I2C_TIME_OUT_INDEX)
    {
        result = kStatus_Timeout;
    }
    result = g_completionStatus;

    return result;
}

status_t I2C_ReadSensor(uint8_t deviceAddress, uint32_t regAddress, uint8_t *regData, size_t dataSize)
{
    status_t result                  = kStatus_Success;
    i2c_master_transfer_t masterXfer = {0};
    uint32_t timeout                 = 0U;

    masterXfer.slaveAddress   = deviceAddress;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = regAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = regData;
    masterXfer.dataSize       = dataSize;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    g_masterCompletionFlag = false;
    g_completionStatus     = kStatus_Success;
    result                 = I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }

    while (!g_masterCompletionFlag)
    {
        timeout++;
        if ((g_completionStatus != kStatus_Success) || (timeout > I2C_TIME_OUT_INDEX))
        {
            break;
        }
    }

    if (timeout == I2C_TIME_OUT_INDEX)
    {
        result = kStatus_Timeout;
    }
    result = g_completionStatus;

    return result;
}


void Temp_Sensor_start(void)
{
    status_t result = kStatus_Success;

    i2c_master_config_t masterConfig;
	p3t1755_config_t p3t1755Config;

    /* Use 16 MHz clock for the FLEXCOMM2 */
	CLOCK_AttachClk(kSFRO_to_FLEXCOMM2);

    /*Init I2C*/
    I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = I2C_BAUDRATE;
	I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);

    /* Create the I2C handle for the non-blocking transfer */
    I2C_MasterTransferCreateHandle(EXAMPLE_I2C_MASTER, &g_m_handle, i2c_master_callback, NULL);
	

	/* P3T1755 Temperature Sensor Config */
	p3t1755Config.writeTransfer = I2C_WriteSensor;
	p3t1755Config.readTransfer  = I2C_ReadSensor;
	p3t1755Config.sensorAddress = SENSOR_SLAVE_ADDR;
	result = P3T1755_Init(&p3t1755Handle, &p3t1755Config);
	if (result != kStatus_Success){
		otCliOutputFormat("\r\n Temperature Sensor Init Failed.\r\n");
	} 
    
    xTaskCreate(Temp_Sensor_task, "temperature_sensor_task", configMINIMAL_STACK_SIZE * 8, NULL, configMAX_PRIORITIES - 5, NULL);
}


