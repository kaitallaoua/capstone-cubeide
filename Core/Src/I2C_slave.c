/*
 * I2C_slave.c
 *
 *  Created on: Mar 12, 2024
 *      Author: karim
 */

// https://controllerstech.com/stm32-as-i2c-slave-part-1/
#include "main.h"
#include "I2C_slave.h"

#define RxSize 4
uint8_t RxData[RxSize];
uint8_t count = 0;

// on listen callback, re-enable listen
extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

// If the device address sent by the master matches with the device address of the STM32 I2C, an interrupt will again trigger and the address callback is called.
extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
	{
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, RxData, 4, I2C_FIRST_AND_LAST_FRAME);
	}
	else  // master requesting the data is not supported yet
	{
		Error_Handler();
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	count++;
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}
