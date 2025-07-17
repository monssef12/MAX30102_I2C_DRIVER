/*
 * max30102_driver.c
 *
 *  Created on: Jul 16, 2025
 *      Author: monci
 */
#include "max30102_driver.h"

HAL_StatusTypeDef ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t Register, uint8_t *pData, uint16_t Size){

	return HAL_I2C_Mem_Read(hi2c, DevAddress, Register, 1, pData, Size, HAL_MAX_DELAY);
}
