/*
 * max30102_driver.h
 *
 *  Created on: Jul 16, 2025
 *      Author: monci
 */
#ifndef INC_MAX30102_DRIVER_H_
#define INC_MAX30102_DRIVER_H_
/*
 * The sensor set of register + the device I2C adress
 *
 */
#include "stm32f4xx_hal.h"

#define MAX30102_ADDR   0b01010111  /*The MAX30102 slave adresse*/

/*STATUS REGISTERS*/
#define INTERRUPT_STATUS_1    0x00  /*The Interrupt Status 1 register address*/
#define INTERRUPT_STATUS_2    0x01  /*The Interrupt Status 2 register address*/
#define INTERRUPT_ENABLE_1    0x02  /*The Interrupt Enable 1 register address*/
#define INTERRUPT_ENABLE_2    0x03  /*The Interrupt Enable 2 register address*/

#define FIFO_WRITE_PTR        0x04    /*The FIFO Write Pointer register address*/
#define OVERFLOW_COUNTER      0x05    /*The Overflow Counter register address*/
#define FIFO_READ_PTR         0x06    /*The FIFO Read Pointer register address*/
#define FIFO_DATA             0x07    /*The  FIFO Data register address*/

/*CONFIGURATION REGISTERS*/
#define FIFO_CONFIGURATION        0X08    /*The FIFO CONFIGURATION register address*/
#define MODE_CONFIGURATION        0X09    /*The MODE CONFIGURATION register address*/
#define SPO2_CONFIGURATION        0X0A    /*The SpO2 Configuration register address*/
#define LED_PULSE_AMPLITUDE1      0X0C    /*The LED Pulse Amplitude register address*/
#define LED_PULSE_AMPLITUDE2      0X0D    /*The LED Pulse Amplitude register address*/
#define MULTI_LED_MODE_CONTROL    0X11    /*The Multi-LED Mode Control register address*/
#define MULTI_LED_MODE_CONTROL    0X11    /*The Multi-LED Mode Control register address*/

/*DIE TEMPERATURE*/
#define DIE_TEMPERATURE_INTEGER    0X1F    /*The DIE TEMPERATURE INTEGER register address*/
#define DIE_TEMPERATURE_FRACTION   0X20    /*The DIE TEMPERATURE FRACTION register address*/
#define DIE_TEMPERATURE_CONFIG     0X20    /*The DIE TEMPERATURE CONFIG register address*/

/*READ AND WRITE FUNCTIONS*/
HAL_StatusTypeDef ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t Register, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData);

#endif /* INC_MAX30102_DRIVER_H_ */

