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
#define LED_PULSE_1               0X0C    /*The LED Pulse Amplitude register address*/
#define LED_PULSE_2               0X0D    /*The LED Pulse Amplitude register address*/
#define MULTI_LED_MODE_CONTROL    0X11    /*The Multi-LED Mode Control register address*/
#define MULTI_LED_MODE_CONTROL    0X11    /*The Multi-LED Mode Control register address*/

/*DIE TEMPERATURE*/
#define DIE_TEMPERATURE_INTEGER    0X1F    /*The DIE TEMPERATURE INTEGER register address*/
#define DIE_TEMPERATURE_FRACTION   0X20    /*The DIE TEMPERATURE FRACTION register address*/
#define DIE_TEMPERATURE_CONFIG     0X20    /*The DIE TEMPERATURE CONFIG register address*/

/*Mode Configuration structure*/
typedef struct {
	uint8_t SHDN; //  Shutdown Control
	uint8_t RESET; // Reset Control
	uint8_t MODE; //  Mode Control

} MODE_CONFIG;

/*SPO2 Configuration structure*/
typedef struct {
	uint8_t SPO2_ADC_RGE; // SpO2 ADC Range Control: maximum measured current
	uint8_t SPO2_SR; //  SpO2 Sample Rate Control
	uint8_t LED_PW; // LED Pulse Width Control and ADC Resolution

} SPO2_CONFIG;

/*FIFO Configuration structure*/
typedef struct {
	uint8_t SMP_AVE; // Sample Averaging
	uint8_t FIFO_ROLLOVER_EN; //  FIFO Rolls on Full
	uint8_t FIFO_A_FULL; // FIFO Almost Full Value

} FIFO_CONFIG;


/*LEDs Configuration structure*/
typedef struct {  // the current level of each LED
	uint8_t LED1_PA;
	uint8_t LED2_PA;
} LED_PULSE;


/*DATA structure for the sensor readings*/
typedef struct {   // Define an instance of this to store the data fetched from the sensor FIFO
	uint32_t IR_DATA;
	uint32_t RED_DATA;

} MAX30102_DATA ;



/*TEMPERATURE DATA STRUCTURE*/

/*
 * final temp data =>  float Tem = float(TEMP_INTEGER) + 0.0625*TEMP_FRACTION in degree celisius
 */
typedef struct {
	uint8_t TEMP_INTEGER;
	uint8_t TEMP_FRACTION;

} TEMPERATURE_DATA;



/*REGISTERS READ AND WRITE FUNCTIONS*/
HAL_StatusTypeDef ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t Register, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t Register, uint8_t *pData, uint16_t Size);

/*Configuration and Reading of the sensor */
HAL_StatusTypeDef SetModeConfiguration(I2C_HandleTypeDef *hi2c, MODE_CONFIG *mode_configuration);
HAL_StatusTypeDef SetSpO2Configuration(I2C_HandleTypeDef *hi2c, SPO2_CONFIG *spo2_configuration);
HAL_StatusTypeDef SetLEDPulseAmplitude(I2C_HandleTypeDef *hi2c, LED_PULSE *led_pulse);
HAL_StatusTypeDef SetFIFOConfiguration(I2C_HandleTypeDef *hi2c, FIFO_CONFIG *fifo_configuration);
HAL_StatusTypeDef ResetFIFOPointers(I2C_HandleTypeDef *hi2c);


HAL_StatusTypeDef ReadSample(I2C_HandleTypeDef *hi2c, MAX30102_DATA *Sampled_Data, MODE_CONFIG *mode_configuration);

/*The temperature ADC config and reading*/
HAL_StatusTypeDef EnableTemperature(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ReadTemperature(I2C_HandleTypeDef *hi2c, TEMPERATURE_DATA *temperature_data);

/*GetTemperature function*/

float GetTemperature(uint8_t TEMP_INTEGER, uint8_t TEMP_FRACTION);

#endif /* INC_MAX30102_DRIVER_H_ */
