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

HAL_StatusTypeDef WriteRegister(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t Register, uint8_t *pData, uint16_t Size){

	return HAL_I2C_Mem_Write(hi2c, DevAddress, Register, 1, pData, Size, HAL_MAX_DELAY);
}


HAL_StatusTypeDef SetModeConfiguration(I2C_HandleTypeDef *hi2c, MODE_CONFIG *mode_configuration) {
    uint8_t mode = (mode_configuration->SHDN << 7) |
                   (mode_configuration->RESET << 6) |
                   (mode_configuration->MODE);
    return WriteRegister(hi2c, (MAX30102_ADDR << 1), MODE_CONFIGURATION, &mode, 1);
}

HAL_StatusTypeDef SetSpO2Configuration(I2C_HandleTypeDef *hi2c, SPO2_CONFIG *spo2_configuration) {
    uint8_t spo2 = (spo2_configuration->SPO2_ADC_RGE << 5) |
                   (spo2_configuration->SPO2_SR << 2) |
                   (spo2_configuration->LED_PW);
    return WriteRegister(hi2c, (MAX30102_ADDR << 1), SPO2_CONFIGURATION, &spo2, 1);
}

HAL_StatusTypeDef SetLEDPulseAmplitude(I2C_HandleTypeDef *hi2c, LED_PULSE *led_pulse) {
    HAL_StatusTypeDef status;

    uint8_t led1_pa = led_pulse->LED1_PA;
    status = WriteRegister(hi2c, (MAX30102_ADDR << 1), LED_PULSE_1, &led1_pa, 1);
    if (status != HAL_OK) return status;

    uint8_t led2_pa = led_pulse->LED2_PA;
    return WriteRegister(hi2c, (MAX30102_ADDR << 1), LED_PULSE_2, &led2_pa, 1);
}

HAL_StatusTypeDef SetFIFOConfiguration(I2C_HandleTypeDef *hi2c, FIFO_CONFIG *fifo_configuration) {
    uint8_t fifo_config = (fifo_configuration->SMP_AVE << 5) |
                          (fifo_configuration->FIFO_ROLLOVER_EN << 4) |
                          fifo_configuration->FIFO_A_FULL;
    return WriteRegister(hi2c, (MAX30102_ADDR << 1), FIFO_CONFIGURATION, &fifo_config, 1);
}

HAL_StatusTypeDef ResetFIFOPointers(I2C_HandleTypeDef *hi2c) {
    uint8_t reset = 0;
    HAL_StatusTypeDef status;

    status = WriteRegister(hi2c, (MAX30102_ADDR << 1), FIFO_READ_PTR, &reset, 1);
    if (status != HAL_OK) return status;

    return WriteRegister(hi2c, (MAX30102_ADDR << 1), FIFO_WRITE_PTR, &reset, 1);
}


HAL_StatusTypeDef ReadSample(I2C_HandleTypeDef *hi2c, MAX30102_DATA *Sampled_Data, MODE_CONFIG *mode_configuration){

	uint8_t READ_PTR;
	uint8_t WRITE_PTR;
	uint8_t Temp_Data = 0;
	Sampled_Data -> IR_DATA = 0;
	Sampled_Data -> RED_DATA = 0;

	ReadRegister(hi2c,  (MAX30102_ADDR<<1) | 1, FIFO_READ_PTR, &READ_PTR, 1);
	ReadRegister(hi2c,  (MAX30102_ADDR<<1) | 1, FIFO_WRITE_PTR, &WRITE_PTR, 1);

	uint8_t Unread_samples =  ( WRITE_PTR - READ_PTR ) & 0x1F;

	if (Unread_samples == 1){ // i will assume i will be reading only one sample at the time without overflow
		if(mode_configuration -> MODE == 0b010 || mode_configuration -> MODE == 0b011){ // Heart rate mode
			HAL_StatusTypeDef status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , FIFO_DATA, &Temp_Data, 1);
		    if (status != HAL_OK) return status;
			Sampled_Data -> IR_DATA = ( (uint32_t)Temp_Data  << 16 );

			status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , FIFO_DATA, &Temp_Data, 1);
		    if (status != HAL_OK) return status;
			Sampled_Data -> IR_DATA |= ( (uint32_t)Temp_Data << 8 );

			status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , FIFO_DATA, &Temp_Data, 1);
		    if (status != HAL_OK) return status;
			Sampled_Data -> IR_DATA |= Temp_Data;
		}
		if (mode_configuration -> MODE == 0b011){
			HAL_StatusTypeDef status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , FIFO_DATA, &Temp_Data, 1);
		    if (status != HAL_OK) return status;
			Sampled_Data -> RED_DATA = ( (uint32_t)Temp_Data  << 16 );

			status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , FIFO_DATA, &Temp_Data, 1);
		    if (status != HAL_OK) return status;
			Sampled_Data -> RED_DATA |= ( (uint32_t) Temp_Data << 8 );

			status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , FIFO_DATA, &Temp_Data, 1);
		    if (status != HAL_OK) return status;
			Sampled_Data -> RED_DATA |= Temp_Data;
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef EnableTemperature(I2C_HandleTypeDef *hi2c){
	uint8_t Set = 1;
	HAL_StatusTypeDef status = WriteRegister(hi2c, (MAX30102_ADDR<<1) , DIE_TEMPERATURE_CONFIG, &Set, 1); // Enable the temperature conversion
	return status;
}

HAL_StatusTypeDef ReadTemperature(I2C_HandleTypeDef *hi2c, TEMPERATURE_DATA *temperature_data){
	uint8_t Temp_Ready;
	HAL_StatusTypeDef status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , DIE_TEMPERATURE_INTEGER, &Temp_Ready, 1);

    if (status != HAL_OK) return status;
    if (Temp_Ready == 0){
    	HAL_StatusTypeDef status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , DIE_TEMPERATURE_INTEGER, &(temperature_data->TEMP_INTEGER), 1);
        if (status != HAL_OK) return status;

    	status = ReadRegister(hi2c, (MAX30102_ADDR<<1) , DIE_TEMPERATURE_FRACTION, &(temperature_data->TEMP_FRACTION), 1);
        if (status != HAL_OK) return status;
    }
    return HAL_OK;
}

// float Tem = float(TEMP_INTEGER) + 0.0625*TEMP_FRACTION in degree celisius
float GetTemperature(uint8_t TEMP_INTEGER, uint8_t TEMP_FRACTION){
	return (float)TEMP_INTEGER + 0.0625f*TEMP_FRACTION;

}
