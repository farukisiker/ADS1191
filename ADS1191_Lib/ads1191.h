/*
 * ads1191.h
 *
 *  Created on: Sep 14, 2024
 *      Author: mfaruk
 */

#ifndef ADS1191_H
#define ADS1191_H

#include "stm32f4xx_hal.h"
#include "ads1191_regs.h"
// Define SPI TIMEOUT in ms
#define SPI_MAX_TIMEOUT 100



typedef enum IO_TYPE{
	EXTERNAL = 0,
	INTERNAL = 1
}IO_TYPE;


typedef struct __ADS1191_PINS_HandleTypeDef{
	GPIO_TypeDef *cs_port,*drdy_port,*rst_port,*start_port;
	uint16_t	cs_pin,drdy_pin,rst_pin,start_pin;
	IO_TYPE 	clk;
	IO_TYPE 	rst_mechanims;
}ADS1191_PINS_HandleTypeDef;

typedef struct __ADS1191_HandleTypeDef{

	SPI_HandleTypeDef* spi;
	ADS1191_PINS_HandleTypeDef gpio;
	void (*power_down)(void);
	void (*wait_until_data_ready)(void);
	void (*set_test_signals)(void);
	void (*set_basic_data_capture)(void);
	GPIO_PinState (*read_drdy)(void);
	HAL_StatusTypeDef (*init)(void);
	HAL_StatusTypeDef (*WriteRegister)( uint8_t reg_adress, uint8_t reg_numbers, const uint8_t* value);
	HAL_StatusTypeDef (*ReadRegister)( uint8_t reg_adress, uint8_t reg_numbers, uint8_t* value);
	HAL_StatusTypeDef (*set_RDataC)(void);
	HAL_StatusTypeDef (*set_SDataC)(void);
	HAL_StatusTypeDef (*ReadData)( void);
	HAL_StatusTypeDef (*ReadDataC)( void);
	HAL_StatusTypeDef (*StartConversion)(void);
	HAL_StatusTypeDef (*Reset)(void);
	uint32_t (*get_status_reg)(void);
	uint16_t (*get_channel_data)(void);
}ADS1191_HandleTypeDef;

// Function prototypes
void ADS1191_Ctor(ADS1191_HandleTypeDef* ads1191);
void delay_ms(uint32_t ms);
#endif // ADS1191_H
