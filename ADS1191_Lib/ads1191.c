/*
 * ads1191.c
 *
 *  Created on: Sep 14, 2024
 *      Author: mfaruk
 */

#include "ads1191.h"
/*
#define ADS1191_CS_Pin GPIO_PIN_4
#define ADS1191_CS_GPIO_Port GPIOC
#define ADS11191_DRDY_Pin GPIO_PIN_5
#define ADS11191_DRDY_GPIO_Port GPIOC
#define ADS1191_RST_Pin GPIO_PIN_0
#define ADS1191_RST_GPIO_Port GPIOB
 */
#define BIT_MASK(_MASK,_BIT) (((_BIT) & (~(_MASK))) | (_BIT))
static uint16_t cs_pin = GPIO_PIN_4 ,drdy_pin = GPIO_PIN_5,reset_pin = GPIO_PIN_0,start_pin;//TODO pins were supposed to be defined as volatile
static GPIO_TypeDef *cs_port = GPIOC,*drdy_port = GPIOC,*reset_port = GPIOB,*start_port;
static SPI_HandleTypeDef *hspi;
static IO_TYPE clk = INTERNAL;
static IO_TYPE rst_mechanims = INTERNAL;
static uint32_t status_reg = 0;
static uint16_t channel_data = 0;



static void gpio_C_Select(void);
static void gpio_C_DSelect(void);
static void gpio_reset_high(void);
static void gpio_reset_low(void);
static void gpio_start_high(void);
static void gpio_start_low(void);
static GPIO_PinState gpio_read_drdy(void);
static void ADS1191_wait_until_data_ready(void);

static void ADS1191_power_down(void);

static HAL_StatusTypeDef ADS1191_SendCommand(uint8_t cmd);
static HAL_StatusTypeDef ADS1191_Init(void);
static HAL_StatusTypeDef ADS1191_WriteRegister(uint8_t reg_adress, uint8_t reg_numbers, const uint8_t* value);
static HAL_StatusTypeDef ADS1191_ReadRegister( uint8_t reg_adress, uint8_t reg_numbers, uint8_t* value);
static HAL_StatusTypeDef ADS1191_set_RDataC(void);
static HAL_StatusTypeDef ADS1191_set_SDataC(void);
static HAL_StatusTypeDef ADS1191_ReadData( void);
static HAL_StatusTypeDef ADS1191_ReadDataC(void);
static HAL_StatusTypeDef ADS1191_StartConversion(void);
static HAL_StatusTypeDef ADS1191_Reset(void);
static void ADS1191_set_basic_data_capture(void);
static void ADS1191_set_test_signals(void);
static uint32_t ADS1191_get_status_reg(void);
static uint16_t ADS1191_get_channel_data(void);
//weak func can be over written
__weak void delay_ms(uint32_t ms)
{
#warning "you can implement your delay function"
	HAL_Delay(ms);
}

static void gpio_C_Select(void)
{
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); // Select ADS1191
}
static void gpio_C_DSelect(void)

{
	HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET); // Deselect ADS1191
}
static void gpio_reset_high(void)
{
	HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_SET);
}
static void gpio_reset_low(void)
{
	HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_RESET);
}
static void gpio_start_high(void)
{
	HAL_GPIO_WritePin(start_port, start_pin, GPIO_PIN_SET);
}
static void gpio_start_low(void)
{
	HAL_GPIO_WritePin(start_port, start_pin, GPIO_PIN_RESET);
}

static GPIO_PinState gpio_read_drdy(void)
{
	volatile GPIO_PinState pin_state;
	pin_state = HAL_GPIO_ReadPin(drdy_port, drdy_pin);
	return pin_state;
}
static void ADS1191_wait_until_data_ready(void)
{
	while(gpio_read_drdy() == GPIO_PIN_SET)
	{

	}
}
static HAL_StatusTypeDef ADS1191_Reset(void)
{
	HAL_StatusTypeDef status;
	if(rst_mechanims == EXTERNAL)
	{
		gpio_reset_low();
		delay_ms(1);
		gpio_reset_high();
		delay_ms(1);
		status = HAL_OK;
	}else if(rst_mechanims == INTERNAL)
	{
		status = ADS1191_SendCommand(ADS1191_CMD_RESET);
		delay_ms(1);
	}else{
		status = HAL_ERROR;
	}
	return status;
}
static void ADS1191_power_down(void)
{
	gpio_reset_low();
	delay_ms(100);
}
static HAL_StatusTypeDef ADS1191_Init(void) {

	HAL_StatusTypeDef status;
	// RST ADS1191 as HW. POWER-UP SEQUENCING, page 55
	status = ADS1191_Reset();
	if(status != HAL_OK)
	{
		return status;
	}
	// Wait for device to stabilize after reset
	HAL_Delay(5);

	 // Send the WAKEUP command to bring the ADS1191 out of standby mode
	status = ADS1191_SendCommand(ADS1191_CMD_WAKEUP);
	if(status != HAL_OK)
	{
		return status;
	}
    // Send the START command to begin continuous data acquisition

    status = ADS1191_SendCommand(ADS1191_CMD_START);

    return status;
}
static HAL_StatusTypeDef ADS1191_SendCommand(uint8_t cmd) {
	HAL_StatusTypeDef status;
	gpio_C_Select();
	status = HAL_SPI_Transmit(hspi, &cmd, 1, SPI_MAX_TIMEOUT);
	gpio_C_DSelect();

	return status;
}
//review : const correctness on value
static HAL_StatusTypeDef ADS1191_WriteRegister(uint8_t reg_adress, uint8_t reg_numbers, const uint8_t* value) {
	HAL_StatusTypeDef status;
    uint8_t opcode[2];
    if((reg_adress <= MAX_REG_LEN) && (reg_numbers <= MAX_REG_LEN))
    {
        opcode[0] = (ADS1191_OPC_WREG | reg_adress);  // Write command + register address
        opcode[1] = (reg_numbers - (uint8_t)(0x01));// The second byte of the opcode specifies the number of registers to write – 1.

        gpio_C_Select(); // Select ADS1191
        status = HAL_SPI_Transmit(hspi, opcode, sizeof(opcode), SPI_MAX_TIMEOUT);
        if(status == HAL_OK)
        {
        	status = HAL_SPI_Transmit(hspi, value, (uint16_t)reg_numbers, SPI_MAX_TIMEOUT);
        }
        gpio_C_DSelect();   // Deselect ADS1191
    }else
    {
    	status = HAL_ERROR; //TODO: can be printed out a message.
    }
    return status;
}

static HAL_StatusTypeDef ADS1191_ReadRegister( uint8_t reg_adress, uint8_t reg_numbers, uint8_t* value) {
	HAL_StatusTypeDef status;
    uint8_t opcode[2];
    if((reg_adress <= MAX_REG_LEN) && (reg_numbers <= MAX_REG_LEN))
    {
        opcode[0] = (ADS1191_OPC_RREG | reg_adress);  // Write command + register address
        opcode[1] = (reg_numbers - (uint8_t)(0x01));// The second byte of the opcode specifies the number of registers to write – 1.

        gpio_C_Select(); // Select ADS1191
        status = HAL_SPI_Transmit(hspi, opcode, sizeof(opcode), SPI_MAX_TIMEOUT);
        if(status == HAL_OK)
        {
        	status = HAL_SPI_Receive(hspi, value, (uint16_t)reg_numbers, SPI_MAX_TIMEOUT);
        }
        gpio_C_DSelect();   // Deselect ADS1191
    }else
    {
    	status = HAL_ERROR; //TODO: can be printed out a message.
    }
    return status;
}

static HAL_StatusTypeDef ADS1191_set_SDataC(void)
{
	HAL_StatusTypeDef status;
	gpio_C_Select(); // Select ADS1191
	status = ADS1191_SendCommand(ADS1191_CMD_SDATAC);
	gpio_C_DSelect();   // Deselect ADS1191


	return status;
}
static HAL_StatusTypeDef ADS1191_set_RDataC(void)
{
	HAL_StatusTypeDef status;
	gpio_C_Select(); // Select ADS1191
	status = ADS1191_SendCommand(ADS1191_CMD_RDATAC);
	gpio_C_DSelect();   // Deselect ADS1191

	return status;
}
static HAL_StatusTypeDef ADS1191_ReadData(void) {
	HAL_StatusTypeDef status;
	uint8_t data[5];

	ADS1191_wait_until_data_ready();
	gpio_C_Select(); // Select ADS1191
	ADS1191_SendCommand(ADS1191_CMD_RDATA);
    status = HAL_SPI_Receive(hspi, data, 5, SPI_MAX_TIMEOUT);
    gpio_C_DSelect();   // Deselect ADS1191
    status_reg = ((uint32_t)data[0] << 16 ) | ((uint32_t)data[1] << 8 ) | (uint32_t)data[2];//TODO error 2
    channel_data = ((uint16_t)data[3] << 8 ) | (uint16_t)data[4];

    return status;
}
static HAL_StatusTypeDef ADS1191_ReadDataC(void) {
	HAL_StatusTypeDef status;
	uint8_t data[5];

	ADS1191_wait_until_data_ready();
	gpio_C_Select(); // Select ADS1191
    status = HAL_SPI_Receive(hspi, data, 5, SPI_MAX_TIMEOUT);
    gpio_C_DSelect();   // Deselect ADS1191
    status_reg = ((uint32_t)data[0] << 16 ) | ((uint32_t)data[1] << 8 ) | (uint32_t)data[2];
    channel_data = ((uint16_t)data[3] << 8 ) | (uint16_t)data[4];

    return status;
}
static HAL_StatusTypeDef ADS1191_StartConversion(void) {
	HAL_StatusTypeDef status;
    uint8_t cmd = ADS1191_CMD_START;
    gpio_C_Select(); // Select ADS1191
    status = HAL_SPI_Transmit(hspi, &cmd, 1, SPI_MAX_TIMEOUT);
    gpio_C_DSelect();   // Deselect ADS1191

    return status;
}


static void ADS1191_set_basic_data_capture(void)
{
	ADS1191_Reset();
	delay_ms(1000);	// Wait for 1 s for Power-On Reset
	ADS1191_SendCommand(ADS1191_CMD_SDATAC);
	uint8_t value = ADS1191_CONFIG2_PDB_REFBUF;
	if(clk == INTERNAL)
	{
		ADS1191_WriteRegister(ADS1191_REG_CONFIG2, 1,  &value);//// If Using Internal Reference, Send This Command -- WREG CONFIG2 A0h
		delay_ms(1);
	}
	value = BIT_MASK(ADS1191_CONFIG1_DR_MASK,ADS1191_CONFIG1_DR_500SPS);//(ADS1191_CONFIG1_DR_500SPS & ~ADS1191_CONFIG1_DR_MASK) | ADS1191_CONFIG1_DR_MASK; //
	ADS1191_WriteRegister(ADS1191_REG_CONFIG1, 1,  &value);
	value = BIT_MASK(ADS1191_CHSET_MUX_MASK,ADS1191_CHSET_MUX_SHORT);//
	ADS1191_WriteRegister(ADS1191_REG_CH1SET, 1,  &value);
	ADS1191_StartConversion();
	delay_ms(1);
	ADS1191_set_RDataC();
}

static void ADS1191_set_test_signals(void)
{
	ADS1191_SendCommand(ADS1191_CMD_SDATAC);
	uint8_t value = 0xA3;
	ADS1191_WriteRegister(ADS1191_REG_CONFIG2, 1, &value);
	value = 0x05;
	ADS1191_WriteRegister(ADS1191_REG_CH1SET, 1, &value);
}
static uint16_t ADS1191_get_channel_data(void)
{
	return channel_data;
}
static uint32_t ADS1191_get_status_reg(void)
{
	return status_reg;
}
void ADS1191_Ctor(ADS1191_HandleTypeDef* ads1191)
{
	hspi 						= ads1191->spi;

	rst_mechanims 				= ads1191->gpio.rst_mechanims;
	clk							= ads1191->gpio.clk;
	cs_pin 						= ads1191->gpio.cs_pin;
	cs_port 					= ads1191->gpio.cs_port;
	drdy_pin 					= ads1191->gpio.drdy_pin;
	drdy_port 					= ads1191->gpio.drdy_port;
	reset_pin 					= ads1191->gpio.rst_pin;
	reset_port 					= ads1191->gpio.rst_port;
	start_pin 					= ads1191->gpio.start_pin;
	start_port 					= ads1191->gpio.start_port;

	ads1191->set_basic_data_capture	= ADS1191_set_basic_data_capture;
	ads1191->power_down				= ADS1191_power_down;
	ads1191->wait_until_data_ready 	= ADS1191_wait_until_data_ready;
	ads1191->set_test_signals 		= ADS1191_set_test_signals;
	ads1191->read_drdy				= gpio_read_drdy;
	ads1191->init 					= ADS1191_Init;
	ads1191->WriteRegister 			= ADS1191_WriteRegister;
	ads1191->ReadRegister 			= ADS1191_ReadRegister;
	ads1191->Reset 					= ADS1191_Reset;
	ads1191->set_SDataC				= ADS1191_set_SDataC;
	ads1191->set_RDataC				= ADS1191_set_RDataC;
	ads1191->ReadData 				= ADS1191_ReadData;
	ads1191->ReadDataC				= ADS1191_ReadDataC;
	ads1191->StartConversion 		= ADS1191_StartConversion;
	ads1191->get_status_reg			= ADS1191_get_status_reg;
	ads1191->get_channel_data		= ADS1191_get_channel_data;
}
