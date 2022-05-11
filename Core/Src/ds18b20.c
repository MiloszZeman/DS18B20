/*
 * ds18b20.c
 *
 *  Created on: 10 May 2022
 *      Author: Milosz
 */


#include "ds18b20.h"

DS18B20_Inst ONE_WIRE_BUS[MAX_DEVICES_ON_BUS];
DS18B20_Inst* p_ONE_WIRE_BUS = ONE_WIRE_BUS;

void delay_us(uint32_t us){
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while(__HAL_TIM_GET_COUNTER(&htim4) < us);
}

ONE_WIRE_StatusTypeDef initialize_comm(void){
	HAL_GPIO_WritePin(DQ_Port, DQ, GPIO_PIN_RESET);
	delay_us(500);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
	delay_us(70);
	if (HAL_GPIO_ReadPin(DQ_Port, DQ)) return ONE_WIRE_INIT_ERROR;
	delay_us(240);
	if (!HAL_GPIO_ReadPin(DQ_Port, DQ)) return ONE_WIRE_INIT_ERROR;
	delay_us(190);
	return ONE_WIRE_OK;
}

void write_bit(uint8_t bit){
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
	if(bit == 0){
		delay_us(90);
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);}
	else {
		delay_us(3);
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
		delay_us(87);}
	delay_us(15);
}

void write_byte(uint8_t byte){
	for(uint8_t i = 0; i < 8; i++){
		uint8_t bit = 0x01 & (byte >> i);
		write_bit(bit);}
}

uint8_t read_bit(void){
	uint8_t bit = 0;
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
	delay_us(5);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
	delay_us(20);
	if(HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin)) bit = 1;
	delay_us(38);
	return bit;
}

uint8_t read_byte(void){
	uint8_t received = 0;
	uint8_t bit = 0;
	for(uint8_t i = 0; i < 8; i++){
		received |= (bit << i);
	}

	return received;
}
