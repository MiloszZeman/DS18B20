/*
 * ds18b20.c
 *
 *  Created on: 10 May 2022
 *      Author: Milosz
 */


#include "ds18b20.h"

DS18B20_Inst _one_wire_bus[MAX_DEVICES_ON_BUS];

extern inline uint8_t _get_bit(uint8_t byte, uint8_t bit_number);
extern inline void _send_ROM(DS18B20_Inst* device);

void _delay_us(uint32_t us){
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while(__HAL_TIM_GET_COUNTER(&htim4) < us);
}

// Returns pointer to the last identified device on the bus
DS18B20_Inst* identify_devices(DS18B20_Inst* one_wire_bus){
	uint8_t done = 0;
	uint8_t last_discrepancy = 0;
	uint8_t discrepancy_marker = 0;
	uint8_t idx = 0;
	uint8_t rom_bit_idx = 0;
	uint8_t rom_byte_idx = 0;
	uint8_t bits_read;
	while(1){
		if(idx == 0){last_discrepancy = 0; done = 0;}
		if(done == 1){done = 0; return one_wire_bus;}
		if(_reset_pulse() != ONE_WIRE_OK){last_discrepancy = 0; return one_wire_bus;}
		rom_bit_idx = 0;
		discrepancy_marker = 0;
		(_one_wire_bus->ROM)[rom_bit_idx/8] = 0;
		_write_byte(0xF0);
		while(rom_bit_idx < 64){
			rom_byte_idx = rom_bit_idx/8;
			bits_read = 0;
			bits_read |= _read_bit() << 1;
			bits_read |= _read_bit();
			if(bits_read == 3){last_discrepancy = 0; return one_wire_bus;}
			if(bits_read != 0){
				(_one_wire_bus->ROM)[rom_byte_idx] |= ((bits_read >> 1) << (rom_bit_idx % 8));}
			else if (rom_bit_idx == last_discrepancy){
				_one_wire_bus->ROM[rom_byte_idx] |= 1 << (rom_bit_idx % 8);}
			else if (rom_bit_idx > last_discrepancy){
				discrepancy_marker = rom_bit_idx;}
			else if (_get_bit((_one_wire_bus->ROM)[rom_byte_idx], (rom_bit_idx % 8)) == 0){
				discrepancy_marker = rom_bit_idx;}
			_write_bit(_get_bit((_one_wire_bus->ROM)[rom_byte_idx], (rom_bit_idx % 8)));
			rom_bit_idx++;
		}
		last_discrepancy = discrepancy_marker;
		if(last_discrepancy != 0) {idx++; one_wire_bus++;}
		else return one_wire_bus;
	}
};

ONE_WIRE_StatusTypeDef _reset_pulse(void){
	HAL_GPIO_WritePin(DQ_Port, DQ, GPIO_PIN_RESET);
	_delay_us(500);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
	_delay_us(70);
	if (HAL_GPIO_ReadPin(DQ_Port, DQ)) return ONE_WIRE_INIT_ERROR;
	_delay_us(240);
	if (!HAL_GPIO_ReadPin(DQ_Port, DQ)) return ONE_WIRE_INIT_ERROR;
	_delay_us(190);
	return ONE_WIRE_OK;
}

void _write_bit(uint8_t bit){
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
	if(bit == 0){
		_delay_us(90);
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);}
	else {
		_delay_us(3);
		HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
		_delay_us(87);}
	_delay_us(15);
}

void _write_byte(uint8_t byte){
	for(uint8_t i = 0; i < 8; i++)
		_write_bit((byte >> i) & 1);
}

uint8_t _read_bit(void){
	uint8_t bit = 0;
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);
	_delay_us(5);
	HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);
	_delay_us(20);
	if(HAL_GPIO_ReadPin(DS18B20_GPIO_Port, DS18B20_Pin)) bit = 1;
	_delay_us(38);
	return bit;
}

uint8_t _read_byte(void){
	uint8_t received = 0;
	for(uint8_t i = 0; i < 8; i++)
		received |= (_read_bit() << i);
	return received;
}

uint8_t _update_CRC(uint8_t* actual_crc, uint8_t input_byte){
	uint8_t new_crc = 0;
	uint8_t input_bit = 0;
	uint8_t mask = 1;
	// Dla każdego bitu z bajtu wejściowego
	for (uint8_t i = 0; i < 8; i++){
		input_bit = (((input_byte >> i) & 1) ^ (*actual_crc & 1)); // XOR na wejściu
		// Aktualizuj shift register
		for(uint8_t j = 0; j < 8; j++){
			mask <<= 1;
			new_crc &= ~mask;  // Wyzeruj aktualnie rozważany bit
			// Funcje ustawiające shift register na podstawie datasheetu
			if (j == 2 || j == 3)
				new_crc |= ((*actual_crc & (mask << 1)) ^ (input_bit << (j+1))) >> 1;
			else if(j == 7)
				new_crc |= input_bit << 7;
			else
				new_crc |= (*actual_crc & (mask << 1)) >> 1;
		}
		*actual_crc = new_crc;
	}
	return new_crc;
}


