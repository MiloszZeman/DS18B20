/*
 * ds18b20.h
 *
 *  Created on: 10 May 2022
 *      Author: Milosz
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include <main.h>
#include <stm32f1xx_hal.h> // Change this file for appropriate HAl header for your board version

// Choose your maximum expected number of devices
#define MAX_DEVICES_ON_BUS 10

// Pin and port for  1-wire communication - insert user label of pin before the ##
// Remember that the pin needs to be configured as Open-Drain
#define DQ	DS18B20 ## _Pin
#define DQ_Port	DS18B20 ## _GPIO_Port


// Configure a timer of your choice so that 1 tick takes 1us
extern TIM_HandleTypeDef htim4;

typedef enum ONE_WIRE_SupplyTypeDef{
	PARASITE_SUPPLY = 0U,
	BATTERY_SUPPLY = 1U
} ONE_WIRE_SupplyTypeDef;

typedef enum ONE_WIRE_StatusTypeDef{
	ONE_WIRE_OK = 0U,
	ONE_WIRE_INIT_ERROR = 1U
} ONE_WIRE_StatusTypeDef;

typedef struct DS18B20_Inst{
	uint8_t ROM[8];
	uint16_t temperature;
	uint8_t Th;
	uint8_t Tl;
	uint8_t config;
} DS18B20_Inst;

extern DS18B20_Inst _one_wire_bus[MAX_DEVICES_ON_BUS];

inline uint8_t _get_bit(uint8_t byte, uint8_t bit_number) { return (byte >> bit_number) & 1;};

DS18B20_Inst* identify_devices(DS18B20_Inst* one_wire_bus);

ONE_WIRE_StatusTypeDef _reset_pulse(void);

void _delay_us(uint32_t us);

ONE_WIRE_StatusTypeDef _initialize_comm(void);

void _write_bit(uint8_t bit);

void _write_byte(uint8_t bit);

uint8_t _read_bit(void);

uint8_t _read_byte(void);

uint8_t _update_CRC(uint8_t* actual_crc, uint8_t input_byte);

inline void _send_ROM(DS18B20_Inst* device){ for(uint8_t i = 0; i < 7; i++) _write_byte((device->ROM)[i]);}



#endif /* INC_DS18B20_H_ */
