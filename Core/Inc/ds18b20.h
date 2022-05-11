/*
 * ds18b20.h
 *
 *  Created on: 10 May 2022
 *      Author: Milosz
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include <stm32f1xx_hal.h>
#include <main.h>

#define MAX_DEVICES_ON_BUS 64

// Pin and port for  1-wire communication - insert user label of pin before the ##
#define DQ	DS18B20 ## _Pin
#define DQ_Port	DS18B20 ## _GPIO_Port

// Configure a timer of your choice so that 1 tick takes 1us
extern TIM_HandleTypeDef htim4;

typedef enum ONE_WIRE_SupplyTypeDef{
	PARASITE_SUPPLY = 0U,
	MASTER_SUPPLY = 1U
} ONE_WIRE_SupplyTypeDef;

typedef enum ONE_WIRE_StatusTypeDef{
	ONE_WIRE_OK = 0U,
	ONE_WIRE_INIT_ERROR = 1U
} ONE_WIRE_StatusTypeDef;

typedef struct DS18B20_Inst{
	uint8_t ROM[8];
	uint16_t temperature;
	uint16_t precision;
	uint16_t Th;
	uint16_t Tl;
	ONE_WIRE_SupplyTypeDef supply;
} DS18B20_Inst;

void delay_us(uint32_t us);

ONE_WIRE_StatusTypeDef initialize_comm(void);

void write_bit(uint8_t bit);

void write_byte(uint8_t bit);

uint8_t read_bit(void);

uint8_t read_byte(void);

#endif /* INC_DS18B20_H_ */
