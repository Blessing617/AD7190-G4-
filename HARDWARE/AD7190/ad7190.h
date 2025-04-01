#ifndef _AD7190_H
#define _AD7190_H

#include "main.h"
#include "delay.h"
#include "math.h"

#define AD7190_CS_PIN 	GPIO_PIN_3
#define AD7190_SCK_PIN 	GPIO_PIN_2
#define AD7190_DIN_PIN 	GPIO_PIN_0  // DIN: STM32->AD7190
#define AD7190_DOUT_PIN GPIO_PIN_1  // DOUT: AD7190->STM32

#define AD7190_CS_GPIO 		GPIOC
#define AD7190_SCK_GPIO 	GPIOC
#define AD7190_DIN_GPIO 	GPIOC
#define AD7190_DOUT_GPIO 	GPIOC

#define AD7190_CS_CLKEN() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define AD7190_SCK_CLKEN() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define AD7190_DIN_CLKEN() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define AD7190_DOUT_CLKEN() __HAL_RCC_GPIOC_CLK_ENABLE()

#define AD7190_CS(__ST__) 	(HAL_GPIO_WritePin(AD7190_CS_GPIO, AD7190_CS_PIN, (__ST__)))
#define AD7190_SCK(__ST__) 	(HAL_GPIO_WritePin(AD7190_SCK_GPIO, AD7190_SCK_PIN, (__ST__)))
#define AD7190_DIN(__ST__) 	(HAL_GPIO_WritePin(AD7190_DIN_GPIO, AD7190_DIN_PIN, (__ST__)))
#define AD7190_DOUT() 		(HAL_GPIO_ReadPin(AD7190_DOUT_GPIO, AD7190_DOUT_PIN))
#define AD7190_Delay() 		delay_us(1)

void 			AD7190_Init(void);
uint8_t 		WaitDataRDY(void);
void 			WriteToAD7190(unsigned char count, unsigned char *buf);
void 			ReadFromAD7190(unsigned char count, unsigned char *buf);
unsigned long 	GET_AD7190(void);
void 			AD7190_Mode_Init(void);
float 			ADC_Votage(void);
uint32_t		ADC_Num(void);
extern uint8_t 	ADC_Channel;
uint8_t 		AdcGroupRead(long int adcResult[4]);

#endif
