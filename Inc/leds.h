#pragma once

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define LD_1_GROUND_PIN GPIO_PIN_15
#define LD_1_GROUND_PORT GPIOC

#define LD_2_GROUND_PIN GPIO_PIN_0
#define LD_2_GROUND_PORT GPIOA

#define LD_1_PORT	GPIOB
#define LD_1_PIN	GPIO_PIN_7

#define LD_3_PORT	GPIOB
#define LD_3_PIN	GPIO_PIN_8

#define LD_4_PORT	GPIOB
#define LD_4_PIN	GPIO_PIN_9

#define LD_8_PORT	LD_4_PORT
#define LD_8_PIN	LD_4_PIN

#define LD_9_PORT	LD_3_PORT
#define LD_9_PIN	LD_3_PIN

#define LD_10_PORT	LD_1_PORT
#define LD_10_PIN	LD_1_PIN


typedef enum 
{
	LD_1  = 1,
	LD_3  = 3,
	LD_4  = 4,

	LD_8  = 8,
	LD_9  = 9,
	LD_10 = 10
} Led_t;


void SwitchOnLD(Led_t led);
void SwitchOffLD(Led_t led);
void ToggleLD(Led_t led);


