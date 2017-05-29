
#include "leds.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"



void SwitchOnLedG1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led);
void SwitchOnLedG2(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led);
void SwitchOffLed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led);
void ToggleLedG1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led);
void ToggleLedG2(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led);


uint16_t ledStatus = 0;



void SwitchOnLD(Led_t led)
{

	switch (led)
	{
	case LD_1:
		SwitchOnLedG1(LD_1_PORT, LD_1_PIN, LD_1);
		break;
	case LD_3:
		SwitchOnLedG1(LD_3_PORT, LD_3_PIN, LD_3);
		break;
	case LD_4:
		SwitchOnLedG1(LD_4_PORT, LD_4_PIN, LD_4);
		break;
	case LD_8:
		SwitchOnLedG2(LD_8_PORT, LD_8_PIN, LD_8);
		break;	
	case LD_9:
		SwitchOnLedG2(LD_9_PORT, LD_9_PIN, LD_9);
		break;	
	case LD_10:
		SwitchOnLedG2(LD_10_PORT, LD_10_PIN, LD_10);
		break;
	default:
		break;
	}
}

void SwitchOffLD(Led_t led)
{

	switch (led)
	{
	case LD_1:
		SwitchOffLed(LD_1_PORT, LD_1_PIN, LD_1);
		break;
	case LD_3:
		SwitchOffLed(LD_3_PORT, LD_3_PIN, LD_3);
		break;
	case LD_4:
		SwitchOffLed(LD_4_PORT, LD_4_PIN, LD_4);
		break;
	case LD_8:
		SwitchOffLed(LD_8_PORT, LD_8_PIN, LD_8);
		break;	
	case LD_9:
		SwitchOffLed(LD_9_PORT, LD_9_PIN, LD_9);
		break;	
	case LD_10:
		SwitchOffLed(LD_10_PORT, LD_10_PIN, LD_10);
		break;
	default:
		break;
	}
}


void ToggleLD(Led_t led)
{

	switch (led)
	{
	case LD_1:
		ToggleLedG1(LD_1_PORT, LD_1_PIN, LD_1);
		break;
	case LD_3:
		ToggleLedG1(LD_3_PORT, LD_3_PIN, LD_3);
		break;
	case LD_4:
		ToggleLedG1(LD_4_PORT, LD_4_PIN, LD_4);
		break;
	case LD_8:
		ToggleLedG2(LD_8_PORT, LD_8_PIN, LD_8);
		break;	
	case LD_9:
		ToggleLedG2(LD_9_PORT, LD_9_PIN, LD_9);
		break;	
	case LD_10:
		ToggleLedG2(LD_10_PORT, LD_10_PIN, LD_10);
		break;
	default:
		break;
	}
}



void SwitchOnLedG1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led)
{
	u_int16_t mask = 0;

	HAL_GPIO_WritePin(LD_1_GROUND_PORT, LD_1_GROUND_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	mask = 1 << ((int)led - 1);
	ledStatus = ledStatus | mask;
}

void SwitchOnLedG2(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led)
{
	u_int16_t mask = 1 << ((int)led - 1);

	HAL_GPIO_WritePin(LD_2_GROUND_PORT, LD_2_GROUND_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	ledStatus = ledStatus | mask;
}


void SwitchOffLed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led)
{
	u_int16_t mask = 1 << ((int)led - 1);
	mask = mask ^ (u_int16_t)0xFFFF;

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	ledStatus = ledStatus & mask;

	u_int16_t mask1g = 0b0000001101;
	u_int16_t mask2g = 0b1110000000;
	if ((ledStatus & mask1g) == 0)
	{
		HAL_GPIO_WritePin(LD_1_GROUND_PORT, LD_1_GROUND_PIN, GPIO_PIN_SET);
	}
	if ((ledStatus & mask2g) == 0)
	{
		HAL_GPIO_WritePin(LD_2_GROUND_PORT, LD_2_GROUND_PIN, GPIO_PIN_SET);
	}
}


void ToggleLedG1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led)
{
	u_int16_t mask = 1 << ((int)led - 1);
	u_int16_t result = ledStatus & mask;
	if (result == mask)
	{
		SwitchOffLed(GPIOx, GPIO_Pin, led);
	}
	else
	{
		SwitchOnLedG1(GPIOx, GPIO_Pin, led);
	}
}

void ToggleLedG2(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, Led_t led)
{
	u_int16_t mask = 1 << ((int)led - 1);
	u_int16_t result = ledStatus & mask;
	if (result == mask)
	{
		SwitchOffLed(GPIOx, GPIO_Pin, led);
	}
	else
	{
		SwitchOnLedG2(GPIOx, GPIO_Pin, led);
	}
}
