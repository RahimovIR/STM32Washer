

#include "engine.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "buttons.h"
#include "leds.h"

extern int impulsShift;
extern int maxShift;
extern int minShift;

void StartMainEngineTask(void const * argument)
{
	uint32_t ulNotificationValue;
	//const TickType_t xBlockTime = pdMS_TO_TICS(50);
	for (;;)
	{
		ulNotificationValue = ulTaskNotifyTake(pdFALSE, 20000);
		if (ulNotificationValue == 1)
		{
			switch (buttonNotyfiParametrs.pressButton)
			{
			case BUTTON_START:
				if (buttonNotyfiParametrs.action == SHORT_CLICK)
				{
					impulsShift -= 100;
					if (impulsShift < minShift) impulsShift = minShift;
				}
				if (buttonNotyfiParametrs.action == LONG_CLICK)
				{
					impulsShift += 100;
					if (impulsShift > maxShift) impulsShift = maxShift;
				}
				break;

			case BUTTON_6:
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					SwitchOnLD(LD_8);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					SwitchOffLD(LD_8);
				}
				break;
			case BUTTON_4:
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					SwitchOnLD(LD_9);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					SwitchOffLD(LD_9);
				}
				break;
			default:
				break;
			}

		}

	}
}