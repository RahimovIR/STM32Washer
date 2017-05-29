
#include "buttons.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define BUTTON_START_VAL_MIN	4000
#define BUTTON_START_VAL_MAX	4100

#define BUTTON_4_VAL_MIN	500
#define BUTTON_4_VAL_MAX	600

#define BUTTON_5_VAL_MIN	830
#define BUTTON_5_VAL_MAX	930

#define BUTTON_6_VAL_MIN	1100
#define BUTTON_6_VAL_MAX	1200

void ProcessPress(void);
Button_t GetCurentButton(ADC_HandleTypeDef* hadc);

extern osThreadId handleMainEngineTask;

Button_t lastPress = BUTTON_NO;

Button_t processButton = BUTTON_NO;

void StartMainButtonTask(void const * argument)
{
	ADC_HandleTypeDef* hadc = (ADC_HandleTypeDef*)argument;

	enum 
	{
		UP,
		DOWN,
		AL
	} processState;

	Button_t currentButton = BUTTON_NO;
	processState = UP;
	uint32_t time = 0;
	for (;;)
	{
		osDelay(20);
		currentButton = GetCurentButton(hadc);
		if (currentButton != BUTTON_NO && currentButton != processButton)
		{
			processButton = currentButton;
			processState = UP;
		}

		switch (processState)
		{
		case UP:
			if (currentButton == BUTTON_NO) continue;
			processState = DOWN;
			time = 0;
			continue;
		case DOWN:
			if (currentButton == BUTTON_NO || currentButton != processButton)
			{
				processState = UP;
				time = 0;
				buttonNotyfiParametrs.pressButton = processButton;
				buttonNotyfiParametrs.action = SHORT_CLICK;
				ProcessPress();
				continue;
			}
			if (time > 20)
			{
				processState = AL;
				time = 0;
				buttonNotyfiParametrs.pressButton = processButton;
				buttonNotyfiParametrs.action = LONG_CLICK;
				ProcessPress();
				continue;
			}
			time++;
			continue;
		case AL:
			if (currentButton == BUTTON_NO)
			{
				processState = UP;
				time = 0;
			}
			continue;
		default:
			break;
		}
	}
}

Button_t GetCurentButton(ADC_HandleTypeDef* hadc)
{
	uint32_t aVal = HAL_ADC_GetValue(hadc);

	if (aVal < BUTTON_4_VAL_MIN)
	{
		return BUTTON_NO;	
	}
	else if (aVal > BUTTON_4_VAL_MIN && aVal < BUTTON_4_VAL_MAX)  //option4
	{
		return BUTTON_4;	
	}
	else if (aVal > BUTTON_5_VAL_MIN && aVal < BUTTON_5_VAL_MAX) //option5
	{
		return BUTTON_5;	
	}
	else if (aVal > BUTTON_6_VAL_MIN && aVal < BUTTON_6_VAL_MAX) //option6
	{
		return BUTTON_6;	
	}
	else if (aVal > BUTTON_START_VAL_MIN) //start
	{
		return BUTTON_START;	
	}

	//return BUTTON_START;
	return BUTTON_NO;

}

void ProcessPress(void)
{
	xTaskNotifyGive(handleMainEngineTask);
	Button_t button = buttonNotyfiParametrs.pressButton;
	switch (button)
	{
	case BUTTON_START:
//		ToggleLD(LD_1);
		break;
	case BUTTON_4:
//		ToggleLD(LD_9);
		break;
	case BUTTON_5:
//		ToggleLD(LD_10);
		break;
	default:
		break;
	}
}