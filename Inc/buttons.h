#include "stm32f1xx_hal.h"


typedef enum 
{
	BUTTON_START = 0,
	BUTTON_4 = 4,
	BUTTON_5 = 5,
	BUTTON_6 = 6,
	BUTTON_NO = 255
} Button_t;

//u_int8_t buttonPress = BUTTON_NO;

void StartMainButtonTask(void const * argument);
