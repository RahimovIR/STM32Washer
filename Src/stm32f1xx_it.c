/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
extern xSemaphoreHandle zeroCross;
extern uint16_t buffer[DMA_BUFFER_SIZE];
extern uint8_t transmitBuffer[32];
ushort firstDma = 1;
ushort itCompare3Enable = 0;
ushort itCompare4Enable = 0;
float powerPrecent = 30;
float power = (1.0 - 0.01 * 30) / 2;
extern UART_HandleTypeDef huart1;

void sendUart(uint8_t *pData, uint16_t Size);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim2_ch2_ch4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
	powerPrecent += 10;
	if (powerPrecent >= 100) powerPrecent = 10;
	power = (1.0 - 0.01 * powerPrecent) / 2;
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel7 global interrupt.
*/
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch2_ch4);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */
	int t = 0;
	int curId = hdma_tim2_ch2_ch4.Instance->CNDTR - 1;
	int lastId = DMA_BUFFER_SIZE  - hdma_tim2_ch2_ch4.Instance->CNDTR;
	if (firstDma == 1)
	{
		firstDma = 0;
		return;
	}

	if (buffer[curId] > buffer[lastId])
	{
		t = buffer[curId] - buffer[lastId];
	}
	else
	{
		t = buffer[curId] + htim2.Init.Period + 1 - buffer[lastId];
	}


	int newCompare = buffer[curId] + t + t * power;
	if (newCompare > htim2.Init.Period)
	{
		newCompare -= htim2.Init.Period;
		while (__HAL_TIM_GET_COUNTER(&htim2) >= newCompare)
		{
			
		}
	}
	while (itCompare3Enable)
	{
			
	}
	if (newCompare > (int)__HAL_TIM_GET_COUNTER(&htim2) )
	{
		itCompare3Enable = 1;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, newCompare);
	}

//		sprintf((char*)transmitBuffer, "%d, %d, t %d, n %d\r\n", buffer[curId], buffer[lastId], t, newCompare);
//		sendUart(transmitBuffer, 64);

	newCompare = newCompare + t / 2;
	if (newCompare > htim2.Init.Period)
	{
		newCompare -= htim2.Init.Period;
		while (__HAL_TIM_GET_COUNTER(&htim2) >= newCompare)
		{
			
		}
	}
	while (itCompare4Enable)
	{
			
	}
	if (newCompare > (int)__HAL_TIM_GET_COUNTER(&htim2) )
	{
		itCompare4Enable = 1;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, newCompare);
	}

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  

	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC3) != RESET && itCompare3Enable)
		{
//			sprintf((char*)transmitBuffer, "cnt3. %d\r\n", (int)__HAL_TIM_GET_COUNTER(&htim2));
//			sendUart(transmitBuffer, 64);

			itCompare3Enable = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			for (int i = 0; i < 500; i++)
			{
				
			}
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

		}
	}

	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC4) != RESET && itCompare4Enable)
		{
//			sprintf((char*)transmitBuffer, "cnt4. %d\r\n", (int)__HAL_TIM_GET_COUNTER(&htim2));
//			sendUart(transmitBuffer, 64);

			itCompare4Enable = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			for (int i = 0; i < 500; i++)
			{
				
			}
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		}
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void sendUart(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit(&huart1, pData, Size, 1000);
	for (int i = 0; i < Size; i++)
	{
		pData[i] = '\0';
	}
}

//static BaseType_t xHigherPriorityTaskWoken;
//xHigherPriorityTaskWoken = pdFALSE;
//xSemaphoreGiveFromISR(zeroCross, &xHigherPriorityTaskWoken);
//			sprintf((char*)transmitBuffer, "cnt4. %d\r\n", (int)__HAL_TIM_GET_COUNTER(&htim2));
//			HAL_UART_Transmit(&huart1, transmitBuffer, 64, 1000);
//			for (int i = 0; i < 64; i++)
//			{
//				transmitBuffer[i] = '\0';
//			}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
