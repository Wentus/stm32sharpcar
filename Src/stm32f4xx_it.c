/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "arm_math.h"

uint16_t angel = 90;
#include <math.h>
#include<stdbool.h>

extern NN neurons[7];
extern uint16_t speed,angle, angle1;
extern TIM_HandleTypeDef htim11;
extern uint16_t sharp[5];
extern void set_speed(int speed);
extern uint8_t sharp_set[5];
//extern void convert(uint32_t *mass1, uint8_t* mass2);
extern void convert(uint16_t *mass1, uint8_t* mass2);
extern int teach(NN *neuron, uint8_t * data);
extern int  check(NN* neurons1, uint8_t* data);
extern arm_pid_instance_f32 PID;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
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
* @brief This function handles Pre-fetch fault, memory access fault.
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
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
/*
	static bool a = 0;
	a = !a;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, a);
	*/
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
	
	
  /* USER CODE BEGIN TIM7_IRQn 0 */
	
	convert(sharp, sharp_set);
//	angle = 	check(neurons, sharp_set);
	
	sharp_set[0] = 65*pow((sharp[0]/4)*0.0048828125, -1.10);
	sharp_set[2] = 65*pow((sharp[2]/4)*0.0048828125, -1.10);
	
	int middle = 90 ;
	int speed = 75;
	
		
	
		
		
	//	middle = middle +(sharp_set[0]-sharp_set[4]);
					
				if( sharp_set[1] > 2)
				{
          

					//else middle=80;
             //     middle=78;
            // rule=middle-4*(right_distance-left_distance)/(4+centr_distance/8);      //mulyiplier 16 was near good +20 to keep right
            //  rule=middle+4*(left_distance-25)/(4+centr_distance/32);

								if(1.3*sharp_set[2]-sharp_set[0] > 0)
                angle= middle+(1.2*sharp_set[2]-sharp_set[0])/3 +sharp_set[1] ;
								else
									angle= middle+(1.2*sharp_set[2]-sharp_set[0])/3 -sharp_set[1] ;
	
	
				 }
				else
				{
					speed = -100;
			if( sharp_set[0] >sharp_set[2])	
		middle = 105;
	else
		middle = 75;
	angle = middle;	
		
		
				}
			
			//if(sharp_set[0] - sharp_set[4] > 3 || sharp_set[2] < 7)
/*
if( fabs( sharp_set[1] - sharp_set[3]) >5

		&& sharp_set[4] < 5)	
		{
			
				if(sharp_set[3] >sharp_set [1])
				angle1 =  75;
				else
					angle1 =   105;
		}
			else
			{
				if(sharp_set[2] > sharp_set[0])
				angle = 105;
				else angle =75;
	
			}
	*/
		//else
			//angle = middle +4*(sharp_set[1] - sharp_set[3])/(sharp_set[2]*0.2);
	/*
	else		
	{
		speed = -50;
			if( sharp_set[0] >sharp_set[4])	
		middle = 80;
	else
		middle = 100;
	angle = middle;	
	}
		*/
		
		if( angle > 105) angle = 105;
if(angle < 75) angle = 75;

	
	
		
	
	
	


	
	//for(int i = 0; i < 5; i++)
	//HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Yura1", 5);
		//void convert(uint32_t *mass1, uint32_t* mass2);
	//__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, speed);
		
	
	
	set_speed(speed);
	//servo 90 delta 10
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, angle);
	
	//distance = 65*pow(volt, -1.10);
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
