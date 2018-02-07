/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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

#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
//#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


		
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int  check(NN* neurons1, uint8_t* data);
int teach(NN *nero, uint8_t * mass);
void copy_n_to_flash(NN* nero);
void copy_flash_to_n(NN* nero);
void set_speed(int  speed);
void convert(uint16_t *mass1, uint8_t* mass2);

#define PID_PARAM_KP 1
#define PID_PARAM_KI 10
#define PID_PARAM_KD 0

//TM_PWM_TIM_t TIM_Data;



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef	flash_ok = HAL_ERROR;


uint8_t Recv[2];
uint8_t Send[8];
uint16_t angle1;
uint16_t sharp[5];
uint16_t speed = 0, angle =0;
NN neurons[7];

arm_pid_instance_f32 PID;
uint8_t sharp_set[5];

/* USER CODE END 0 */


int main(void)
{

  /* USER CODE BEGIN 1 */

	PID.Kp = PID_PARAM_KP;
PID.Ki = PID_PARAM_KI;
PID.Kd = PID_PARAM_KD;
	arm_pid_init_f32(&PID, 1);
	//arm_pid_init_f32(&PID, 1);

	


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_TIM7_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();

		copy_flash_to_n(neurons);

  /* USER CODE BEGIN 2 */
//	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim7);
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_Base_Start(&htim11);
	//HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&sharp, 5);







/* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
//	HAL_UART_Transmit_IT(&huart2, Buffer, 8);
  //HAL_UART_Receive_IT(&huart2, Buffer, 8);
		//for(uint32_t i = 0 ; i < 0xFFFFF; i++);
	while(1)
  {
		
HAL_UART_Transmit_IT(&huart2, &sharp_set[1], 1);
		HAL_Delay(1000);	
		//convert(sharp, sharp_set);
	//for(int i = 0; i < 5; i++)
	//sharp_set[i] = sharp[i]/16 ;



		/*
		HAL_UART_Receive_IT(&huart2, Recv, 2);
	HAL_Delay(1000);	
		
		if(Recv[0] == '.')
		{
		
	 //HAL_UART_Transmit_IT(&huart2, &Recv[1], 1);
			
		teach(&neurons[Recv[1]-48], sharp_set); 	
			
			Recv[0] = 0;
			Recv[1] = 0;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
			HAL_Delay(1000);	
				
			HAL_UART_Transmit_IT(&huart2, sharp_set, 5);
		}
		
		if(Recv[0] == 'S')
		{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		copy_n_to_flash(neurons);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Done!", 5);
		}
		Recv[0] = 0;
			Recv[1] = 0;
//		uint8_t sharp_set[5];
		
	//		for(int i = 0; i < 5; i++)
//		sharp_set[i] = sharp[i]/16 ;	
	//	convert(sharp, sharp_set);
		//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)&sharp_set[3] , 1);
		//HAL_Delay(1000);
	//HAL_UART_Receive_IT(&huart2, Recv, 8);
	//for(uint32_t i =0; i < 0xFFFFFF; i++);
  //HAL_UART_Transmit_IT(&huart2, Recv, 8);
	//for(uint32_t i =0; i < 0xFFFFFF; i++);
	*/
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 40000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 210;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 2800;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 200;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 90;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim10);

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 5600;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 300;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim11);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int teach(NN *neuron, uint8_t * data) {
	//
	//static int neuronus = 0;
	   for (int i = 0; i < 5; i++){
		for(int k = 0; k < data[i]+1;k++){
			neuron->mass[i][k]++;
			neuron->weight++;
		}
		for(int l =  data[i]+1; l < 10; l++)
		neuron->mass[i][l]--;

	   }

}



int  check(NN* neurons1, uint8_t* data) {
	float percents[7] = {0};
	float percent = 0;
	float weight = 0;
	for (int i = 0; i < 7; i++) {  //

	      for (int j = 0; j < 5; j++){  //???????? ?? ???? 
	           for(int k = 0; k < data[j]+1; k++ )
	               percent += neurons1[i].mass[j][k];

	           for(int l = 0; l < 10; l++)
	               if(neurons1[i].mass[j][l] > 0) weight+=neurons1[i].mass[j][l];
               // printf("%f" ,weight);
	           percents[i]+= percent/weight;
	      }
	       percents[i] = percents[i] / 5;

	       percent = 0;
	        weight = 0;
	}


	int neuron = 0;
	for (int k = 0; k < 7; k++)
		if (percents[k] > percents[neuron]) neuron = k;
	
	return neurons1[neuron].alpha;

}





void copy_n_to_flash(NN* nero)
{
	uint32_t adres = 0x080E0000;
	
		while(flash_ok != HAL_OK){
		flash_ok = HAL_FLASH_Unlock();
	}
		
	FLASH_Erase_Sector(FLASH_SECTOR_11, VOLTAGE_RANGE_3);
	
	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK){
		flash_ok = HAL_FLASH_Lock();
	}
	
		
		flash_ok = HAL_ERROR;
			while(flash_ok != HAL_OK){
		flash_ok = HAL_FLASH_Unlock();
	}
		
		for(int i = 0; i < 7; i++)
	{
		for(int j = 0; j < 5; j++) {
		for(int k = 0; k < 9; k++)	{
		flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK){
		flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, adres++,nero[i].mass[j][k]);
		}
	}		
}	
}
	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK){
		flash_ok = HAL_FLASH_Lock();

	}

}

void set_speed(int speed)
{
	if(speed > 0)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 1);
	}
	else if( speed < 0)
	{
	
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 0);
	
	}
	else 
	{
	
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, 0);
	}
			if( speed > 0)
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, speed);
			else 
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, -1*speed);
}

uint32_t flash_read(uint32_t address) {
return (*(__IO uint32_t*) address);

}

void copy_flash_to_n(NN* nero) 
	{
		 uint32_t adres = 0x080E0000;
		
			for(int i = 0; i < 7; i++)
	{
		for(int j = 0; j < 5; j++) 
		for(int k = 0; k < 10; k++)	
			{
		nero[i].mass[j][k] = flash_read(adres);
			adres++;
			}


		
	}
}
	

void convert(uint16_t *mass1, uint8_t* mass2)
{

 if( ((3.3/4096)*mass1[1]) > 1.6) mass2[1] = 1; //right
		else if((3.3/4096)*mass1[1] > 1.1) mass2[1] = 2;
					else if((3.3/4096)*mass1[1] > 0.85) mass2[1] =3;
						else if((3.3/4096)*mass1[1] > 0.65) mass2[1] =4;
							else if((3.3/4096)*mass1[1] > 0.56) mass2[1] =5;
										else if((3.3/4096)*mass1[1] > 0.48) mass2[1] =6;
											else if((3.3/4096)*mass1[1] > 0.42) mass2[1] =7;
												else mass2[1] = 8;
		
	if( ((3.3/4096)*mass1[3]) > 1.6) mass2[3] = 1; // left
		else if((3.3/4096)*mass1[3] > 1.1) mass2[3] = 2;
					else if((3.3/4096)*mass1[3] > 0.85) mass2[3] =3;
						else if((3.3/4096)*mass1[3] > 0.65) mass2[3] =4;
							else if((3.3/4096)*mass1[3] > 0.56) mass2[3] =5;
										else if((3.3/4096)*mass1[3] > 0.48) mass2[3] =6;
											else if((3.3/4096)*mass1[3] > 0.42) mass2[3] =7;
												else mass2[3] = 8;
		
	



	if( ((3.3/4096)*mass1[4]) > 1.6) mass2[4] = 1; //center
		else if((3.3/4096)*mass1[4] > 1.1) mass2[4] = 2;
					else if((3.3/4096)*mass1[4] > 0.85) mass2[4] =3;
						else if((3.3/4096)*mass1[4] > 0.65) mass2[4] =4;
							else if((3.3/4096)*mass1[4] > 0.56) mass2[4] =5;
										else if((3.3/4096)*mass1[4] > 0.48) mass2[4] =6;
											else if((3.3/4096)*mass1[4] > 0.42) mass2[4] =7;
												else mass2[4] = 8;
		
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
