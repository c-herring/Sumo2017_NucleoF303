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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "dwt_stm32_delay.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t IRSensors[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  if(DWT_Delay_Init())
  {
    Error_Handler(); /* Call Error Handler */
  }

  // Temp test stuff
  GPIO_PinState pinState;
  uint32_t LineSensorStopwatch = HAL_GetTick();
  //changePinMode(GPIO_MODE_OUTPUT_PP);
  uint32_t LineSensorPollPeriod_ms = 500;
  uint32_t LineSensorDelayCharge_us = 10;
  uint32_t LineSensorDelay_us = 200;
  // End of temp stuff

  // Create the sensorStares variable TODO: REMOVE TO HEADER
  SensorStates sensorStates;
  // Start ADC writing to DMA on scan complete
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(sensorStates.IRSensors), 4);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)(sensorStates.IRSensors)+4, 4);
  // Start the ADC
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  HAL_UART_Receive_DMA(&huart2, &(RXCommand.rxByte), 1);

  // Initialise the motor strucutures
  L_motor.period = 7999;
  L_motor.pulseWidth = 0;
  L_motor.channel = MOTORS_PWM_CHANNEL_L;
  R_motor.period = 7999;
  R_motor.pulseWidth = 0;
  R_motor.channel = MOTORS_PWM_CHANNEL_R;

  // Start both channels of the PWM
  setMotorPWM(); // Make sure they start with zero duty cycle
  HAL_TIM_PWM_Start(&htim2, L_motor.channel);
  HAL_TIM_PWM_Start(&htim2, R_motor.channel);



  // Initialise variables
  uint32_t stopwatch = HAL_GetTick(); // Start the main loop timer
  uint32_t timerA; // Timer A and B are debugging timers
  uint32_t timerB;
  uint8_t buffer[200]; // Debug buffer

  // Transmit buffer

  int TXBufHeaderLen = 5;
  int TXBufFooterLen = 3;
  int TXDataLen = 4*8+8; //4*8+8+2;
  int TXBuffLen = TXBufHeaderLen +  TXBufFooterLen + TXDataLen;   //5+3+8*4+8;
  uint8_t TXBuff[TXBuffLen];
  uint8_t TXBuffHeader[] = {'s', 't', 'a', 'r', 't'};
  uint8_t TXBuffFooter[] = {'e', 'n', 'd'};

  RXCommand.headerLength = 5;
  RXCommand.footerLength = 3;
  memcpy(RXCommand.cmdHeader, TXBuffHeader, 5);
  memcpy(RXCommand.cmdFooter, TXBuffFooter, 3);
  //RXCommand.buff[5+3+11] = '\0';
  RXCommand.RXMotorsWatchdog = HAL_GetTick();


  // Copy in the head and footers
  memcpy(TXBuff, TXBuffHeader, 5);
  memcpy(TXBuff+5+TXDataLen, TXBuffFooter, 3);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int attemptNo = 11;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if (HAL_GetTick() - stopwatch > 19)
	  {
		  // Toggle the indicator
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

		  // Reset the stopwatch
		  stopwatch = HAL_GetTick();

		  // Check the watchdog timer. If it has expired then turn motors off.
		  if (HAL_GetTick() - RXCommand.RXMotorsWatchdog > MOTOR_WATCHDOG_MS)
		  {
			  L_motor.pulseWidth = 0;
			  R_motor.pulseWidth = 0;
		  }

		  // Update motor pwm
		  setMotorPWM();

		  // Update the state of the line sensors
		  //updateLineSensorState(&(sensorStates.LineSensors));

		  // Create the TX string. For loop places the bytes of the uint32_t IR sensor data into the string.
		  int i;
		  for (i = 0; i < TXDataLen; i++)
		  {
			  // If this is a multiple of 5 then it is the sensor identifier
			  if(i%5 == 0)
			  {
				  TXBuff[TXBufHeaderLen+i] = '0'+i/5;
			  }
			  // Otherwise it is data
			  else
			  {
				  TXBuff[TXBufHeaderLen+i] = (uint8_t)(sensorStates.IRSensors[(uint8_t)(i/5)] >> 8*(i%5-1) & 0xFF);
						  //'0'+(uint8_t)(i%5);//IRSensors[i/5];// >> 8 ;
			  }
		  }
		  // Almost done, just need to send the line sensor state byte.
		  //TXBuff[TXBufHeaderLen+i++] = 'l';
		  //TXBuff[TXBufHeaderLen+i] = sensorStates.LineSensors;

		  // TRansmit the TX string
		  HAL_UART_Transmit(&huart2, TXBuff, TXBuffLen, 0xFF);



		  /*			-------- DEBUGGING STUFF -------
		  changePinMode(GPIO_MODE_OUTPUT_PP);
		  DWT_Delay_us(LineSensorDelayCharge_us);
		  //LineSensorDelay_us_clkCycles = LineSensorDelay_us * 5 * (SystemCoreClock / 1000000) / 14;
		  //while (LineSensorDelay_us_clkCycles--);
		  changePinMode(GPIO_MODE_INPUT);//GPIO_MODE_INPUT GPIO_MODE_OUTPUT_PP
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		  //LineSensorDelay_us_clkCycles = 1 * (SystemCoreClock / 1000000) / 14;
		  DWT_Delay_us(LineSensorDelay_us);
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		  //while (LineSensorDelay_us_clkCycles--);

		  pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
*/


		  //sprintf((char*)buffer, "(%d)\t\tADC1- 1: %lu\t2: %lu\t4: %lu\t12: %lu\tADC2- 1: %lu\t2: %lu\t3: %lu\t4: %lu\t", attemptNo,
		  //		  				  IRSensors[0], IRSensors[1], IRSensors[2], IRSensors[3],
		  //						  IRSensors[4], IRSensors[5], IRSensors[6], IRSensors[7]);

		  //sprintf((char*)buffer, "5HI Guillaumemesd... delay = %lu, State = %d  ", LineSensorDelay_us, pinState);
		  //LineSensorDelay_us_clkCycles = (10000 / 14 * (SystemCoreClock / 1000000));
		  //LineSensorDelay_us_clkCycles = LineSensorDelay_us * (SystemCoreClock / 1000000) / 14;

// 			-------- END OF DEBUGGING STUFF -------

		  // DEBUGGING: Increment the pulse width
		  //L_motor.pulseWidth += 100;
		  //R_motor.pulseWidth += 200;
		  //L_motor.pulseWidth %= 7999;
		  //R_motor.pulseWidth %= 7999;



		  //HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFF);
		  //HAL_UART_Transmit(&huart2, TXBuff, TXBuffLen, 0xFF);
		  timerA = timerB;
		  timerB = HAL_GetTick();


		  // The last thing we do is append a debug string.
		  // As far as any driver goes, this is just jibberish invalid ahit and will be ignored.
		  sprintf((char*)buffer, "<-- hi4 That took about %lums\tstrstate = %d\tL pwm = %ld\tR pwm = %ld \n\r", timerB-timerA, RXCommand.index, L_motor.pulseWidth, R_motor.pulseWidth);
		  //sprintf((char*)buffer, "<-- hi2 That took about %lums, rxIndex = %lu\tL_Motor = %#010x\t%s \n\r", timerB-timerA, RXCommand.index, L_motor.pulseWidth, RXCommand.buff);
				  //(char)L_motor.pulseWidth&0xFF, ((char)L_motor.pulseWidth >> 8)&0xFF, ((char)L_motor.pulseWidth >> 16)&0xFF, ((char)L_motor.pulseWidth >> 24)&0xFF);
		  HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFF);

	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * Change the IR sensors form input to output or visa versa
 *
 */
void changePinMode(uint32_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pins : PB3 PB4 PB5 PB6
	                         PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
	                        |GPIO_PIN_7;
	GPIO_InitStruct.Mode = mode;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * Check the state of the line sensors. Populate the object
 */
void updateLineSensorState(uint8_t *state)
{
	*state =	HAL_GPIO_ReadPin(LINE_SENSOR_PORT, LINE_SENSOR_PINA1) & 0x01
			|	(HAL_GPIO_ReadPin(LINE_SENSOR_PORT, LINE_SENSOR_PINA2) & 0x01) << 1
			|	(HAL_GPIO_ReadPin(LINE_SENSOR_PORT, LINE_SENSOR_PINB1) & 0x01) << 2
			|	(HAL_GPIO_ReadPin(LINE_SENSOR_PORT, LINE_SENSOR_PINB2) & 0x01) << 3;
}

/**
 * Just some abstraction to set PWM duty cycle.
 */
void setMotorPWM()
{
	//uint32_t L_dir = (L_motor.pulseWidth < 0) ? -1 : 1;
	//uint32_t R_dir = (R_motor.pulseWidth < 0) ? -1 : 1;

	HAL_GPIO_WritePin(MOTORS_DIR_PORT, MOTORS_DIR_PIN_L, (L_motor.pulseWidth < 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTORS_DIR_PORT, MOTORS_DIR_PIN_R, (R_motor.pulseWidth < 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);


	TIM2->MOTORS_PWM_CCRx_L = (uint32_t)(abs(L_motor.pulseWidth));
	TIM2->MOTORS_PWM_CCRx_R = (uint32_t)(abs(R_motor.pulseWidth));
}

/**
 * Callback inside the serial character DMA.
 * Command structure:
 * 	Header: "start"
 * 	First Byte is Command Type
 * 		-! Set motor PWM
 * 	Payload
 * 		!'L'<uint32_t>'R'<uint32_t>
 * 	Footer: "end"
 *
 * 	Command Byte !motor PWM Payload: header!'0'<32bit>'1'<32bit>footer
 */
void parseCommand()
{
	// Check what command we received, parse it.
	switch (RXCommand.buff[RXCommand.headerLength]) // Switching the command byte
	{
	case '!': // Set motor speed
		RXCommand.RXMotorsWatchdog = HAL_GetTick();
		L_motor.pulseWidth = 0x00000000;
		R_motor.pulseWidth = 0x00000000;
		for (int i=0; i<4;i++)
		{
			L_motor.pulseWidth |= ((int32_t)RXCommand.buff[RXCommand.headerLength+2+i] & 0xFF) << i*8;
			R_motor.pulseWidth |= ((int32_t)RXCommand.buff[RXCommand.headerLength+7+i] & 0xFF) << i*8;
		}
		//L_motor.pulseWidth = RXCommand.buff[RXCommand.headerLength+1+1] | (RXCommand.buff[RXCommand.headerLength+1+2] & 0xFF) << 8 | (RXCommand.buff[RXCommand.headerLength+1+3] & 0xFF) << 16 | (RXCommand.buff[RXCommand.headerLength+1+4] & 0xFF) << 24;
		break;
	default:
		break;
	}
	resetCommandStruct();
	//L_motor.pulseWidth += 100;
	//L_motor.pulseWidth %= 7800;
}

/**
 * Reset the command byte back to its default state
 */
void resetCommandStruct()
{
	RXCommand.index = 0;
	RXCommand.cmdLength = RXCommand.headerLength + RXCommand.footerLength;
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
