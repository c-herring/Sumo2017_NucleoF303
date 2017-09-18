/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */


// ---- PIN PORT DEFINITIONS ---- //

#define MOTORS_PWM_PORT 		GPIOA
#define MOTORS_PWM_PIN_LEFT 	GPIO_PIN_9
#define MOTORS_PWM_CHANNEL_L	TIM_CHANNEL_3
#define MOTORS_PWM_PIN_RIGHT 	GPIO_PIN_10
#define MOTORS_PWM_CHANNEL_R	TIM_CHANNEL_4

#define LINE_SENSOR_PORT 	GPIOB
#define LINE_SENSOR_PINA1	GPIO_PIN_4
#define LINE_SENSOR_PINA2	GPIO_PIN_5
#define LINE_SENSOR_PINB1	GPIO_PIN_6
#define LINE_SENSOR_PINB2	GPIO_PIN_7

#define IR0_PORT			GPIOA
#define IR0_PIN				GPIO_PIN_0
#define IR1_PORT			GPIOA
#define IR1_PIN				GPIO_PIN_1
#define IR2_PORT			GPIOA
#define IR2_PIN				GPIO_PIN_3
#define IR3_PORT			GPIOA
#define IR3_PIN				GPIO_PIN_4
#define IR4_PORT			GPIOA
#define IR4_PIN				GPIO_PIN_5
#define IR5_PORT			GPIOA
#define IR5_PIN				GPIO_PIN_6
#define IR6_PORT			GPIOA
#define IR6_PIN				GPIO_PIN_7
#define IR7_PORT			GPIOB
#define IR7_PIN				GPIO_PIN_1


// ---- Global Variables ---- //
typedef struct _SensorStates{
	// 4 LSB represent line sensors: [0 0 0 0 B2 B1 A2 A1]
	uint8_t LineSensors;
	uint32_t IRSensors[8];
} SensorStates;

typedef struct _Motor {
	uint32_t pulseWidth;
	uint32_t period;
	uint32_t channel;
} Motor;

// Create global instances of the motor structures
Motor L_motor;
Motor R_motor;

// ---- Function Prototypes ---- //
void changePinMode(uint32_t mode);
void updateLineSensorState(uint8_t *state);
void setMotorPWM(void);

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
