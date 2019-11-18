/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CLK8_IN_Pin GPIO_PIN_0
#define CLK8_IN_GPIO_Port GPIOF
#define USART4_A0_Pin GPIO_PIN_4
#define USART4_A0_GPIO_Port GPIOA
#define USART4_A1_Pin GPIO_PIN_5
#define USART4_A1_GPIO_Port GPIOA
#define USART4_A2_Pin GPIO_PIN_6
#define USART4_A2_GPIO_Port GPIOA
#define USART4_A3_Pin GPIO_PIN_7
#define USART4_A3_GPIO_Port GPIOA
#define USART3_A0_Pin GPIO_PIN_0
#define USART3_A0_GPIO_Port GPIOB
#define USART3_A1_Pin GPIO_PIN_1
#define USART3_A1_GPIO_Port GPIOB
#define USART3_A2_Pin GPIO_PIN_2
#define USART3_A2_GPIO_Port GPIOB
#define USART1_A0_Pin GPIO_PIN_12
#define USART1_A0_GPIO_Port GPIOB
#define USART1_A1_Pin GPIO_PIN_13
#define USART1_A1_GPIO_Port GPIOB
#define USART1_A2_Pin GPIO_PIN_14
#define USART1_A2_GPIO_Port GPIOB
#define USART1_A3_Pin GPIO_PIN_15
#define USART1_A3_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define USART3_A3_Pin GPIO_PIN_3
#define USART3_A3_GPIO_Port GPIOB
#define USART2_A0_Pin GPIO_PIN_4
#define USART2_A0_GPIO_Port GPIOB
#define USART2_A1_Pin GPIO_PIN_5
#define USART2_A1_GPIO_Port GPIOB
#define USART2_A2_Pin GPIO_PIN_6
#define USART2_A2_GPIO_Port GPIOB
#define USART2_A3_Pin GPIO_PIN_7
#define USART2_A3_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_8
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_9
#define LED_GREEN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define FIFO_SIZE 4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
