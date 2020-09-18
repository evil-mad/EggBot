/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SRV1_Pin GPIO_PIN_0
#define SRV1_GPIO_Port GPIOC
#define SRV2_Pin GPIO_PIN_1
#define SRV2_GPIO_Port GPIOC
#define SRV3_Pin GPIO_PIN_2
#define SRV3_GPIO_Port GPIOC
#define SRV4_Pin GPIO_PIN_3
#define SRV4_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOA
#define Step3IO_Pin GPIO_PIN_10
#define Step3IO_GPIO_Port GPIOB
#define Dir3IO_Pin GPIO_PIN_8
#define Dir3IO_GPIO_Port GPIOA
#define Step1IO_Pin GPIO_PIN_10
#define Step1IO_GPIO_Port GPIOA
#define Dir1IO_Pin GPIO_PIN_3
#define Dir1IO_GPIO_Port GPIOB
#define Dir2IO_Pin GPIO_PIN_4
#define Dir2IO_GPIO_Port GPIOB
#define Step2IO_Pin GPIO_PIN_5
#define Step2IO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
