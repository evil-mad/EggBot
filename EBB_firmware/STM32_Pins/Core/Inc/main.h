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
#define D9_Pin GPIO_PIN_13
#define D9_GPIO_Port GPIOC
#define D10_Pin GPIO_PIN_14
#define D10_GPIO_Port GPIOC
#define D11_Pin GPIO_PIN_15
#define D11_GPIO_Port GPIOC
#define D6_Pin GPIO_PIN_0
#define D6_GPIO_Port GPIOF
#define D8_Pin GPIO_PIN_1
#define D8_GPIO_Port GPIOF
#define STEP1_Pin GPIO_PIN_0
#define STEP1_GPIO_Port GPIOA
#define STEP2_Pin GPIO_PIN_1
#define STEP2_GPIO_Port GPIOA
#define STEP3_Pin GPIO_PIN_2
#define STEP3_GPIO_Port GPIOA
#define CUR_SNS_Pin GPIO_PIN_3
#define CUR_SNS_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_4
#define D2_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_5
#define D3_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_6
#define D4_GPIO_Port GPIOA
#define D5_Pin GPIO_PIN_7
#define D5_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_4
#define D7_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_0
#define DIR1_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_2
#define DIR3_GPIO_Port GPIOB
#define DBG_RX_Pin GPIO_PIN_10
#define DBG_RX_GPIO_Port GPIOB
#define DBG_TX_Pin GPIO_PIN_11
#define DBG_TX_GPIO_Port GPIOB
#define SCALED_5V_Pin GPIO_PIN_12
#define SCALED_5V_GPIO_Port GPIOB
#define SVO_EN_Pin GPIO_PIN_13
#define SVO_EN_GPIO_Port GPIOB
#define SCALED_V__Pin GPIO_PIN_14
#define SCALED_V__GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_15
#define EN_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_6
#define S0_GPIO_Port GPIOC
#define USR_LED_Pin GPIO_PIN_8
#define USR_LED_GPIO_Port GPIOA
#define STP_UART_TX_Pin GPIO_PIN_9
#define STP_UART_TX_GPIO_Port GPIOA
#define STP_UART_RX_Pin GPIO_PIN_10
#define STP_UART_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define D12_Pin GPIO_PIN_15
#define D12_GPIO_Port GPIOA
#define D0_Pin GPIO_PIN_10
#define D0_GPIO_Port GPIOC
#define D1_Pin GPIO_PIN_11
#define D1_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_4
#define S1_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_5
#define S2_GPIO_Port GPIOB
#define S3_Pin GPIO_PIN_6
#define S3_GPIO_Port GPIOB
#define S4_Pin GPIO_PIN_7
#define S4_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
#define S5_Pin GPIO_PIN_9
#define S5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
