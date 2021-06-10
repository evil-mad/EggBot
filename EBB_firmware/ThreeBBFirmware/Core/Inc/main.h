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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define G9_Pin GPIO_PIN_13
#define G9_GPIO_Port GPIOC
#define G10_Pin GPIO_PIN_14
#define G10_GPIO_Port GPIOC
#define G11_Pin GPIO_PIN_15
#define G11_GPIO_Port GPIOC
#define G6_Pin GPIO_PIN_0
#define G6_GPIO_Port GPIOF
#define G8_Pin GPIO_PIN_1
#define G8_GPIO_Port GPIOF
#define STEP1_Pin GPIO_PIN_0
#define STEP1_GPIO_Port GPIOA
#define STEP2_Pin GPIO_PIN_1
#define STEP2_GPIO_Port GPIOA
#define STEP3_Pin GPIO_PIN_2
#define STEP3_GPIO_Port GPIOA
#define CUR_SNS_Pin GPIO_PIN_3
#define CUR_SNS_GPIO_Port GPIOA
#define G2_Pin GPIO_PIN_4
#define G2_GPIO_Port GPIOA
#define G3_Pin GPIO_PIN_5
#define G3_GPIO_Port GPIOA
#define G4_Pin GPIO_PIN_6
#define G4_GPIO_Port GPIOA
#define G5_Pin GPIO_PIN_7
#define G5_GPIO_Port GPIOA
#define G7_Pin GPIO_PIN_4
#define G7_GPIO_Port GPIOC
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
#define P0_Pin GPIO_PIN_6
#define P0_GPIO_Port GPIOC
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
#define G12_Pin GPIO_PIN_15
#define G12_GPIO_Port GPIOA
#define G0_Pin GPIO_PIN_10
#define G0_GPIO_Port GPIOC
#define G1_Pin GPIO_PIN_11
#define G1_GPIO_Port GPIOC
#define P1_Pin GPIO_PIN_4
#define P1_GPIO_Port GPIOB
#define P2_Pin GPIO_PIN_5
#define P2_GPIO_Port GPIOB
#define P3_Pin GPIO_PIN_6
#define P3_GPIO_Port GPIOB
#define P4_Pin GPIO_PIN_7
#define P4_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
#define P5_Pin GPIO_PIN_9
#define P5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
