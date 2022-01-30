/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc4;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC4_Init(void);

/* USER CODE BEGIN Prototypes */

/*
 * Returns the previously acquired SCALED_V+ value
 * Returns raw 12 bit ADC conversion
 */
uint16_t adc_AcquireScaledVPlus(void);

/*
 * Returns the previously acquired SCALED_5V value
 * Returns raw 12 bit ADC conversion
 */
uint16_t adc_AcquireScaled5V(void);

/*
 * Returns the previously acquired CUR_SNS value
 * Returns raw 12 bit ADC conversion
 */
uint16_t adc_AcquireMotorCurrent(void);

/*
 * Triggers one ADC conversion on all three channels
 */
void ADC_StartConversions(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

