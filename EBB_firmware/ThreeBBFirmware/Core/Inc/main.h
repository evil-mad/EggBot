/* USER CODE BEGIN Header */
/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        main.h
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Software License Agreement
 *
 * Copyright (c) 2020-2021, Brian Schmalz of Schmalz Haus LLC
 * All rights reserved.
 * Based on EiBotBoard (EBB) Firmware, written by Brian Schmalz of
 *   Schmalz Haus LLC
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials
 * provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of
 * its contributors may be used to endorse or promote products
 * derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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
#include "printf.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// Linkerscript defines 64 bytes (16 uint32s) that are never initialized
// from 0x20000000 to 0x2000003F. These defines are 'variables' located there
// that we can use for various things.

#define BL_FORCE_RUN_MAGIC  0xFEEDBEEF

// When run, bootloader looks for this to be BL_FORCE_RUN_MAGIC and if so forces
// bootloader mode to be entered
#define BL_FORCE_RAMLOC     (*((volatile uint32_t *) 0x20000000))

// Keeps track of time between RST button pushes
#define BL_RST_CNT_RAMLOC   (*((volatile uint32_t *) 0x20000004))

// Address of the beginning of the bootloader image
#define BOOTLOADER_ADDR     0x08000000

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

// How many stepper motors does this board support? (3BB has 3)
#define NUMBER_OF_STEPPERS      3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
