/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        debug.c
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

/************** INCLUDES ******************************************************/
#include "main.h"   // Pull in all of the CubeMX pin definition names
#include "debug.h"

/************** PRIVATE TYPEDEFS **********************************************/

/************** PRIVATE DEFINES ***********************************************/

/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// Boolean, set true if ITM (SWO) is turned on, used to gate _putchar()
static  uint32_t    bItmAvailable;

/************** PRIVATE FUNCTION PROTOTYPES ***********************************/

/************** PRIVATE FUNCTIONS *********************************************/

/************** PUBLIC FUNCTIONS **********************************************/

/*
 * Our own function for the printf.c library that actually outputs a byte
 */
void _putchar(char character)
{
  if (bItmAvailable)
  {
    ITM_SendChar(character);
  }
}

/* Perform any initialization of debug resources. For example, make all debug pins outputs
 * Also test out all debug pins by setting them high then low again.
 */
void DebugInit(void)
{
#if defined(DEBUG)
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure debug pin Output Levels to start out low */
  HAL_GPIO_WritePin(GPIOA, G2_Pin | G3_Pin | G4_Pin | G5_Pin | G12_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, G0_Pin | G1_Pin | G7_Pin | G9_Pin | G10_Pin | G11_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOF, G6_Pin | G8_Pin, GPIO_PIN_RESET);

  /*Configure debug pins to be outputs */
  GPIO_InitStruct.Pin = G2_Pin | G3_Pin | G4_Pin | G5_Pin | G6_Pin | G8_Pin | G12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =  G0_Pin | G1_Pin | G7_Pin | G9_Pin | G10_Pin | G11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =  G6_Pin | G8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* Test each debug pin */
  DEBUG_G0_SET();
  DEBUG_G1_SET();
  DEBUG_G2_SET();
  DEBUG_G3_SET();
  DEBUG_G4_SET();
  DEBUG_G5_SET();
  DEBUG_G6_SET();
  DEBUG_G7_SET();
  DEBUG_G8_SET();
  DEBUG_G9_SET();
  DEBUG_G10_SET();
  DEBUG_G11_SET();
  DEBUG_G12_SET();
  DEBUG_G0_RESET();
  DEBUG_G1_RESET();
  DEBUG_G2_RESET();
  DEBUG_G3_RESET();
  DEBUG_G4_RESET();
  DEBUG_G5_RESET();
  DEBUG_G6_RESET();
  DEBUG_G7_RESET();
  DEBUG_G8_RESET();
  DEBUG_G9_RESET();
  DEBUG_G10_RESET();
  DEBUG_G11_RESET();
  DEBUG_G12_RESET();
#endif
}

/*
    Initialize the SWO trace port for debug message printing
    portMask : Port bit mask to be configured
    cpuCoreFreqHz : CPU core clock frequency in Hz
    baudrate : SWO frequency in Hz
*/

void Debug_SWOInit(uint32_t portMask, uint32_t cpuCoreFreqHz, uint32_t baudrate)
{
  uint32_t SWOPrescaler = (cpuCoreFreqHz / baudrate) - 1 ;

  // Debug Exception and Monitor Control Register: enable trace in core debug
  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;

  // DBGMCU_CR : TRACE_IOEN DBG_STANDBY DBG_STOP  DBG_SLEEP
  DBGMCU->CR   = 0x00000027;

  // Selected PIN Protocol Register: Select which protocol to use for trace output (2: SWO)
  TPI->SPPR    = 0x00000002 ;

  // Async Clock Prescaler Register: Scale the baud rate of the asynchronous output
  TPI->ACPR    = SWOPrescaler ;

  // ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC
  ITM->LAR = 0xC5ACCE55 ;

  // ITM Trace Control Register
  ITM->TCR = 0x0001000D ;

  // ITM Trace Privilege Register: All stimulus ports
  ITM->TPR = ITM_TPR_PRIVMASK_Msk ;

  // ITM Trace Enable Register: Enabled tracing on stimulus ports. One bit per stimulus port.
  ITM->TER = portMask;

  // Data Watchpoint and Trace Register
  DWT->CTRL    = 0x400003FE ;

  // Formatter and Flush Control Register
  TPI->FFCR    = 0x00000100 ;

  // ITM/SWO works only if enabled from debugger.
  // If ITM stimulus 0 is not free, don't try to send data to SWO
  if (ITM->PORT [0].u8 == 1)
  {
    bItmAvailable = 1 ;
  }
}

