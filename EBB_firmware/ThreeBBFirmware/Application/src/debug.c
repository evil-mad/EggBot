/*
 * debug.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Brian Schmalz
 */

#include "main.h"   // Pull in all of the CubeMX pin definition names
#include "debug.h"

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

void SWO_PrintString(const char *s)
{
/// TODO: Enabling this hangs us for some reason (after a few commands). Why? This should work.
//  while(*s != '\0')
//  {
//    ITM_SendChar((uint32_t)*s++);
//  }
}

/*!
 * \brief Initialize the SWO trace port for debug message printing
 * \param portBits Port bit mask to be configured
 * \param cpuCoreFreqHz CPU core clock frequency in Hz
 */
void SWO_Init(uint32_t portBits, uint32_t cpuCoreFreqHz)
{
  uint32_t SWOSpeed = 2000000; /* default 2000kHz baud rate */
  uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1; /* SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock */

  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk; /* enable trace in core debug */
  *((volatile unsigned *)(ITM_BASE + 0x400F0)) = 0x00000002; /* "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO NRZ, 1: SWO Manchester encoding) */
  *((volatile unsigned *)(ITM_BASE + 0x40010)) = SWOPrescaler; /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */
  *((volatile unsigned *)(ITM_BASE + 0x00FB0)) = 0xC5ACCE55; /* ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC */
  ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk; /* ITM Trace Control Register */
  ITM->TPR = ITM_TPR_PRIVMASK_Msk; /* ITM Trace Privilege Register */
  ITM->TER = portBits; /* ITM Trace Enable Register. Enabled tracing on stimulus ports. One bit per stimulus port. */
  *((volatile unsigned *)(ITM_BASE + 0x01000)) = 0x400003FE; /* DWT_CTRL */
  *((volatile unsigned *)(ITM_BASE + 0x40304)) = 0x00000100; /* Formatter and Flush Control Register */
}

