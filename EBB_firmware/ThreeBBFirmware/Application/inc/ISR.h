/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        ISR.c
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Software License Agreement
 *
 * Copyright (c) 2021, Brian Schmalz of Schmalz Haus LLC
 * All rights reserved.
 * Based on EiBotBoard (EBB) Firmware, written by Brian Schmalz of
 *   Schmalz Haus LLC
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ISR_H
#define	ISR_H

/* Includes ------------------------------------------------------------------*/

/* TMR6 fires high_ISR() at 100 KHz rate, so this value is used in math to take this
 * rate into account.
 * TODO: Make this derive from CubeMX somehow?
 */
#define HIGH_ISR_TICKS_PER_MS (100)

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

// Tick counter. Increments every 1ms in interrupt
volatile extern uint32_t TickCounterMS;
volatile extern uint8_t GlobalDelayMS;
volatile extern uint8_t DriverInitDelayMS;

void ISR_MotionISR(void);

///void low_ISR(void);

#endif	/* ISR_H */

