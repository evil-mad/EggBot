/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        stepper.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STEPPER_H__
#define __STEPPER_H__

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "FIFO.h"

/* Exported types ------------------------------------------------------------*/

// Structure to hold step and direction pin values for a stepper
// This is separate from Steppers array because this never changes
typedef struct {
  GPIO_TypeDef * DirPort;       // The port where this stepper's direction pin is located
  uint16_t DirPin;              // The pin of <DirPort> where this stepper's direction pin is located
  GPIO_TypeDef * StepPort;      // The port where this stepper's step pin is located
  uint16_t StepPin;             // The pin of <StepPort> where this stepper's step pin is located
} SteppersIO_t;

// Structure for all important changing values related to one stepper in the Steppers table
/// Todo: Make some of these volatile and not the whole struct?
typedef struct {
  uint32_t  StepAdd;            // Amount to add to SteppAcc every ISR (determines speed)
  int32_t   StepAddInc;         // (future) For acceleration, how much to add to StepAdd each ISR
  uint32_t  StepAcc;            // Step accumulator
  int32_t   GlobalPosition;     // Current global position in 1/256th macro steps
  uint8_t   Microsteps;         // Current microstep value - same as set by SP command (0 = 256th, 8 = full steps)
} Steppers_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

uint8_t stepper_Step(StepperCommand_t * cmdPtr);
void stepper_SMCommand(void);
void stepper_AMCommand(void);
void stepper_LMCommand(void);
void stepper_HMCommand(void);
void stepper_XMCommand(void);
void stepper_EMCommand(void);
void stepper_QMCommand(void);
void stepper_ESCommand(void);
void stepper_QSCommand(void);
void stepper_CSCommand(void);
uint8_t process_QM(void);
void process_SM(
  uint32_t Duration,
  int32_t A1Stp,
  int32_t A2Stp,
  int32_t A3Stp
);

#endif	/* STEPPER_H */
