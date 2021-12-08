/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        FIFO.h
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
#ifndef __FIFO_H__
#define __FIFO_H__

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Enum that lists each type of command that can be put in the motion control queue */
typedef enum
{
  COMMAND_NONE = 0,
  COMMAND_MOTOR_MOVE, // StepAdd(12), StepAddInc(12), StepsCounter(12), DirBits(1))
  COMMAND_DELAY,      // DelayCounter
  COMMAND_SERVO_MOVE, // ServoRPn, ServoChannel, ServoRate
  COMMAND_SE          // SEState, SEPower
} CommandType;

typedef union {
  int32_t   StepAdd2;
  uint16_t  ServoPosition;
  uint16_t  SEPower;
} StepGeneric1_t;

typedef union {
  uint32_t  StepsCounter0;
  uint8_t   ServoRPn;
  uint32_t  DelayCounter;
  uint8_t   SEState;
} StepGeneric2_t;

typedef union {
  uint32_t  StepsCounter1;
  uint16_t  ServoRate;
} StepGeneric3_t;

typedef union {
  uint32_t  StepsCounter2;
  uint8_t   ServoChannel;
} StepGeneric4_t;

typedef union {
  int32_t   StepAddInc0;
  uint32_t  DelayCounter;
} StepGeneric5_t;

typedef struct StepperCommand
{
  int32_t       StepAdd[NUMBER_OF_STEPPERS];
  int32_t       StepAddInc[NUMBER_OF_STEPPERS];
  uint32_t      StepsCounter[NUMBER_OF_STEPPERS];
  uint8_t       Dir[NUMBER_OF_STEPPERS];
} StepperCommand_t;

struct ServoCommand
{
  uint16_t      ServoPosition;
  uint8_t       ServoPin;
  uint16_t      ServoRate;
};

struct EngraverCommand
{
  uint8_t       SEState;
  uint16_t      SEPower;
};

// This structure defines the elements of the move commands in the queue that
// are sent from the command parser to the ISR move engine.
typedef struct
{
  CommandType   Command;
  uint32_t      DelayCounter;   // NOT Milliseconds! In 100KHz units
  union
  {
    StepperCommand_t   Stepper;
    struct ServoCommand     Servo;
    struct EngraverCommand  Engraver;
  } Data;
} MoveCommandType;  // 27 bytes

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void queue_Init(void);
void queue_WaitForRoom(void);
void queue_WaitForEmpty(void);
void queue_AddStepperCommandToQueue(
    int32_t stepAdd[NUMBER_OF_STEPPERS],
    int32_t stepAddInc[NUMBER_OF_STEPPERS],
    uint32_t stepsCounter[NUMBER_OF_STEPPERS],
    uint8_t dir[NUMBER_OF_STEPPERS]);
void queue_AddServoCommandToQueue(uint16_t position, uint8_t pin, uint16_t rate, uint32_t delay);
void queue_AddEngraverCommandToQueue(uint8_t state, uint16_t power);
void queue_AddDelayCommandToQueue(uint32_t delay);
bool queue_PullNextCommand(MoveCommandType * move);

#endif	/* FIFO_H */
