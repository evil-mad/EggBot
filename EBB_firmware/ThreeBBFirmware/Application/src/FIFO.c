/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        FIFO.c
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

#include "fifo.h"
#include <string.h>
#include "HardwareProfile.h"

/************** PRIVATE TYPEDEFS **********************************************/

/************** PRIVATE DEFINES ***********************************************/

/************** MODULE DEFINES ************************************************/

// Maximum size of queue in elements
#define DEFAULT_QUEUE_SIZE          64

#define MAX_QUEUE_SIZE              100

/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// How many elements of the queue will we be using by default?
static volatile uint8_t QueueSize = DEFAULT_QUEUE_SIZE;
// The next element of the queue to put a command into
static volatile uint8_t QueueIn;
// The queue command we are currently executing
static volatile uint8_t QueueOut;
// The number of commands in the queue that are not done being executed yet
static volatile uint8_t QueueDepth;

// The motion command queue
static MoveCommand_t Queue[MAX_QUEUE_SIZE];

/************** PRIVATE FUNCTION PROTOTYPES ***********************************/

static void incrementQueue(void);


/************** PRIVATE FUNCTIONS *********************************************/

// After adding a command to the queue, move the input pointer ahead one
static void incrementQueue(void)
{
  QueueIn++;
  if (QueueIn >= MAX_QUEUE_SIZE)
  {
    QueueIn = 0;
  }
  QueueDepth++;
}


/************** PUBLIC FUNCTIONS **********************************************/

uint8_t queue_GetSize(void)
{
  return QueueDepth;
}

void queue_Init(void)
{
  QueueSize = DEFAULT_QUEUE_SIZE;
  QueueIn = 0;
  QueueOut = 0;
  QueueDepth = 0;
  
  // Clear the queue out
  memset(Queue, 0x00, sizeof(Queue));
}

// Wait until there is at least one empty spot in the motion queue
void queue_WaitForRoom(void)
{
  while(QueueDepth >= QueueSize)
    ;
}

// Wait until the motion queue is completely drained
void queue_WaitForEmpty(void)
{
  while(QueueDepth)
    ;
}

// Add a new stepper motion command to the motion queue
void queue_AddStepperCommandToQueue(
    int32_t stepAdd[NUMBER_OF_STEPPERS],
    int32_t stepAddInc[NUMBER_OF_STEPPERS],
    uint32_t stepsCounter[NUMBER_OF_STEPPERS],
    uint8_t dir[NUMBER_OF_STEPPERS])
{
  uint8_t i;

  queue_WaitForRoom();

  for (i=0; i < NUMBER_OF_STEPPERS; i++)
  {
    Queue[QueueIn].Data.Stepper.StepAdd[i] = stepAdd[i];
    Queue[QueueIn].Data.Stepper.StepsCounter[i] = stepsCounter[i];
    Queue[QueueIn].Data.Stepper.StepAddInc[i] = stepAddInc[i];
    Queue[QueueIn].Data.Stepper.Dir[i] = dir[i];
  }
  Queue[QueueIn].DelayCounter = 0;
  Queue[QueueIn].Command = COMMAND_MOTOR_MOVE;

  incrementQueue();
}

// Add a new servo motion command to the motion queue
void queue_AddServoCommandToQueue(uint16_t position, uint8_t pin, uint16_t rate, uint32_t delay)
{
  queue_WaitForRoom();

  Queue[QueueIn].Data.Servo.ServoPosition = position;
  Queue[QueueIn].Data.Servo.ServoRate = rate;
  Queue[QueueIn].Data.Servo.ServoPin = pin;
  Queue[QueueIn].DelayCounter = delay;
  Queue[QueueIn].Command = COMMAND_SERVO_MOVE;

  incrementQueue();
}

// Add a new engraver motion command to the motion queue
void queue_AddEngraverCommandToQueue(uint8_t state, uint16_t power)
{
  queue_WaitForRoom();

  Queue[QueueIn].Data.Engraver.SEPower = power;
  Queue[QueueIn].Data.Engraver.SEState = state;
  Queue[QueueIn].DelayCounter = 0;
  Queue[QueueIn].Command = COMMAND_SE;

  incrementQueue();
}


// Add a new delay command to the motion queue
void queue_AddDelayCommandToQueue(uint32_t delay)
{
  queue_WaitForRoom();

  Queue[QueueIn].DelayCounter = delay;
  Queue[QueueIn].Command = COMMAND_DELAY;

  incrementQueue();
}

// Try to pull a command off the queue. If there are none, then return false. Otherwise return true.
// If there is one available, copy it into 'move'.
bool queue_PullNextCommand(MoveCommand_t * move)
{
  if (QueueDepth)
  {
    // Copy over the move structure data
    *move = Queue[QueueOut];

    QueueOut++;
    if (QueueOut >= MAX_QUEUE_SIZE)
    {
      QueueOut = 0;
    }
    QueueDepth--;

    return true;
  }
  else
  {
    return false;
  }
}

