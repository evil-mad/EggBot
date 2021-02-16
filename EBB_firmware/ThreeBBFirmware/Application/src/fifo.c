

#include "fifo.h"
#include "HardwareProfile.h"

// Working registers

// How many elements of the queue will we be using by default?
volatile uint8_t queueSize = 3;
// The next element of the queue to put a command into
volatile uint8_t queueIn = 0;
// The queue command we are currently executing
volatile uint8_t queueOut = 0;
// The number of commands in the queue that are not done being executed yet
volatile uint8_t queueDepth = 0;

// The motion command queue
MoveCommandType Queue[COMMAND_QUEUE_LENGTH];

void queue_Init(void)
{
  uint8_t i, j;

  queueSize = 1;
  queueIn = 0;
  queueOut = 0;
  queueDepth = 0;
  
  // Initialize all queue values
  /// TODO : use memset() or something?
  for(i=0; i < COMMAND_QUEUE_LENGTH; i++)
  {
    Queue[i].Command = COMMAND_NONE;
    Queue[i].DelayCounter = 0;
    for (j=0; j < NUMBER_OF_STEPPERS; j++)
    {
      Queue[i].Data.Stepper.StepAdd[j] = 0;
      Queue[i].Data.Stepper.StepsCounter[j] = 0;
      Queue[i].Data.Stepper.StepAddInc[j] = 0;
    }
    Queue[i].Data.Stepper.DirBits = 0;
  }
}

// Wait until QueueDepth has gone below queueSize
void WaitForRoomInQueue(void)
{
  while(queueDepth >= queueSize)
    ;
}

// Wait until queueDepth has gone to zero
void WaitForEmptyQueue(void)
{
  while(queueDepth)
    ;
}

void queue_Inc(void)
{
  queueIn++;
  if (queueIn >= COMMAND_QUEUE_LENGTH)
  {
    queueIn = 0;
  }
  queueDepth++;
}

