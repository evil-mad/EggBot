

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

/// MoveCommandType Commandqueue[COMMAND_QUEUE_LENGTH];
// What used to be an array of structures is now separate arrays. Why? To
// get around the 256 byte (1 bank)/variable limitation of C18/PIC18
int32_t           queue_StepAdd0[COMMAND_QUEUE_LENGTH];
int32_t           queue_StepAdd1[COMMAND_QUEUE_LENGTH];
StepGeneric1_t  queue_G1[COMMAND_QUEUE_LENGTH];  // (INT32) StepAdd2, (UINT16) queue_ServoPosition, (UINT16) queue_SEPower
StepGeneric2_t  queue_G2[COMMAND_QUEUE_LENGTH];  // (UINT32) StepsCounter0, (UINT8) queue_ServoRPn, (UINT8) queue_SEState
StepGeneric3_t  queue_G3[COMMAND_QUEUE_LENGTH];  // (UINT32) StepsCoutner1, (UINT16) queue_ServoRate, (UINT8) queue_ServoChannel
StepGeneric4_t  queue_G4[COMMAND_QUEUE_LENGTH];  // (UINT32) StepsCoutner2, (UINT8) queue_ServoChannel
StepGeneric5_t  queue_G5[COMMAND_QUEUE_LENGTH];  // (INT32) StepAddInc0, (UINT32) queue_DelayCounter,
int32_t           queue_StepAddInc1[COMMAND_QUEUE_LENGTH];
int32_t           queue_StepAddInc2[COMMAND_QUEUE_LENGTH];
uint8_t           queue_DirBits[COMMAND_QUEUE_LENGTH];
CommandType     queue_Command[COMMAND_QUEUE_LENGTH];


void queue_Init(void)
{
  uint8_t i;

  queueSize = 1;
  queueIn = 0;
  queueOut = 0;
  queueDepth = 0;
  
  // Initialize all queue values
  /// TODO : use memset() or something?
  for(i=0; i < COMMAND_QUEUE_LENGTH; i++)
  {
    queue_Command[i] = COMMAND_NONE;
    queue_StepAdd0[i] = 0;
    queue_StepAdd1[i] = 0;
    queue_G1[i].StepAdd2 = 0;
    queue_G2[i].StepsCounter0 = 0;
    queue_G3[i].StepsCounter1 = 0;
    queue_G4[i].StepsCounter2 = 0;
    queue_G5[i].StepAddInc0 = 0;
    queue_StepAddInc1[i] = 0;
    queue_StepAddInc2[i] = 0;
    queue_DirBits[i] = 0;
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

