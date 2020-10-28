

#include "fifo.h"
#include "HardwareProfile.h"

// Working registers

// How many elements of the FIFO will we be using by default?
volatile uint8_t FIFOSize = 3;
// The next element of the FIFO to put a command into
volatile uint8_t FIFOIn = 0;
// The FIFO command we are currently executing
volatile uint8_t FIFOOut = 0;
// The number of commands in the FIFO that are not done being executed yet
volatile uint8_t FIFODepth = 0;

/// MoveCommandType CommandFIFO[COMMAND_FIFO_LENGTH];
// What used to be an array of structures is now separate arrays. Why? To
// get around the 256 byte (1 bank)/variable limitation of C18/PIC18
int32_t           FIFO_StepAdd0[COMMAND_FIFO_LENGTH];
int32_t           FIFO_StepAdd1[COMMAND_FIFO_LENGTH];
StepGeneric1_t  FIFO_G1[COMMAND_FIFO_LENGTH];  // (INT32) StepAdd2, (UINT16) FIFO_ServoPosition, (UINT16) FIFO_SEPower
StepGeneric2_t  FIFO_G2[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCounter0, (UINT8) FIFO_ServoRPn, (UINT8) FIFO_SEState
StepGeneric3_t  FIFO_G3[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCoutner1, (UINT16) FIFO_ServoRate, (UINT8) FIFO_ServoChannel
StepGeneric4_t  FIFO_G4[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCoutner2, (UINT8) FIFO_ServoChannel
StepGeneric5_t  FIFO_G5[COMMAND_FIFO_LENGTH];  // (INT32) StepAddInc0, (UINT32) FIFO_DelayCounter,
int32_t           FIFO_StepAddInc1[COMMAND_FIFO_LENGTH];
int32_t           FIFO_StepAddInc2[COMMAND_FIFO_LENGTH];
uint8_t           FIFO_DirBits[COMMAND_FIFO_LENGTH];
CommandType     FIFO_Command[COMMAND_FIFO_LENGTH];


void fifo_Init(void)
{
  uint8_t i;

  FIFOSize = 1;
  FIFOIn = 0;
  FIFOOut = 0;
  FIFODepth = 0;
  
  // Initialize all FIFO values
  /// TODO : use memset() or something?
  for(i=0; i < COMMAND_FIFO_LENGTH; i++)
  {
    FIFO_Command[i] = COMMAND_NONE;
    FIFO_StepAdd0[i] = 0;
    FIFO_StepAdd1[i] = 0;
    FIFO_G1[i].StepAdd2 = 0;
    FIFO_G2[i].StepsCounter0 = 0;
    FIFO_G3[i].StepsCounter1 = 0;
    FIFO_G4[i].StepsCounter2 = 0;
    FIFO_G5[i].StepAddInc0 = 0;
    FIFO_StepAddInc1[i] = 0;
    FIFO_StepAddInc2[i] = 0;
    FIFO_DirBits[i] = 0;
  }
}

// Wait until FIFODepth has gone below FIFOSize
void WaitForRoomInFIFO(void)
{
  while(FIFODepth >= FIFOSize)
    ;
}

// Wait until FIFODepth has gone to zero
void WaitForEmptyFIFO(void)
{
  while(FIFODepth)
    ;
}

void fifo_Inc(void)
{
  FIFOIn++;
  if (FIFOIn >= COMMAND_FIFO_LENGTH)
  {
    FIFOIn = 0;
  }
  FIFODepth++;
}

