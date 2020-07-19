

#include "fifo.h"

// Working registers

//#pragma udata access fast_vars
// How many elements of the FIFO will we be using by default?
volatile UINT8 FIFOSize = 3;
// The next element of the FIFO to put a command into
volatile UINT8 FIFOIn = 0;
// The FIFO command we are currently executing
volatile UINT8 FIFOOut = 0;
// The number of commands in the FIFO that are not done being executed yet
volatile UINT8 FIFODepth = 0;

#pragma udata FIFO=0x800
/// MoveCommandType CommandFIFO[COMMAND_FIFO_LENGTH];
// What used to be an array of structures is now separate arrays. Why? To
// get around the 256 byte (1 bank)/variable limitation of C18/PIC18
INT32           FIFO_StepAdd0[COMMAND_FIFO_LENGTH];
INT32           FIFO_StepAdd1[COMMAND_FIFO_LENGTH];
#pragma udata FIFO2=0x900
StepGeneric1_t  FIFO_G1[COMMAND_FIFO_LENGTH];  // (INT32) StepAdd2, (UINT16) FIFO_ServoPosition, (UINT16) FIFO_SEPower
StepGeneric2_t  FIFO_G2[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCounter0, (UINT8) FIFO_ServoRPn, (UINT8) FIFO_SEState
#pragma udata FIFO3=0xA00
StepGeneric3_t  FIFO_G3[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCoutner1, (UINT16) FIFO_ServoRate, (UINT8) FIFO_ServoChannel
StepGeneric4_t  FIFO_G4[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCoutner2, (UINT8) FIFO_ServoChannel
#pragma udata FIFO4=0xB00
StepGeneric5_t  FIFO_G5[COMMAND_FIFO_LENGTH];  // (INT32) StepAddInc0, (UINT32) FIFO_DelayCounter,
INT32           FIFO_StepAddInc1[COMMAND_FIFO_LENGTH];
#pragma udata FIFO5=0xC00
INT32           FIFO_StepAddInc2[COMMAND_FIFO_LENGTH];
UINT8           FIFO_DirBits[COMMAND_FIFO_LENGTH];
CommandType     FIFO_Command[COMMAND_FIFO_LENGTH];

#pragma udata

void fifo_Init(void)
{
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

