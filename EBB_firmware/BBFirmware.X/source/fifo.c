

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

//volatile unsigned int ISR_A_FIFO[16];                       // Stores the most recent analog conversions



#pragma udata FIFO=0x800
/// MoveCommandType CommandFIFO[COMMAND_FIFO_LENGTH];
// What used to be an array of structures is now separate arrays. Why? To
// get around the 256 byte (1 bank)/variable limitation of C18/PIC18
INT32           FIFO_StepAdd[NUMBER_OF_STEPPERS][COMMAND_FIFO_LENGTH];
UINT16          FIFO_SEPower[COMMAND_FIFO_LENGTH];
#pragma udata FIFO2=0x900
UINT32          FIFO_StepsCounter[NUMBER_OF_STEPPERS][COMMAND_FIFO_LENGTH];
UINT8           FIFO_DirBits[COMMAND_FIFO_LENGTH];
UINT8           FIFO_ServoChannel[COMMAND_FIFO_LENGTH];
#pragma udata FIFO3=0xA00
UINT32          FIFO_DelayCounter[COMMAND_FIFO_LENGTH];   // NOT Milliseconds! In 25KHz units
UINT16          FIFO_ServoPosition[COMMAND_FIFO_LENGTH];
#pragma udata FIFO4=0xB00
UINT16          FIFO_ServoRate[COMMAND_FIFO_LENGTH];
INT32           FIFO_StepAddInc[NUMBER_OF_STEPPERS][COMMAND_FIFO_LENGTH];
#pragma udata FIFO5=0xC00
CommandType     FIFO_Command[COMMAND_FIFO_LENGTH];
UINT8           FIFO_ServoRPn[COMMAND_FIFO_LENGTH];
UINT8           FIFO_SEState[COMMAND_FIFO_LENGTH];

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

