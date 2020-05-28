/* 
 * File:   fifo.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 2:29 PM
 */

#ifndef FIFO_H
#define	FIFO_H

#include "HardwareProfile.h"
#include <GenericTypeDefs.h>

// Maximum number of elements in the command FIFO
#define COMMAND_FIFO_LENGTH     (25)

#define kISR_FIFO_A_DEPTH       3
#define kISR_FIFO_D_DEPTH       3


/* Enum that lists each type of command that can be put in the motion control FIFO */
typedef enum
{
  COMMAND_NONE = 0,
  COMMAND_MOTOR_MOVE,
  COMMAND_DELAY,
  COMMAND_SERVO_MOVE,
  COMMAND_SE
} CommandType;

// This structure defines the elements of the move commands in the FIFO that
// are sent from the command parser to the ISR move engine.
typedef struct
{
  CommandType     Command;
  INT32           StepAdd[NUMBER_OF_STEPPERS];
  INT32           StepAddInc[NUMBER_OF_STEPPERS];
  UINT32          StepsCounter[NUMBER_OF_STEPPERS];
  UINT8           DirBits;
  UINT32          DelayCounter;   // NOT Milliseconds! In 25KHz units
  UINT16          ServoPosition;
  UINT8           ServoRPn;
  UINT8           ServoChannel;
  UINT16          ServoRate;
  UINT8           SEState;
  UINT16          SEPower;
} MoveCommandType;  // 27 bytes


extern volatile UINT8 FIFODepth;
extern volatile UINT8 FIFOSize;
extern volatile UINT8 FIFOIn;
extern volatile UINT8 FIFOOut;
extern volatile unsigned int ISR_A_FIFO[16];                       // Stores the most recent analog conversions

///extern MoveCommandType CommandFIFO[];
extern CommandType     FIFO_Command[COMMAND_FIFO_LENGTH];
extern INT32           FIFO_StepAdd[NUMBER_OF_STEPPERS][COMMAND_FIFO_LENGTH];
extern INT32           FIFO_StepAddInc[NUMBER_OF_STEPPERS][COMMAND_FIFO_LENGTH];
extern UINT32          FIFO_StepsCounter[NUMBER_OF_STEPPERS][COMMAND_FIFO_LENGTH];
extern UINT8           FIFO_DirBits[COMMAND_FIFO_LENGTH];
extern UINT32          FIFO_DelayCounter[COMMAND_FIFO_LENGTH];   // NOT Milliseconds! In 25KHz units
extern UINT16          FIFO_ServoPosition[COMMAND_FIFO_LENGTH];
extern UINT8           FIFO_ServoRPn[COMMAND_FIFO_LENGTH];
extern UINT8           FIFO_ServoChannel[COMMAND_FIFO_LENGTH];
extern UINT16          FIFO_ServoRate[COMMAND_FIFO_LENGTH];
extern UINT8           FIFO_SEState[COMMAND_FIFO_LENGTH];
extern UINT16          FIFO_SEPower[COMMAND_FIFO_LENGTH];



void fifo_Init(void);
void fifo_Inc(void);
void WaitForRoomInFIFO(void);
void WaitForEmptyFIFO(void);

#endif	/* FIFO_H */

