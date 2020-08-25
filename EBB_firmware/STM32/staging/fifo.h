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

// Bits used within the OutByte to keep track of what direction and step bits need to be at the end of the ISR
// (Not tied to physical pins at all)
#define STEP1_BIT (0x01)
#define DIR1_BIT  (0x02)
#define STEP2_BIT (0x04)
#define DIR2_BIT  (0x08)
#define STEP3_BIT (0x10)
#define DIR3_BIT  (0x20)

/* Enum that lists each type of command that can be put in the motion control FIFO */
typedef enum
{
  COMMAND_NONE = 0,
  COMMAND_MOTOR_MOVE, // StepAdd(12), StepAddInc(12), StepsCounter(12), DirBits(1))
  COMMAND_DELAY,      // DelayCounter
  COMMAND_SERVO_MOVE, // ServoRPn, ServoChannel, ServoRate
  COMMAND_SE          // SEState, SEPower
} CommandType;

typedef union {
  INT32   StepAdd2;
  UINT16  ServoPosition;
  UINT16  SEPower;
} StepGeneric1_t;

typedef union {
  UINT32  StepsCounter0;
  UINT8   ServoRPn;
  UINT32  DelayCounter;
  UINT8   SEState;
} StepGeneric2_t;

typedef union {
  UINT32  StepsCounter1;
  UINT16  ServoRate;
} StepGeneric3_t;

typedef union {
  UINT32  StepsCounter2;
  UINT8   ServoChannel;
} StepGeneric4_t;

typedef union {
  INT32   StepAddInc0;
  UINT32  DelayCounter;
} StepGeneric5_t;

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


extern INT32           FIFO_StepAdd0[COMMAND_FIFO_LENGTH];
extern INT32           FIFO_StepAdd1[COMMAND_FIFO_LENGTH];
extern StepGeneric1_t  FIFO_G1[COMMAND_FIFO_LENGTH];  // (INT32) StepAdd2, (UINT16) FIFO_ServoPosition, (UINT16) FIFO_SEPower
extern StepGeneric2_t  FIFO_G2[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCounter0, (UINT8) FIFO_ServoRPn, (UINT32) FIFO_DelayCounter, (UINT8) FIFO_SEState
extern StepGeneric3_t  FIFO_G3[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCoutner1, (UINT16) FIFO_ServoRate
extern StepGeneric4_t  FIFO_G4[COMMAND_FIFO_LENGTH];  // (UINT32) StepsCoutner2, (UINT8) FIFO_ServoChannel
extern StepGeneric5_t  FIFO_G5[COMMAND_FIFO_LENGTH];  // (INT32) StepAddInc0, (UINT32) FIFO_DelayCounter,
extern INT32           FIFO_StepAddInc1[COMMAND_FIFO_LENGTH];
extern INT32           FIFO_StepAddInc2[COMMAND_FIFO_LENGTH];
extern UINT8           FIFO_DirBits[COMMAND_FIFO_LENGTH];
extern CommandType     FIFO_Command[COMMAND_FIFO_LENGTH];

void fifo_Init(void);
void fifo_Inc(void);
void WaitForRoomInFIFO(void);
void WaitForEmptyFIFO(void);

#endif	/* FIFO_H */

