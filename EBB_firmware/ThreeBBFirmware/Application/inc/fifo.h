/* 
 * File:   fifo.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 2:29 PM
 */

#ifndef QUEUE_H
#define	QUEUE_H

#include <stdbool.h>
#include <stdint.h>
#include "HardwareProfile.h"

// Bits used within the OutByte to keep track of what direction and step bits need to be at the end of the ISR
// (Not tied to physical pins at all)
#define STEP1_BIT (0x01)
#define DIR1_BIT  (0x02)
#define STEP2_BIT (0x04)
#define DIR2_BIT  (0x08)
#define STEP3_BIT (0x10)
#define DIR3_BIT  (0x20)

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

// This structure defines the elements of the move commands in the queue that
// are sent from the command parser to the ISR move engine.
typedef struct
{
  CommandType     Command;
  int32_t           StepAdd[NUMBER_OF_STEPPERS];
  int32_t           StepAddInc[NUMBER_OF_STEPPERS];
  uint32_t          StepsCounter[NUMBER_OF_STEPPERS];
  uint8_t           DirBits;
  uint32_t          DelayCounter;   // NOT Milliseconds! In 25KHz units
  uint16_t          ServoPosition;
  uint8_t           ServoRPn;
  uint8_t           ServoChannel;
  uint16_t          ServoRate;
  uint8_t           SEState;
  uint16_t          SEPower;
} MoveCommandType;  // 27 bytes


extern volatile uint8_t queueDepth;
extern volatile uint8_t queueSize;
extern volatile uint8_t queueIn;
extern volatile uint8_t queueOut;


extern int32_t           queue_StepAdd0[COMMAND_QUEUE_LENGTH];
extern int32_t           queue_StepAdd1[COMMAND_QUEUE_LENGTH];
extern StepGeneric1_t  queue_G1[COMMAND_QUEUE_LENGTH];  // (INT32) StepAdd2, (UINT16) FIFO_ServoPosition, (UINT16) FIFO_SEPower
extern StepGeneric2_t  queue_G2[COMMAND_QUEUE_LENGTH];  // (UINT32) StepsCounter0, (UINT8) FIFO_ServoRPn, (UINT32) FIFO_DelayCounter, (UINT8) FIFO_SEState
extern StepGeneric3_t  queue_G3[COMMAND_QUEUE_LENGTH];  // (UINT32) StepsCoutner1, (UINT16) FIFO_ServoRate
extern StepGeneric4_t  queue_G4[COMMAND_QUEUE_LENGTH];  // (UINT32) StepsCoutner2, (UINT8) FIFO_ServoChannel
extern StepGeneric5_t  queue_G5[COMMAND_QUEUE_LENGTH];  // (INT32) StepAddInc0, (UINT32) FIFO_DelayCounter,
extern int32_t           queue_StepAddInc1[COMMAND_QUEUE_LENGTH];
extern int32_t           queue_StepAddInc2[COMMAND_QUEUE_LENGTH];
extern uint8_t           queue_DirBits[COMMAND_QUEUE_LENGTH];
extern CommandType     queue_Command[COMMAND_QUEUE_LENGTH];

void queue_Init(void);
void queue_Inc(void);
void WaitForRoomInQueue(void);
void WaitForEmptyQueue(void);

#endif	/* FIFO_H */

