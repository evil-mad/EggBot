/*********************************************************************
 *
 *                EiBotBoard Firmware
 *
 *********************************************************************
 * FileName:        ebb.h
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Based on original files by Microchip Inc. in MAL USB example.
 *
 * Software License Agreement
 *
 * Copyright (c) 2014, Brian Schmalz of Schmalz Haus LLC
 * All rights reserved.
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

#ifndef EBB_H
#define EBB_H

// Enable this line to compile with a lot of debug prints for motion commands
//#define DEBUG_VALUE_PRINT

// Define this to turn on some GPIO pin timing debug for stepper commands
//#define GPIO_DEBUG

// 	These are used for Enable<X>IO to control the enable lines for the driver
#define ENABLE_MOTOR        0
#define DISABLE_MOTOR       1

// How many stepper motors does this board support? (EBB is always 2)
#define NUMBER_OF_STEPPERS  2

typedef enum
{
	PEN_DOWN = 0,
	PEN_UP
} PenStateType;

/* Enum that lists each type of command that can be put in the motion control FIFO */
typedef enum
{
	COMMAND_NONE = 0,
	COMMAND_MOTOR_MOVE,
	COMMAND_DELAY,
	COMMAND_SERVO_MOVE,
  COMMAND_SE,
  COMMAND_MOTOR_MOVE_TIMED
} CommandType;

// Byte union used for accumulator (unsigned))
typedef union union32b4 {
  struct byte_map {
      UINT8 b1; // Low byte
      UINT8 b2;
      UINT8 b3;
      UINT8 b4; // High byte
  } bytes;
  UINT32 value;
} u32b4_t;

// Byte union used for rate (signed)
typedef union union32b4 {
  struct byte_map {
      UINT8 b1; // Low byte
      UINT8 b2;
      UINT8 b3;
      UINT8 b4; // High byte
  } bytes;
  INT32 value;
} uS32b4_t;

// This structure defines the elements of the move commands in the FIFO that
// are sent from the command parser to the ISR move engine.
typedef struct
{
  CommandType     Command;
  uS32b4_t        Rate[NUMBER_OF_STEPPERS];
  INT32           Accel[NUMBER_OF_STEPPERS];
  UINT32          Steps[NUMBER_OF_STEPPERS];
  UINT8           DirBits;
  UINT32          DelayCounter;   // NOT Milliseconds! In 25KHz units
  UINT16          ServoPosition;
  UINT8           ServoRPn;
  UINT8           ServoChannel;
  UINT16          ServoRate;
  UINT8           SEState;
  UINT16          SEPower;
  UINT8           Active[NUMBER_OF_STEPPERS];
} MoveCommandType;

// Define global things that depend on the board type
#define STEP1_BIT	(0x01)
#define DIR1_BIT	(0x02)
#define STEP2_BIT	(0x04)
#define DIR2_BIT	(0x08)

#define NUMBER_OF_STEPPERS  2

// Reload value for TIMER1
// We need a 25KHz ISR to fire, so we take Fosc (48Mhz), divide by 4
// (normal CPU instruction rate of Fosc/4)
// Then we use a reload value of 480 (0x1E0) to give us
// a rate of 48MHz/4/480 = 25KHz.
// Note that because we can't reload the timer _exactly_ after it fires,
// we have to decrease our 480 value by a few to account for the instructions
// that happen after the timer fires but before we can reload the timer with new
// values.
// The values here are hand tuned for 25KHz ISR operation
// 0xFFFF - 0x01E0 = 0xFE1F
#define TIMER1_L_RELOAD (61)  // 0x3D
#define TIMER1_H_RELOAD (254) // 0xFE
//#define TIMER1_L_RELOAD (0x3F)
//#define TIMER1_H_RELOAD (0xED)


#define HIGH_ISR_TICKS_PER_MS (25)  // Note: computed by hand, could be formula


extern MoveCommandType CommandFIFO[];
extern unsigned int DemoModeActive;
extern BOOL FIFOEmpty;
extern unsigned int comd_counter;
extern unsigned char QC_ms_timer;
extern BOOL gLimitChecks;

// Default to on, comes out on pin RB4 for EBB v1.3 and above
extern BOOL gUseSolenoid;
void parse_SM_packet(void);
void parse_SC_packet(void);
void parse_SP_packet(void);
void parse_TP_packet(void);
void parse_QP_packet(void);
void parse_SN_packet(void);
void parse_QN_packet(void);
void parse_NI_packet(void);
void parse_ND_packet(void);
void parse_SL_packet(void);
void parse_QL_packet(void);
void parse_QB_packet(void);
void parse_EM_packet(void);
void parse_QC_packet(void);
void parse_QG_packet(void);
void parse_SE_packet(void);
void parse_RM_packet(void);
void parse_QM_packet(void);
void parse_ES_packet(void);
void parse_XM_packet(void);
void parse_QS_packet(void);
void parse_CS_packet(void);
void parse_LM_packet(void);
void parse_LT_packet(void);
void parse_HM_packet(void);
void EBB_Init(void);
void process_SP(PenStateType NewState, UINT16 CommandDuration);
#endif