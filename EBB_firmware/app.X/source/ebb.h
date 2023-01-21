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
#define GPIO_DEBUG

// Define this to output every byte received from PC out UART1 (TX1 = RC6)
// for debugging PC comms.
#define UART_OUTPUT_DEBUG

// 	These are used for Enable<X>IO to control the enable lines for the driver
#define ENABLE_MOTOR        0u
#define DISABLE_MOTOR       1u

// How many stepper motors does this board support? (EBB is always 2)
#define NUMBER_OF_STEPPERS  2u

typedef enum
{
  PEN_DOWN = 0,
  PEN_UP
} PenStateType;

// Bitfield defines the CommandType BYTE in the MoveCommandType
// We use bits now in this byte in order to trigger a command,
// one bit per command. Thus we can only have 8 total commands that are passed
// through the FIFO to the ISR. Note that three USB commands (SM, XM HM) are
// all the same and represented by one bit.
// Only one bit must be set at a time or undefined behavior will occur.
// If no bits are set, then the command is effectively COMMAND_NONE.
#define COMMAND_DELAY_BIT             0u
#define COMMAND_SERVO_MOVE_BIT        1u
#define COMMAND_SE_BIT                2u
#define COMMAND_EM_BIT                3u
#define COMMAND_SM_XM_HM_MOVE_BIT     4u
#define COMMAND_LM_MOVE_BIT           5u
#define COMMAND_LT_MOVE_BIT           6u

#define COMMAND_NONE                  0u
#define COMMAND_DELAY                 1u
#define COMMAND_SERVO_MOVE            (1u << COMMAND_SERVO_MOVE_BIT)
#define COMMAND_SE                    (1u << COMMAND_SE_BIT)
#define COMMAND_EM                    (1u << COMMAND_EM_BIT)
#define COMMAND_SM_XM_HM_MOVE         (1u << COMMAND_SM_XM_HM_MOVE_BIT)
#define COMMAND_LM_MOVE               (1u << COMMAND_LM_MOVE_BIT)
#define COMMAND_LT_MOVE               (1u << COMMAND_LT_MOVE_BIT)

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
{                                                 // Used in which commands? (SM = SM/XM/HM, DL = Delay, S2 = any servo move)
  UINT8           Command;                        // SM DL S2 SE EM LM LT
  uS32b4_t        Rate[NUMBER_OF_STEPPERS];       // SM             LM LT
  INT32           Accel[NUMBER_OF_STEPPERS];      //                LM LT
  UINT32          Steps[NUMBER_OF_STEPPERS];      // SM             LM LT
  UINT8           DirBits;                        // SM          EM LM LT
  UINT32          DelayCounter;                   // SM DL S2 SE    LM LT   NOT Milliseconds! In 25KHz units
  UINT16          ServoPosition;                  //       S2
  UINT8           ServoRPn;                       //       S2    EM LM LT
  UINT8           ServoChannel;                   //       S2
  UINT16          ServoRate;                      //       S2
  UINT8           SEState;                        // SM       SE    LM LT
  UINT16          SEPower;                        //          SE
  UINT32          TicksToFlip[NUMBER_OF_STEPPERS];//                LM LT
} MoveCommandType;

// Define global things that depend on the board type
// STEP2 = RD4
#define STEP2_BIT_NUM   4u
#define STEP2_BIT       (1u << STEP2_BIT_NUM)
// DIR2 = RD5
#define DIR2_BIT_NUM    5u
#define DIR2_BIT        (1u << DIR2_BIT_NUM)
// STEP1 = RD6
#define STEP1_BIT_NUM   6u
#define STEP1_BIT       (1u << STEP1_BIT_NUM)
// DIR1 = RD7
#define DIR1_BIT_NUM    7u
#define DIR1_BIT        (1u << DIR1_BIT_NUM)

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
#define TIMER1_L_RELOAD (61u)  // 0x3D
#define TIMER1_H_RELOAD (254u) // 0xFE
//#define TIMER1_L_RELOAD (0x3F)
//#define TIMER1_H_RELOAD (0xED)


#define HIGH_ISR_TICKS_PER_MS (25u)  // Note: computed by hand, could be formula


extern MoveCommandType CommandFIFO[];
extern unsigned int DemoModeActive;
extern UINT8 FIFOEmpty;
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
void parse_QE_packet(void);
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