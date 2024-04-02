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
 * Copyright (c) 2014-2023, Brian Schmalz of Schmalz Haus LLC
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

#include <usart.h>

/*
 *  Bitfield defines for the TestModes variable.
 */
// Set to use RC0 as indicator that a command is being parsed
#define TEST_MODE_PARSING_COMMAND_NUM     0u
#define TEST_MODE_PARSING_COMMAND_BIT     (1u)
// Set to make D0, D1, A1 outputs for Next Command, In ISR and FIFO Empty
#define TEST_MODE_GPIO_NUM                1u
#define TEST_MODE_GPIO_BIT                (1u << TEST_MODE_GPIO_BIT_NUM)
// Set for printing ISR debug info to UART at end of every move
#define TEST_MODE_USART_ISR_NUM           2u
#define TEST_MODE_USART_ISR_BIT           (1u << TEST_MODE_USART_ISR_BIT_NUM)
// Set this and TEST_MODE_USART_ISR_BIT_NUM for printing every ISR, not just end of move
#define TEST_MODE_USART_ISR_FULL_NUM      3u
#define TEST_MODE_USART_ISR_FULL_BIT      (1u << TEST_MODE_USART_ISR_FULL_BIT_NUM)
// Prints every received byte out to debug UART
#define TEST_MODE_USART_COMMAND_NUM       4u
#define TEST_MODE_USART_COMMAND_BIT       (1u << TEST_MODE_USART_COMMAND_BIT_NUM)
// Prints additional command debugging info to USB back to PC
#define TEST_MODE_DEBUG_COMMAND_NUM       5u
#define TEST_MODE_DEBUG_COMMAND_BIT       (1u << TEST_MODE_DEBUG_COMMAND_BIT_NUM)
// When 1, commands are parsed but not sent to FIFO. 0 (default) has commands go to FIFO
#define TEST_MODE_DEBUG_BLOCK_FIFO_NUM    6u
#define TEST_MODE_DEBUG_BLOCK_FIFO_BIT    (1u << TEST_MODE_DEBUG_BLOCK_FIFO_NUM)
// This last bit is used during the ISR and is not available as a general test mode bit
#define TEST_MODE_PRINT_TRIGGER_NUM       7u
#define TEST_MODE_PRINT_TRIGGER_BIT       (1u << TEST_MDOE_PRINT_TRIGGER_BIT_NUM)

// 	These are used for Enable<X>IO to control the enable lines for the driver
#define ENABLE_MOTOR        0u
#define DISABLE_MOTOR       1u

// How many stepper motors does this board support? (EBB is always 2)
#define NUMBER_OF_STEPPERS  2u

// Maximum number of elements in the command FIFO (5 is largest we can have
// in one bank). With the FIFO_scn in the linker file going from 0x600 to 0xE00
// there is 2048 bytes of RAM available for the FIFO. At 47 bytes for each 
// element (command) that gives us a maximum of 43 for the FIFO depth.
// Because we want to leave some room for FIFO command expansion in the future,
// we artifically set this to 32 elements. That gives us room to grow the size
// of the command structure without needing to decrease the maximum size
// of the FIFO in elements.
#define COMMAND_FIFO_MAX_LENGTH     32u

// The total number of bytes of space we want the FIFO to be (i.e. reserve
// maximum 2048 bytes, even if we're not using all of it)
#define COMMAND_FIFO_SIZE_BYTES 0x800


typedef enum
{
  PEN_DOWN =                0,
  PEN_UP =                  1,
  PEN_UP_IMMEDIATE =        2,
  PEN_UP_IMMEDIATE_CLEAR =  3
} PenStateType;

typedef enum
{
  SOLENOID_OFF = 0,
  SOLENOID_ON,
  SOLENOID_PWM
} SolenoidStateType;


// Defines for the CommandType BYTE in the MoveCommandType
// Note that three USB commands (SM, XM HM) are
// all the same and represented by one command value.
#define COMMAND_NONE                  0u
#define COMMAND_DELAY                 1u
#define COMMAND_SERVO_MOVE            2u
#define COMMAND_SE                    3u
#define COMMAND_EM                    4u
#define COMMAND_SM_XM_HM_MOVE         5u
#define COMMAND_LM_MOVE               6u
#define COMMAND_LT_MOVE               7u
#define COMMAND_CM_OUTER_MOVE         8u
#define COMMAND_CM_INNER_MOVE         9u

typedef enum
{
  PIC_CONTROLS_DRIVERS = 0,
  PIC_CONTROLS_EXTERNAL,
  EXTERNAL_CONTROLS_DRIVERS
} DriverConfigurationType;

// Byte union used for accumulator (unsigned))
typedef union union32b4 {
  struct byte_map {
    UINT8 b1; // Low bytegTmpIntervals =
    UINT8 b2;
    UINT8 b3;
    UINT8 b4; // High byte
  } bytes;
  UINT32 value;
  UINT32 circle_accum;
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
} s32b4_t;

// This structure defines the elements of the move commands in the FIFO that
// are sent from the command parser to the ISR move engine.
// Currently 47 bytes long
typedef struct
{
  UINT8           Command;                        // SM DL S2 SE EM LM LT
  union {
    struct {
                                                      // Used in which commands? (SM = SM/XM/HM, DL = Delay, S2 = any servo move)
  UINT8           DirBits;                        // SM          EM LM LT
  UINT32          DelayCounter;                   // SM DL S2 SE    LM LT   NOT Milliseconds! In 25KHz units
  UINT8           SEState;                        // SM       SE    LM LT
  s32b4_t         Rate[NUMBER_OF_STEPPERS];       // SM             LM LT
  UINT32          Steps[NUMBER_OF_STEPPERS];      // SM             LM LT
  INT32           Jerk[NUMBER_OF_STEPPERS];       //                LM LT
  INT32           Accel[NUMBER_OF_STEPPERS];      //                LM LT
  UINT8           ServoRPn;                       //       S2    EM LM LT
  UINT16          ServoPosition;                  //       S2
  UINT8           ServoChannel;                   //       S2
  UINT16          ServoRate;                      //       S2
  UINT16          SEPower;                        //          SE
    } sm;
    struct {  // Currently 39 bytes long
      UINT8           DirBits;
      s32b4_t         Rate[NUMBER_OF_STEPPERS];
      UINT32          Steps[NUMBER_OF_STEPPERS];
      UINT16          VScaleK;
      UINT8           m_alpha;
      INT8            bits_left;
      INT16           x_f;
      INT16           y_f;
      INT32           x_t;
      INT32           y_t;
      UINT8           direction;
      INT16           x_pos_last;
      INT16           y_pos_last;
      UINT8           typ_seg;
    } cm;  
  } m;
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

// Bitfield used in motion commands for communicating accumulator handling
// messages to ISR
#define SESTATE_CLEAR_ACC1_BIT      0x01
#define SESTATE_CLEAR_ACC2_BIT      0x02
#define SESTATE_NEGATE_ACC1_BIT     0x04
#define SESTATE_NEGATE_ACC2_BIT     0x08
#define SESTATE_ARBITRARY_ACC1_BIT  0x10
#define SESTATE_ARBITRARY_ACC2_BIT  0x20

// Reload value for TIMER0
// We need a 25KHz ISR to fire, so we take Fosc (48Mhz) and divide by 4
// (normal CPU instruction rate of Fosc/4). We then set up Timer0 so that it
// has a 1:4 clock prescaler. Thus Timer0 is being clocked by a 
// 3MHz clock. We use a reload value of 0x8A to give us an ISR rate of 
// 48MHz/4/4/120 = 25KHz. In order to get the timer to count 120 clocks,
// we give it a reload value of 256-120=136 (since it's a count up timer)
// but we also need to add 2 timer counts to bring it to 138 in order to account
// for the additional time required to get into the ISR and read out the value
// of the timer in order to compute the next reload value.

#define TIMER0_RELOAD 0x8A

#define HIGH_ISR_TICKS_PER_MS (25u)  // Note: computed by hand, could be formula

// A define to wait until the transmit register is empty, the print one byte
#define PrintChar(print_val)          \
  while(Busy1USART())                 \
  { }                                 \
  TXREG1 = print_val;


// A define to print out a 32-bit hex value as eight ASCII characters
#define HexPrint(print_val)           \
        xx.value = print_val;         \
        nib = xx.bytes.b4 >> 4;       \
        if (nib <= 9u)                \
        {                             \
          PrintChar(nib + '0')        \
        }                             \
        else                          \
        {                             \
          PrintChar(nib + 'A' - 10)   \
        }                             \
        nib = xx.bytes.b4 & 0x0F;     \
        if (nib <= 9u)                \
        {                             \
          PrintChar(nib + '0')        \
        }                             \
        else                          \
        {                             \
          PrintChar(nib + 'A' - 10)   \
        }                             \
        nib = xx.bytes.b3 >> 4;       \
        if (nib <= 9u)                \
        {                             \
          PrintChar(nib + '0')        \
        }                             \
        else                          \
        {                             \
          PrintChar(nib + 'A' - 10)   \
        }                             \
        nib = xx.bytes.b3 & 0x0F;     \
        if (nib <= 9u)                \
        {                             \
          PrintChar(nib + '0')        \
        }                             \
        else                          \
        {                             \
          PrintChar(nib + 'A' - 10)   \
        }                             \
        nib = xx.bytes.b2 >> 4;       \
        if (nib <= 9u)                \
        {                             \
          PrintChar(nib + '0')        \
        }                             \
        else                          \
        {                             \
          PrintChar(nib + 'A' - 10)   \
        }                             \
        nib = xx.bytes.b2 & 0x0F;     \
        if (nib <= 9u)                \
        {                             \
          PrintChar(nib + '0')        \
        }                             \
        else                          \
        {                             \
          PrintChar(nib + 'A' - 10)   \
        }                             \
        nib = xx.bytes.b1 >> 4;       \
        if (nib <= 9u)                \
        {                             \
          PrintChar(nib + '0')        \
        }                             \
        else                          \
        {                             \
          PrintChar(nib + 'A' - 10)   \
        }                             \
        nib = xx.bytes.b1 & 0x0F;     \
        if (nib <= 9u)                \
        {                             \
          PrintChar(nib + '0')        \
        }                             \
        else                          \
        {                             \
          PrintChar(nib + 'A' - 10)   \
        }                             


// A define to print out a 32-bit hex value in binary (4 bytes), MSB first
#define BinPrint(print_val)           \
        xx.value = print_val;         \
        PrintChar(xx.bytes.b4)        \
        PrintChar(xx.bytes.b3)        \
        PrintChar(xx.bytes.b2)        \
        PrintChar(xx.bytes.b1)        \


extern near MoveCommandType * FIFOPtr;
extern near volatile UINT8 gFIFOLength;
extern near volatile UINT8 gFIFOIn;
extern near volatile UINT8 gFIFOOut;
extern unsigned int DemoModeActive;
extern unsigned int comd_counter;
extern unsigned char QC_ms_timer;
extern BOOL gLimitChecks;
extern volatile near UINT8 TestMode;
extern volatile near UINT8 gLimitSwitchMask;
extern volatile near UINT8 gLimitSwitchTarget;
extern volatile near UINT8 gLimitSwitchTriggered;
extern UINT8 gStandardizedCommandFormat;
extern volatile near UINT8 gCurrentFIFOLength;
extern near DriverConfigurationType DriverConfiguration;
extern near u32b4_t acc_union[2];
extern UINT16 Sqrt(UINT32 val);

// Externs for assembly square root function
extern UINT8 ARGA0;
extern UINT8 ARGA1;
extern UINT8 ARGA2;
extern UINT8 ARGA3;
extern UINT8 RES0;
extern UINT8 RES1;

extern UINT16 Sqrt(UINT32 val);

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
void parse_QM_packet(void);
void parse_ES_packet(void);
void parse_XM_packet(void);
void parse_QS_packet(void);
void parse_CS_packet(void);
void parse_LM_packet(void);
void parse_LT_packet(void);
void parse_HM_packet(void);
void parse_T3_packet(void);
void parse_L3_packet(void);
void parse_CM_packet(void);
void parse_TR_packet(void);
void EBB_Init(void);
void process_SP(PenStateType NewState, UINT16 CommandDuration);
UINT8 process_QM(void);
#endif