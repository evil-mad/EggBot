/*********************************************************************
 *
 *                EiBotBoard Firmware
 *
 *********************************************************************
 * FileName:        servo.c
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

/*
 * This module implements the RC Servo outputs, method 2
 * 
 * Started by Brian Schmalz (www.schmalzhaus.com) on 8/22/09
 * For the Egg Bot Firmware
 *
 * There are several parts of this file. It is written in a modular
 * way to ease incorporation into different UBW (www.schmalzhaus.com/UBW) 
 * hardware builds. Because we don't want to have any function calls in the ISR,
 * the part of the ISR that this module contributes is implemented as a big
 * macro. There is a section for the API calls (for other modules) to turn
 * on/off RC method 2 and to set values and configure outputs. Then there is the
 * user command section that handles parsing user input. This module (the .c and
 * .h files) should be able to be dropped into any UBW firmware that is 'module
 * aware' with minimal changes. There is also an init section that gets called
 * on bootup.
 *
 * MODULE THEORY
 *
 * The idea is that we want to generate between zero and eight RC servo outputs. 
 * We want a maximum of resolution in their timing, and a minimum of CPU and ISR
 * overhead. We want maximum flexibility with respect to which pins receive the
 * output pulses. This 'method 2' only works with PICs that have the PPS
 * (Peripheral Pin Select) hardware. Using this method, we will be able to
 * generate servo output pulses (positive going) on up to eight output pins
 * (selectable using PPS), with times from 0ms to 3ms, at a repetition rate of
 * 24ms. The resolution of the pulse will be 83ns (Fosc/4). So a value of 0 will
 * represent 0ms pulse, and 36000 will represent 3ms.
 *
 * On UBW firmware, TIMER1 generates an interrupt every 1ms. This interrupt is 
 * used to schedule the four RC servo outputs. Every 3 fires of this ISR, we
 * check to see if we need to set the next RC output pin. If so, we select the
 * proper pin using the PPS, set it high, record what state we are now in, and
 * then set up TMR3 and ECCP2 to fire after X 83ns ticks of TMR3. ECCP2 will
 * wait until TMR3 fires, then will set the output pin low. This method allows
 * us to choose any of the PPS pins as outputs, and only requires CPU time every
 * 3 ms to start off the pulses.
 *
 * As of version 2.2.0, we have changed the way that the S2 command works.
 * Under the hood, ALL servo commands are now handled by the motion engine
 * ISR. This means that the S2 command now simply fills in a data structure
 * and puts it in the motion queue. The motion engine ISR pulls it off the
 * queue and then 'executes' the command. (All it does there is transfer the
 * proper values to the data structures that the actual RC servo ISR uses to
 * update the various output pins for each servo pulse.) But the key here is
 * that the S2 command no longer takes effect immediately - RC servo commands
 * are now handled in the same way that SM commands are handled. Which means
 * they can be queued up and delays added, etc.
 *
 * The other effect of this change is that the normal SP pen up/pen down
 * commands are now just normal S2 commands in disguise.
 */

/************** INCLUDES ******************************************************/

#include <p18cxxx.h>
#include <stdio.h>
#include <ctype.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "ebb.h"
#include "servo.h"
#include "HardwareProfile.h"
#include "fifo.h"
#include "parse.h"
#include "utility.h"
#include "isr.h"
#include "stepper.h"

/************** MODULE DEFINES ************************************************/


// Power on default value for gPenMoveDuration in milliseconds
#define PEN_MOVE_DURATION_DEFAULT_MS  500

#if defined(BOARD_EBB)
#define DEFAULT_PEN_MAX_POSITION_SERVO    15302   // max = down (SC,5,15302)
#define DEFAULT_PEN_MIN_POSITION_SERVO    22565   // min = up (SC,4,22565)
#else
#define DEFAULT_PEN_MAX_POSITION_STEPPER  (200*32/4)  // max = down (SC,5) - 1/4 turn from home
#define DEFAULT_PEN_MIN_POSITION_STEPPER  (200*32/2)  // min = up (SC,4) - 1/2 turn from home
#endif

/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// Counts from 0 to gRC2SlotMS
UINT8  gRC2msCounter;
// Current RC servo position in 83uS units for each channel
UINT16 gRC2Value[MAX_RC2_SERVOS];
// RPn pin associated with this channel
UINT8  gRC2RPn[MAX_RC2_SERVOS];
// Target position for this channel in 83uS units
UINT16 gRC2Target[MAX_RC2_SERVOS];
// Amount of change from Value to Target each 24ms
UINT16 gRC2Rate[MAX_RC2_SERVOS];
// 
UINT8  gRC2Ptr;
// How many RC servos can we currently simultainously service (default 8)
UINT8  gRC2Slots;
// How many 1ms ISR ticks before switching to the next channel (default 3)
UINT8  gRC2SlotMS;
// Pointer into PPS output registers
far ram UINT8 * gRC2RPORPtr;

// Records the current pen state (up/down) in reality
PenStateType gPenStateActual;

// Records the pen state that the commands coming from the PC think the pen is in
// (Prevents duplicate pen up or pen down commands.)
static PenStateType PenStateCommand;


// These are the min, max, and default duration values for SP pen move command
// They can be changed by using SC,x commands.
INT16 gPenMaxPosition;
INT16 gPenMinPosition;
UINT16 gPenMoveDuration;

#if defined(BOARD_EBB)
// Counts down milliseconds until zero. At zero shuts off power to RC servo (via RA3))
volatile UINT32 gRCServoPoweroffCounterMS = 0;
volatile UINT32 gRCServoPoweroffCounterReloadMS = RCSERVO_POWEROFF_DEFAULT_MS;
#endif

/************** LOCAL FUNCTION PROTOTYPES *************************************/

static void PenHome(void);
static UINT8 servo_Move(UINT16 Position, UINT8 RPn, UINT16 Rate, UINT16 Delay);
static UINT8 servo_get_channel_from_RPn(UINT8 RPn);

/************** LOCAL FUNCTIONS ***********************************************/

// Perform all of the things necessary to initialize the pen position.
// Not much to do when using a servo, but for a stepper Motor3 must be 
// driven past the lowest position, limped, a pause, 
// then a move from the home (lowest) position to the bottom position 
// (gPenMaxPosition)
// /// TODO : Once suitable mechanicals have been constructed, convert the
// simplified code below over to what is talked about above (using hardstop
// homing)
static void PenHome(void)
{  
  gPenStateActual = PEN_DOWN;
  PenStateCommand = PEN_DOWN;

  // Execute a move from homed to the lower pen position
  process_SM(500, 0, 0, gPenMaxPosition);
}

// Return the current channel that is associated with the PPS output pin
// RPn. If there is no channel yet assigned for this RPn, then pick the
// next available one. If there are none available, then return channel 0
// (which is considered an error.)
// Remember, channels are from 1 through 8 (Normally - can be increased with 
// SC,8 command). Channel 0 is the 'error' channel.
static UINT8 servo_get_channel_from_RPn(UINT8 RPn)
{
  UINT8 i;

  // Search through the existing channels, and see if our RPn is there
  for (i=0; i < MAX_RC2_SERVOS; i++)
  {
    if (gRC2RPn[i] == RPn)
    {
      // Found it! Return the channel number
      return (i + 1);
    }
  }

  // We have not found it, so we need to allocate a new channel for this RPn
  for (i=0; i < MAX_RC2_SERVOS; i++)
  {
    if (gRC2RPn[i] == 0)
    {
      // Found one that's free! Return the channel number
      return (i + 1);
    }
  }

  // We do not have room for another channel, so return an error
  return 0;
}

// Function to set up an RC Servo move. Takes Duration, RPn, and Rate
// and adds them to the motion control fifo.
// <Position> is the new target position for the servo, in 83uS units. So
//      32,000 is 3ms. To turn off a servo output, use 0 for Duration.
// <RPn> is the PPS RP# number for the pin that you want to use as the output
//      (See schematic for a list of each RPn number for each GPIO pin.)
// <Rate> is how quickly to move to the new position. Use 0 for instant change.
//      Unit is 83uS of pulse width change every 24ms of time.
// <Delay> is how many milliseconds after this command is excuted before the
//      next command in the motion control FIFO is executed. 0 will run the next
//      command immediatly.
// This function will allocate a new channel for RPn if the pin is not already
// assigned to a channel. It will return the channel number used when it
// returns. If you send in 0 for Duration, the channel for RPn will be de-
// allocated.
// Another thing we do here is to make sure that the proper pin is an output,
// And, if this is the first time we're starting up the channel, make sure that
// it starts out low.
static UINT8 servo_Move(
  UINT16 Position,
  UINT8  RPn,
  UINT16 Rate,
  UINT16 Delay
)
{
  UINT8 i;
  UINT8 Channel;

  // Get the channel that's already assigned to the RPn, or assign a new one
  // if possible. If this returns zero, then do nothing as we're out of
  // channels.
  Channel = servo_get_channel_from_RPn(RPn);

  // Error out if there were no available channels left
  if (Channel == 0)
  {
    return 0;
  }

  // If Duration is zero, then caller wants to shut down this channel
  if (0 == Position)
  {
    // Turn off the PPS routing to the pin
    *(gRC2RPORPtr + gRC2RPn[Channel - 1]) = 0;
    gRC2Rate[Channel - 1] = 0;
    gRC2Target[Channel - 1] = 0;
    gRC2RPn[Channel - 1] = 0;
    gRC2Value[Channel - 1] = 0;
  }
  else
  {
    // If we have a valid channel, and RPn, then make the move
    if ((Channel - 1) < gRC2Slots && RPn <= 24)
    {
      // As a special case, if the pin is the same as the pin
      // used for the solenoid, then turn off the solenoid function
      // so that we can output PWM on that pin
      if (RPn == PEN_UP_DOWN_RPN)
      {
        gUseSolenoid = FALSE;
      }

      // Is this the first time we've used this channel?
      if (gRC2Value[Channel - 1] == 0)
      {
        // Make sure the pin is set as an output, or this won't do much good
        SetPinTRISFromRPn(RPn, OUTPUT_PIN);

        // For v2.1.5, found bug where if a pin is HIGH when we start doing
        // RC output, the output is totally messed up. So make sure to set
        // the pin low first.
        SetPinLATFromRPn(RPn, 0);
      }

      // Wait until we have a free spot in the FIFO, and add our new
      // command in
      WaitForRoomInFIFO();

      /// TODO: Move this to FIFO function?
      // Now copy the values over into the FIFO element
      FIFO_Command[FIFOIn] = COMMAND_SERVO_MOVE;
      FIFO_DelayCounter[FIFOIn] = HIGH_ISR_TICKS_PER_MS * (UINT32)Delay;
      FIFO_ServoChannel[FIFOIn] = Channel;
      FIFO_ServoRPn[FIFOIn] = RPn;
      FIFO_ServoPosition[FIFOIn] = Position;
      FIFO_ServoRate[FIFOIn] = Rate;

      fifo_Inc();
    }
  }
  return Channel;
}

/************** GLOBAL FUNCTIONS **********************************************/

/*
The idea with servo is to use the ECCP2 module and timer 3.
We divide time into 24ms periods. Inside each 24ms period, we
can fire up to 8 RC servo's pulses (slots). Each pulse can be between
0ms and 3ms long, controlled entirely by the ECCP2 hardware,
so there is no jitter in the high time of the pulse.

We want to go from 0ms to 3ms so we can accomodate RC servos
who need really short or really long pulses to reach the
physical extremes of its motion.

This servo method will only be available on the 18F45J50 based
EggBotBoards, because it requires the PPS (peripheral pin select)
facility to be possible.

Timer3 will be configured to clock at Fosc/4 = 12MHz.
At this rate, a 1ms high pulse would need a CCPR2 value of 12,000.

Variables:

gRC2msCounter - counts from 0 to 2, in ms, to know when to fire each servo
gRC2Ptr - index into gRC2Value and gRC2Pin arrays = next RC servo to fire
gRC2Value - uint array of values (times) to load into Timer3
gRC2Pin - what PPS pin is affected by this RC servo slot.
gRC2RPORPtr - a RAM pointer into the RPORxx register

If a slot's gRC2Value[] = 0, then that slot is disabled and its pin will be low.
The value in gRC2Pin is the PPS RPx pin number that this slot controls

/*
 * Module Init Function
 *
 * Put a call to this function inside the UserInit() call in UBW.c
 */
void servo_Init(void)
{
  unsigned char i;

  gRC2msCounter = 0;
  gRC2Ptr = 0;
  gPenMoveDuration = PEN_MOVE_DURATION_DEFAULT_MS;

  for (i=0; i < MAX_RC2_SERVOS; i++)
  {
    gRC2Value[i] = 0;
    gRC2RPn[i] = 0;
    gRC2Target[i] = 0;
    gRC2Rate[i] = 0;
  }
  // Initialize the RPOR pointer
  gRC2RPORPtr = &RPOR0;

  // Set up TIMER3
  T3CONbits.TMR3CS = 0b00;    // Use Fosc/4 as input
  T3CONbits.T3CKPS = 0b00;    // Prescale is 1:1
  T3CONbits.RD16 = 1;         // Enable 16 bit mode
  TMR3H = 0;
  TMR3L = 0;
  T3CONbits.TMR3ON = 0;       // Keep timer off for now

  TCLKCONbits.T3CCP1 = 1;     // ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4
  TCLKCONbits.T3CCP2 = 0;     // ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4

  CCP2CONbits.CCP2M = 0b1001; // Set EECP2 as compare, clear output on match

  // We start out with 8 slots because that is good for RC servos (3ms * 8 = 24ms)
  gRC2Slots = INITAL_RC2_SLOTS;

  // We start out with 3ms slot duration because it's good for RC servos
  gRC2SlotMS = 3;

  // Start with some reasonable default values for min and max pen positions
#if defined(BOARD_EBB)
  gPenMaxPosition = DEFAULT_PEN_MAX_POSITION_SERVO;
  gPenMinPosition = DEFAULT_PEN_MIN_POSITION_SERVO;
#else
  gPenMaxPosition = DEFAULT_PEN_MAX_POSITION_STEPPER;
  gPenMinPosition = DEFAULT_PEN_MIN_POSITION_STEPPER;
#endif
  
  PenHome();
  
#if defined(BOARD_EBB)
  RCServoPowerIO = RCSERVO_POWER_OFF;
#endif
}

// Set Pen (modified for v3.0.0 and above firmware)
// Usage: SP,<state>[,duration]<CR>
// <state> is 0 (for goto servo_max) or 1 (for goto servo_min)
// <duration> (optional) is the length of time this move should take in ms 
//  (<duration> is a 16 bit unsigned int)
//  (Note that the global <pen_move_duration> will be used if no parameter
//   value is specified for <duration>.)
//
// This is a command that the user can send from the PC to set the pen state.
//
// Sending an SP command will get it inserted into the motion queue just like
// any other motion command. The SP command will not begin until the previous
// motion command has finished. Thus there will be no stepper movement during
// the pen move. As soon as the SP move is complete, the next command in the 
// motion queue will be executed.
//
// This function will use the values for <serv_min>, <servo_max>,
// (SC,4 SC,5 commands) when it schedules the pen move for the destination
// position.
// 
// Internally, the parse_SP_packet() function makes a call to
// process_SP() function to actually make the change in the servo output.
//
void parse_SP_packet(void)
{
  UINT8 State = 0;
  UINT16 CommandDuration = gPenMoveDuration;

  // Extract each of the values.
  extract_number (kUCHAR, &State, kREQUIRED);
  extract_number (kUINT, &CommandDuration, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (State > 1)
  {
    State = 1;
  }

  // Execute the servo state change
  process_SP(State, CommandDuration);
    
  print_ack();
}

// Toggle Pen
// Usage: TP,<duration><CR>
// Returns: OK<CR>
// <duration> is optional, and defaults to 0mS
// Just toggles state of pen arm, then delays for the optional <duration>
// Duration is in units of 1ms
void parse_TP_packet(void)
{
  UINT16 CommandDuration = gPenMoveDuration;

  // Extract each of the values.
  extract_number (kUINT, &CommandDuration, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (PenStateCommand == PEN_UP)
  {
    process_SP(PEN_DOWN, CommandDuration);
  }
  else
  {
    process_SP(PEN_UP, CommandDuration);
  }

  print_ack();
}

// Helper function :
// Perform a state change on the pen. Move it up or move it down.
// For 3BB, this will result in Motor3 moving to the new position.
// For EBB, the 3rd stepper axis movement is mapped to RB1 as an RC servo pulse
//
// <NewState> is either PEN_UP or PEN_DOWN.
// <CommandDuration> is the number of milliseconds that the move must take
//  (Note: If <CommandDuration> is 0, then the global default value of
//   gPenMoveDuration is used.)
//
// This function uses the gPenMinPosition and gPenMaxPosition as targets for the
// pen's movement. It simply generates an process_SM() call to schedule the
// move.
void process_SP(PenStateType newState, UINT16 commandDuration)
{
  INT16 steps;
  
  if (commandDuration == 0)
  {
    commandDuration = gPenMoveDuration;
  }
  
  // Only send a new command if the pen state is changing from what our last
  // commanded state was. We don't want to send multiple relative moves commands
  // in the same direction to the stepper as it will go out of bounds.
  if (PenStateCommand != newState)
  {
    if (newState == PEN_UP)
    {
      steps = gPenMinPosition-gPenMaxPosition;
    }
    else
    {
      steps = -(gPenMinPosition-gPenMaxPosition);
    }

#if defined(BOARD_EBB)
    RCServoPowerIO = RCSERVO_POWER_ON;
    gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
#endif
  
    // Use the process_SM() command to schedule the move
    process_SM(commandDuration, 0, 0, steps);
    
    // Now that we've sent the move command off to the motion FIFO, record
    // the new commanded pen state
    PenStateCommand = newState;
  }
}

// Servo method 2 enable command
// S2,<duration>,<output_pin>,<rate>,<delay><CR>
//  will set RC output <channel> for <duration> on output pin <output_pin>
//    <duration> can be 0 (output off) to 32,000 (3ms on time)
//      (a 0 for <duration> de-allocates the channel for this output_pin)
//    <output_pin> is an RPn pin number (0 through 24)
//    <rate> is the rate to change (optional, defaults to 0 = instant)
//    <delay> is the number of milliseconds to delay the start of the next command
//      (optional, defaults to 0 = instant)

void servo_S2_command (void)
{
  UINT16 Duration = 0;
  UINT8 Pin = 0;
  UINT16 Rate = 0;
  UINT16 Delay = 0;

  // Extract each of the values.
  extract_number (kUINT, &Duration, kOPTIONAL);
  extract_number (kUCHAR, &Pin, kOPTIONAL);
  extract_number (kUINT, &Rate, kOPTIONAL);
  extract_number (kUINT, &Delay, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (Pin > 24)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  servo_Move(Duration, Pin, Rate, Delay);

  print_ack();
}

