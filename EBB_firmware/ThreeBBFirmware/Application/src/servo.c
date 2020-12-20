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
 * This module implements the RC Servo outputs. The 3BB implements RC Servo
 * outputs differently than the EBB. It uses dedicated timer channels instead of
 * a single timer and an ISR to switch signals between outputs. Also, on the
 * 3BB, there are only 6 possible RC servo outputs rather than 8 as on the EBB.
 *
 * 3BB Servo Outputs and Timer Channels
 * 
 * MCU Pin  Timer  Channel  Silk Screen
 * ------------------------------------
 * PC6      TIM8   CH1      P0
 * PB4      TIM3   CH1      P1
 * PB5      TIM3   CH2      P2
 * PB6      TIM4   CH1      P3
 * PB7      TIM4   CH2      P4
 * PB9      TIM4   CH4      P5
 */

/************** INCLUDES ******************************************************/

#include <stdio.h>
#include <ctype.h>
#include "main.h"
#include "servo.h"

//#include "ebb.h"
//#include "HardwareProfile.h"
//#include "fifo.h"
//#include "parse.h"
//#include "utility.h"
//#include "isr.h"
//#include "stepper.h"

/************** MODULE DEFINES ************************************************/

#define MAX_RC2_SERVOS    6       // This is 6 because there are 6 timer channels dedicated to RC servo pulses

#define RCSERVO_POWEROFF_DEFAULT_MS (60ul*1000ul)  // Number of milliseconds to default the RCServo power autotimeout (5min)

// Power on default value for gPenMoveDuration in milliseconds
#define PEN_MOVE_DURATION_DEFAULT_MS  500

#if defined(BOARD_EBB)
#define DEFAULT_PEN_MAX_POSITION_SERVO    15302   // max = down (SC,5,15302)
#define DEFAULT_PEN_MIN_POSITION_SERVO    22565   // min = up (SC,4,22565)
#else
#define DEFAULT_PEN_MAX_POSITION_STEPPER  (200*16/4)  // max = down (SC,5) - 1/4 turn from home
#define DEFAULT_PEN_MIN_POSITION_STEPPER  (200*16/2)  // min = up (SC,4) - 1/2 turn from home
#endif

/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// Current RC servo position in 83uS units for each channel
static uint16_t gRC2Value[MAX_RC2_SERVOS];
// Target position for this channel in 83uS units
static uint16_t gRC2Target[MAX_RC2_SERVOS];
// Amount of change from Value to Target each 24ms
static uint16_t gRC2Rate[MAX_RC2_SERVOS];

// Records the current pen state (up/down) in reality
static PenStateType gPenStateActual;

// Records the pen state that the commands coming from the PC think the pen is in
// (Prevents duplicate pen up or pen down commands.)
static PenStateType PenStateCommand;


// These are the min, max, and default duration values for SP pen move command
// They can be changed by using SC,x commands.
static int16_t gPenMaxPosition;
static uint16_t gPenMinPosition;
static uint16_t gPenMoveDuration;

// Counts down milliseconds until zero. At zero shuts off power to RC servo (via RA3))
volatile uint32_t gRCServoPoweroffCounterMS = 0;
volatile uint32_t gRCServoPoweroffCounterReloadMS = RCSERVO_POWEROFF_DEFAULT_MS;


/************** LOCAL FUNCTION PROTOTYPES *************************************/

//static uint8_t servo_Move(UINT16 Position, UINT8 RPn, UINT16 Rate, UINT16 Delay);
static void servo_SetChannel(uint8_t channel, uint16_t width);

/************** LOCAL FUNCTIONS ***********************************************/
#if 0



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
      FIFO_G1[FIFOIn].ServoPosition = Position;
      FIFO_G2[FIFOIn].ServoRPn = RPn;
      FIFO_G3[FIFOIn].ServoRate = Rate;
      FIFO_G4[FIFOIn].ServoChannel = Channel;
      FIFO_G5[FIFOIn].DelayCounter = HIGH_ISR_TICKS_PER_MS * (UINT32)Delay;

      fifo_Inc();
    }
  }
  return Channel;
}

/************** GLOBAL FUNCTIONS **********************************************/
#endif

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
*/

/*
 * Module Init Function
 *
 * Put a call to this function inside the UserInit() call in UBW.c
 */
void servo_Init(void)
{
  unsigned char i;

  gPenMoveDuration = PEN_MOVE_DURATION_DEFAULT_MS;

  for (i=0; i < MAX_RC2_SERVOS; i++)
  {
    gRC2Value[i] = 0;
    gRC2Target[i] = 0;
    gRC2Rate[i] = 0;
  }


  // Start with some reasonable default values for min and max pen positions
#if defined(BOARD_EBB)
  gPenMaxPosition = DEFAULT_PEN_MAX_POSITION_SERVO;
  gPenMinPosition = DEFAULT_PEN_MIN_POSITION_SERVO;
#else
  gPenMaxPosition = DEFAULT_PEN_MAX_POSITION_STEPPER;
  gPenMinPosition = DEFAULT_PEN_MIN_POSITION_STEPPER;
#endif
  
#if defined(BOARD_EBB)
  RCServoPowerIO = RCSERVO_POWER_OFF;
#endif

  // JUST A TEST
  servo_SetChannel(0, 10000);

}

// servo_SetChannel
// This function sets the PWM width of any of the 6 RC servo channels (P0 to P5)
// If width is zero, then the RC servo output on that channel is disabled and it is
// available for GPIO or other uses. In this case, it will be set to be an output
// and driven low (as a PWM width of 0 would normally do)
// For every timer used in RC Servo signal generation, the frequency is 49.88 Hz.
// The clock is 170Mhz, and a divider of 52 is used. So the full 65536 width of
// the PMW pulse will be 20.05ms. And the PWM resolution is in units of 306ns.
// If width is from 1 to 65535 then the pin will stay high for that number of
// 306ns units.
// channel is from 0 to 5
static void servo_SetChannel(uint8_t channel, uint16_t width)
{
	switch (channel)
	{
	case 0:



		break;
	default:
		break;
	}
}

#if 0
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
// Internally, the parseSPCommand() function makes a call to
// process_SP() function to actually make the change in the servo output.
//
void parseSPCommand(void)
{
  UINT8 State = 0;
  UINT16 CommandDuration = gPenMoveDuration;

  // Extract each of the values.
  extract_number (kUINT8, &State, kREQUIRED);
  extract_number (kUINT16, &CommandDuration, kOPTIONAL);

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
void parseTPCommand(void)
{
  UINT16 CommandDuration = gPenMoveDuration;

  // Extract each of the values.
  extract_number (kUINT16, &CommandDuration, kOPTIONAL);

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

void parseS2Command(void)
{
  UINT16 Duration = 0;
  UINT8 Pin = 0;
  UINT16 Rate = 0;
  UINT16 Delay = 0;

  // Extract each of the values.
  extract_number (kUINT16, &Duration, kOPTIONAL);
  extract_number (kUINT8, &Pin, kOPTIONAL);
  extract_number (kUINT16, &Rate, kOPTIONAL);
  extract_number (kUINT16, &Delay, kOPTIONAL);

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

// Perform all of the things necessary to initialize the pen position.
// Not much to do when using a servo, but for a stepper Motor3 must be 
// driven past the lowest position, limped, a pause, 
// then a move from the home (lowest) position to the bottom position 
// (gPenMaxPosition)
// /// TODO : Once suitable mechanicals have been constructed, convert the
// simplified code below over to what is talked about above (using hardstop
// homing)
void servoPenHome(void)
{  
  gPenStateActual = PEN_DOWN;
  PenStateCommand = PEN_DOWN;

  // Execute a move from homed to the lower pen position
  process_SM(500, 0, 0, gPenMaxPosition);
}

#endif
