/*********************************************************************
 *
 *                EiBotBoard Firmware
 *
 *********************************************************************
 * FileName:        RCServo2.c
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
 * There are several paarts of this file. It is written in a modular
 * way to ease incorporation into different UBW (www.schmalzhaus.com/UBW) 
 * hardware builds. Because we don't want to have any function calls in the ISR,
 * the part of the ISR that this module contributes is implemented as a big
 * macro. There is a section for the API calls (for other modules) to turn
 * on/off RC method 2 and to set values and configure outputs. Then there is the
 * user command section that handles parsing user input. This module (the .c and
 * .h files) should be able to be dropped into any UBW firmware that is 'module
 * aware' with minimial changes. There is also an init section that gets called
 * on bootup.
 *
 * MODULE THEORY
 *
 * The idea is that we want to generate between zero and eight RC servo outputs. 
 * We want a maximum of resolution in their timing, and a minimum of CPU and ISR
 * overhead. We want maximum flexibility with respect to which pins receive the
 * output pulses. This 'method 2' only works with PICs that have the PPS
 * (Perhipheral Pin Select) hardware. Using this method, we will be able to
 * generate servo output pulses (positive going) on up to eight output pins
 * (selectable using PPS), with times from 0ms to 3ms, at a repitition rate of
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
 * that the S2 command no longer takes effect immediatly - RC servo commands
 * are now handleded in the same way that SM commands are handled. Which means
 * they can be queued up and delays added, etc.
 *
 * The other effect of this change is that the normal SP pen up/pen down
 * commands are now just normal S2 commands in disguise.
 */

#include <p18cxxx.h>
#include <stdio.h>
#include <ctype.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "ebb.h"
#include "UBW.h"
#include "RCServo2.h"
#include "HardwareProfile.h"

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

// These are the min, max, up rate, down rate, and RPn for the Pen Servo
// They can be changed by using SC,x commands.
UINT16 g_servo2_max;
UINT16 g_servo2_min;
UINT16 g_servo2_rate_up;
UINT16 g_servo2_rate_down;
UINT8  g_servo2_RPn;

/*
The idea with RCServo2 is to use the ECCP2 module and timer 3.
We divide time into 24ms periods. Inside each 24ms period, we
can fire up to 8 RC servo's pulses (slots). Each pulse can be between
0ms and 3ms long, controlled entirely by the ECCP2 hardware,
so there is no jitter in the high time of the pulse.

We want to go from 0ms to 3ms so we can accomodate RC servos
who need really short or really long pulses to reach the
physical extremes of its motion.

This RCServo2 method will only be available on the 18F45J50 based
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
void RCServo2_Init(void)
{
	unsigned char i;

	gRC2msCounter = 0;
	gRC2Ptr = 0;

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
	T3CONbits.TMR3CS = 0b00;  	// Use Fosc/4 as input
	T3CONbits.T3CKPS = 0b00;  	// Prescale is 1:1
	T3CONbits.RD16 = 1;			// Enable 16 bit mode
	TMR3H = 0;
	TMR3L = 0;
	T3CONbits.TMR3ON = 0;		// Keep timer off for now
	
	TCLKCONbits.T3CCP1 = 1;		// ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4
	TCLKCONbits.T3CCP2 = 0;		// ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4

	CCP2CONbits.CCP2M = 0b1001;	// Set EECP2 as compare, clear output on match

	// We start out with 8 slots because that is good for RC servos (3ms * 8 = 24ms)
	gRC2Slots = INITAL_RC2_SLOTS;

	// We start out with 3ms slot duration because it's good for RC servos
	gRC2SlotMS = 3;

    // Start with some reasonable default values for min and max
	g_servo2_max = 15302;           // max = down (SC,5,15302)
	g_servo2_min = 22565;           // min = up (SC,4,22565)

	g_servo2_RPn = DEFAULT_EBB_SERVO_RPN;		// Always start out with RP4 as the output (just for this test version of code)
	
	g_servo2_rate_up = 400;
	g_servo2_rate_down = 400;
	process_SP(PEN_UP, 0);			// Start servo up
  RCServoPowerIO = RCSERVO_POWER_OFF;
}

// Return the current channel that is associated with the PPS output pin
// RPn. If there is no channel yet assigned for this RPn, then pick the
// next available one. If there are none available, then return channel 0
// (which is considered an error.)
// Remember, channels are from 1 through 8 (Normally - can be increased with 
// SC,8 command). Channel 0 is the 'error' channel.
UINT8 RCServo2_get_channel_from_RPn(UINT8 RPn)
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

// Servo method 2 enable command
// S2,<duration>,<output_pin>,<rate>,<delay><CR>
//  will set RC output <channel> for <duration> on output pin <output_pin>
//	<duration> can be 0 (output off) to 32,000 (3ms on time)
//      (a 0 for <duration> de-allocates the channel for this output_pin)
//	<output_pin> is an RPn pin number (0 through 24)
//  <rate> is the rate to change (optional, defaults to 0 = instant)
//  <delay> is the number of milliseconds to delay the start of the next command
//      (optional, defaults to 0 = instant)

void RCServo2_S2_command (void)
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
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	
	RCServo2_Move(Duration, Pin, Rate, Delay);

	print_ack();
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
UINT8 RCServo2_Move(
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
  Channel = RCServo2_get_channel_from_RPn(RPn);

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
      while(!FIFOEmpty)
      ;
      
      // If the pin we're controlling is B1 (the normal servo output) then
      // always make sure to turn power on and start the countdown timer
      // for that servo port. (issue #144)
      if (RPn == 4)
      {
        RCServoPowerIO = RCSERVO_POWER_ON;
        gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
      }

      // Now copy the values over into the FIFO element
      CommandFIFO[0].Command = COMMAND_SERVO_MOVE;
      CommandFIFO[0].DelayCounter = HIGH_ISR_TICKS_PER_MS * (UINT32)Delay;
      CommandFIFO[0].ServoChannel = Channel;
      CommandFIFO[0].ServoRPn = RPn;
      CommandFIFO[0].ServoPosition = Position;
      CommandFIFO[0].ServoRate = Rate;

      FIFOEmpty = FALSE;
    }
	}
  return Channel;
}