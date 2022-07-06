/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        ISR.c
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Software License Agreement
 *
 * Copyright (c) 2020-2021, Brian Schmalz of Schmalz Haus LLC
 * All rights reserved.
 * Based on EiBotBoard (EBB) Firmware, written by Brian Schmalz of
 *   Schmalz Haus LLC
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

/************** INCLUDES ******************************************************/

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "stepper.h"
#include "ISR.h"
#include "HardwareProfile.h"
#include "debug.h"
#include "FIFO.h"
#include "servo.h"
#include "commands.h"
#include "utility.h"


/************** PRIVATE TYPEDEFS **********************************************/

/************** PRIVATE DEFINES ***********************************************/

/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// Constantly incrementing every 1ms in ISR global tick counter
volatile uint32_t TickCounterMS;

// Constantly decrementing until it reaches zero. Use in local functions for
// short local delays from 1 to 255 milliseconds
volatile uint8_t GlobalDelayMS;

volatile uint8_t DriverInitDelayMS;

// Holds the data for the currently executing command
/// TODO: Can this just be the currently pointed to element in the FIFO? Why
/// have it be a separate variable?
volatile MoveCommand_t CurrentCommand;


/************** PRIVATE FUNCTION PROTOTYPES ***********************************/

/************** PRIVATE FUNCTIONS *********************************************/

/************** PUBLIC FUNCTIONS **********************************************/

/* For functions outside this module that need access to information about the
 * current command. Returns a pointer to the currently executing command.
 */
/// TODO: This is dangerous. It would be much better to somehow have accessor
/// functions which would allow external modules to get read-only access to
/// the currently executing command right?
volatile MoveCommand_t * ISR_GetCurrentCommand(void)
{
  return &CurrentCommand;
}

/*
 * Main interrupt service routine
 * Runs at 100kHz (EBB is at 25kHz)
 * Called from tim.c's HAL_TIM_PeriodElapsedCallback() when TIM6 fires
 * This routine looks at the currently executing command in the move fifo,
 * and 'runs' it. For example, the primary command is moving stepper motors:
 *   Step through each of the 3 stepper motors. For each one, check to see
 *   if it is moving, and if so, see if it needs to take a step. Do the math
 *   for acceleration as well, just like on EBB.
 * But also check for servo changes and other move commands in the fifo.
 */
void ISR_MotionISR(void)
{
  // Accumulators (at 100kHz) used to determine when to take a step
  static bool AllDone;
  AllDone = true;

///DEBUG_G0_SET();
  // If we are not already processing a command, see if there are any
  // waiting for us on the queue
  if (CurrentCommand.Command == COMMAND_NONE)
  {
    if (queue_PullNextCommand(&CurrentCommand) == false)
    {
      // Nope, queue is empty. So just mark that we do not have a command
      // to process and be done
      CurrentCommand.Command = COMMAND_NONE;
      return;
    }
  }

  if (CurrentCommand.Command == COMMAND_MOTOR_MOVE)
  {
    AllDone = stepper_Step(&(CurrentCommand.Data.Stepper));
  }
  // Check to see if we should start or stop the engraver
  else if (CurrentCommand.Command == COMMAND_SE)
  {
    // Now act on the State of the SE command
    if (CurrentCommand.Data.Engraver.SEState)
    {
      // Set RB3 to StoredEngraverPower
///          CCPR1L = queue_G1[queueOut].SEPower >> 2;
///          CCP1CON = (CCP1CON & 0b11001111) | ((StoredEngraverPower << 4) & 0b00110000);
    }
    else
    {
      // Set RB3 to low by setting PWM duty cycle to zero
///          CCPR1L = 0;
///          CCP1CON = (CCP1CON & 0b11001111);
    }
    AllDone = true;
  }
  // Do we have an RC servo move?
  else if (CurrentCommand.Command == COMMAND_SERVO_MOVE)
  {
    // Set up a new target and rate for one of the servos
    servo_SetTarget(CurrentCommand.Data.Servo.ServoPosition, CurrentCommand.Data.Servo.ServoPin, CurrentCommand.Data.Servo.ServoRate);
    AllDone = true;
  }
  // Note that we can have a delay with a COMMAND_DELAY or a COMMAND_SERVO_MOVE
  // That's why this is not an elseif here.
  if (
    CurrentCommand.Command == COMMAND_DELAY
    ||
    CurrentCommand.Command == COMMAND_SERVO_MOVE
  )
  {
    if (CurrentCommand.DelayCounter)
    {
      // Double check that things aren't way too big
      if (CurrentCommand.DelayCounter > HIGH_ISR_TICKS_PER_MS * (uint32_t)0x10000)
      {
        CurrentCommand.DelayCounter = 0;
      }
      else
      {
        CurrentCommand.DelayCounter--;
      }
    }

    if (CurrentCommand.DelayCounter)
    {
      AllDone = false;
    }
  }

  // If we're done with our current command, load in the next one, if there's more
  if (AllDone)
  {
    // "Erase" the current command from the queue
    CurrentCommand.Command = COMMAND_NONE;
  }
  // Check for button being pushed
///    if (
///      (!swProgram)
///      ||
///      (
///        UseAltPause
///        &&
///        !PORTBbits.RB0
///      )
///    )
///    {
///      ButtonPushed = true;
///    }
///DEBUG_G0_RESET();
}


#if 0
void low_ISR(void)
{
  unsigned int i;
  signed int RC2Difference = 0;

  // Did we get an edge on SCALED_V+? (i.e. 2209 power went away or came back?)
  if (INTCON3bits.INT1IF)
  {
    DEBUG_C7_SET();
    // Clear the interrupt
    INTCON3bits.INT1IF = 0;
    
    // Always disable the motors when we see DIAG trigger - either we just lost
    // power, or we just got power. In either case we don't want the drivers to
    // consume power.
    EnableIO = 1;

//    // Set the interrupt to happen on the other edge
//    if (INTCON2bits.INTEDG1)
//    {
      // We just got a rising edge, power has been applied, so init the drivers
      DriversNeedInit = TRUE;
      queue_NeedsInit = TRUE;
//      INTCON2bits.INTEDG1 = 0;
//    }
//    else
//    {
//      INTCON2bits.INTEDG1 = 1;
//      DEBUG_C7_CLEAR();
//    }
    DEBUG_C7_CLEAR();
  }
  
  // Do we have a Timer4 interrupt? (1ms rate)
  if (PIR3bits.TMR4IF)
  {
    // Clear the interrupt 
    PIR3bits.TMR4IF = 0;

#if 0
    /// TODO: IS this necessary to have in ISR?
    // Only start analog conversions if there are channels enabled
    if (AnalogEnabledChannels)
    {
      // Only start every so many ms
      if (AnalogInitiate >= ANALOG_INITATE_MS_BETWEEN_STARTS)
      {
        // Always start off with calibration
        ADCON1bits.ADCAL = 1;

        // Clear the interrupt
        PIR1bits.ADIF = 0;

        // Set the interrupt enable
        PIE1bits.ADIE = 1;

        // Make sure it's on!
        ADCON0bits.ADON = 1;

        // And tell the A/D to GO!
        ADCON0bits.GO_DONE = 1;

        // Reset AnalogInitiate counter
        AnalogInitiate = 0;
      }
      // Otherwise, increment each 1ms
      else
      {
        AnalogInitiate++;
      }
    }
#endif
    
    // Is Pulse Mode on?
    if (gPulsesOn)
    {
      // Loop across the four pins
      for (i=0; i<4; i++)
      {
        // Only pulse the pin if there is a length for the pin
        if (gPulseLen[i])
        {
          // If this is the beginning of the pulse, turn the pin on
          if (gPulseCounters[i] == 0)
          {
            // Turn the pin
            if (i==0) PORTBbits.RB0 = 1;
            if (i==1) PORTBbits.RB1 = 1;
            if (i==2) PORTBbits.RB2 = 1;
            if (i==3) PORTBbits.RB3 = 1;
          }

          // If we've reached the end of the pulse, turn the pin off
          if (gPulseCounters[i] == gPulseLen[i])
          {
            // Turn the pin off
            if (i==0) PORTBbits.RB0 = 0;
            if (i==1) PORTBbits.RB1 = 0;
            if (i==2) PORTBbits.RB2 = 0;
            if (i==3) PORTBbits.RB3 = 0;
          }

          // Now increment the counter
          gPulseCounters[i]++;

          // And check to see if we've reached the end of the rate
          if (gPulseCounters[i] >= gPulseRate[i])
          {
            // If so, start over from zero
            gPulseCounters[i] = 0;
          }
        }
        else
        {
          // Turn the pin off
          if (i==0) PORTBbits.RB0 = 0;
          if (i==1) PORTBbits.RB1 = 0;
          if (i==2) PORTBbits.RB2 = 0;
          if (i==3) PORTBbits.RB3 = 0;
        }
      }
    }
    
    // Global delay (for short local delays in functions)
    if (GlobalDelayMS)
    {
      GlobalDelayMS--;
    }

    // Global delay after DIAG rising edge before we take action in utlity_run())
    if (DriverInitDelayMS)
    {
      DriverInitDelayMS--;
    }
    /// TODO: Refactor this into something nicer?
#if defined(BOARD_EBB)
    // Software timer for RCServo power control
    if (gRCServoPoweroffCounterMS)
    {
      gRCServoPoweroffCounterMS--;
      // If we just timed out, then shut off RC Servo power
      if (gRCServoPoweroffCounterMS == 0)
      {
        RCServoPowerIO = RCSERVO_POWER_OFF;
      }
    }
#endif
    
    // Keep track of global time
    TickCounterMS++;
    
  } // end of 1ms interrupt

#if 0
  // Do we have an analog interrupt?
  if (PIR1bits.ADIF)
  {
    // Clear the interrupt
    PIR1bits.ADIF = 0;
    // If we just had a calibration, means we just started, so clear things
    // out and begin our sequence.
    if (ADCON1bits.ADCAL)
    {
      ADCON1bits.ADCAL = 0;
      ChannelBit = 0x0001;
      A_cur_channel = 0;
    }
    else
    {
      // Read out the value that we just converted, and store it.
      ISR_A_queue[A_cur_channel] =
        (unsigned int)ADRESL
        |
        ((unsigned int)ADRESH << 8);

      // increment the channel and mask bit
      ChannelBit = ChannelBit << 1;
      A_cur_channel++;
    }

    // Walk through the enabled channels until we find the next one
    while (A_cur_channel < 16)
    {
      if (ChannelBit & AnalogEnabledChannels)
      {
        break;
      }
      else
      {
        // increment the channel and write the new one in
        A_cur_channel++;
        ChannelBit = ChannelBit << 1;
      }
    }

    if (A_cur_channel >= 16)
    {
      // We're done, so just sit and wait
      // Turn off our interrupts though.
      PIE1bits.ADIE = 0;
    }
    else
    {
      // Update the channel number
      ADCON0 = (A_cur_channel << 2) + 1;
      // And start the next conversion
      ADCON0bits.GO_DONE = 1;
    }
  }
#endif
}
#endif
