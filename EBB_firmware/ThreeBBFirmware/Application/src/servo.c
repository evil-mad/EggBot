/*********************************************************************
 *
 *                ThreeBotBoard Firmware
 *
 *********************************************************************
 * FileName:        servo.c
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Software License Agreement
 *
 * Copyright (c) 2020-2021, Brian Schmalz of Schmalz Haus LLC
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
 * For every timer used in RC Servo signal generation, the frequency is 49.88 Hz.
 * The clock is 170Mhz, and a divider of 51 is used (so there is a divide by 52).
 * So the full 65536 width of the PMW pulse will be 20.05ms. And the PWM resolution
 * is in units of 306ns.
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

/// TODO: Put into user docs: The fact that P1/2 and P3/4/5 will all have synchronized
/// rising edges, but 0, 1/2, and 3/4/5 may not be synchronized.

/************** INCLUDES ******************************************************/

#include <stdio.h>
#include <ctype.h>
#include "main.h"
#include "servo.h"
#include "tim.h"
#include "parse.h"
#include "utility.h"
#include "fifo.h"
#include "isr.h"
#include "debug.h"


/************** MODULE DEFINES ************************************************/

// This is 6 because there are 6 timer channels dedicated to RC servo pulses
#define MAX_SERVOS                              6

// Number of milliseconds to default the RCServo power autotimeout (60s)
#define RCSERVO_POWEROFF_DEFAULT_MS             (60ul*1000ul)

// Initialized default move time for RC Servo Pen moves in milliseconds
#define PEN_DEFAULT_MOVE_DURATION_SERVO_MS      500

// Initialized default move rate for RC Servo Pen moves in Servo Time Units/20ms
#define PEN_DEFAULT_RATE_SERVO                  500

/// TODO: Update these for 3BB
#define PEN_DEFAULT_MAX_POSITION_SERVO          15302   // max = down (SC,5,15302)
#define PEN_DEFAULT_MIN_POSITION_SERVO          22565   // min = up (SC,4,22565)


/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// Current RC servo 'position' in 306nS units for each channel
static volatile uint16_t servo_Position[MAX_SERVOS];
// Target position for this channel in 306uS units
static volatile uint16_t servo_Target[MAX_SERVOS];
// Amount of change (in 306uS units) applied to Position to get to Target each 20ms
static volatile uint16_t servo_Rate[MAX_SERVOS];
// True when the servo 'channel' is enabled and controlling the pin
static volatile bool servo_Enable[MAX_SERVOS];

// Records the current pen state (up/down) in reality (i.e. after move is complete)
static volatile PenStateType servo_PenActualState;

// Records the pen state that the commands coming from the PC think the pen is in
// (Prevents duplicate pen up or pen down commands.)
static PenStateType servo_PenLastState;

// Min and Max target positions for SP/TP RC Servo Pen commands
static uint16_t servo_PenMaxPosition;
static uint16_t servo_PenMinPosition;

// Duration and Rate values for SP/TP RC Servo Pen commands
static uint16_t servo_PenDuration;
static uint16_t servo_PenRate;

// Counts down milliseconds until zero. At zero shuts off power to RC servo (via RA3))
volatile uint32_t gRCServoPoweroffCounterMS = 0;
volatile uint32_t gRCServoPoweroffCounterReloadMS = RCSERVO_POWEROFF_DEFAULT_MS;


/************** LOCAL FUNCTION PROTOTYPES *************************************/

static void servo_Move(uint16_t position, uint8_t pin, uint16_t rate, uint16_t delay);
static void process_SP(PenStateType new_state, uint16_t delay);

/************** LOCAL FUNCTIONS ***********************************************/

/*
 * Function to set up an RC Servo move. Takes Position, Pin, Rate and Delay
 * and adds them to the motion control queue.
 * <Duration> is the new target position for the servo, in 83uS units. So
 *     65535 is 20.05ms. To turn off a servo output, use 0 for Duration.
 * <Pin> is the RC Servo Pin number for the pin that you want to use as the output
 *      (0 through 5)
 * <Rate> is how quickly to move to the new position. Use 0 for instant change.
 *      Unit is 83uS of pulse width change every 24ms of time.
 * <Delay> is how many milliseconds after this command is executed before the
 *     next command in the motion control FIFO is executed. 0 will run the next
 *     command immediately.
 * This function will allocate a new channel for RPn if the pin is not already
 * assigned to a channel. It will return the channel number used when it
 * returns. If you send in 0 for Duration, the channel for RPn will be deallocated.
 * Another thing we do here is to make sure that the proper pin is an output,
 * And, if this is the first time we're starting up the channel, make sure that
 * it starts out low.
 */
static void servo_Move(
  uint16_t position,
  uint8_t  pin,
  uint16_t rate,
  uint16_t delay
)
{
  if (pin >= MAX_SERVOS)
  {
    return;
  }

  // As a special case, if the pin is the same as the pin
  // used for the solenoid, then turn off the solenoid function
  // so that we can output PWM on that pin
///    if (Pin == SERVO_PEN_UP_DOWN_SERVO_PIN)
///    {
///      gUseSolenoid = FALSE;
///    }

  // Wait until we have a free spot in the FIFO, and add our new
  // command in
  WaitForRoomInFIFO();

  /// TODO: Move this to a FIFO module function?
  // Now copy the values over into the FIFO element
  FIFO_Command[FIFOIn] = COMMAND_SERVO_MOVE;
  FIFO_G1[FIFOIn].ServoPosition = position;
  FIFO_G3[FIFOIn].ServoRate = rate;
  FIFO_G4[FIFOIn].ServoChannel = pin;
  FIFO_G5[FIFOIn].DelayCounter = HIGH_ISR_TICKS_PER_MS * (uint32_t)delay;

  fifo_Inc();
}

/*
 *  Helper function :
 * Perform a state change on the pen. Move it up or move it down.
 * For 3BB, this will result in Motor3 moving to the new position.
 * For EBB, the 3rd stepper axis movement is mapped to RB1 as an RC servo pulse
 *
 * <NewState> is either PEN_UP or PEN_DOWN.
 * <CommandDuration> is the number of milliseconds that the move must take
 *  (Note: If <CommandDuration> is 0, then the global default value of
 *   gPenMoveDuration is used.)
 *
 * This function uses the gPenMinPosition and gPenMaxPosition as targets for the
 * pen's movement. It simply generates an process_SM() call to schedule the
 * move.
 */
static void process_SP(PenStateType new_state, uint16_t delay)
{
  if (delay == 0)
  {
    delay = servo_PenDuration;
  }

  // Only send a new command if the pen state is changing from what our last
  // commanded state was. We don't want to send multiple relative moves commands
  // in the same direction to the stepper as it will go out of bounds.
  if (servo_PenLastState != new_state)
  {
    ///    RCServoPowerIO = RCSERVO_POWER_ON;
    ///    gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;

    if (new_state == PEN_UP)
    {
      // Schedule the move on the motion queue
      servo_Move(servo_PenMaxPosition, SERVO_PEN_UP_DOWN_SERVO_PIN, servo_PenRate, delay);
    }
    else
    {
      servo_Move(servo_PenMinPosition, SERVO_PEN_UP_DOWN_SERVO_PIN, servo_PenRate, delay);
    }

    // Now that we've sent the move command off to the motion FIFO, record
    // the new commanded pen state
    servo_PenLastState = new_state;
  }
}

/************** PUBLIC FUNCTIONS **********************************************/

/*
 * Module Init Function
 *
 * Put a call to this function inside the UserInit() call in UBW.c
 */
void servo_Init(void)
{
  unsigned char i;


  for (i=0; i < MAX_SERVOS; i++)
  {
    servo_Enable[i] = false;
    servo_Position[i] = 0;
    servo_Rate[i] = 0;
    servo_Target[i] = 0;
  }

  // Start out at init by loading default values for these module globals
  servo_PenDuration = PEN_DEFAULT_MOVE_DURATION_SERVO_MS;
  servo_PenMaxPosition = PEN_DEFAULT_MAX_POSITION_SERVO;
  servo_PenMinPosition = PEN_DEFAULT_MIN_POSITION_SERVO;
  servo_PenRate = PEN_DEFAULT_RATE_SERVO;
  
///  RCServoPowerIO = RCSERVO_POWER_OFF;
}

/*
 * servo_SetTarget()
 * This function updates the target position and rate for an RC servo output
 * It is called from the 100KHz motion ISR at the start of an RC servo 'move'
 */
void servo_SetTarget(uint16_t position, uint8_t pin, uint16_t rate)
{
  // If duration is zero, then user wants to shut down this channel
  if (0 == position)
  {
    servo_Enable[pin] = false;
    servo_Position[pin] = 0;
    servo_Rate[pin] = 0;
    servo_Target[pin] = 0;

    /// TODO: Turn off PWM on this pin, make it a GPIO output and make it low

    // For now, just set the output to zero immediately
    servo_SetOutput(position, pin);
  }
  else
  {
    // Is this the first time we've used this channel?
    if (servo_Enable[pin] == false)
    {
      // Make sure the pin is set as an output, or this won't do much good

      // And turn on the PWM output for this pin
      servo_Enable[pin] = true;
    }

    // If the rate is zero, this means that the new position should happen
    // immediately and not wait for the next 20ms update in servo_ProcessoTargets()
    if (rate == 0)
    {
      servo_Rate[pin] = 0;
      servo_Target[pin] = position;
      servo_Position[pin] = position;
      servo_SetOutput(position, pin);
    }
    else
    {
      servo_Rate[pin] = rate;
      servo_Target[pin] = position;
    }
  }
}

/*
 * servo_SetOutput()
 * This function sets the PWM width of any of the 6 RC servo channels (P0 to P5)
 * If width is zero, then the RC servo output on that channel is disabled and it is
 * available for GPIO or other uses. In this case, it will be set to be an output
 * and driven low (as a PWM width of 0 would normally do)
 * For every timer used in RC Servo signal generation, the frequency is 49.88 Hz.
 * The clock is 170Mhz, and a divider of 52 is used. So the full 65535 width of
 * the PMW pulse will be 20.046ms (49.885Hz). And the PWM resolution is in units of 306ns.
 * If width is from 1 to 65534 then the pin will stay high for that number of
 * 306ns units.
 * <channel> is from 0 to 5, larger values will cause no change
 * This function can be called in mainline or interrupt contexts.
 */
void servo_SetOutput(uint16_t position, uint8_t pin)
{
  switch (pin)
  {
	case 0:
	{
	  if (position != 0)
	  {
	    TIM8->CCR1 = position;
	  }
	  else
	  {
	    // TODO: Change this : Turn off the timer for this I/O pin so it's just a GPIO output driven low
	    TIM8->CCR1 = position;
	  }
	  // TODO: Should this only get done if we don't already have the channel on? (to save time)
      if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
      {
        Error_Handler();
      }
      break;
	}
    case 1:
    {
      if (position != 0)
      {
        TIM3->CCR1 = position;
      }
      else
      {
        // TODO: Change this : Turn off the timer for this I/O pin so it's just a GPIO output driven low
        TIM3->CCR1 = position;
      }
      // TODO: Should this only get done if we don't already have the channel on? (to save time)
      if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
      {
        Error_Handler();
      }
      break;
    }
    case 2:
    {
      if (position != 0)
      {
        TIM3->CCR2 = position;
      }
      else
      {
        // TODO: Change this : Turn off the timer for this I/O pin so it's just a GPIO output driven low
        TIM3->CCR2 = position;
      }
      // TODO: Should this only get done if we don't already have the channel on? (to save time)
      if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
      {
        Error_Handler();
      }
      break;
    }
    case 3:
    {
      if (position != 0)
      {
        TIM4->CCR1 = position;
      }
      else
      {
        // TODO: Change this : Turn off the timer for this I/O pin so it's just a GPIO output driven low
        TIM4->CCR1 = position;
      }
      // TODO: Should this only get done if we don't already have the channel on? (to save time)
      if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
      {
        Error_Handler();
      }
      break;
    }
    case 4:
    {
      if (position != 0)
      {
        TIM4->CCR2 = position;
      }
      else
      {
        // TODO: Change this : Turn off the timer for this I/O pin so it's just a GPIO output driven low
        TIM4->CCR2 = position;
      }
      // TODO: Should this only get done if we don't already have the channel on? (to save time)
      if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
      {
        Error_Handler();
      }
      break;
    }
    case 5:
    {
      if (position != 0)
      {
        TIM4->CCR4 = position;
      }
      else
      {
        // TODO: Change this : Turn off the timer for this I/O pin so it's just a GPIO output driven low
        TIM4->CCR4 = position;
      }
      // TODO: Should this only get done if we don't already have the channel on? (to save time)
      if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
      {
        Error_Handler();
      }
      break;
    }

	default:
	  break;
  }
}

/*
 * Set Pen
 * Usage: SP,<state>[,duration]<CR>
 * <state> is 0 (for goto servo_max) or 1 (for goto servo_min)
 * <duration> (optional) is the length of time this move should take in ms
 *  (<duration> is a 16 bit unsigned int)
 *  (Note that the global <pen_move_duration> will be used if no parameter
 *   value is specified for <duration>.)
 *
 * This is a command that the user can send from the PC to set the pen state.
 *
 * Sending an SP command will get it inserted into the motion queue just like
 * any other motion command. The SP command will not begin until the previous
 * motion command has finished. Thus there will be no stepper movement during
 * the pen move. As soon as the SP move is complete, the next command in the
 * motion queue will be executed.
 *
 * This function will use the values for <serv_min>, <servo_max>,
 * (SC,4 SC,5 commands) when it schedules the pen move for the destination
 * position.
 *
 * Internally, the parseSPCommand() function makes a call to
 * process_SP() function to actually make the change in the servo output.
 */
void parseSPCommand(void)
{
  uint8_t state = 0;
  uint16_t delay = servo_PenDuration;

  // Extract each of the values.
  extract_number(kUINT8, &state, kREQUIRED);
  extract_number(kUINT16, &delay, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (state > 1)
  {
    state = 1;
  }

  // Execute the servo state change
  process_SP(state, delay);
    
  print_ack();
}

/*
 * Toggle Pen
 * Usage: TP,<duration><CR>
 * Returns: OK<CR>
 * <duration> is optional, and defaults to 0mS
 * Just toggles state of pen arm, then delays for the optional <duration>
 * Duration is in units of 1ms
 */
void parseTPCommand(void)
{
  uint16_t delay = servo_PenDuration;

  // Extract each of the values.
  extract_number(kUINT16, &delay, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (servo_PenLastState == PEN_UP)
  {
    process_SP(PEN_DOWN, delay);
  }
  else
  {
    process_SP(PEN_UP, delay);
  }

  print_ack();
}


/*
 * RC Servo Output command
 * S2,<position>,<pin>,<rate>,<delay><CR>
 * It will turn on/off RC pulses on pin <pin> for <duration>.
 *    <duration> can be 0 (output off) to 65535 (20.05ms, or 100% on time)
 *      A 0 for <duration> sets the output pin to be a GPIO output pin and sets it low.
 *      16-bit unsigned integer
 *    <pin> is an RC Servo output pin number (P0 through P5)
 *      Values 0 through 5 are available, anything over will trigger error
 *    <rate> is the rate to change (optional, defaults to 0 = instant)
 *      /// TODO: Add details about units
 *    <delay> is the number of milliseconds to delay the start of the next motion command
 *      after this command begins. Gives the ability to 'overlap' servo moves with stepper
 *      moves. (optional, defaults to 0 = instant)
 *      16-bit unsigned integer
 * This command is added to the motion queue.
 */
void parseS2Command(void)
{
  uint16_t position = 0;
  uint8_t pin = 0;
  uint16_t rate = 0;
  uint16_t delay = 0;

  // Extract each of the values.
  extract_number(kUINT16, &position, kREQUIRED);
  extract_number(kUINT8, &pin, kREQUIRED);
  extract_number(kUINT16, &rate, kOPTIONAL);
  extract_number(kUINT16, &delay, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (pin > 5)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  servo_Move(position, pin, rate, delay);

  print_ack();
}

#if 0

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

/*
 * This function gets called every 1m from the SysTick handler
 * It's job is to walk through all servo outputs that are currently enabled
 * and see if any need their position updated to get closer to their target.
 * Even though this function is called every 1ms, we only want to apply 'rate'
 * change every 20ms (once per pulse).
 */
void servo_ProcessTargets(void)
{
  static uint8_t ProcessTargetsCounter = 0;
  uint8_t pin;
  int32_t temp;

  ProcessTargetsCounter++;
  if (ProcessTargetsCounter >= 20)
  {
    ProcessTargetsCounter = 0;

    for(pin=0; pin < MAX_SERVOS; pin++)
    {
      if (servo_Enable[pin])
      {
        if (servo_Target[pin] != servo_Position[pin])
        {
          DEBUG_G1_SET();
          if ((servo_Position[pin] - servo_Target[pin]) > 0)
          {
            temp = servo_Position[pin] - servo_Rate[pin];
            if (temp < servo_Target[pin])
            {
              servo_Position[pin] = servo_Target[pin];
            }
            else
            {
              servo_Position[pin] = temp;
            }
          }
          else
          {
            temp = servo_Position[pin] + servo_Rate[pin];
            if (temp > servo_Target[pin])
            {
              servo_Position[pin] = servo_Target[pin];
            }
            else
            {
              servo_Position[pin] = temp;
            }
          }

          servo_SetOutput(servo_Position[pin], pin);
          DEBUG_G1_RESET();
        }
      }
    }
  }
}

