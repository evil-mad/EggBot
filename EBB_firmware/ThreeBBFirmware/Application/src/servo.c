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
 * is in units of 306ns. These are called Servo Time Units.
 *
 * Each RC servo move is placed into the motion queue so that it can be sequenced
 * properly with the other motion commands. Once the motion ISR begins the RC servo
 * move, it can happen either immediately (if the rate parameter is zero) or it
 * will happen over a period of time as determined by the rate parameter.
 *
 * When the rate parameter is not zero for an RC servo move, the servo's position
 * (PWM width) will be updated every 20ms from the SysTick ISR. Every 20ms, 'rate'
 * will be added (or subtracted) from the current position until the target is
 * achieved. The units for 'rate' are Servo Time Units per 20ms.
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
#include "tim.h"
#include "parse.h"
#include "utility.h"
#include "fifo.h"
#include "isr.h"
#include "debug.h"
#include "main.h"


/************** MODULE TYPEDEFS ************************************************/

typedef enum
{
  PEN_DOWN = 0,
  PEN_UP
} PenStateType;


/************** MODULE DEFINES ************************************************/

// Which of the six RC servo outputs (P0-P5) is the pen lift servo on
#define SERVO_PEN_UP_DOWN_SERVO_PIN             1

// Macro to convert from floating point milliseconds to Servo Time Units (305ns)
#define MS_TO_SERVO_TIME_UNITS(x)               (uint16_t)((65535.0f/20.05f) * (x))

// This is 6 because there are 6 timer channels dedicated to RC servo pulses
#define MAX_SERVOS                              6

// Number of milliseconds to default the RCServo power autotimeout (60s)
#define PEN_SERVO_POWER_COUNTER_DEFAULT_MS      (60ul*1000ul)

// Default delay before next motion queue command for RC Servo Pen moves (ms)
#define PEN_DEFAULT_MOVE_DELAY_SERVO_MS         500

// Default move rate for RC Servo Pen moves in Servo Time Units/20ms
#define PEN_DEFAULT_RATE_SERVO                  500

// Default min and max positions for RC Servo Pen. Can be changed by SC command
#define PEN_DEFAULT_MAX_POSITION_SERVO          MS_TO_SERVO_TIME_UNITS(1.5) // SP,1
#define PEN_DEFAULT_MIN_POSITION_SERVO          MS_TO_SERVO_TIME_UNITS(0.5) // SP,0


/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// Current RC servo 'position' in 306nS units for each channel
static volatile uint16_t servo_Position[MAX_SERVOS];
// Target position for this channel in 306uS units
static volatile uint16_t servo_Target[MAX_SERVOS];
// Amount of change (in 306uS units) applied to Position to get to Target each 20ms
static volatile uint16_t servo_Rate[MAX_SERVOS];

// Records the current pen state (up/down) in reality (i.e. after move is complete)
static volatile PenStateType servo_PenActualState;

// Records the pen state that the commands coming from the PC think the pen is in
// (Prevents duplicate pen up or pen down commands.)
static PenStateType servo_PenLastState;

// Values used for SP/TP RC Servo Pen commands
static uint16_t servo_PenMaxPosition;   // Max position (Servo Time Units)
static uint16_t servo_PenMinPosition;   // Min position (Servo Time Units)
static uint16_t servo_PenDelay;         // Delay before next motion command (ms)
static uint16_t servo_PenRate;          // Rate for pen move (Servo Time Units/20ms)

// Counts down milliseconds until zero. At zero shuts off power to RC servo (via RA3)
volatile static uint32_t servo_PenServoPowerCounterMS = 0;
volatile static uint32_t servo_PenServoPowerCounterReloadMS = PEN_SERVO_POWER_COUNTER_DEFAULT_MS;


/************** LOCAL FUNCTION PROTOTYPES *************************************/

static void servo_Move(uint16_t position, uint8_t pin, uint16_t rate, uint16_t delay);
static void process_SP(PenStateType newState, uint16_t delay);
static void PenServoPowerEnable(bool state);


/************** LOCAL FUNCTIONS ***********************************************/

/*
 * Set a new state for the Pen Servo's 5V power connection via the SVO_EN pin
 * <state> should be true to enable power to the Pen Servo, and false to disable it
 */
static void PenServoPowerEnable(bool state)
{
  if (state)
  {
    HAL_GPIO_WritePin(SVO_EN_GPIO_Port, SVO_EN_Pin, GPIO_PIN_SET);
    servo_PenServoPowerCounterMS = servo_PenServoPowerCounterReloadMS;
  }
  else
  {
    HAL_GPIO_WritePin(SVO_EN_GPIO_Port, SVO_EN_Pin, GPIO_PIN_RESET);
    servo_PenServoPowerCounterMS = 0;
  }
}

/*
 * Function to schedule an RC Servo move. Takes position, pin, rate and delay
 * and adds the move to the motion control queue.
 * <position> is the new target position for the servo, in Servo Time Units. So
 *     65535 is 20.05ms. To turn off a servo output, use 0 for position.
 * <pin> is the RC Servo Pin number for the pin that you want to use as the output
 *      (0 through 5)
 * <rate> is how quickly to move to the new position. Use 0 for instant change.
 *      Units are Servo Time Units per 20ms.
 * <delay> is how many milliseconds after this command is executed before the
 *     next command in the motion control queue is executed. 0 will run the next
 *     command immediately. Note that <rate> and <delay> are independent.
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

  // Wait until we have at least one free spot in the queue, and add our new
  // command in
  WaitForRoomInQueue();

  /// TODO: Move this to a queue module function?
  // Now copy the values over into the queue element
  queue_Command[queueIn] = COMMAND_SERVO_MOVE;
  queue_G1[queueIn].ServoPosition = position;
  queue_G3[queueIn].ServoRate = rate;
  queue_G4[queueIn].ServoChannel = pin;
  queue_G5[queueIn].DelayCounter = HIGH_ISR_TICKS_PER_MS * (uint32_t)delay;

  queue_Inc();
}

/*
 * Pen move helper function
 * Perform a state change on the pen. Move it up or move it down.
 * At this point, this is limited to the pen servo for 3BB. The z-axis stepper
 * will be controlled entirely by PC "SM" commands, not by 'pen' command here.
 *
 * <NewState> is either PEN_UP or PEN_DOWN.
 * <CommandDuration> is the number of milliseconds that the move must take
 *  (Note: If <CommandDuration> is 0, then the global default value of
 *   servo_PenDuration is used.)
 *
 * This function uses the servo_PenMaxPosition and servo_PenMinPosition as targets for the
 * pen's movement.
 */
static void process_SP(PenStateType newState, uint16_t delay)
{
  if (delay == 0)
  {
    delay = servo_PenDelay;
  }

  // Only send a new command if the pen state is changing from what our last
  // commanded state was. We don't want to send multiple relative moves commands
  // in the same direction to the stepper as it will go out of bounds.
  if (servo_PenLastState != newState)
  {
    PenServoPowerEnable(true);

    if (newState == PEN_UP)
    {
      // Schedule the move on the motion queue
      servo_Move(servo_PenMaxPosition, SERVO_PEN_UP_DOWN_SERVO_PIN, servo_PenRate, delay);
    }
    else
    {
      servo_Move(servo_PenMinPosition, SERVO_PEN_UP_DOWN_SERVO_PIN, servo_PenRate, delay);
    }

    // Now that we've sent the move command off to the motion queue record
    // the new commanded pen state. This LastState also gets used to set the
    // global current state once the move is complete.
    servo_PenLastState = newState;
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

  // Zero out all of the low level parameters for the servo outputs
  for (i=0; i < MAX_SERVOS; i++)
  {
    servo_Position[i] = 0;
    servo_Rate[i] = 0;
    servo_Target[i] = 0;
  }

  // Start out at init by loading default values for these module globals
  servo_PenMaxPosition = PEN_DEFAULT_MAX_POSITION_SERVO;
  servo_PenMinPosition = PEN_DEFAULT_MIN_POSITION_SERVO;
  servo_PenDelay = PEN_DEFAULT_MOVE_DELAY_SERVO_MS;
  servo_PenRate = PEN_DEFAULT_RATE_SERVO;
  
  // Always start out with servo power off
  PenServoPowerEnable(false);
}

/*
 * servo_SetTarget()
 * This function updates the target position and rate for an RC servo output
 * It is called from the 100KHz motion ISR at the start of an RC servo 'move'
 * <position> is the new target position for the servo output (PWM width)
 * <pin> is the pin (0-5) to change
 * <rate> is how quickly the move should happen
 */
void servo_SetTarget(uint16_t position, uint8_t pin, uint16_t rate)
{
  if (rate == 0)
  {
    // If the rate is zero, this means that the new position should happen
    // immediately and not wait for the next 20ms update in servo_ProcessoTargets()
    servo_Rate[pin] = 0;
    servo_Target[pin] = position;
    servo_Position[pin] = position;
    servo_SetOutput(position, pin);
  }
  else
  {
    // Just update target and rate and let servo_ProcessTargets() take care of moving there
    servo_Rate[pin] = rate;
    servo_Target[pin] = position;
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
 * 306ns units, with a position of 65535 being high all the time.
 * <pin> is from 0 to 5, larger values will cause no change
 * This function can be called in mainline or interrupt contexts.
 */
void servo_SetOutput(uint16_t position, uint8_t pin)
{
  uint32_t oldPosition;

  switch (pin)
  {
	case 0: // Pin P0 is on Timer8 channel 1
	{
	  oldPosition = TIM8->CCR1;
      TIM8->CCR1 = position;

      if (position == 0)
      {
	    // New position is 0, so set GPIO pin low and turn off PWM
        HAL_GPIO_WritePin(P0_GPIO_Port, P0_Pin, GPIO_PIN_SET);
        if (HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1) != HAL_OK)
        {
          Error_Handler();
        }
	  }
      else
      {
        if (oldPosition == 0)
        {
          // New position is not zero, but old position was. Turn on PWM for this pin
          if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK)
          {
            Error_Handler();
          }
        }
      }
      break;
	}
    case 1: // Pin P1 is on Timer 3 channel 1
    {
      oldPosition = TIM3->CCR1;
      TIM3->CCR1 = position;

      if (position == 0)
      {
        // New position is 0, so set GPIO pin low and turn off PWM
        HAL_GPIO_WritePin(P1_GPIO_Port, P1_Pin, GPIO_PIN_SET);
        if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1) != HAL_OK)
        {
          Error_Handler();
        }
      }
      else
      {
        if (oldPosition == 0)
        {
          // New position is not zero, but old position was. Turn on PWM for this pin
          if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
          {
            Error_Handler();
          }
        }
      }
      break;
    }
    case 2: // Pin P2 is on Timer3 channel 2
    {
      oldPosition = TIM3->CCR2;
      TIM3->CCR2 = position;

      if (position == 0)
      {
        // New position is 0, so set GPIO pin low and turn off PWM
        HAL_GPIO_WritePin(P2_GPIO_Port, P2_Pin, GPIO_PIN_SET);
        if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2) != HAL_OK)
        {
          Error_Handler();
        }
      }
      else
      {
        if (oldPosition == 0)
        {
          // New position is not zero, but old position was. Turn on PWM for this pin
          if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
          {
            Error_Handler();
          }
        }
      }
      break;
    }
    case 3: // Pin P3 is on Timer4 channel 1
    {
      oldPosition = TIM4->CCR1;
      TIM4->CCR1 = position;

      if (position == 0)
      {
        // New position is 0, so set GPIO pin low and turn off PWM
        HAL_GPIO_WritePin(P3_GPIO_Port, P3_Pin, GPIO_PIN_SET);
        if (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1) != HAL_OK)
        {
          Error_Handler();
        }
      }
      else
      {
        if (oldPosition == 0)
        {
          // New position is not zero, but old position was. Turn on PWM for this pin
          if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
          {
            Error_Handler();
          }
        }
      }
      break;
    }
    case 4: // Pin P4 is on Timer4 channel 2
    {
      oldPosition = TIM4->CCR2;
      TIM4->CCR2 = position;

      if (position == 0)
      {
        // New position is 0, so set GPIO pin low and turn off PWM
        HAL_GPIO_WritePin(P4_GPIO_Port, P4_Pin, GPIO_PIN_SET);
        if (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2) != HAL_OK)
        {
          Error_Handler();
        }
      }
      else
      {
        if (oldPosition == 0)
        {
          // New position is not zero, but old position was. Turn on PWM for this pin
          if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
          {
            Error_Handler();
          }
        }
      }
      break;
    }
    case 5: // Pin P5 is on Timer4 channel 4
    {
      oldPosition = TIM4->CCR4;
      TIM4->CCR4 = position;

      if (position == 0)
      {
        // New position is 0, so set GPIO pin low and turn off PWM
        HAL_GPIO_WritePin(P5_GPIO_Port, P5_Pin, GPIO_PIN_SET);
        if (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4) != HAL_OK)
        {
          Error_Handler();
        }
      }
      else
      {
        if (oldPosition == 0)
        {
          // New position is not zero, but old position was. Turn on PWM for this pin
          if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
          {
            Error_Handler();
          }
        }
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
 * <duration> (optional) is the length of time between when this move
 *  begins and when the next move in the motion queue is started (ms)
 *  (<duration> is a 16 bit unsigned int)
 *  (Note that the global <servo_PenDuration> will be used if no parameter
 *   value is specified for <duration>.)
 *
 * This is a command that the user can send from the PC to set the pen state.
 *
 * Sending an SP command will get it inserted into the motion queue just like
 * any other motion command. The SP command will not begin until all previous
 * motion command has finished.
 *
 * This function will use the values for <serv_min>, <servo_max>,
 * (SC,4 SC,5 commands) when it schedules the pen move for the destination
 * position.
 */
void servo_SPCommand(void)
{
  uint8_t state = 0;
  uint16_t delay = servo_PenDelay;

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
 * Usage: TP,<delay><CR>
 * Returns: OK<CR>
 * <delay> is optional, and defaults to 0mS. It represents the amount of time
 *   after the RC servo command begins before the next motion queue command is
 *   allowed to start.
 * This command toggles state of pen arm, then delays for the optional <delay>
 */
void servo_TPCommand(void)
{
  uint16_t delay = servo_PenDelay;

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
 *      Units for <rate> are Servo Time Units (or 306ns).
 *    <delay> is the number of milliseconds to delay the start of the next motion command
 *      after this command begins. Gives the ability to 'overlap' servo moves with stepper
 *      moves. (optional, defaults to 0 = instant)
 *      16-bit unsigned integer
 * This command is added to the motion queue.
 */
void servo_S2Command(void)
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

/*
 * Query Pen
 * Usage: QP<CR>
 * Returns: 0 for down, 1 for up, then OK<CR>
 */
void servo_QPCommand(void)
{
  if (servo_PenActualState == PEN_UP)
  {
    printf("1\n");
  }
  else
  {
    printf("0\n");
  }
  print_ack();
}

/*
 * QR Query RC Servo power state command
 * Example: "QR<CR>"
 * Returns "0<LF>OK<LF>" or "1<LF>OK<LF>"
 * 0 = power to RC servo off
 * 1 = power to RC servo on
 */
void servo_QRCommand()
{
  if (HAL_GPIO_ReadPin(SVO_EN_GPIO_Port, SVO_EN_Pin) == GPIO_PIN_SET)
  {
    printf("1\n");
  }
  else
  {
    printf("0\n");
  }
  print_ack();
}

/*
 * SR Set RC Servo power timeout
 * Example: "SR,<new_time_ms>,<new_power_state><CR><LF>"
 * Returns "OK<CR><LF>"
 * <new_time_ms> is a 32-bit unsigned integer, representing the new RC servo
 * poweroff timeout in milliseconds. This value is not saved across reboots.
 * It is the length of time the system will wait after any command that uses
 * the motors or servo before killing power to the RC servo.
 * Use a value of 0 for <new_time_ms> to completely disable the poweroff timer.
 * <new_power_state> is an optional parameter of either 0 or 1. It will
 * immediately affect the servo's power state, where 0 turns it off and 1
 * turns it on.
 */
void servo_SRCommand(void)
{
  uint32_t delay = 0;
  uint8_t state = 0;
  ExtractReturnType got_state;

  extract_number(kUINT32, &delay, kREQUIRED);
  got_state = extract_number(kUINT8, &state, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Update the reload value for the power off timer
  servo_PenServoPowerCounterReloadMS = delay;

  // Did the user want to set the state?
  if (got_state == kEXTRACT_OK)
  {
    // Yup, so set new power state
    PenServoPowerEnable(state);
  }

  print_ack();
}

/*
 * Sets new value for pen's maximum position value (used in SP/TP)
 */
void servo_SetPenMaxPosition(uint16_t position)
{
  servo_PenMaxPosition = position;
}

/*
 * Sets new value for pen's minimum position value (used in SP/TP)
 */
void servo_SetPenMinPosition(uint16_t position)
{
  servo_PenMinPosition = position;
}

/*
 * Sets new value for pen's delay - time before next motion command is allow to run
 */
void servo_SetPenDelay(uint16_t delay)
{
  servo_PenDelay = delay;
}

/*
 * Sets new value for pen's rate - how fast it moves, in Servo Time Units/20ms
 */
void servo_SetPenRate(uint16_t rate)
{
  servo_PenRate = rate;
}


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

    for (pin=0; pin < MAX_SERVOS; pin++)
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

        // For special case of pen servo pin, if this move has now completed, then
        // update the global pen status to be in the new position
        if (pin == SERVO_PEN_UP_DOWN_SERVO_PIN)
        {
          if (servo_Position[pin] == servo_Target[pin])
          {
            servo_PenActualState = servo_PenLastState;
          }
        }
        DEBUG_G1_RESET();
      }
    }
  }
}

/*
 * Call this every 1ms from SysTick interrupt
 * It checks to see if it is time to turn off power to the Pen Servo
 */
void servo_CheckPenServoPowerTimeout(void)
{
  if (servo_PenServoPowerCounterMS > 0)
  {
    servo_PenServoPowerCounterMS--;
    if (servo_PenServoPowerCounterMS == 0)
    {
      PenServoPowerEnable(false);
    }
  }
}
