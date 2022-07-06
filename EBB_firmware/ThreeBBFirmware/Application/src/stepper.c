/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        stepper.c
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

/*
 * This module implements the stepper command functionality
 */

/************** INCLUDES ******************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "parse.h"
#include "HardwareProfile.h"
#include "utility.h"
#include "isr.h"
#include "fifo.h"
#include "main.h"
#include "debug.h"
#include "servo.h"
#include "commands.h"
#include "stepper.h"
#include "serial.h"
#include "tim.h"

/************** PRIVATE TYPEDEFS **********************************************/

/************** PRIVATE DEFINES ***********************************************/

// This is the value that gets multiplied by Steps/Duration to compute
// the StepAdd values.
#define OVERFLOW_MUL            (0x8000 / HIGH_ISR_TICKS_PER_MS)

/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// When FALSE, we skip parameter checks for motor move commands so they can run faster
bool gLimitChecks = true;

/* Pin table for stepper control */
const SteppersIO_t SteppersIO[NUMBER_OF_STEPPERS] = {
    {DIR1_GPIO_Port,  DIR1_Pin, STEP1_GPIO_Port,  STEP1_Pin},   // MOTOR1
    {DIR2_GPIO_Port,  DIR2_Pin, STEP2_GPIO_Port,  STEP2_Pin},   // MOTOR2
    {DIR3_GPIO_Port,  DIR3_Pin, STEP3_GPIO_Port,  STEP3_Pin},   // MOTOR3
};

// Actual array for all values used for each stepper
volatile Steppers_t Steppers[NUMBER_OF_STEPPERS] = {
    {0, 0x10000000, 0, 0, 0},   // MOTOR1
    {0, 0x10000000, 0, 0, 0},   // MOTOR2
    {0, 0x10000000, 0, 0, 0},   // MOTOR3
};


/************** PRIVATE FUNCTION PROTOTYPES ***********************************/

void clear_StepCounters(void);

/************** PRIVATE FUNCTIONS *********************************************/

/************** PUBLIC FUNCTIONS **********************************************/

/*
 * Called from the 100kHz ISR if the current FIFO command is COMMAND_MOTOR_MOVE
 * Loop through all motors and Perform all of the work of see if any motors
 * need a step, acceleration, etc.
 * Returns true if the command is complete (no more steps to take)
 */
uint8_t stepper_Step(StepperCommand_t * cmdPtr)
{
  uint8_t stepper;
  int8_t dir;
  uint8_t done = true;

  /// TODO: We only need to set up and output the DIR bits once, when we first start the move. No need to waste time after that.

  // Loop through all 3 steppers
  for (stepper = 0; stepper < NUMBER_OF_STEPPERS; stepper++)
  {
    // Only check for needed stepping if there are steps left to take
    if (cmdPtr->StepsCounter[stepper] != 0)
    {
      done = false;
      // Make sure to Set the direction bit properly
      if (cmdPtr->Dir[stepper])
      {
        SteppersIO[stepper].DirPort->BSRR = (uint32_t)SteppersIO[stepper].DirPin;
        dir = 1;
      }
      else
      {
        SteppersIO[stepper].DirPort->BRR = (uint32_t)SteppersIO[stepper].DirPin;
        dir = -1;
      }

      // Add increment value to accumulator, then see if highest bit is set
      Steppers[stepper].StepAcc += cmdPtr->StepAdd[stepper];

      if (Steppers[stepper].StepAcc & 0x80000000)
      {
        // Set the step bit
        SteppersIO[stepper].StepPort->BSRR = (uint32_t)SteppersIO[stepper].StepPin;
        // Do the housekeeping math
        Steppers[stepper].StepAcc = Steppers[stepper].StepAcc & 0x7FFFFFFF;
        cmdPtr->StepsCounter[stepper]--;
        Steppers[stepper].GlobalPosition += dir;
        // Clear the step bit
        SteppersIO[stepper].StepPort->BRR = (uint32_t)SteppersIO[stepper].StepPin;
      }
      // For acceleration, we now add a bit to StepAdd each time through as well
      cmdPtr->StepAdd[stepper] += cmdPtr->StepAddInc[stepper];
    }
  }
  return done;
}





// Enable Motor
// Usage: EM,<EnableAxis1>,<EnableAxis2><CR>
// Everything after EnableAxis1 is optional
// Each parameter can have a value of
//    0 to disable that motor driver
// Legacy Mode: (default at power up)
// (only first parameter applies, and it applies to both drivers)
//    1 to enable the driver in 1/16th step mode
//    2 to enable the driver in 1/8 step mode
//    3 to enable the driver in 1/4 step mode
//    4 to enable the driver in 1/2 step mode
//    5 to enable the driver in full step mode
// 3BB Mode:
//    1 to
//
// If you disable a motor (0 for parameter), it goes 'limp'
// Note that when using 0 or 1 for a parameter, you can use both axis even
// on a 'new' driver chip board. (i.e. EM,0,1 will disable motor 1 and enable 2)
// Note that the MSx lines do not come to any headers, so even when an external
// source is controlling the drivers, the PIC still needs to control the
// MSx lines.
void stepper_EMCommand(void)
{
  unsigned char EA1, EA2, EA3;
  ExtractReturnType RetVal1, RetVal2, RetVal3;

  // Extract each of the values.
  RetVal1 = extract_number (kUINT8, &EA1, kREQUIRED);
  RetVal2 = extract_number (kUINT8, &EA2, kOPTIONAL);
  RetVal3 = extract_number (kUINT8, &EA3, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // NOTE: Because 3BB only has one enable line going to all three steppers,
  // we just take the first parameter to the command as the enable state for
  // all three.
  if (utility_LegacyModeEnabled())
  {

    if (kEXTRACT_OK == RetVal1)
    {
      switch (EA1)
      {
        case 0:
          stepper_Enable(false);
          break;

        case 1:   // 1/16th microstep mode
          stepper_Enable(true);
          serial_SetMicrosteps(1, MICROSTEP_16);
          serial_SetMicrosteps(2, MICROSTEP_16);
          serial_SetMicrosteps(3, MICROSTEP_16);
          break;

        case 2:   // 1/8th microstep mode
          stepper_Enable(true);
          serial_SetMicrosteps(1, MICROSTEP_8);
          serial_SetMicrosteps(2, MICROSTEP_8);
          serial_SetMicrosteps(3, MICROSTEP_8);
          break;

        case 3:   // 1/4h microstep mode
          stepper_Enable(true);
          serial_SetMicrosteps(1, MICROSTEP_4);
          serial_SetMicrosteps(2, MICROSTEP_4);
          serial_SetMicrosteps(3, MICROSTEP_4);
          break;

        case 4:   // 1/2 microstep mode
          stepper_Enable(true);
          serial_SetMicrosteps(1, MICROSTEP_2);
          serial_SetMicrosteps(2, MICROSTEP_2);
          serial_SetMicrosteps(3, MICROSTEP_2);
          break;

        case 5:   // full step mode
          stepper_Enable(true);
          serial_SetMicrosteps(1, MICROSTEP_1);
          serial_SetMicrosteps(2, MICROSTEP_1);
          serial_SetMicrosteps(3, MICROSTEP_1);
          break;

        default:
          break;
      }
      if (kEXTRACT_OK == RetVal2)
      {
        if (EA2 == 0)
        {

        }
        else
        {

        }
      }
      else
      {

      }

    }
  }

/// TODO: Add proper code for enabling the motors at various levels
#if defined(BOARD_EBB)
    if (
      (DriverConfiguration == PIC_CONTROLS_DRIVERS)
      ||
      (DriverConfiguration == EXTERNAL_CONTROLS_DRIVERS)
    )
    {
      if (EA1 > 0)
      {
        if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
        {
          Enable1IO = ENABLE_MOTOR;
          RCServoPowerIO = RCSERVO_POWER_ON;
          gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
        }
        if (EA1 == 1)
        {
          MS1_IO = 1;
          MS2_IO = 1;
          MS3_IO = 1;
        }
        if (EA1 == 2)
        {
          MS1_IO = 1;
          MS2_IO = 1;
          MS3_IO = 0;
        }
        if (EA1 == 3)
        {
          MS1_IO = 0;
          MS2_IO = 1;
          MS3_IO = 0;
        }
        if (EA1 == 4)
        {
          MS1_IO = 1;
          MS2_IO = 0;
          MS3_IO = 0;
        }
        if (EA1 == 5)
        {
          MS1_IO = 0;
          MS2_IO = 0;
          MS3_IO = 0;
        }
      }
      else
      {
        if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
                {
        Enable1IO = DISABLE_MOTOR;
                }
      }
    }
    else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
    {
      if (EA1 > 0)
      {
        Enable1AltIO = ENABLE_MOTOR;
        RCServoPowerIO = RCSERVO_POWER_ON;
        gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
      }
      else
      {
        Enable1AltIO = DISABLE_MOTOR;
      }
    }
#endif

#if defined(BOARD_EBB)
    if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
    {
      if (EA2 > 0)
      {
        Enable2IO = ENABLE_MOTOR;
        RCServoPowerIO = RCSERVO_POWER_ON;
        gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
      }
      else
      {
        Enable2IO = DISABLE_MOTOR;
      }
    }
    else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
    {
      if (EA2 > 0)
      {
        Enable2AltIO = ENABLE_MOTOR;
        RCServoPowerIO = RCSERVO_POWER_ON;
        gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
      }
      else
      {
        Enable2AltIO = DISABLE_MOTOR;
      }
    }
#endif


  // Always clear the step counts if motors are enabled/disabled or 
  // resolution is changed.
/// TODO: Create this function
///  clear_StepCounters();
  
  print_ack();
}

#if 0

// RM command
// For Run Motor - allows completely independent running of the three stepper motors
/// TODO : do we even need this?
void parseRMCommand(void)
{
}

#endif

// The Stepper Motor command
// Usage: SM,<move_duration>,<axis1_steps>,<axis2_steps>,<axis3_steps><CR>
// <move_duration> is a number from 1 to 16777215, indicating the number of milliseconds this move should take
// <axisX_steps> is a signed 24 bit number indicating how many steps (and what direction) the axis should take
// NOTE1: <axis2_steps> and <axis3_steps> are optional and can be left off
// If the *BB can not make the move in the specified time, it will take as long as it needs to at max speed
// i.e. SM,1,1000 will not produce 1000steps in 1ms. Instead, it will take 40ms (25KHz max step rate)
// NOTE2: If you specify zero steps for the the three axis values, then you effectively create a delay. Use for small
// pauses before raising or lowering the pen, for example.
void stepper_SMCommand(void)
{
  uint32_t Duration = 0;
  int32_t A1Steps = 0, A2Steps = 0, A3Steps = 0;
  int32_t Steps = 0;

  // Extract each of the values.
  extract_number(kUINT32, &Duration, kREQUIRED);
  extract_number(kINT32, &A1Steps, kREQUIRED);
  extract_number(kINT32, &A2Steps, kOPTIONAL);
  extract_number(kINT32, &A3Steps, kOPTIONAL);

  if (gLimitChecks)
  {
    // Check for invalid duration
    if (Duration == 0) 
    {
      bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
    // Bail if we got a conversion error
    if (error_byte)
    {
      return;
    }
    // Limit each parameter to just 3 bytes
    if (Duration > 0xFFFFFF) 
    {
      printf("!0 Err: <move_duration> larger than 16777215 ms.\n");
      return;
    }
    // Check for too-fast step request (>25KHz)
    // First get absolute value of steps, then check if it's asking for >25KHz
    if (A1Steps > 0) 
    {
      Steps = A1Steps;
    }
    else 
    {
      Steps = -A1Steps;
    }
    if (Steps > 0xFFFFFF) 
    {
      printf("!0 Err: <axis1> larger than 16777215 steps.\n");
      return;
    }
    // Check for too fast
    if ((Steps/Duration) > HIGH_ISR_TICKS_PER_MS) 
    {
      printf("!0 Err: <axis1> step rate > 25K steps/second.\n");
      return;
    }
    // And check for too slow
    if ((Duration/1311) >= Steps && Steps != 0) 
    {
      printf("!0 Err: <axis1> step rate < 1.31Hz.\n");
      return;
    }

    if (A2Steps > 0) 
    {
      Steps = A2Steps;
    }
    else {
      Steps = -A2Steps;
    }    

    if (Steps > 0xFFFFFF) 
    {
      printf("!0 Err: <axis2> larger than 16777215 steps.\n");
      return;
    }
    if ((Steps/Duration) > HIGH_ISR_TICKS_PER_MS) 
    {
      printf("!0 Err: <axis2> step rate > 25K steps/second.\n");
      return;
    }
    if ((Duration/1311) >= Steps && Steps != 0) 
    {
      printf("!0 Err: <axis2> step rate < 1.31Hz.\n");
      return;
    }

    if (A3Steps > 0) 
    {
      Steps = A3Steps;
    }
    else {
      Steps = -A3Steps;
    }    

    if (Steps > 0xFFFFFF) 
    {
      printf("!0 Err: <axis3> larger than 16777215 steps.\n");
      return;
    }
    if ((Steps/Duration) > HIGH_ISR_TICKS_PER_MS) 
    {
      printf("!0 Err: <axis3> step rate > 25K steps/second.\n");
      return;
    }
    if ((Duration/1311) >= Steps && Steps != 0) 
    {
      printf("!0 Err: <axis3> step rate < 1.31Hz.\n");
      return;
    }
  }

  // If we get here, we know that step rate for both A1 and A2 is
  // between 25KHz and 1.31Hz which are the limits of what EBB can do.
  process_SM(Duration, A1Steps, A2Steps, A3Steps);

  if (g_ack_enable)
  {
    print_ack();
  }
}

#if 0
// The Accelerated Motion command
// Usage: SM,<inital_velocity>,<final_velocity>,<axis1_steps>,<axis2_steps><CR>
// <inital_velocity> is a number from 1 to 10000 in steps/second indicating the initial velocity
// <final_velocity> is a number from 1 to 10000 in steps/second indicating the final velocity
// <axisX_steps> is a signed 24 bit number indicating how many steps (and what direction) the axis should take
// Note that the two velocities are of the combined move - i.e. the tip of the pen, not the individual
// axies velocities.
void parseAMCommand(void)
{
  UINT32 temp = 0;
  UINT16 VelocityInital = 0;
  UINT16 VelocityFinal = 0;
  INT32 A1Steps = 0, A2Steps = 0;
  UINT32 Duration = 0;
  UINT32 Distance;
  float distance_temp;
  float accel_temp;

  // Extract each of the values.
  extract_number (kUINT32, &VelocityInital, kREQUIRED);
  extract_number (kUINT32, &VelocityFinal, kREQUIRED);
  extract_number (kINT32, &A1Steps, kREQUIRED);
  extract_number (kINT32, &A2Steps, kREQUIRED);

  // Check for too-fast step request (>25KHz)
  if (VelocityInital > 25000)
  {
    printf((far rom char *)"!0 Err: <velocity_initial> larger than 25000.\n");
    return;
  }
  if (VelocityFinal > 25000)
  {
    printf((far rom char *)"!0 Err: <velocity_final> larger than 25000.\n");
    return;
  }
  if (VelocityInital < 4)
  {
    printf((far rom char *)"!0 Err: <velocity_initial> less than 4.\n");
    return;
  }
  if (VelocityFinal < 4)
  {
    printf((far rom char *)"!0 Err: <velocity_final> less than 4.\n");
    return;
  }
    
  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }
    
  // Compute total pen distance
  // fprint(ftemp);
  Distance = (UINT32)(sqrt((float)((A1Steps * A1Steps) + (A2Steps * A2Steps))));
    
  // For debug
  //printf((far rom char *)"Distance= %lu\n", Distance);

  WaitForRoomInQueue();

  Queue_DirBits[queueIn] = 0;

  // Always enable both motors when we want to move them
/// TODO: Fix this for 3BB motor enable
#if defined(BOARD_EBB)
  Enable1IO = ENABLE_MOTOR;
  Enable2IO = ENABLE_MOTOR;
  RCServoPowerIO = RCSERVO_POWER_ON;
  gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
#endif
  
  // First, set the direction bits
  if (A1Steps < 0)
  {
    queue_DirBits[queueIn] = DIR1_BIT;
    A1Steps = -A1Steps;
  }
  if (A2Steps < 0)
  {
    queue_DirBits[queueIn] = queue_DirBits[queueIn] | DIR2_BIT;
    A2Steps = -A2Steps;
  }

  if (A1Steps > 0xFFFFFF) {
    printf((far rom char *)"!0 Err: <axis1> larger than 16777215 steps.\n");
    return;
  }
  if (A2Steps > 0xFFFFFF) {
    printf((far rom char *)"!0 Err: <axis2> larger than 16777215 steps.\n");
    return;
  }

  // To compute StepAdd values from Duration.
  // A1Stp is from 0x000001 to 0xFFFFFF.
  // HIGH_ISR_TICKS_PER_MS = 25
  // Duration is from 0x000001 to 0xFFFFFF.
  // temp needs to be from 0x0001 to 0x7FFF.
  // Temp is added to accumulator every 25KHz. So slowest step rate
  // we can do is 1 step every 25KHz / 0x7FFF or 1 every 763mS. 
  // Fastest step rate is obviously 25KHz.
  // If A1Stp is 1, then duration must be 763 or less.
  // If A1Stp is 2, then duration must be 763 * 2 or less.
  // If A1Stp is 0xFFFFFF, then duration must be at least 671088.

  // Test values
  // Duration = 2000;
  // Distance = 600;
  // Acceleration = 200;
  // VelocityInital = 100;
  // VelocityFinal = 500;
    
  /* Compute StepAdd Axis 1 Initial */
  //temp = ((UINT32)A1Steps*(((UINT32)VelocityInital * (UINT32)0x8000)/(UINT32)25000)/(UINT32)Distance);
  //printf((far rom char *)"VelocityInital = %d\n", VelocityInital);
  //printf((far rom char *)"Distance = %ld\n", Distance);
  //temp = (UINT32)((float)A1Steps*(((float)VelocityInital * (float)0x80000000UL)/(float)25000)/(float)Distance);
  distance_temp = ((float)VelocityInital * 85899.34592)/Distance;
  //printf((far rom char *)"distance_temp =");
  //fprint(distance_temp);
  //ftemp = distance_temp * A1Steps;
  //fprint(ftemp);
  //temp = (UINT32)ftemp;

  /* Amount to add to accumulator each 25KHz */
  FIFO_StepAdd0[FIFOIn] = (UINT32)(distance_temp * (float)A1Steps);

  // For debug
  //printf((far rom char *)"SAxi = %lu\n", CommandFIFO[0].StepAdd[0]);

  /* Total number of steps for this axis for this move */
  FIFO_G2[FIFOIn].StepsCounter0 = A1Steps;

  //ftemp = (float)VelocityFinal * 2147483648.0;
  //fprint(ftemp);
  //ftemp = ftemp / 25000;
  //fprint(ftemp);
  //ftemp = ftemp / Distance;
  //fprint(ftemp);
  //ftemp = ftemp * A1Steps;
  //fprint(ftemp);
  //temp = (UINT32)ftemp;

  // For debug
  //printf((far rom char *)"SAxf = %lu\n", temp);
    
  /* Compute StepAddInc for axis 1 */
  accel_temp = (((float)VelocityFinal * (float)VelocityFinal) - ((float)VelocityInital * (float)VelocityInital))/((float)Distance * (float)Distance * 2);
  //Accel1 = ((float)A1Steps * accel_temp);
  //printf((far rom char *)"accel_temp : ");
  //fprint(accel_temp);
  //stemp = (INT32)((Accel1 * (float)0x8000 * (float)0x10000)/((float)25000 * (float)25000));
  //stemp = (INT32)(Accel1 * 343.59738);
  //printf((far rom char *)"SAxinc = %ld\n", stemp);

  /* Amount to add to StepAdd each 25KHz */
  FIFO_G5[FIFOIn].StepAddInc0 = (INT32)(((float)A1Steps * accel_temp) * 3.435921);

  /* Compute StepAdd Axis 2 Initial */
  // temp = ((UINT32)A2Steps*(((UINT32)VelocityInital * (UINT32)0x8000)/(UINT32)25000)/(UINT32)Distance);
  // temp = (UINT32)((float)A2Steps*(((float)VelocityInital * (float)0x80000000)/(float)25000)/(float)Distance);

  //printf((far rom char *)"VelocityInital = %d\n", VelocityInital);
  //printf((far rom char *)"Distance = %ld\n", Distance);
  //    temp = (UINT32)((float)A1Steps*(((float)VelocityInital * (float)0x80000000UL)/(float)25000)/(float)Distance);
  //    ftemp = (float)VelocityInital * 2147483648.0;
  //    fprint(ftemp);
  //    ftemp = ftemp / 25000;
  //    fprint(ftemp);
  //    ftemp = ftemp / Distance;
  //    fprint(ftemp);
  //    ftemp = ftemp * A2Steps;
  //    fprint(ftemp);
  //    temp = (UINT32)ftemp;
    
  // For debug
  //printf((far rom char *)"SAyi = %lu\n", temp);

  FIFO_StepAdd1[FIFOIn] = (UINT32)(distance_temp * A2Steps);
  FIFO_G3[FIFOIn].StepsCounter1 = A2Steps;
    
  /* Compute StepAddInc for axis 2 */
  //    Accel2 = ((float)A2Steps * accel_temp);
  //printf((far rom char *)"Accel2 : ");
  //fprint(Accel2);
  //    stemp = (INT32)((Accel2 * (float)0x8000 * (float)0x10000)/((float)25000 * (float)25000));
  //    stemp = (INT32)(((float)A2Steps * accel_temp) * 343.59738);
    
  FIFO_StepAddInc1[FIFOIn] = (INT32)(((float)A2Steps * accel_temp) * 3.435921);

  if (VelocityInital != VelocityFinal && FIFO_G5[FIFOIn].StepAddInc0 == 0 && FIFO_G2[FIFOIn].StepsCounter0 > 0)
  {
     printf((far rom char *)"!0 Err: <axis1> acceleration value is 0.\n");
     return;
  }
  if (VelocityInital != VelocityFinal && FIFO_StepAddInc1[FIFOIn] == 0 && FIFO_G3[FIFOIn].StepsCounter1 > 0)
  {
     printf((far rom char *)"!0 Err: <axis2> acceleration value is 0.\n");
     return;
  }

  FIFO_Command[FIFOIn] = COMMAND_MOTOR_MOVE;
  
  fifo_Inc();
    
  print_ack();
}

// Low Level Move command
// Usage: LM,<StepAdd1>,<StepsCounter1>,<StepAddInc1>,<StepAdd2>,<StepsCounter2>,<StepAddInc2><CR>
void parseLMCommand(void)
{
  UINT32 StepAdd1, StepAddInc1, StepAdd2, StepAddInc2 = 0;
  INT32 StepsCounter1, StepsCounter2 = 0;
    
  // Extract each of the values.
  extract_number (kUINT32, &StepAdd1, kREQUIRED);
  extract_number (kINT32,  &StepsCounter1, kREQUIRED);
  extract_number (kINT32, &StepAddInc1, kREQUIRED);
  extract_number (kUINT32, &StepAdd2, kREQUIRED);
  extract_number (kINT32,  &StepsCounter2, kREQUIRED);
  extract_number (kINT32, &StepAddInc2, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
      return;
  }

  /* Quickly eliminate obvious invalid parameter combinations,
   * like LM,0,0,0,0,0,0. Or LM,0,1000,0,100000,0,100 GH issue #78 */
  if (
      (
        ((StepAdd1 == 0) && (StepAddInc1 == 0))
        ||
        (StepsCounter1 == 0)
      )
      &&
      (
        ((StepAdd2 == 0) && (StepAddInc2 == 0))
        ||
        (StepsCounter2 == 0)
      )
  )
  {
    return;
  }
  
/// TODO: Get this to work for 3BB motor enable as well  
#if defined(BOARD_EBB)
  // Always enable both motors when we want to move them
  Enable1IO = ENABLE_MOTOR;
  Enable2IO = ENABLE_MOTOR;
  RCServoPowerIO = RCSERVO_POWER_ON;
  gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
#endif
  
  // Spin here until there's space in the fifo
  WaitForRoomInQueue();

  queue_DirBits[queueIn] = 0;

  // First, set the direction bits
  if (StepsCounter1 < 0)
  {
    FIFO_DirBits[FIFOIn] = FIFO_DirBits[FIFOIn] | DIR1_BIT;
    StepsCounter1 = -StepsCounter1;
  }
  if (StepsCounter2 < 0)
  {
    FIFO_DirBits[FIFOIn] = FIFO_DirBits[FIFOIn] | DIR2_BIT;
    StepsCounter2 = -StepsCounter2;
  }

  FIFO_StepAdd0[FIFOIn] = StepAdd1;
  FIFO_G2[FIFOIn].StepsCounter0 = StepsCounter1;
  FIFO_G5[FIFOIn].StepAddInc0 = StepAddInc1;
  FIFO_StepAdd1[FIFOIn] = StepAdd2;
  FIFO_G3[FIFOIn].StepsCounter1 = StepsCounter2;
  FIFO_StepAddInc1[FIFOIn] = StepAddInc2;
  FIFO_Command[FIFOIn] = COMMAND_MOTOR_MOVE;

  /* For debugging step motion , uncomment the next line */
  /*
   * printf((far rom char *)"SA1=%lu SC1=%lu SA2=%lu SC2=%lu\n",
          CommandFIFO[0].StepAdd[0],
          CommandFIFO[0].StepsCounter[0],
          CommandFIFO[0].StepAdd[1],
          CommandFIFO[0].StepsCounter[1]
      );
   */

  fifo_Inc();

  if (g_ack_enable)
  {
    print_ack();
  }
}



// Home the motors
// "HM,<step_rate><CR>"
// <step_rate> is the desired rate of the primary (larger) axis in steps/s.
// Use the current global step counts to "undo" the current pen position so
// it gets back to zero.
// To figure out what the duration of the move should be, look at the axis
// with more steps to move (the 'primary' axis).
// There are a couple of special cases to consider:
// 1) If <step_rate> causes a step rate that's too high on the primary axis, 
//    then just use a duration which is slow enough.
// 2) If the <step_rate> causes the non-primary axis to need to move too slowly,
//    break the move into two parts. (i.e. "dog-leg" it) First move will move
//    with a duration such that the minor axis moves at the minimum step rate.
//    Second move will move at <step_rate> but with minor axis moving zero steps.
// If the step rate is too high or too low, don't error out, just use a legal value
// May need to make 2 moves if one of the axis has few steps to go and the other has
// lots.
// When parsing this command, always wait until both the FIFO is empty and the motion 
// commands are finished. That way two SM commands can be issued back to back to take 
// care of both moves (or just one if a 'dog leg' move is needed)
//
// TODO: This code can't handle steps counts above 4,294,967 in either axis. Is
// there a way to allow it to handle steps counts up to 16,777,215 easily?
void parseHMCommand(void)
{
  UINT32 StepRate = 0;
  INT32 Steps1 = 0, Steps2 = 0, Steps3 = 0;
  INT32 AbsSteps1 = 0, AbsSteps2 = 0, AbsSteps3 = 0;
  UINT32 Duration = 0;
  UINT8 CommandExecuting = 1;
  INT32 XSteps = 0;

  // Extract the step rate.
  extract_number (kUINT32, &StepRate, kREQUIRED);

  // Wait until FIFO is completely empty
  WaitForEmptyFIFO();

  // We now know that the motors have stopped moving
  // This should not be necessary, but just in case,
  // turn off high priority interrupts breifly here to read out value that ISR uses
  INTCONbits.GIEH = 0;  // Turn high priority interrupts off
  
  // Make a local copy of the things we care about. This is how far we need to move.
  Steps1 = -globalStepCounter1;
  Steps2 = -globalStepCounter2;
  Steps3 = -globalStepCounter3;

  // Re-enable interrupts
  INTCONbits.GIEH = 1;  // Turn high priority interrupts on

  // Compute absolute value versions of steps for computation
  if (Steps1 < 0)
  {
    AbsSteps1 = -Steps1;
  }
  else
  {
    AbsSteps1 = Steps1;
  }
  if (Steps2 < 0)
  {
    AbsSteps2 = -Steps2;
  }
  else
  {
    AbsSteps2 = Steps2;
  }
  if (Steps3 < 0)
  {
    AbsSteps3 = -Steps3;
  }
  else
  {
    AbsSteps3 = Steps3;
  }
    
  // Check for too many steps to step
  if ((AbsSteps1 > 0xFFFFFF) || (AbsSteps2 > 0xFFFFFF) || (AbsSteps3 > 0xFFFFFF))
  {
    printf((far rom char *)"!0 Err: steps to home larger than 16,777,215 on at least one axis.\n");
    return;
  }
  
  // Compute duration based on step rate user requested. Take bigger step count to use for calculation
/// TODO : UPdate this somehow to use third axis
  if (AbsSteps1 > AbsSteps2)
  {
    Duration = (AbsSteps1 * 1000) / StepRate;
    // Axis1 is primary
    // Check for too fast 
    if ((StepRate/1000) > HIGH_ISR_TICKS_PER_MS)
    {
      printf((far rom char *)"!0 Err: HM <axis1> step rate > 25K steps/second.\n");
      return;
    }
    // Check for too slow, on the non-primary axis
    if ((Duration/1311) >= AbsSteps2 && AbsSteps2 != 0)
    {
      // We need to break apart the home into two moves.
      // The first will be to get the non-primary axis down to zero.
      // Recompute duration for the first move
      Duration = (AbsSteps2 * 1000) / StepRate;
      if (Steps1 > 0 && Steps2 > 0)       // C
      {
        XSteps = Steps2;
      }
      else if (Steps1 < 0 && Steps2 > 0)  // B
      {
        XSteps = -Steps2;
      }
      else if (Steps1 > 0 && Steps2 < 0)  // D
      {
        XSteps = -Steps2;
      }
      else if (Steps1 < 0 && Steps2 < 0)  // A
      {
        XSteps = Steps2;
      }
      /// TODO: Add 3rd axis?
      process_SM(Duration, XSteps, Steps2, 0);
      // Update both steps count for final move
      Steps1 = Steps1 - XSteps;
      Steps2 = 0;
      // Recompute duration
      Duration = (AbsSteps1 * 1000) / StepRate;
    }
  }
  else
  {
    Duration = (AbsSteps2 * 1000) / StepRate;        
    // Axis2 is primary
    // Check for too fast 
    if ((StepRate/1000) > HIGH_ISR_TICKS_PER_MS)
    {
      printf((far rom char *)"!0 Err: HM <axis2> step rate > 25K steps/second.\n");
      return;
    }
    // Check for too slow, on the non-primary axis
    if ((Duration/1311) >= AbsSteps1 && AbsSteps1 != 0)
    {
      // We need to break apart the home into two moves.
      // The first will be to get the non-primary axis down to zero.
      // Recompute duration for the first move
      Duration = (AbsSteps1 * 1000) / StepRate;
      if (Steps2 > 0 && Steps1 > 0)       // C
      {
        XSteps = Steps1;
      }
      else if (Steps2 < 0 && Steps1 > 0)  // B
      {
        XSteps = -Steps1;
      }
      else if (Steps2 > 0 && Steps1 < 0)  // D
      {
        XSteps = -Steps1;
      }
      else if (Steps2 < 0 && Steps1 < 0)  // A
      {
        XSteps = Steps1;
      }
      /// TODO: add 3rd axis?
      process_SM(Duration, Steps1, XSteps, 0);
      // Update both steps count for final move
      Steps2 = Steps2 - XSteps;
      Steps1 = 0;
      // Recompute duration
      Duration = (AbsSteps2 * 1000) / StepRate;
    }
  }

  if (Duration < 10)
  {
    Duration = 10;
  }
  //printf((far rom char *)"HM Duration=%lu SA1=%li SA2=%li\n",
  //  Duration,
  //  Steps1,
  //  Steps2
  //);

  // If we get here, we know that step rate for both A1 and A2 is
  // between 25KHz and 1.31Hz which are the limits of what EBB can do.
  /// TODO: Add 3rd axis?
  process_SM(Duration, Steps1, Steps2, Steps3);

  if (g_ack_enable)
  {
    print_ack();
  }
}

#endif

// The X Stepper Motor command
// Usage: XM,<move_duration>,<axisA_steps>,<axisB_steps>,<axisZ_steps><CR>
// <move_duration> is a number from 1 to 16777215, indicating the number of milliseconds this move should take
// <axisA_steps> and <axisB_stetsp> are signed 24 bit numbers.
// This command differs from the normal "SM" command in that it is designed to drive 'mixed-axis' geometry
// machines like H-Bot and CoreXY. Using XM will effectively call SM with Axis1 = <axisA_steps> + <axisB_steps> and
// Axis2 = <axisA_steps> - <axisB_steps>.
// <axisZ_steps> is an optional signed 32 bit number which represents the number of steps that the Z Axis (3rd stepper - pen up/down)
//  should move during this <move_duration>.
void stepper_XMCommand(void)
{
  uint32_t Duration = 0;
  int32_t A1Steps = 0, A2Steps = 0;
  int32_t ASteps = 0, BSteps = 0;
  int32_t Steps = 0;
  int32_t ZSteps = 0;

  // Extract each of the values.
  extract_number(kUINT32, &Duration, kREQUIRED);
  extract_number(kINT32, &ASteps, kREQUIRED);
  extract_number(kINT32, &BSteps, kREQUIRED);
  extract_number(kINT32, &ZSteps, kOPTIONAL);

  // Check for invalid duration
  if (Duration == 0) 
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
  }

  // Do the math to convert to Axis1 and Axis2
  A1Steps = ASteps + BSteps;
  A2Steps = ASteps - BSteps;

  /// TODO: Can this limit checking be turned into a function (it's used three times here and also in SM)
  // Check for too-fast step request (>25KHz)
  // First get absolute value of steps, then check if it's asking for >25KHz
  if (A1Steps > 0) 
  {
    Steps = A1Steps;
  }
  else 
  {
    Steps = -A1Steps;
  }
  // Limit each parameter to just 3 bytes
  if (Duration > 0xFFFFFF) 
  {
    printf("!0 Err: <move_duration> larger than 16777215 ms.\n");
    return;
  }
  if (Steps > 0xFFFFFF) 
  {
    printf("!0 Err: <axis1> larger than 16777215 steps.\n");
    return;
  }
  // Check for too fast
  if ((Steps/Duration) > HIGH_ISR_TICKS_PER_MS) 
  {
    printf("!0 Err: <axis1> step rate > 25K steps/second.\n");
    return;
  }
  // And check for too slow
  if ((Duration/1311) >= Steps && Steps != 0)
  {
    printf("!0 Err: <axis1> step rate < 1.31Hz.\n");
    return;
  }

  if (A2Steps > 0) 
  {
    Steps = A2Steps;
  }
  else {
    Steps = -A2Steps;
  }    
  if (Steps > 0xFFFFFF) 
  {
    printf("!0 Err: <axis2> larger than 16777215 steps.\n");
    return;
  }
  if ((Steps/Duration) > HIGH_ISR_TICKS_PER_MS) 
  {
    printf("!0 Err: <axis2> step rate > 25K steps/second.\n");
    return;
  }
  if ((Duration/1311) >= Steps && Steps != 0) 
  {
    printf("!0 Err: <axis2> step rate < 1.31Hz.\n");
    return;
  }

  if (ZSteps > 0)
  {
    Steps = ZSteps;
  }
  else {
    Steps = -ZSteps;
  }
  if (Steps > 0xFFFFFF)
  {
    printf("!0 Err: <axis3> larger than 16777215 steps.\n");
    return;
  }
  if ((Steps/Duration) > HIGH_ISR_TICKS_PER_MS)
  {
    printf("!0 Err: <axis3> step rate > 25K steps/second.\n");
    return;
  }
  if ((Duration/1311) >= Steps && Steps != 0)
  {
    printf("!0 Err: <axis3> step rate < 1.31Hz.\n");
    return;
  }

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // If we get here, we know that step rate for both A1, A2 and Z are
  // between 25KHz and 1.31Hz which are the limits of what EBB can do.
  process_SM(Duration, A1Steps, A2Steps, ZSteps);

  print_ack();
}


// Main stepper move function. This is the reason EBB exists.
// <Duration> is a 3 byte unsigned int, the number of mS that the move should take
// <A1Stp> and <A2Stp> are the Axis 1 and Axis 2 number of steps to take in
//  <Duration> mS, as 3 byte signed values, where the sign determines the motor
//  direction.
// This function waits until there is room in the 1-deep FIFO before placing
// the data in the FIFO. The ISR then sees this data when it is done with its
// current move, and starts this new move.
//
// In the future, making the FIFO more elements deep may be cool.
// 
void process_SM(uint32_t duration, int32_t a1Stp, int32_t a2Stp, int32_t a3Stp)
{
  uint32_t temp = 0;
  uint32_t temp1 = 0;
  uint32_t temp2 = 0;
  uint32_t remainder = 0;
  uint8_t dir[NUMBER_OF_STEPPERS];
  int32_t stepAdd[NUMBER_OF_STEPPERS];
  int32_t stepAddInc[NUMBER_OF_STEPPERS];
  uint32_t stepsCounter[NUMBER_OF_STEPPERS];

  // Uncomment the following printf() for debugging
  //printf((far rom char *)"Duration=%lu SA1=%li SA2=%li\n",
  //        Duration,
  //        A1Stp,
  //        A2Stp
  //    );

  // Check for delay (if no step counts)
  if (a1Stp == 0 && a2Stp == 0 && a3Stp == 0)
  {
    queue_AddDelayCommandToQueue(HIGH_ISR_TICKS_PER_MS * duration);
  }
  else
  {
    dir[0] = 0;
    dir[1] = 0;
    dir[2] = 0;

    // Always enable both motors when we want to move them
///    Enable1IO = ENABLE_MOTOR;
///    Enable2IO = ENABLE_MOTOR;
///    RCServoPowerIO = RCSERVO_POWER_ON;
///    gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;
    
    // First, set the direction bits
    if (a1Stp < 0)
    {
      dir[0] = 1;
      a1Stp = -a1Stp;
    }
    if (a2Stp < 0)
    {
      dir[1] = 1;
      a2Stp = -a2Stp;
    }
    if (a3Stp < 0)
    {
      dir[2] = 1;
      a3Stp = -a3Stp;
    }

    // To compute StepAdd values from Duration.
    // A1Stp is from 0x000001 to 0xFFFFFF.
    // HIGH_ISR_TICKS_PER_MS = 25
    // Duration is from 0x000001 to 0xFFFFFF.
    // temp needs to be from 0x0001 to 0x7FFF.
    // Temp is added to accumulator every 25KHz. So slowest step rate
    // we can do is 1 step every 25KHz / 0x7FFF or 1 every 763mS. 
    // Fastest step rate is obviously 25KHz.
    // If A1Stp is 1, then duration must be 763 or less.
    // If A1Stp is 2, then duration must be 763 * 2 or less.
    // If A1Stp is 0xFFFFFF, then duration must be at least 671088.
        
    // First check for duration to large.
    //        if (A1Stp < (0xFFFFFF/763)) {
    //            if (duration > (A1Stp * 763)) {
    //                printf((far rom char *)"Major malfunction Axis1 duration too long : %lu\n", duration);
    //                temp = 0;
    //                A1Stp = 0;
    //            }
    //        }
        
    if (a1Stp < 0x1FFFF)
    {
      temp1 = HIGH_ISR_TICKS_PER_MS * duration;
      temp = (a1Stp << 15)/temp1;
      temp2 = (a1Stp << 15) % temp1;
      /* Because it takes us about 5ms extra time to do this division,
       * we only perform this extra step if our move is long enough to
       * warrant it. That way, for really short moves (where the extra
       * precision isn't necessary) we don't take up extra time. Without
       * this optimization, our minimum move time is 20ms. With it, it
       * drops down to about 15ms.
       */
      if (duration > 30)
      {
        remainder = (temp2 << 16) / temp1;
      }
    }
    else 
    {
      temp = (((a1Stp/duration) * (uint32_t)0x8000)/(uint32_t)HIGH_ISR_TICKS_PER_MS);
      remainder = 0;
    }
    if (temp > 0x8000) 
    {
      printf("Major malfunction Axis1 StepCounter too high : %lu\n", temp);
      temp = 0x8000;
    }
    if (temp == 0 && a1Stp != 0)
    {
      printf("Major malfunction Axis1 StepCounter zero\n");
      temp = 1;
    }
    if (duration > 30)
    {
      temp = (temp << 16) + remainder;
    }
    else
    {
      temp = (temp << 16);
    }

    stepAdd[0] = temp;
    stepsCounter[0] = a1Stp;
    stepAddInc[0] = 0;

    if (a2Stp < 0x1FFFF)
    {
      temp = (a2Stp << 15)/temp1;
      temp2 = (a2Stp << 15) % temp1;
      if (duration > 30)
      {
        remainder = (temp2 << 16) / temp1;
      }
    }
    else 
    {
      temp = (((a2Stp/duration) * (uint32_t)0x8000)/(uint32_t)HIGH_ISR_TICKS_PER_MS);
      remainder = 0;
    }
    if (temp > 0x8000) 
    {
      printf("Major malfunction Axis2 StepCounter too high : %lu\n", temp);
      temp = 0x8000;
    }
    if (temp == 0 && a2Stp != 0)
    {
      printf("Major malfunction Axis2 StepCounter zero\n");
      temp = 1;
    }
    if (duration > 30)
    {
      temp = (temp << 16) + remainder;
    }
    else
    {
      temp = (temp << 16);
    }
        
    stepAdd[1] = temp;
    stepsCounter[1] = a2Stp;
    stepAddInc[1] = 0;
     
    if (a3Stp < 0x1FFFF)
    {
      temp = (a3Stp << 15)/temp1;
      temp2 = (a3Stp << 15) % temp1;
      if (duration > 30)
      {
        remainder = (temp2 << 16) / temp1;
      }
    }
    else 
    {
      temp = (((a3Stp/duration) * (uint32_t)0x8000)/(uint32_t)HIGH_ISR_TICKS_PER_MS);
      remainder = 0;
    }
    if (temp > 0x8000) 
    {
      printf("Major malfunction Axis3 StepCounter too high : %lu\n", temp);
      temp = 0x8000;
    }
    if (temp == 0 && a3Stp != 0)
    {
      printf("Major malfunction Axis3 StepCounter zero\n");
      temp = 1;
    }
    if (duration > 30)
    {
      temp = (temp << 16) + remainder;
    }
    else
    {
      temp = (temp << 16);
    }
        
    stepAdd[2] = temp;
    stepsCounter[2] = a3Stp;
    stepAddInc[2] = 0;
    
    /* For debugging step motion , uncomment the next line */

    //printf((far rom char *)"SA1=%lu SC1=%lu SA2=%lu SC2=%lu\n",
    //        move.StepAdd[0],
    //        move.StepsCounter[0],
    //        move.StepAdd[1],
    //        move.StepsCounter[1]
    //    );

    // Always enable the stepper motors before we start moving them
    stepper_Enable(true);

    // Add the stepper command to the motion queue
    queue_AddStepperCommandToQueue(stepAdd, stepAddInc, stepsCounter, dir);
  }
}

#if 0

// Handle the EStop functionality: stop all motor motion in steppers and servo,
// print out the interrupted state (if printResult == TRUE), and start the 
// motor limp timer if it was non zero which will limp the 3 motors after
// the a delay.
void process_EStop(BOOL printResult)
{
  UINT8 command_interrupted = 0;
  UINT32 remaining_steps1 = 0;
  UINT32 remaining_steps2 = 0;
  UINT32 fifo_steps1 = 0;
  UINT32 fifo_steps2 = 0;
#if 0
  // If there is a command waiting in the FIFO and it is a move command
  // or the current command is a move command, then remember that for later.
  if (
      (!FIFOEmpty && FIFO_Command[0] == COMMAND_MOTOR_MOVE)
      || 
      CurrentCommand_Command == COMMAND_MOTOR_MOVE
  )
  {
    command_interrupted = 1;
  }

  // If the FIFO has a move command in it, remove it.
  if (FIFO_Command[0] == COMMAND_MOTOR_MOVE)
  {
    FIFO_Command[0] = COMMAND_NONE;
    fifo_steps1 = FIFO_StepsCounter[0][0];
    fifo_steps2 = FIFO_StepsCounter[1][0];
    FIFO_StepsCounter[0][0] = 0;
    FIFO_StepsCounter[1][0] = 0;
    FIFO_StepAddInc[0][0] = 0;
    FIFO_StepAddInc[1][0] = 0;
    FIFOEmpty = TRUE;
  }

  // If the current command is a move command, then stop the move.
  if (CurrentCommand_Command == COMMAND_MOTOR_MOVE)
  {
    CurrentCommand_Command = COMMAND_NONE;
    remaining_steps1 = CurrentCommand_StepsCounter[0];
    remaining_steps2 = CurrentCommand_StepsCounter[1];
    CurrentCommand_StepsCounter[0] = 0;
    CurrentCommand_StepsCounter[1] = 0;
    CurrentCommand_StepAddInc[0] = 0;
    CurrentCommand_StepAddInc[1] = 0;
  }
  
  // Stop the servo's motion

  // Check to see if there is a limp delay set. If so, start the countdown
  // timer that will limp the 3 motors after that delay.
///  if (EStopLimpDelayMS)
///  {
///    EStopLimpDelayTimerMS = EStopLimpDelayMS;
///  }
#endif
  if (printResult)
  {
    printf((far rom char *)"%d,%lu,%lu,%lu,%lu\n",
            command_interrupted,
            fifo_steps1,
            fifo_steps2,
            remaining_steps1,
            remaining_steps2
        );
    print_ack();
  }
}

// E-Stop
// Usage: ES<CR>
// Returns: <command_interrupted>,<fifo_steps1>,<fifo_steps2>,<steps_remaining1>,<steps_remaining2><CR>OK<CR>
// This command will abort any in-progress motor move (SM) command.
// It will also clear out any pending command(s) in the FIFO.
// <command_interrupted> = 0 if no FIFO or in-progress move commands were interrupted,
//                         1 if a motor move command was in progress or in the FIFO
// <fifo_steps1> and <fifo_steps1> = 24 bit unsigned integers with the number of steps
//                         in any SM command sitting in the fifo for axis1 and axis2.
// <steps_remaining1> and <steps_remaining2> = 24 bit unsigned integers with the number of
//                         steps left in the currently executing SM command (if any) for
//                         axis1 and axis2.
// It will return 0,0,0,0,0 if no SM command was executing at the time, and no SM
// command was in the FIFO.
void parseESCommand(void)
{
  process_EStop(TRUE);
}

#endif

// Do the work of the QM command so we can use this same code for QM and
// for QG commands.
uint8_t process_QM(void)
{
  uint8_t commandExecuting = 0;
  uint8_t motor1Running = 0;
  uint8_t motor2Running = 0;
  uint8_t motor3Running = 0;
  uint8_t FIFOStatus = 0;
  volatile MoveCommand_t * currentCommand;

  // Disable stepper interrupt briefly here to read out value that ISR uses
  TIM_TIM6InterruptStop();

  currentCommand = ISR_GetCurrentCommand();

  // Create our output values to print back to the PC
  if (currentCommand->Command != COMMAND_NONE)
  {
    commandExecuting = 1;
  }
  if (queue_GetSize() > 0)
  {
    FIFOStatus = 1;
  }
  if (commandExecuting && currentCommand->Data.Stepper.StepsCounter[0] != 0)
  {
    motor1Running = 1;
  }
  if (commandExecuting && currentCommand->Data.Stepper.StepsCounter[1] != 0)
  {
    motor2Running = 1;
  }
  if (commandExecuting && currentCommand->Data.Stepper.StepsCounter[2] != 0)
  {
    motor3Running = 1;
  }

  // Re-enable stepper interrupt
  TIM_TIM6InterruptStart();
    
  return ((motor3Running << 4) | (commandExecuting << 3) | (motor1Running << 2) | (motor2Running << 1) | FIFOStatus);
}

// Query General
// Three forms of the command:
// Form 1:
// Usage: QG<CR> or QG,0<CR>
// Returns: <status><NL><CR>
// <status> is a single byte, printed as a hexadecimal number from "00" to "FF".
// Each bit in the byte represents the status of a single bit of information in the EBB.
// Bit 1 : Motion FIFO status (0 = FIFO empty, 1 = FIFO not empty)
// Bit 2 : Motor2 status (0 = not moving, 1 = moving)
// Bit 3 : Motor1 status (0 = not moving, 1 = moving)
// Bit 4 : CommandExecuting (0 = no command currently executing, 1 = a command is currently executing)
// Bit 5 : Pen status (0 = up, 1 = down)
// Bit 6 : PRG button status (0 = not pressed since last query, 1 = pressed since last query)
// Bit 7 : GPIO Pin RB2 state (0 = low, 1 = high)
// Bit 8 : GPIO Pin RB5 state (0 = low, 1 = high)
// Just like the QB command, the PRG button status is cleared (after being printed) if pressed since last QB/QG command
//
// Form 2:
// Usage: QG,1<CR>
// Returns: <status>,<input_state><NL><CR>
// <status> is exactly as in Form 1
// <input_state> is for future expansion and will always be 0 for now
//
// Form 3:
// Usage: QG,2<CR>
// Returns: <status>,<input_state>,<queue_depth><NL><CR>
// <status> is exactly as in Form 1
// <input_state> is for future expansion and will always be 0 for now
// <queue_depth> is an unsigned 2 byte integer representing how many motion commands are currently
//   sitting in the motion queue, waiting to be executed. This value will never be more than the
//   <fifo_size> parameter set with the CU,3,<fifo_size> command or COMMAND_FIFO_LENGTH whichever
//   is smaller.
//
void stepper_QGCommand(void)
{
  uint8_t result = process_QM();
  uint8_t param = 0;

  extract_number (kUINT8, &param, kOPTIONAL);

  // process_QM() gives us the low 4 bits of our output result.
  result = result & 0x0F;

//  if (gPenStateActual)
//  {
//    result = result | (1 << 4);
//  }
//  if (ButtonPushed)
//  {
//    result = result | (1 << 5);
//  }
//  if (PORTBbits.RB2)
//  {
//    result = result | (1 << 6);
//  }
//  if (PORTBbits.RB5)
//  {
//    result = result | (1 << 7);
//  }

  if (param == 1)
  {
    printf("%02X,0\n", result);
  }
  else if (param == 2)
  {
    printf("%02X,0,%u\n", result, queue_GetSize());
  }
  else
  {
    printf("%02X\n", result);
  }

  // Reset the button pushed flag
//  if (ButtonPushed)
//  {
//    ButtonPushed = FALSE;
//  }
}


// QM command
// For Query Motor - returns the current status of each motor
// QM takes no parameters, so usage is just QM<CR>
// QM returns:
// QM,<CommandExecutingStatus>,<Motor1Satus>,<Motor2Status><CR>
// where:
//   <CommandExecutingStatus>: 0 if no 'motion command' is executing, > 0 if some 'motion command' is executing
//   <Motor1Status>: 0 if motor 1 is idle, 1 if motor is moving
//   <Motor2Status>: 0 if motor 2 is idle, 1 if motor is moving
//   <Motor3Status>: 0 if motor 3 is idle, 1 if motor is moving
//
// As of version 2.4.4, there is now a fourth parameter at the end of the reply packet.
// QM,<CommandExecutingStatus>,<Motor1Satus>,<Motor2Status>,<FIFOStatus><CR>
// Where <FIFOStatus> is either 1 (if there are any commands in the FIFO) or 0 (if the FIFO is empty)
void stepper_QMCommand(void)
{
  uint8_t CommandExecuting = 0;
  uint8_t Motor1Running = 0;
  uint8_t Motor2Running = 0;
  uint8_t Motor3Running = 0;
  uint8_t FIFOStatus = 0;
  uint8_t result = process_QM();

  if (result & 0x01)
  {
    FIFOStatus = 1;
  }
  if (result & 0x02)
  {
    Motor2Running = 1;
  }
  if (result & 0x04)
  {
    Motor1Running = 1;
  }
  if (result & 0x08)
  {
    CommandExecuting = 1;
  }
  if (result & 0x10)
  {
    Motor3Running = 1;
  }

  if(utility_LegacyModeEnabled())
  {
    printf("QM,%i,%i,%i,%i\n", CommandExecuting, Motor1Running, Motor2Running, FIFOStatus);
  }
  else
  {
    printf("QM,%i,%i,%i,%i,%i\n", CommandExecuting, Motor1Running, Motor2Running, Motor3Running, FIFOStatus);
  }
}

// QS command
// For Query Step position - returns the current x and y global step positions
// QS takes no parameters, so usage is just CS<CR>
// QS returns:
// QS,<global_step1_position>,<global_step2_position>,<global_step3_position><CR>
// where:
//   <global_step1_position>: signed 32 bit value, current global motor 1 step position
//   <global_step2_position>: signed 32 bit value, current global motor 2 step position
//   <global_step2_position>: signed 32 bit value, current global motor 3 step position
void stepper_QSCommand(void)
{
  int32_t step1, step2, step3;

  // Disable stepper interrupt briefly here to read out value that ISR uses
  TIM_TIM6InterruptStop();

  // Make a local copy of the things we care about
  step1 = Steppers[0].GlobalPosition;
  step2 = Steppers[1].GlobalPosition;
  step3 = Steppers[2].GlobalPosition;

  // Re-enable stepper interrupt
  TIM_TIM6InterruptStart();

  if(utility_LegacyModeEnabled())
  {
    printf("%li,%li\n", step1, step2);
  }
  else
  {
    printf("%li,%li,%li\n", step1, step2, step3);
  }

  print_ack();
}

// Perform the actual clearing of the step counters (used from several places)
void clear_StepCounters(void)
{
  // Disable stepper interrupt briefly here to read out value that ISR uses
  TIM_TIM6InterruptStop();

  // Make a local copy of the things we care about
  Steppers[0].GlobalPosition = 0;
  Steppers[1].GlobalPosition = 0;
  Steppers[2].GlobalPosition = 0;

  // Re-enable stepper interrupt
  TIM_TIM6InterruptStart();
}

// CS command
// For Clear Stepper position - zeros out both step1 and step2 global positions
// CS takes no parameters, so usage is just CS<CR>
// QS returns:
// OK<CR>
void stepper_CSCommand(void)
{
  clear_StepCounters();
  
  print_ack();
}

void stepper_Init(void)
{
  TIM_TIM6Start();
  stepper_Enable(false);
}

void stepper_Enable(bool state)
{
  if (state)
  {
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
  }
}
