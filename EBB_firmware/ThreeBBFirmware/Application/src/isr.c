#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "isr.h"
#include "HardwareProfile.h"
#include "debug.h"
//#include "ebb.h"
#include "fifo.h"
//#include "analog.h"
//#include "delays.h"
#include "stepper.h"
#include "servo.h"
//#include "solenoid.h"
#include "commands.h"
#include "utility.h"

// Constantly incrementing every 1ms in ISR global tick counter
volatile uint32_t TickCounterMS;

// Constantly decrementing until it reaches zero. Use in local functions for
// short local delays from 1 to 255 milliseconds
volatile uint8_t GlobalDelayMS;

volatile uint8_t DriverInitDelayMS;

// ISR now at 100KHz
void high_ISR(void)
{
  // Accumulators (at 25Khz) used to determine when to take a step
  static uint32_t StepAcc[NUMBER_OF_STEPPERS];
  static uint8_t OutByte;
  static bool TookStep;
  static bool AllDone;
  static MoveCommandType move = {0};
  TookStep = false;
  AllDone = true;

///DEBUG_G0_SET();
  // If we are not already processing a command, see if there are any
  // waiting for us on the queue
  if (move.Command == COMMAND_NONE)
  {
    if (queue_PullNextCommand(&move) == false)
    {
      // Nope, queue is empty. So just mark that we do not have a command
      // to process and be done
      move.Command = COMMAND_NONE;
      return;
    }
  }

  if (move.Command == COMMAND_MOTOR_MOVE)
  {
    /// TODO: We only need to set up and output the DIR bits once, when we first start the move. No need to waste time after that.

    // Only output DIR bits if we are actually doing something
    if (
         move.Data.Stepper.StepsCounter[0]
         ||
         move.Data.Stepper.StepsCounter[1]
         ||
         move.Data.Stepper.StepsCounter[2]
    )
    {
      if (move.Data.Stepper.DirBits & DIR1_BIT)
      {
        DIR1_GPIO_Port->BSRR = (uint32_t)DIR1_Pin;
      }
      else
      {
        DIR1_GPIO_Port->BRR = (uint32_t)DIR1_Pin;
      }
      if (move.Data.Stepper.DirBits & DIR2_BIT)
      {
        DIR2_GPIO_Port->BSRR = (uint32_t)DIR2_Pin;
      }
      else
      {
        DIR2_GPIO_Port->BRR = (uint32_t)DIR2_Pin;
      }
      if (move.Data.Stepper.DirBits & DIR3_BIT)
      {
        DIR3_GPIO_Port->BSRR = (uint32_t)DIR3_Pin;
      }
      else
      {
        DIR3_GPIO_Port->BRR = (uint32_t)DIR3_Pin;
      }

      OutByte = move.Data.Stepper.DirBits;

      // Only do this if there are steps left to take
      if (move.Data.Stepper.StepsCounter[0])
      {
        StepAcc[0] = StepAcc[0] + move.Data.Stepper.StepAdd[0];
        if (StepAcc[0] & 0x80000000)
        {
          StepAcc[0] = StepAcc[0] & 0x7FFFFFFF;
          OutByte = OutByte | STEP1_BIT;
          TookStep = true;
          move.Data.Stepper.StepsCounter[0]--;
          if (move.Data.Stepper.DirBits & DIR1_BIT)
          {
            globalStepCounter1--;
          }
          else
          {
            globalStepCounter1++;
          }
        }
        // For acceleration, we now add a bit to StepAdd each time through as well
        move.Data.Stepper.StepAdd[0] += move.Data.Stepper.StepAddInc[0];
        AllDone = false;
      }
      if (move.Data.Stepper.StepsCounter[1])
      {
        StepAcc[1] = StepAcc[1] + move.Data.Stepper.StepAdd[1];
        if (StepAcc[1] & 0x80000000)
        {
          StepAcc[1] = StepAcc[1] & 0x7FFFFFFF;
          OutByte = OutByte | STEP2_BIT;
          TookStep = true;
          move.Data.Stepper.StepsCounter[1]--;
          if (move.Data.Stepper.DirBits & DIR2_BIT)
          {
            globalStepCounter2--;
          }
          else
          {
            globalStepCounter2++;
          }
        }
        // For acceleration, we now add a bit to StepAdd each time through as well
        move.Data.Stepper.StepAdd[1] += move.Data.Stepper.StepAddInc[1];
        AllDone = false;
      }
      if (move.Data.Stepper.StepsCounter[2])
      {
        StepAcc[2] = StepAcc[2] + move.Data.Stepper.StepAdd[2];
        if (StepAcc[2] & 0x80000000)
        {
          StepAcc[2] = StepAcc[2] & 0x7FFFFFFF;
          OutByte = OutByte | STEP3_BIT;
          TookStep = true;
          move.Data.Stepper.StepsCounter[2]--;
          if (move.Data.Stepper.DirBits & DIR3_BIT)
          {
            globalStepCounter3--;
          }
          else
          {
            globalStepCounter3++;
          }
        }
        // For acceleration, we now add a bit to StepAdd each time through as well
        move.Data.Stepper.StepAdd[2] += move.Data.Stepper.StepAddInc[2];
        AllDone = false;
      }
      if (TookStep)
      {
        if (OutByte & STEP1_BIT)
        {
          STEP1_GPIO_Port->BSRR = (uint32_t)STEP1_Pin;
        }
        if (OutByte & STEP2_BIT)
        {
          STEP2_GPIO_Port->BSRR = (uint32_t)STEP2_Pin;
        }
        if (OutByte & STEP3_BIT)
        {
          STEP3_GPIO_Port->BSRR = (uint32_t)STEP3_Pin;
        }
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
        if (OutByte & STEP1_BIT)
        {
          STEP1_GPIO_Port->BRR = (uint32_t)STEP1_Pin;
        }
        if (OutByte & STEP2_BIT)
        {
          STEP2_GPIO_Port->BRR = (uint32_t)STEP2_Pin;
        }
        if (OutByte & STEP3_BIT)
        {
          STEP3_GPIO_Port->BRR = (uint32_t)STEP3_Pin;
        }
      }
    }
  }
  // Check to see if we should start or stop the engraver
  else if (move.Command == COMMAND_SE)
  {
    // Now act on the State of the SE command
    if (move.Data.Engraver.SEState)
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
  else if (move.Command == COMMAND_SERVO_MOVE)
  {
    // Set up a new target and rate for one of the servos
    servo_SetTarget(move.Data.Servo.ServoPosition, move.Data.Servo.ServoPin, move.Data.Servo.ServoRate);
    AllDone = true;
  }
  // Note that we can have a delay with a COMMAND_DELAY or a COMMAND_SERVO_MOVE
  // That's why this is not an elseif here.
  if (
    move.Command == COMMAND_DELAY
    ||
    move.Command == COMMAND_SERVO_MOVE
  )
  {
    if (move.DelayCounter)
    {
      // Double check that things aren't way too big
      if (move.DelayCounter > HIGH_ISR_TICKS_PER_MS * (uint32_t)0x10000)
      {
        move.DelayCounter = 0;
      }
      else
      {
        move.DelayCounter--;
      }
    }

    if (move.DelayCounter)
    {
      AllDone = false;
    }
  }

  // If we're done with our current command, load in the next one, if there's more
  if (AllDone)
  {
    // "Erase" the current command from the queue
    move.Command = COMMAND_NONE;
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
