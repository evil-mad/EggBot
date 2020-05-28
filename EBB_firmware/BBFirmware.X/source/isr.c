

#include <p18cxxx.h>
#include "isr.h"
#include "HardwareProfile.h"
#include "ebb.h"
#include "fifo.h"
#include "analog.h"
#include "delays.h"
#include "stepper.h"
#include "servo.h"
#include "solenoid.h"
#include "commands.h"

/// TODO: Move these into ISR function body?
static unsigned char OutByte;
static unsigned char TookStep;
static unsigned char AllDone;

static UINT32 StepAcc[NUMBER_OF_STEPPERS] = {0,0};


// Used only in LowISR

// ISR
#pragma interrupt high_ISR
void high_ISR(void)
{
  //Check which interrupt flag caused the interrupt.
  //Service the interrupt
  //Clear the interrupt flag
  //Etc.
  #if defined(USB_INTERRUPT)
    USBDeviceTasks();
  #endif

  // 25KHz ISR fire
  if (PIR1bits.TMR1IF)
  {
    // Clear the interrupt 
    PIR1bits.TMR1IF = 0;
    TMR1H = TIMER1_H_RELOAD;  //
    TMR1L = TIMER1_L_RELOAD;  // Reload for 25KHz ISR fire

    OutByte = FIFO_DirBits[FIFOOut];
    TookStep = FALSE;
    AllDone = TRUE;
PORTDbits.RD0 = 1;
    
    if (FIFODepth)
    {
PORTDbits.RD1 = 1;

      // Note, you don't even need a command to delay. Any command can have
      // a delay associated with it, if DelayCounter is != 0.
      if (FIFO_DelayCounter[FIFOOut])
      {
        // Double check that things aren't way too big
        if (FIFO_DelayCounter[FIFOOut] > HIGH_ISR_TICKS_PER_MS * (UINT32)0x10000)
        {
          FIFO_DelayCounter[FIFOOut] = 0;
        }
        else {
          FIFO_DelayCounter[FIFOOut]--;
        }
      }

      if (FIFO_DelayCounter[FIFOOut])
      {
          AllDone = FALSE;
      }

      // Not sure why this is here? For debugging? If so, then #ifdef it out for release build
      //PORTDbits.RD1 = 0;

      // Note: by not making this an else-if, we have our DelayCounter
      // counting done at the same time as our motor move or servo move.
      // This allows the delay time to start counting at the beginning of the
      // command execution.
      if (FIFO_Command[FIFOOut] == COMMAND_MOTOR_MOVE)
      {
        // Only output DIR bits if we are actually doing something
        if (FIFO_StepsCounter[0][FIFOOut] || FIFO_StepsCounter[1][FIFOOut])
        {
          if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
          {
            if (FIFO_DirBits[FIFOOut] & DIR1_BIT)
            {
              Dir1IO = 1;
            }
            else
            {
              Dir1IO = 0;
            }
            if (FIFO_DirBits[FIFOOut] & DIR2_BIT)
            {
              Dir2IO = 1;
            }
            else
            {
              Dir2IO = 0;
            }
          }
          else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
          {
            if (FIFO_DirBits[FIFOOut] & DIR1_BIT)
            {
              Dir1AltIO = 1;
            }
            else
            {
              Dir1AltIO = 0;
            }
            if (FIFO_DirBits[FIFOOut] & DIR2_BIT)
            {
              Dir2AltIO = 1;
            }
            else
            {
              Dir2AltIO = 0;
            }
          }

          // Only do this if there are steps left to take
          if (FIFO_StepsCounter[0][FIFOOut])
          {
            StepAcc[0] = StepAcc[0] + FIFO_StepAdd[0][FIFOOut];
            if (StepAcc[0] & 0x80000000)
            {
              StepAcc[0] = StepAcc[0] & 0x7FFFFFFF;
              OutByte = OutByte | STEP1_BIT;
              TookStep = TRUE;
              FIFO_StepsCounter[0][FIFOOut]--;
              if (FIFO_DirBits[FIFOOut] & DIR1_BIT)
              {
                globalStepCounter1--;
              }
              else
              {
                globalStepCounter1++;
              }
            }
            // For acceleration, we now add a bit to StepAdd each time through as well
            FIFO_StepAdd[0][FIFOOut] += FIFO_StepAddInc[0][FIFOOut];
            AllDone = FALSE;
          }
          if (FIFO_StepsCounter[1][FIFOOut])
          {
            StepAcc[1] = StepAcc[1] + FIFO_StepAdd[1][FIFOOut];
            if (StepAcc[1] & 0x80000000)
            {
              StepAcc[1] = StepAcc[1] & 0x7FFFFFFF;
              OutByte = OutByte | STEP2_BIT;
              TookStep = TRUE;
              FIFO_StepsCounter[1][FIFOOut]--;
              if (FIFO_DirBits[FIFOOut] & DIR2_BIT)
              {
                globalStepCounter2--;
              }
              else
              {
                globalStepCounter2++;
              }
            }
            // For acceleration, we now add a bit to StepAdd each time through as well
            FIFO_StepAdd[1][FIFOOut] += FIFO_StepAddInc[1][FIFOOut];
            AllDone = FALSE;
          }

          if (TookStep)
          {
            if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
            {
              if (OutByte & STEP1_BIT)
              {
                Step1IO = 1;
              }
              if (OutByte & STEP2_BIT)
              {
                Step2IO = 1;
              }
            }
            else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
            {
              if (OutByte & STEP1_BIT)
              {
                Step1AltIO = 1;
              }
              if (OutByte & STEP2_BIT)
              {
                Step2AltIO = 1;
              }
            }
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            Delay1TCY();
            if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
            {
              Step1IO = 0;
              Step2IO = 0;
            }
            else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
            {
              Step1AltIO = 0;
              Step2AltIO = 0;
            }
          }
        }
      }
      // Check to see if we should change the state of the pen
      else if (FIFO_Command[FIFOOut] == COMMAND_SERVO_MOVE)
      {
        if (gUseRCPenServo)
        {
          // Precompute the channel, since we use it all over the place
          UINT8 Channel = FIFO_ServoChannel[FIFOOut] - 1;

          // This code below is the meat of the servo_Move() function
          // We have to manually write it in here rather than calling
          // the function because a real function inside the ISR
          // causes the compiler to generate enormous amounts of setup/teardown
          // code and things run way too slowly.

          // If the user is trying to turn off this channel's RC servo output
          if (0 == FIFO_ServoPosition[FIFOOut])
          {
            // Turn off the PPS routing to the pin
            *(gRC2RPORPtr + gRC2RPn[Channel]) = 0;
            // Clear everything else out for this channel
            gRC2Rate[Channel] = 0;
            gRC2Target[Channel] = 0;
            gRC2RPn[Channel] = 0;
            gRC2Value[Channel] = 0;
          }
          else
          {
            // Otherwise, set all of the values that start this RC servo moving
            gRC2Rate[Channel] = FIFO_ServoRate[FIFOOut];
            gRC2Target[Channel] = FIFO_ServoPosition[FIFOOut];
            gRC2RPn[Channel] = FIFO_ServoRPn[FIFOOut];
            if (gRC2Value[Channel] == 0)
            {
              gRC2Value[Channel] = FIFO_ServoPosition[FIFOOut];
            }
          }
        }

        // If this servo is the pen servo (on g_servo2_RPn)
        if (FIFO_ServoRPn[FIFOOut] == g_servo2_RPn)
        {
          // Then set its new state based on the new position
          if (FIFO_ServoPosition[FIFOOut] == g_servo2_min)
          {
            PenState = PEN_UP;
            SolenoidState = SOLENOID_OFF;
            if (gUseSolenoid)
            {
              PenUpDownIO = 0;
            }
          }
          else
          {
            PenState = PEN_DOWN;
            SolenoidState = SOLENOID_ON;
            if (gUseSolenoid)
            {
              PenUpDownIO = 1;
            }
          }
        }
      }
      // Check to see if we should start or stop the engraver
      else if (FIFO_Command[FIFOOut] == COMMAND_SE)
      {
        // Now act on the State of the SE command
        if (FIFO_SEState[FIFOOut])
        {
          // Set RB3 to StoredEngraverPower
          CCPR1L = FIFO_SEPower[FIFOOut] >> 2;
          CCP1CON = (CCP1CON & 0b11001111) | ((StoredEngraverPower << 4) & 0b00110000);
        }
        else
        {
          // Set RB3 to low by setting PWM duty cycle to zero
          CCPR1L = 0;
          CCP1CON = (CCP1CON & 0b11001111);
        }
        AllDone = TRUE;
      }

      // If we're done with our current command, load in the next one, if there's more
      if (AllDone && FIFO_DelayCounter[FIFOOut] == 0)
      {
        // "Erase" the current command from the FIFO
        FIFO_Command[FIFOOut] = COMMAND_NONE;

        // There should be at least one command in FIFODepth right now (the one we just finished)
        // Remove it
        FIFOOut++;
        if (FIFOOut >= COMMAND_FIFO_LENGTH)
        {
          FIFOOut = 0;
        }
        FIFODepth--;
      }
    }
    
    // Check for button being pushed
    if (
      (!swProgram)
      ||
      (
        UseAltPause
        &&
        !PORTBbits.RB0
      )
    )
    {
      ButtonPushed = TRUE;
    }
  }
PORTDbits.RD0 = 0;
PORTDbits.RD1 = 0;
}


#pragma interruptlow low_ISR
void low_ISR(void)
{
  unsigned int i;
  signed int RC2Difference = 0;

  // Do we have a Timer4 interrupt? (1ms rate)
  if (PIR3bits.TMR4IF)
  {
    // Clear the interrupt 
    PIR3bits.TMR4IF = 0;

    // Handle RC servo pulse generation (for next pulse/channel)
    // Always increment the gRCServo2msCounter
    gRC2msCounter++;

    if (gRC2msCounter >= gRC2SlotMS)
    {
      // Clear the RC2 ms counter
      gRC2msCounter = 0;

      // Turn off the PPS routing to the 'old' pin
      *(gRC2RPORPtr + gRC2RPn[gRC2Ptr]) = 0;

      // Turn off TIMER3 for now
      T3CONbits.TMR3ON = 0;

      // And clear TIMER3 to zero
      TMR3H = 0;
      TMR3L = 0;

      // And always advance the main pointer
      gRC2Ptr++;
      if (gRC2Ptr >= gRC2Slots)
      {
        gRC2Ptr = 0;
      }

      // If the value is zero, we do nothing to this pin
      // otherwise, prime it for sending a pulse
      if (gRC2Value[gRC2Ptr] != 0)
      {
        // Now, to move 'slowly', we update gRC2Value[] by
        // seeing if we are at gRC2Target[] yet. If not, then
        // we add (or subtract) gRC2Rate[] to try and get there.
        if (gRC2Target[gRC2Ptr] != gRC2Value[gRC2Ptr])
        {
          // If the rate is zero, then we always move instantly
          // to the target.
          if (gRC2Rate[gRC2Ptr] == 0)
          {
            gRC2Value[gRC2Ptr] = gRC2Target[gRC2Ptr];
          }
          else
          {
            // Otherwise, add gRC2Rate[] each time through until we
            // get to our desired pulse width.
            RC2Difference = (gRC2Target[gRC2Ptr] - gRC2Value[gRC2Ptr]);
            if (RC2Difference > 0)
            {
              if (RC2Difference > gRC2Rate[gRC2Ptr])
              {
                gRC2Value[gRC2Ptr] += gRC2Rate[gRC2Ptr];
              }
              else
              {
                gRC2Value[gRC2Ptr] = gRC2Target[gRC2Ptr];
              }
            }
            else
            {
              if (-RC2Difference > gRC2Rate[gRC2Ptr])
              {
                gRC2Value[gRC2Ptr] -= gRC2Rate[gRC2Ptr];
              }
              else
              {
                gRC2Value[gRC2Ptr] = gRC2Target[gRC2Ptr];
              }
            }

          }
        }

        // Set up the PPS routing for the CCP2
        *(gRC2RPORPtr + gRC2RPn[gRC2Ptr]) = 18; // 18 = CCP2

        // Disable interrupts (high)
        INTCONbits.GIEH = 0;

        // Load up the new compare time
        CCPR2H = gRC2Value[gRC2Ptr] >> 8;
        CCPR2L = gRC2Value[gRC2Ptr] & 0xFF;
        CCP2CONbits.CCP2M = 0b0000;
        CCP2CONbits.CCP2M = 0b1001;

        // Turn TIMER3 back on
        T3CONbits.TMR3ON = 1;

        // Re-enable interrupts
        INTCONbits.GIEH = 1;
      }
    }

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

    // Software timer for QC command
    if (QC_ms_timer)
    {
      QC_ms_timer--;
    }

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

  } // end of 1ms interrupt

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
      ISR_A_FIFO[A_cur_channel] =
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
}
