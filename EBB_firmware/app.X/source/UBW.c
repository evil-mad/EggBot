/*********************************************************************
 *
 *                UBW Firmware
 *
 *********************************************************************
 * FileName:        UBW.c
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
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       11/19/04    Original.
 * Brian Schmalz        03/15/06    Added user code to implement
 *                      firmware version D v1.0 for UBW
 *                      project. See www.greta.dhs.org/UBW
 * Brian Schmalz        05/04/06    Starting version 1.1, which will 
 *                      include several fixes. See website.
 * BPS                  06/21/06  Starting v1.2 -
 * - Fixed problem with I packets (from T command) filling up TX buffer
 *    and not letting any incoming commands be received. (strange)
 * - Adding several commands - Analog inputs being the biggest set.
 * - Also Byte read/Byte write (PEEK/POKE) anywhere in memory
 * - Individual pin I/O and direction
 * BPS                  08/16/06  v1.3 - Fixed bug with USB startup
 * BPS                  09/09/06  v1.4 - Starting 1.4
 * - Fixed Microchip bug with early silicon - UCONbits.PKTDIS = 0;
 * - Adding BO and BC commands for parallel output to graphics panels
 * BPS                  12/06/06  v1.4 - More work on 1.4
 * - Re-wrote all I/O buffering code for increased speed and functionality
 * - Re-wrote error handling code
 * - Added delays to BC/BO commands to help Corey
 * BPS                  01/06/07  v1.4 - Added RC command for servos
 * BPS                  03/07/07  v1.4.1 - Changed blink rate for SFE
 * BPS                  05/24/07  v1.4.2 - Fixed RC command bug - it
 *                      wouldn't shut off.
 * BPS                  08/28/07  v1.4.3 - Allowed UBW to run without
 *                      USB connected.
 *
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include <usart.h>
#include <ctype.h>
#include <delays.h>
#include <flash.h>
#include "Usb\usb.h"
#include "Usb\usb_function_cdc.h"
#include "usb_config.h"
#include "HardwareProfile.h"
#include "UBW.h"
#include "ebb.h"
#include "RCServo2.h"
#include "ebb_print.h"

/** D E F I N E S ********************************************************/

#define kUSART_TX_BUF_SIZE    10u               // In bytes
#define kUSART_RX_BUF_SIZE    10u               // In bytes

#define kISR_FIFO_A_DEPTH     3u
#define kISR_FIFO_D_DEPTH     3u
#define kPR4_RELOAD           250u              // For 1ms TMR4 tick
#define kCR                   0x0Du
#define kLF                   0x0Au

#define ANALOG_INITATE_MS_BETWEEN_STARTS 5u     // Number of ms between analog converts (all enabled channels)

#define FLASH_NAME_ADDRESS    0xF800            // Starting address in FLASH where we store our EBB's name
#define FLASH_NAME_LENGTH     16u               // Size of store for EBB's name in FLASH

#define RCSERVO_POWEROFF_DEFAULT_MS (60ul*1000ul)  // Number of milliseconds to default the RCServo power autotimeout (5min)

/** V A R I A B L E S ********************************************************/
//#pragma udata access fast_vars

// Rate variable - how fast does interrupt fire to capture inputs?
unsigned int time_between_updates;

volatile unsigned int ISR_D_RepeatRate;       // How many 1ms ticks between Digital updates
volatile unsigned char ISR_D_FIFO_in;         // In pointer
volatile unsigned char ISR_D_FIFO_out;        // Out pointer
volatile unsigned char ISR_D_FIFO_length;     // Current FIFO depth

volatile unsigned int ISR_A_RepeatRate;       // How many 1ms ticks between Analog updates
volatile unsigned char ISR_A_FIFO_in;         // In pointer
volatile unsigned char ISR_A_FIFO_out;        // Out pointer
volatile unsigned char ISR_A_FIFO_length;     // Current FIFO depth

// This byte has each of its bits used as a separate error flag
BYTE error_byte;

// RC servo variables
// First the main array of data for each servo
unsigned char g_RC_primed_ptr;
unsigned char g_RC_next_ptr;
unsigned char g_RC_timing_ptr;

// Used only in LowISR
unsigned int D_tick_counter;
unsigned int A_tick_counter;
unsigned char A_cur_channel;
unsigned char AnalogInitiate;
volatile unsigned int AnalogEnabledChannels;
volatile unsigned int ChannelBit;
volatile UINT16 g_PowerMonitorThresholdADC;     // 0-1023 ADC counts, below which
volatile BOOL g_PowerDropDetected;              // True if power drops below PowerMonitorThreshold

volatile tStepperDisableTimeout g_StepperDisableState;  // Stores state of stepper timeout disable feature
volatile UINT16 g_StepperDisableTimeoutS;       // Seconds of no motion before motors are disabled
volatile UINT16 g_StepperDisableSecondCounter;  // Counts milliseconds up to 1 s for stepper disable timeout
volatile UINT16 g_StepperDisableCountdownS;     // After motion is done, counts down in seconds from g_StepperDisableTimeoutS to zero

const rom char st_version[] = {"EBBv13_and_above EB Firmware Version 3.0.0-a29"};

#pragma udata ISR_buf = 0x100
volatile unsigned int ISR_A_FIFO[16];                     // Stores the most recent analog conversions
volatile unsigned char ISR_D_FIFO[3][kISR_FIFO_D_DEPTH];  // FIFO of actual data

#pragma udata com_tx_buf = 0x200
// USB Transmit buffer for packets (back to PC)
unsigned char g_TX_buf[kTX_BUF_SIZE];

unsigned char g_RX_command_buf[kRX_COMMAND_BUF_SIZE];

#pragma udata com_rx_buf = 0x300
// USB Receiving buffer for commands as they come from PC
unsigned char g_RX_buf[kRX_BUF_SIZE];

// These variables are in normal storage space
#pragma udata

// USART Receiving buffer for data coming from the USART
unsigned char g_USART_RX_buf[kUSART_RX_BUF_SIZE];

// USART Transmit buffer for data going to the USART
unsigned char g_USART_TX_buf[kUSART_TX_BUF_SIZE];

// These are used for the Fast Parallel Output routines
unsigned char g_BO_init;
unsigned char g_BO_strobe_mask;
unsigned char g_BO_wait_mask;
unsigned char g_BO_wait_delay;
unsigned char g_BO_strobe_delay;

// Pointers to USB transmit (back to PC) buffer
unsigned char g_TX_buf_in;
unsigned char g_TX_buf_out;

// Pointers to USB receive (from PC) buffer
unsigned char g_RX_buf_in;
unsigned char g_RX_buf_out;

// In and out pointers to our USART input buffer
unsigned char g_USART_RX_buf_in;
unsigned char g_USART_RX_buf_out;

// In and out pointers to our USART output buffer
unsigned char g_USART_TX_buf_in;
unsigned char g_USART_TX_buf_out;

// Normally set to TRUE. Able to set FALSE to not send "OK" message after packet reception
BOOL g_ack_enable;

#if PC_PG_T_COMMANDS_ENABLED
// Set to TRUE to turn Pulse Mode on
unsigned char gPulsesOn;
// For Pulse Mode, how long should each pulse be on for in ms?
unsigned int gPulseLen[4];
// For Pulse Mode, how many ms between rising edges of pulses?
unsigned int gPulseRate[4];
// For Pulse Mode, counters keeping track of where we are
unsigned int gPulseCounters[4];
#endif

// Counts down milliseconds until zero. At zero shuts off power to RC servo (via RA3))
volatile UINT32 gRCServoPoweroffCounterMS;
volatile UINT32 gRCServoPoweroffCounterReloadMS;

// When true, any stepper motion command will automatically enable both motors
BOOL gAutomaticMotorEnable;

// These store the first and second characters of the latest command from the PC
unsigned char gCommand_Char1;
unsigned char gCommand_Char2;

// Global variables used for limit switch feature
volatile UINT8 gLimitSwitchPortB;     // Latched PortB value when trigger happens
volatile UINT8 gLimitSwitchReplies;   // Non-zero if we want a reply printed when limit switch trigger goes true
UINT8 gLimitSwitchReplyPrinted;       // True if we've already printed a reply for this limit switch trigger

UINT8 gCommandChecksumRequired;       // True if we are requiring checksums on all commands (defaults to off)

// Stack high water value
static volatile UINT16 gStackHighWater;

// Locals used in low ISR
static INT16 RC2Difference;
static UINT8 tempStackPointerHigh;
static UINT8 tempStackPointerLow;
static UINT16 tempStackPointer;

static unsigned char gDeviceStringName[FLASH_NAME_LENGTH+1];

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void BlinkUSBStatus(void);     // Handles blinking the USB status LED
BOOL SwitchIsPressed(void);    // Check to see if the user (PRG) switch is pressed
void parse_packet(void);       // Take a full packet and dispatch it to the right function
signed char extract_digit(unsigned long * acc, unsigned char digits); // Pull a character out of the packet
void parse_R_packet(void);     // R for resetting UBW
void parse_C_packet(void);     // C for configuring I/O and analog pins
void parse_O_packet(void);     // O for output digital to pins
void parse_I_packet(void);     // I for input digital from pins
void parse_V_packet(void);     // V for printing version
void parse_A_packet(void);     // A for requesting analog inputs
void parse_PI_packet(void);    // PI for reading a single pin
void parse_PO_packet(void);    // PO for setting a single pin state
void parse_PD_packet(void);    // PD for setting a pin's direction
void parse_MR_packet(void);    // MR for Memory Read
void parse_MW_packet(void);    // MW for Memory Write
void parse_CU_packet(void);    // CU configures UBW (system wide parameters)
#if PC_PG_T_COMMANDS_ENABLED
void parse_T_packet(void);     // T for setting up timed I/O (digital or analog)
void parse_PG_packet(void);    // PG Pulse Go
void parse_PC_packet(void);    // PC Pulse Configure
#endif
void parse_BL_packet(void);    // BL Boot Load command
void parse_CK_packet(void);    // CK ChecK command
void parse_MR_packet(void);    // MR Motors Run command
void parse_AC_packet(void);    // AC Analog Configure
void parse_ST_packet(void);    // ST Set Tag command
void parse_QT_packet(void);    // QT Query Tag command
void parse_RB_packet(void);    // RB ReBoot command
void parse_QR_packet(void);    // QR Query RC Servo power state
void parse_SR_packet(void);    // SR Set RC Servo power timeout
void parse_QU_packet(void);    // QU General Query

void check_and_send_TX_data(void); // See if there is any data to send to PC, and if so, do it

/** D E C L A R A T I O N S **************************************************/
#pragma code

#pragma interruptlow low_ISR
void low_ISR(void)
{
  UINT8 i;

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
      if (gRC2Value[gRC2Ptr] != 0u)
      {
        // Now, to move 'slowly', we update gRC2Value[] by
        // seeing if we are at gRC2Target[] yet. If not, then
        // we add (or subtract) gRC2Rate[] to try and get there.
        if (gRC2Target[gRC2Ptr] != gRC2Value[gRC2Ptr])
        {
          // If the rate is zero, then we always move instantly
          // to the target.
          if (gRC2Rate[gRC2Ptr] == 0u)
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
              if (RC2Difference > (INT16)gRC2Rate[gRC2Ptr])
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
              if (-RC2Difference > (INT16)gRC2Rate[gRC2Ptr])
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

    // See if it's time to fire off an I packet
    if (ISR_D_RepeatRate > 0u)
    {
      D_tick_counter++;
      if (D_tick_counter >= ISR_D_RepeatRate)
      {
        D_tick_counter = 0;
        // Tell the main code to send an I packet
        if (ISR_D_FIFO_length < kISR_FIFO_D_DEPTH)
        {
          // And copy over our port values
          ISR_D_FIFO[0][ISR_D_FIFO_in] = PORTA;
          ISR_D_FIFO[1][ISR_D_FIFO_in] = PORTB;
          ISR_D_FIFO[2][ISR_D_FIFO_in] = PORTC;
          ISR_D_FIFO_in++;
          if (ISR_D_FIFO_in >= kISR_FIFO_D_DEPTH)
          {
            ISR_D_FIFO_in = 0;
          }
          ISR_D_FIFO_length++;
        }
        else
        {
          // Stop the madness! Something is wrong, we're
          // not getting our packets out. So kill the 
          // timer.
          ISR_D_RepeatRate = 0;
        }
      }
    }

    // See if it's time to fire off an A packet
    if ((ISR_A_RepeatRate > 0u) && (AnalogEnabledChannels > 0u))
    {
      A_tick_counter++;
      if (A_tick_counter >= ISR_A_RepeatRate)
      {
        A_tick_counter = 0;
        // Tell the main code to send an A packet
        if (ISR_A_FIFO_length < kISR_FIFO_A_DEPTH)
        {
          ISR_A_FIFO_in++;
          if (ISR_A_FIFO_in >= kISR_FIFO_A_DEPTH)
          {
            ISR_A_FIFO_in = 0;
          }
          ISR_A_FIFO_length++;
        }
        else
        {
          // Stop the madness! Something is wrong, we're
          // not getting our packets out. So kill the A
          // packets.
          ISR_A_RepeatRate = 0;
        }
      }
    }

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

#if PC_PG_T_COMMANDS_ENABLED
    // Is Pulse Mode on?
    if (gPulsesOn)
    {
      // Loop across the four pins
      for (i=0; i<4u; i++)
      {
        // Only pulse the pin if there is a length for the pin
        if (gPulseLen[i])
        {
          // If this is the beginning of the pulse, turn the pin on
          if (gPulseCounters[i] == 0u)
          {
            // Turn the pin
            if (i==0u) PORTBbits.RB0 = 1;
            if (i==1u) PORTBbits.RB1 = 1;
            if (i==2u) PORTBbits.RB2 = 1;
            if (i==3u) PORTBbits.RB3 = 1;
          }

          // If we've reached the end of the pulse, turn the pin off
          if (gPulseCounters[i] == gPulseLen[i])
          {
            // Turn the pin off
            if (i==0u) PORTBbits.RB0 = 0;
            if (i==1u) PORTBbits.RB1 = 0;
            if (i==2u) PORTBbits.RB2 = 0;
            if (i==3u) PORTBbits.RB3 = 0;
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
          if (i==0u) PORTBbits.RB0 = 0;
          if (i==1u) PORTBbits.RB1 = 0;
          if (i==2u) PORTBbits.RB2 = 0;
          if (i==3u) PORTBbits.RB3 = 0;
        }
      }
    }
#endif
    
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
      if (gRCServoPoweroffCounterMS == 0u)
      {
        RCServoPowerIO = RCSERVO_POWER_OFF;
      }
    }
    
    // Check for stepper motor disable timeout if enabled
    if (g_StepperDisableState == kSTEPPER_TIMEOUT_TIMING)
    {
      // Count the milliseconds until we get to 1 second
      if (g_StepperDisableSecondCounter)
      {
        g_StepperDisableSecondCounter--;
        
        if (g_StepperDisableSecondCounter == 0u)
        {
          // Then count down the seconds
          if (g_StepperDisableCountdownS)
          {
            g_StepperDisableCountdownS--;

            if (g_StepperDisableCountdownS == 0u)
            {
              // If the countdown gets to zero, then it's time to disable 
              // the steppers.
              if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
              {
                Enable1IO = DISABLE_MOTOR;
                Enable2IO = DISABLE_MOTOR;
              }
              else
              {
                Enable1AltIO = DISABLE_MOTOR;
                Enable2AltIO = DISABLE_MOTOR;
              }
              g_StepperDisableState = kSTEPPER_TIMEOUT_FIRED;
            }
            else
            {
              // Only count the next second if there are still seconds to count
              g_StepperDisableSecondCounter = 1000u;
            }
          }
        }
      }
    }

    // Read out the current value of FSR1 (stack pointer)
    // If the new value is higher than gStackHighWater, then update gStackHighWater
    // This will record the highest value the stack pointer attains
///    _asm
///      MOVFF FSR1H, tempStackPointerHigh
///      MOVFF FSR1L, tempStackPointerLow
///    _endasm
///    tempStackPointer = ((((UINT16)tempStackPointerHigh) << 8) | tempStackPointerLow);
///    if (tempStackPointer > gStackHighWater)
///    {
///      gStackHighWater = tempStackPointer;
///    }
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

      // If this is the V+_VOLTAGE ADC channel, then check to see if the value
      // is below the threshold, and if so, set the bit to record this fact
      if (A_cur_channel == RA11_VPLUS_POWER_ADC_CHANNEL)
      {
        if (ISR_A_FIFO[A_cur_channel] < g_PowerMonitorThresholdADC)
        {
          g_PowerDropDetected = TRUE;
        }
      }
      
      // Increment the channel and mask bit
      ChannelBit = ChannelBit << 1;
      A_cur_channel++;
    }

    // Walk through the enabled channels until we find the next one
    while (A_cur_channel < 16u)
    {
      if (ChannelBit & AnalogEnabledChannels)
      {
        break;
      }
      else
      {
        // Increment the channel and write the new one in
        A_cur_channel++;
        ChannelBit = ChannelBit << 1;
      }
    }

    if (A_cur_channel >= 16u)
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

//////// JUST FOR TESTING - DELETE

void fill_stack(void)
{
  UINT8 * stackPtr = (UINT8 *)0x000;

  _asm
    MOVFF FSR1H, tempStackPointerHigh
    MOVFF FSR1L, tempStackPointerLow
  _endasm

  tempStackPointer = ((((UINT16)tempStackPointerHigh) << 8) | tempStackPointerLow);

  stackPtr = (UINT8 *)tempStackPointer;
  
  stackPtr += 8;
  
  while ((UINT16)stackPtr <= 0xEBFu)
  {
    *stackPtr = 0xEE;
    stackPtr++;
  }
}

// Walk backwards in the stack RAM section looking of 0xEEs. When we stop
// seeing them, then we know that's the highest value of the stack to this point.
// Print that location out.
void check_high_water(void)
{
  UINT8 nib2;
  UINT8 * stackPtr = (UINT8 *)0xEBF;
  
  INTCONbits.GIEL = 1;  // Turn low priority interrupts off
  
  while (*stackPtr == 0xEE)
  {
    stackPtr--;
  }
  
  if ((UINT16)stackPtr > gStackHighWater)
  {
    gStackHighWater = (UINT16)stackPtr;
  }

  INTCONbits.GIEL = 1;  // Turn low priority interrupts on  
}

///////

void UserInit(void)
{
  UINT16 i;
  
  // Make all of 3 digital inputs
  LATA = 0x00;
  TRISA = 0xFF;
  // Turn all analog inputs into digital inputs
  //  ADCON1 = 0x0F;
  // Turn off the ADC
  //  ADCON0bits.ADON = 0;
  // Turn off our own idea of how many analog channels to convert
  AnalogEnabledChannels = 0;
  // Make all of PORTB inputs
  LATB = 0x00;
  TRISB = 0xFF;
  // Make all of PORTC inputs
  LATC = 0x00;
  TRISC = 0xFF;

  // Initialize LED I/Os to outputs
  mInitAllLEDs();
  // Initialize switch as an input
  mInitSwitch();

  // Start off always using "OK" acknowledge.
  g_ack_enable = TRUE;

  // This has been missing for a long time and created problems on boot if not
  // set to zero.
  error_byte = 0;
  
  // Initialize all of the ISR FIFOs
  ISR_A_FIFO_out = 0;
  ISR_A_FIFO_in = 0;
  ISR_A_FIFO_length = 0;
  ISR_D_FIFO_out = 0;
  ISR_D_FIFO_in = 0;
  ISR_D_FIFO_length = 0;

  // Make sure that our timer stuff starts out disabled
  ISR_D_RepeatRate = 0;
  ISR_A_RepeatRate = 0;
  D_tick_counter = 0;
  A_tick_counter = 0;
  A_cur_channel = 0;

  // Now init our registers
  // Initialize Timer4
  // The prescaler will be at 16
  T4CONbits.T4CKPS1 = 1;
  T4CONbits.T4CKPS0 = 1;
  // We want the TMR4 post scaler to be a 3
  T4CONbits.T4OUTPS3 = 0;
  T4CONbits.T4OUTPS2 = 0;
  T4CONbits.T4OUTPS1 = 1;
  T4CONbits.T4OUTPS0 = 0;
  // Set our reload value
  PR4 = kPR4_RELOAD;

  // Set up the Analog to Digital converter
  // Clear out the FIFO data
  for (i = 0; i < 16u; i++)
  {
    ISR_A_FIFO[i] = 0;
  }

  // Initialize USB TX and RX buffer management
  g_RX_buf_in = 0;
  g_RX_buf_out = 0;
  g_TX_buf_in = 0;
  g_TX_buf_out = 0;

  for (i=0; i < kTX_BUF_SIZE; i++)
  {
    g_TX_buf[i] = 0;
  }
  for (i=0; i < kRX_COMMAND_BUF_SIZE; i++)
  {
    g_RX_command_buf[i] = 0;
  }
  for (i=0; i < kRX_BUF_SIZE; i++)
  {
    g_RX_buf[i] = 0;
  }
    
  // And the USART TX and RX buffer management
  g_USART_RX_buf_in = 0;
  g_USART_RX_buf_out = 0;
  g_USART_TX_buf_in = 0;
  g_USART_TX_buf_out = 0;

  // Clear out the RC servo output pointer values
  g_RC_primed_ptr = 0;
  g_RC_next_ptr = 0;
  g_RC_timing_ptr = 0;

  // Turn on band-gap
  ANCON1bits.VBGEN = 1;

  // Set up ADCON1 options
  // A/D Result right justified
  // Normal A/D (no calibration)
  // Acq time = 20 Tad (?)
  // Tad = Fosc/64
  ADCON1 = 0b10111110;

  // Enable interrupt priorities
  RCONbits.IPEN = 1;
  T4CONbits.TMR4ON = 0;

  PIE3bits.TMR4IE = 1;
  IPR3bits.TMR4IP = 0;

  // Call the ebb init function to setup whatever it needs
  EBB_Init();

  RCServo2_Init();

  INTCONbits.GIEH = 1;  // Turn high priority interrupts on
  INTCONbits.GIEL = 1;  // Turn low priority interrupts on

  // Turn on the Timer4
  T4CONbits.TMR4ON = 1;

  // If there's a name in FLASH for us, copy it over to the USB Device
  // descriptor before we enumerate
  populateDeviceStringWithName();

  // Zero out limit switch variables
  gLimitSwitchPortB = 0;
  gLimitSwitchReplies = 0;
  gLimitSwitchReplyPrinted = 0;
  gCommandChecksumRequired = 0;
  gLimitSwitchTriggered = 0;

#if PC_PG_T_COMMANDS_ENABLED
  // Zero out pulse variables
  gPulsesOn = FALSE;
  gPulseLen[0] = 0;
  gPulseLen[1] = 0;
  gPulseLen[2] = 0;
  gPulseLen[3] = 0;
  gPulseRate[0] = 0;
  gPulseRate[1] = 0;
  gPulseRate[2] = 0;
  gPulseRate[3] = 0;
  gPulseCounters[0] = 0;
  gPulseCounters[1] = 0;
  gPulseCounters[2] = 0;
  gPulseCounters[3] = 0;
#endif
  
  gRCServoPoweroffCounterMS = 0;
  gRCServoPoweroffCounterReloadMS = RCSERVO_POWEROFF_DEFAULT_MS;
  gAutomaticMotorEnable = TRUE;
  gStackHighWater = 0;
  g_PowerMonitorThresholdADC = 0;
  g_StepperDisableTimeoutS = 0;
  g_StepperDisableSecondCounter = 0;
  g_StepperDisableCountdownS = 0;
  g_StepperDisableState = kSTEPPER_TIMEOUT_DISABLED;
  g_PowerDropDetected = FALSE;

  
  //// JUST FOR TESTING! REMOVE!  
TRISDbits.TRISD1 = 0;   // D1 high when in ISR
TRISDbits.TRISD0 = 0;   // D0 high when loading next command
TRISAbits.TRISA1 = 0;   // A1 when FIFO empty
TRISCbits.TRISC0 = 0;
TRISAbits.TRISA5 = 0;


Open1USART(
  USART_TX_INT_OFF &
  USART_RX_INT_OFF &
  USART_ASYNCH_MODE &
  USART_EIGHT_BIT &
  USART_CONT_RX &
  USART_BRGH_HIGH &
  USART_ADDEN_OFF,
  2                   // At 48 MHz, this creates 1 Mbaud output
);

////  

}

/******************************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        In this function, we check for a new packet that just
 *                  arrived via USB. We do a few checks on the packet to see
 *                  if it is worthy of us trying to interpret it. If it is,
 *                  we go and call the proper function based upon the first
 *                  character of the packet.
 *                  NOTE: We need to see everything in one packet (i.e. we
 *                  won't treat the USB data as a stream and try to find our
 *                  start and end of packets within the stream. We just look 
 *                  at the first character of each packet for a command and
 *                  check that there's a CR as the last character of the
 *                  packet.
 *
 * Note:            None
 *****************************************************************************/
void ProcessIO(void)
{
  static BOOL in_cr = FALSE;
  static BYTE last_fifo_size;
  static unsigned char button_state = 0;
  static unsigned int button_ctr = 0;

  BYTE i;
  BOOL done = FALSE;
  unsigned char rx_bytes = 0;
  unsigned char byte_cnt = 0;
  unsigned char tst_char;

  BlinkUSBStatus();
  
  // Check for any new I packets (from T command) ready to go out
  while (ISR_D_FIFO_length > 0u)
  {
    // Spit out an I packet first
    parse_I_packet();

    // Then update our I packet FIFO stuff
    ISR_D_FIFO_out++;
    if (ISR_D_FIFO_out == kISR_FIFO_D_DEPTH)
    {
      ISR_D_FIFO_out = 0;
    }
    ISR_D_FIFO_length--;
  }

  // Check for a new A packet (from T command) ready to go out
  while (ISR_A_FIFO_length > 0u)
  {
    // Spit out an A packet first
    parse_A_packet();

    // Then update our A packet FIFO stuff
    ISR_A_FIFO_out++;
    if (ISR_A_FIFO_out == kISR_FIFO_A_DEPTH)
    {
      ISR_A_FIFO_out = 0;
    }
    ISR_A_FIFO_length--;
  }
  // Bail from here if we're not 'plugged in' to a PC or we're suspended
  if(
    (USBDeviceState < CONFIGURED_STATE)
    ||
    (USBSuspendControl == 1u)
  ) 
  {
    return;
  }

  // Pull in some new data if there is new data to pull in
  // And we aren't waiting for the current move to finish
  rx_bytes = getsUSBUSART((char *)g_RX_command_buf, kRX_COMMAND_BUF_SIZE);

  if (rx_bytes > 0u)
  {
    for (byte_cnt = 0; byte_cnt < rx_bytes; byte_cnt++)
    {
      tst_char = g_RX_command_buf[byte_cnt];
      
      if (bittst(TestMode, TEST_MODE_USART_COMMAND_BIT_NUM))
      {
        Write1USART(tst_char);
      }
        
      // Check to see if we are in a CR/LF situation
      if (
        !in_cr 
        && 
        (
          kCR == tst_char
          ||
          kLF == tst_char
        )
      )
      {
        in_cr = TRUE;
        g_RX_buf[g_RX_buf_in] = kCR;
        g_RX_buf_in++;
      
        // At this point, we know we have a full packet
        // of information from the PC to parse

        // Now, if we've gotten a full command (user send <CR>) then
        // go call the code that deals with that command, and then
        // keep parsing. (This allows multiple small commands per packet)
        parse_packet();
        g_RX_buf_in = 0;
        g_RX_buf_out = 0;
      }
      else if (tst_char == 8u && g_RX_buf_in > 0u)
      {
        // Handle the backspace thing
        g_RX_buf_in--;
        g_RX_buf[g_RX_buf_in] = 0x00;
        ebb_print((far rom char *)" \b");
      }
      else if (
        tst_char != kCR
        &&
        tst_char != kLF
        &&
        tst_char >= 32u
      )
      {
        // Only add a byte if it is not a CR or LF
        g_RX_buf[g_RX_buf_in] = tst_char;
        in_cr = FALSE;
        g_RX_buf_in++;
      }
      // Check for buffer wraparound
      if (kRX_BUF_SIZE == g_RX_buf_in)
      {
        bitset(error_byte, kERROR_BYTE_RX_BUFFER_OVERRUN);
        g_RX_buf_in = 0;
        g_RX_buf_out = 0;
      }
    }
  }

  // Check for any errors logged in error_byte that need to be sent out
  if (error_byte)
  {
    if (bittstzero(error_byte))
    {
      // Unused as of yet
      ebb_print((far rom char *)"!0 ");
      print_line_ending(kLE_NORM);
    }
    if (bittst(error_byte, kERROR_BYTE_STEPS_TO_FAST))
    {
      // Unused as of yet
      ebb_print((far rom char *)"!1 Err: Can't step that fast");
      print_line_ending(kLE_NORM);
    }
    if (bittst(error_byte, kERROR_BYTE_TX_BUF_OVERRUN))
    {
      ebb_print((far rom char *)"!2 Err: TX Buffer overrun");
      print_line_ending(kLE_NORM);
    }
    if (bittst(error_byte, kERROR_BYTE_RX_BUFFER_OVERRUN))
    {
      ebb_print((far rom char *)"!3 Err: RX Buffer overrun");
      print_line_ending(kLE_NORM);
    }
    if (bittst(error_byte, kERROR_BYTE_MISSING_PARAMETER))
    {
      ebb_print((far rom char *)"!4 Err: Missing parameter(s)");
      print_line_ending(kLE_NORM);
    }
    if (bittst(error_byte, kERROR_BYTE_PRINTED_ERROR))
    {
      // We don't need to do anything since something has already been printed out
      //printf ((rom char *)"!5");
      //print_line_ending(kLE_NORM);
    }
    if (bittst(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT))
    {
      ebb_print((far rom char *)"!6 Err: Invalid parameter value");
      print_line_ending(kLE_NORM);
    }
    if (bittst(error_byte, kERROR_BYTE_EXTRA_CHARACTERS))
    {
      ebb_print((far rom char *)"!7 Err: Extra parameter");
      print_line_ending(kLE_NORM);
    }
    error_byte = 0;
  }

  // Check to see if we need to print out a "Limit switch triggered" packet to the PC
  if (gLimitSwitchReplies)
  {
    if (bittstzero(gLimitSwitchTriggered) && !gLimitSwitchReplyPrinted)
    {
      ebb_print((far rom char *)"Limit switch triggered. PortB=");
      // We want 2 characters of hex
      ebb_print_hex(gLimitSwitchPortB, 2);
      print_line_ending(kLE_NORM);
      gLimitSwitchReplyPrinted = TRUE;
    }
    else if (!bittstzero(gLimitSwitchTriggered) && gLimitSwitchReplyPrinted)
    {
      gLimitSwitchReplyPrinted = FALSE;
    }
  }
  
  // Go send any data that needs sending to PC
  check_and_send_TX_data();
}

// This is our replacement for the standard putc routine
// This enables ebb_print() and all related functions to print to
// the USB output (i.e. to the PC) buffer
int ebb_putc(char c)
{
  BYTE OldPtr = g_TX_buf_in;

  // Check to see if adding this byte will cause us to be full
  OldPtr++;
  if (kTX_BUF_SIZE == OldPtr)
  {
    OldPtr = 0;
  }
  // If so, then wait until some bytes go away first and make room
  if (OldPtr == g_TX_buf_out)
  {
    check_and_send_TX_data();
  }
  // Copy the character into the output buffer
  g_TX_buf[g_TX_buf_in] = c;
  g_TX_buf_in++;

  // Check for wrap around
  if (kTX_BUF_SIZE == g_TX_buf_in)
  {
    g_TX_buf_in = 0;
  }

  // Also check to see if we bumped up against our output pointer
  if (g_TX_buf_in == g_TX_buf_out)
  {
    bitset(error_byte, kERROR_BYTE_TX_BUF_OVERRUN);
  }
  return(c);
}

// In this function, we check to see if we have anything to transmit. 
// If so then we schedule the data for sending.
void check_and_send_TX_data(void)
{
  char temp;

  // Only send if there's something there to send
  if (g_TX_buf_out != g_TX_buf_in)
  {
    // Yes, Microchip says not to do this. We'll be blocking
    // here until there's room in the USB stack to send
    // something new. But without making our buffers huge,
    // I don't know how else to do it.
    while (!USBUSARTIsTxTrfReady())
    {
      CDCTxService();
#if defined(USB_POLLING)
      USBDeviceTasks();
#endif
    }
    // Now we know that the stack can transmit some new data

    // Now decide if we need to break it up into two parts or not
    if (g_TX_buf_in > g_TX_buf_out)
    {
      // Since IN is beyond OUT, only need one chunk
      temp = g_TX_buf_in - g_TX_buf_out;
      putUSBUSART((char *)&g_TX_buf[g_TX_buf_out], temp);
      // Now that we've scheduled the data for sending,
      // update the pointer
      g_TX_buf_out = g_TX_buf_in;
    }
    else
    {
      // Since IN is before OUT, we need to send from OUT to the end
      // of the buffer, then the next time around we'll catch
      // from the beginning to IN.
      temp = kTX_BUF_SIZE - g_TX_buf_out;
      putUSBUSART((char *)&g_TX_buf[g_TX_buf_out], temp);
      // Now that we've scheduled the data for sending,
      // update the pointer
      g_TX_buf_out = 0;
    }
    CDCTxService();
  }
}

// Look at the new packet, see what command it is, and 
// route it appropriately. We come in knowing that
// our packet is in g_RX_buf[], and that the beginning
// of the packet is at g_RX_buf_out, and the end (CR) is at
// g_RX_buf_in - 1. Note that because of buffer wrapping,
// g_RX_buf_in may be less than g_RX_buf_out.
// New for v3.0.0: if gCommandChecksumRequired is true, then look at the end
// of the command packet for the checksum, and if it is not there or not
// correct, error out.
void parse_packet(void)
{
  UINT16 command = 0;
  UINT8 checksum_cmd = 0;
  UINT8 checksum_calc = 0;
  UINT8 checksum_ptr;
  UINT8 checksum_len = 0;
  BOOL checksumOK = TRUE;   // Starts out true for no checksum case
  UINT8 old_rx_buf_out;
  UINT8 i;
  gCommand_Char1 = 0;
  gCommand_Char2 = 0;
  
  if (gCommandChecksumRequired)
  {
    checksumOK = FALSE;
    
    // Walk backwards from the end of the packet, looking for the first found
    // comma, then read out 
    checksum_ptr = g_RX_buf_in;
    if (checksum_ptr != 0u)
    {
      checksum_ptr--;
    }
    else
    {
      checksum_ptr = kRX_BUF_SIZE - 1;
    }
    
    while ((g_RX_buf[checksum_ptr] != ',') && (checksum_len < 5u) && (checksum_ptr != g_RX_buf_out))
    {
      if (checksum_ptr != 0u)
      {
        checksum_ptr--;
      }
      else
      {
        checksum_ptr = kRX_BUF_SIZE - 1;
      }
      checksum_len++;
    }
    
    // If checksum_ptr isn't on a comma then there is no checksum for sure
    // so let checksumOK stay FALSE
    if (g_RX_buf[checksum_ptr] == ',')
    {
      // Last parameter found, hopefully it's a checksum. Read it in.
      // We have to play some games with the buffer index values here
      // since we're extracting a parameter at the very end of the packet
      // before we extract any from the beginning (as part of normal command
      // processing)
      old_rx_buf_out = g_RX_buf_out;
      g_RX_buf_out = checksum_ptr;
      // We're going to ignore the return value from extract_number() since
      // we'll get a failure anyway since the checksum value won't match.
      extract_number(kUCHAR, &checksum_cmd, kREQUIRED);
      g_RX_buf_out = old_rx_buf_out;
      
      // Compute the checksum of the packet up to checksum_ptr
      i = g_RX_buf_out;
      while(i != checksum_ptr)
      {
        checksum_calc += g_RX_buf[i];
        i++;
        if (i == kRX_BUF_SIZE)
        {
          i = 0;
        }
      }
      checksum_calc = (~checksum_calc)+1;
      
      // See if it matches
      if (checksum_calc == checksum_cmd)
      {
        // All is good, allow command to proceed
        checksumOK = TRUE;
        // Need to fake out the command parsing, make it think the packet
        // ends where it expects it to end. It doesn't want to see the comma
        // before the checksum.
        g_RX_buf[checksum_ptr] = kCR;
      }
      else
      {
        ebb_print((far rom char *)"!8 Err: Checksum incorrect, expected ");
        ebb_print_uint(checksum_calc);
        print_line_ending(kLE_NORM);
      }
    }
    else
    {
      ebb_print((far rom char *)"!8 Err: Checksum not found but required.");
      print_line_ending(kLE_NORM);
    }
  }

  if (checksumOK)
  {
    // Always grab the first character (which is the first byte of the command)
    gCommand_Char1 = toupper(g_RX_buf[g_RX_buf_out]);
    advance_RX_buf_out();
    command = gCommand_Char1;

    // Only grab second one if it is not a comma
    if (g_RX_buf[g_RX_buf_out] != (BYTE)',' && g_RX_buf[g_RX_buf_out] != kCR && g_RX_buf[g_RX_buf_out] != kLF)
    {
      gCommand_Char2 = toupper(g_RX_buf[g_RX_buf_out]);
      advance_RX_buf_out();
      command = ((unsigned int)(gCommand_Char1) << 8) + gCommand_Char2;
    }

    // Now 'command' is equal to one or two bytes of our command
    switch (command)
    {
      case ('L' * 256) + 'T':
      {
        // Low Level Timed Move
        parse_LT_packet();
        break;
      }
      case ('L' * 256) + '3':
      {
        // Low Level 3rd derivative (jerk) move
        parse_L3_packet();
        break;
      }
      case ('T' * 256) + '3':
      {
        // Timed 3rd derivative (jerk) move
        parse_T3_packet();
        break;
      }
      case ('L' * 256) + 'M':
      {
        // Low Level Move
        parse_LM_packet();
        break;
      }
      case 'R':
      {
        // Reset command (resets everything to power-on state)
        parse_R_packet();
        break;
      }
      case 'C':
      {
        // Configure command (configure ports for Input or Output)
        parse_C_packet();
        break;
      }
      case ('C' * 256) + 'U':
      {
        // For configuring UBW
        parse_CU_packet();
        break;
      }
      case 'O':
      {
        // Output command (tell the ports to output something)
        parse_O_packet();
        break;
      }
      case 'I':
      {
        // Input command (return the current status of the ports)
        parse_I_packet();
        break;
      }
      case 'V':
      {
        // Version command
        parse_V_packet();
        break;
      }
      case 'A':
      {
        // Analog command
        parse_A_packet();
        break;
      }
  #if PC_PG_T_COMMANDS_ENABLED
      case 'T':
      {
        // For timed I/O
        parse_T_packet();
        break;
      }
  #endif
      case ('P' * 256) + 'I':
      {
        // PI for reading a single pin
        parse_PI_packet();
        break;
      }
      case ('P' * 256) + 'O':
      {
        // PO for setting a single pin
        parse_PO_packet();
        break;
      }
      case ('P' * 256) + 'D':
      {
        // PD for setting a pin's direction
        parse_PD_packet();
        break;
      }
      case ('M' * 256) + 'R':
      {
        // MR for Memory Read
        parse_MR_packet();
        break;
      }
      case ('M' * 256) + 'W':
      {
        // MW for Memory Write
        parse_MW_packet();
        break;
      }
  #if PC_PG_T_COMMANDS_ENABLED
      case ('P' * 256) + 'C':
      {
        // PC for pulse configure
        parse_PC_packet();
        break;
      }
      case ('P' * 256) + 'G':
      {
        // PG for pulse go command
        parse_PG_packet();
        break;
      }
  #endif
      case ('S' * 256) + 'M':
      {
        // SM for stepper motor
        parse_SM_packet();
        break;
      }
      case ('S' * 256) + 'P':
      {
        // SP for set pen
        parse_SP_packet();
        break;
      }
      case ('T' * 256) + 'P':
      {
        // TP for toggle pen
        parse_TP_packet();
        break;
      }
      case ('Q' * 256) + 'P':
      {
        // QP for query pen
        parse_QP_packet();
        break;
      }
      case ('Q' * 256) + 'E':
      {
        // QE for Query motor Enable and resolution
        parse_QE_packet();
        break;
      }
      case ('E' * 256) + 'M':
      {
        // EM for enable motors
        parse_EM_packet();
        break;
      }
      case ('S' * 256) + 'C':
      {
        // SC for stepper mode configure
        parse_SC_packet();
        break;
      }
      case ('S' * 256) + 'N':
      {
        // SN for Clear Node count
        parse_SN_packet();
        break;
      }
      case ('Q' * 256) + 'N':
      {
        // QN for Query Node count
        parse_QN_packet();
        break;
      }
      case ('S' * 256) + 'L':
      {
        // SL for Set Layer
        parse_SL_packet();
        break;
      }
      case ('Q' * 256) + 'L':
      {
        // QL for Query Layer count
        parse_QL_packet();
        break;
      }
      case ('Q' * 256) + 'B':
      {
        // QL for Query Button (program)
        parse_QB_packet();
        break;
      }
      case ('N' * 256) + 'I':
      {
        // NI for Node count Increment
        parse_NI_packet();
        break;
      }
      case ('N' * 256) + 'D':
      {
        // ND Node count Decrement
        parse_ND_packet();
        break;
      }
      case ('B' * 256) + 'L':
      {
        // BL for Boot Load
        parse_BL_packet();
        break;
      }
      case ('C' * 256) + 'K':
      {
        // CL for Check
        parse_CK_packet();
        break;
      }
      case ('Q' * 256) + 'C':
      {
        // QC for Query Current
        parse_QC_packet();
        break;
      }
      case ('Q' * 256) + 'G':
      {
        // QG for Query General
        parse_QG_packet();
        break;
      }
      case ('S' * 256) + 'E':
      {
        // SE for Set Engraver
        parse_SE_packet();
        break;
      }
      case ('S' * 256) + '2':
      {
        // S2 for RC Servo method 2
        RCServo2_S2_command();
        break;
      }
      case ('Q' * 256) + 'M':
      {
        // QM for Query Motor
        parse_QM_packet();
        break;
      }
      case ('A' * 256) + 'C':
      {
        // AC for Analog Configure
        parse_AC_packet();
        break;
      }
      case ('E' * 256) + 'S':
      {
        // ES for E-Stop
        parse_ES_packet();
        break;
      }
      case ('X' * 256) + 'M':
      {
        // XM for X motor move
        parse_XM_packet();
        break;
      }
      case ('Q' * 256) + 'S':
      {
        // QP for Query Step position
        parse_QS_packet();
        break;
      }
      case ('C' * 256) + 'S':
      {
        // CS for Clear Step position
        parse_CS_packet();
        break;
      }
      case ('S' * 256) + 'T':
      {
        // ST for Set Tag
        parse_ST_packet();
        break;
      }
      case ('Q' * 256) + 'T':
      {
        // QT for Query Tag
        parse_QT_packet();
        break;
      }
      case ('R' * 256) + 'B':
      {
        // RB for ReBoot
        parse_RB_packet();
        break;
      }
      case ('Q' * 256) + 'R':
      {
        // QR is for Query RC Servo power state
        parse_QR_packet();
        break;
      }
      case ('S' * 256) + 'R':
      {
        // SR is for Set RC Servo power timeout
        parse_SR_packet();
        break;
      }
      case ('H' * 256) + 'M':
      {
        // HM is for Home Motor
        parse_HM_packet();
        break;
      }
      case ('Q' * 256) + 'U':
      {
        // QU is for General Query
        parse_QU_packet();
        break;
      }

      default:
      {
        if (0u == gCommand_Char2)
        {
          // Send back 'unknown command' error
          ebb_print((far rom char *)"!8 Err: Unknown command '");
          ebb_print_char(gCommand_Char1);
          ebb_print_char(':');
          ebb_print_hex(gCommand_Char1, 2);
          ebb_print_char(0x27); // the ' character
        }
        else
        {
          // Send back 'unknown command' error
          ebb_print((far rom char *)"!8 Err: Unknown command '");
          ebb_print_char(gCommand_Char1);
          ebb_print_char(gCommand_Char2);
          ebb_print_char(':');
          ebb_print_hex(gCommand_Char1, 2);
          ebb_print_hex(gCommand_Char2, 2);
          ebb_print_char(0x27); // the ' character
        }
        print_line_ending(kLE_NORM);
        break;
      }
    }
    
    // Double check that our output pointer is now at the ending <CR>
    // If it is not, this indicates that there were extra characters that
    // the command parsing routine didn't eat. This would be an error and needs
    // to be reported. (Ignore for Reset command because FIFO pointers get cleared.)
    if (
      (g_RX_buf[g_RX_buf_out] != kCR && 0u == error_byte)
      &&
      ('R' != command)
    )
    {
      bitset(error_byte, kERROR_BYTE_EXTRA_CHARACTERS);
    }
  }
  
  // Clean up by skipping over any bytes we haven't eaten
  // This is safe since we parse each packet as we get a <CR>
  // (i.e. g_RX_buf_in doesn't move while we are in this routine)
  g_RX_buf_out = g_RX_buf_in;
}

// This function will print out the two character command that was just parsed
// if the New line ending mode is set (CU,10,1). Every time a command is sent
// to the EBB, it will respond with the two character command code (at least,
// some commands will return more than that).
// print_always = true : always print command, even in Default Line Ending Mode
// print_always = false : do not print command when in Unified Line Ending Mode
// print_comma = true : if command printed, also print comma after
// print_comma = false : if command printed, do not print comma after
// If Legacy line ending mode is turned on this function will not print anything
void print_command(BOOL print_always, BOOL print_comma)
{
  char comma = ',';
  
  if (bittstzero(gStandardizedCommandFormat) || print_always)
  {
    ebb_putc(gCommand_Char1);
    if (gCommand_Char2 != 0u)
    {
      ebb_putc(gCommand_Char2);
    }
    if (print_comma)
    {
      ebb_putc(comma);
    }
  }
}

// This function prints out the common endings needed on text lines sent back
// to the PC. It operates in two modes, Legacy and New. Legacy mode is used
// when we need to emulate how the EBB's line endings have been done since
// the beginning (as some PC software counts on this). New mode can be turned
// on with CU,10,1 and makes every line ending exactly the same : "\n".
// Because in Legacy mode we also need to print "OK", that is included
// as an option here too.
//   Legacy Mode:
//     <type> = LE_OK_NORM : Print "OK\r\n"
//     <type> = LE_NORM    : Print "\r\n"
//     <type> = LE_REV     : Print "\n\r"
//   New Mode:
//     Regardless of the value of <type>, always print "\n"
void print_line_ending(tLineEnding le_type)
{
  if (bittstzero(gStandardizedCommandFormat))
  {
    ebb_print((far rom char *)"\n");
  }
  else
  {
    if ((g_ack_enable) && (le_type == kLE_OK_NORM))
    {
      ebb_print((far rom char *)"OK");
    }
    if (le_type == kLE_REV)
    {
      ebb_print((far rom char *)"\n\r");
    }
    else
    {
      // le_type == kLE_NORM
      ebb_print((far rom char *)"\r\n");
    }
  }
}

// Return all I/Os to their default power-on values
void parse_R_packet(void)
{
  print_command(FALSE, FALSE);
  print_line_ending(kLE_OK_NORM);
  check_and_send_TX_data();
  UserInit();
}

// CU is "Configure UBW" and controls system-wide configuration values
// "CU,<parameter_number>,<parameter_value><CR>"
// <parameter_number> <parameter_value>
// 1   {1|0} turns on or off the 'ack' ("OK" at end of packets)
// 2   {1|0} turns on or off parameter limit checking (defaults to on))
// 3   {1|0} turns on or off the red LED acting as an empty FIFO indicator (defaults to off)
// 4   <new_fifo_size> sets the FIFO size, in commands. Defaults to 1 at boot
// 10  {1|0} turns on or off the standardized command format replies (defaults to off))
// 50  {1|0} turns on or off the automatic enabling of both motors on any move command (defaults to on)
// 51  <limit_switch_mask> sets the limit_switch_mask value for limit switch checking in ISR. Set to 0 to disable. Any high bit looks for a corresponding bit in the limit_switch_target on PORTB
// 52  <limit_switch_target> set the limit_switch_value for limit switch checking in ISR. 
// 53  {1|0} turns on or off the sending of "Limit switch triggered" replies (defaults to off)
// 54  {1|0} turns on/off command checksum (defaults to off)
// 60  <NewThreshood> Set the power lost threshold. Set to 0 to disable.
// 61  <NewStepperDisableTimeoutS> Sets the stepper timeout in seconds. 0 to disable.
// 250 {1|0} turns on or off the GPIO DEBUG (i/o pins to time moves and the ISR)
// 251 {1|0} turns on or off the UART ISR DEBUG (prints internal numbers at end of each move)
// 252 {1|0} turns on or off the UART ISR DEBUG FULL (prints internal numbers at end of each ISR)
// 253 {1|0} turns on or off the UART COMMAND DEBUG (prints all received command bytes)
// 254 {1} turns on lock up mode. Tight loop of I/O toggles shows true ISR timing. Reset to exit.
// 255 {1|0} turns on or off command parsing debug printing on USB

void parse_CU_packet(void)
{
  UINT8 parameter_number;
  INT16 paramater_value;

  print_command(FALSE, FALSE);

  extract_number(kUCHAR, &parameter_number, kREQUIRED);
  extract_number(kINT, &paramater_value, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // CU,1,1 or CU,1,0 to turn on/off "OK" at end of command reply
  if (1u == parameter_number)
  {
    if (0 == paramater_value || 1 == paramater_value)
    {
      g_ack_enable = paramater_value;
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  // CU,2,1 or CU,2,0 to turn on/off parameter limit checks
  else if (2u == parameter_number)
  {
    if (0 == paramater_value || 1 == paramater_value)
    {
      gLimitChecks = paramater_value;
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  // CU,3,1 or CU,3,0 to turn on/off red LED FIFO empty indicator
  else if (3u == parameter_number)
  {
    if (0 == paramater_value)
    {
      bitclrzero(gRedLEDEmptyFIFO);
      mLED_2_Off()      
    }
    else if (1 == paramater_value)
    {
      bitsetzero(gRedLEDEmptyFIFO);
      mLED_2_Off()
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  // CU,4,<new_FIFO_size>
  else if (4u == parameter_number)
  {
    if (paramater_value > (INT16)COMMAND_FIFO_MAX_LENGTH)
    {
      paramater_value = COMMAND_FIFO_MAX_LENGTH;
    }
    // Spin here until we're certain the FIFO is empty and there are no 
    // command executing. We want the ISR to be completely idle while we
    // change this value.
    while (process_QM())
      ;
    gCurrentFIFOLength = paramater_value;
  }
  // CU,10,1 or CU,10,0 to turn on/off standardized line ending
  else if (10u == parameter_number)
  {
    if (0 == paramater_value)
    {
      bitclrzero(gStandardizedCommandFormat);
    }
    else if (1 == paramater_value)
    {
      bitsetzero(gStandardizedCommandFormat);
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  // CU,50,1 or CU,50,0 to turn on/off automatic motor enable
  else if (50u == parameter_number)
  {
    if (0 == paramater_value)
    {
      gAutomaticMotorEnable = FALSE;
    }
    else
    {
      gAutomaticMotorEnable = TRUE;
    }
  }
  // CU,51,<limit_switch_mask>
  else if (51u == parameter_number)
  {
    gLimitSwitchMask = (paramater_value & 0xFF);
    if (gLimitSwitchMask == 0u)
    {
      bitclrzero(gLimitSwitchTriggered);
    }
  }
  // CU,52,<limit_siwtch_target>
  else if (52u == parameter_number)
  {
    gLimitSwitchTarget = (paramater_value & 0xFF);
  }
  // CU,53,1 turns on the sending of "Limit switch trigger" replies
  else if (53u == parameter_number)
  {
    if (1 == paramater_value)
    {
      gLimitSwitchReplies = TRUE;
    }
    else
    {
      gLimitSwitchReplies = FALSE;
      gLimitSwitchReplyPrinted = FALSE;
    }
  }
  // CU,54,1 turns on command checksums
  else if (54u == parameter_number)
  {
    if (1 == paramater_value)
    {
      gCommandChecksumRequired = TRUE;
    }
    else
    {
      gCommandChecksumRequired = FALSE;
    }
  }
  // CU,60,<NewThreshold>
  else if (60u == parameter_number)
  {
    g_PowerMonitorThresholdADC = (paramater_value & 0x03FF);
  }
  // CU,61,<NewStepperDisableTimeoutThreshold>
  else if (61u == parameter_number)
  {
    INTCONbits.GIEH = 0;    // Turn high priority interrupts off
    INTCONbits.GIEL = 0;    // Turn low priority interrupts off
    
    g_StepperDisableTimeoutS = paramater_value;
    
    if (g_StepperDisableTimeoutS == 0u)
    {
      // Turn feature completely off no matter what state we're in
      g_StepperDisableState = kSTEPPER_TIMEOUT_DISABLED;
      g_StepperDisableSecondCounter = 0;
      g_StepperDisableCountdownS = 0;
    }
    else
    {
      // User wants feature enabled with new timeout. Do different things
      // based on current state.
      switch (g_StepperDisableState)
      {
        case kSTEPPER_TIMEOUT_TIMING:
          // Always start over with new timeout value
          g_StepperDisableCountdownS = g_StepperDisableTimeoutS;
          g_StepperDisableSecondCounter = 1000u;
          break;

        default:
        case kSTEPPER_TIMEOUT_PRIMED:
        case kSTEPPER_TIMEOUT_DISABLED:
          g_StepperDisableState = kSTEPPER_TIMEOUT_PRIMED;
          // Note intentional fall-through
        case kSTEPPER_TIMEOUT_FIRED:
          g_StepperDisableSecondCounter = 0;
          g_StepperDisableCountdownS = 0;
          break;
      }
    }
    INTCONbits.GIEL = 1;    // Turn low priority interrupts on
    INTCONbits.GIEH = 1;    // Turn high priority interrupts on
  }
  // CU,250,1 or CU,250,0 to turn on/off GPIO ISR timing debug
  else if (250u == parameter_number)
  {
    if (0 == paramater_value)
    {
      bitclr(TestMode, TEST_MODE_GPIO_BIT_NUM);
    }
    else if (1 == paramater_value)
    {
      bitset(TestMode, TEST_MODE_GPIO_BIT_NUM);
      TRISDbits.TRISD1 = 0;   // D1 high when in ISR
      TRISDbits.TRISD0 = 0;   // D0 high when loading next command
      TRISAbits.TRISA1 = 0;   // A1 when FIFO empty
      TRISCbits.TRISC0 = 0;
      TRISAbits.TRISA5 = 0;
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  // CU,251,1 or CU,251,0 to turn on/off ISR end of move values printing (On RC6)
  else if (251u == parameter_number)
  {
    if (0 == paramater_value)
    {
      bitclr(TestMode, TEST_MODE_USART_ISR_FULL_BIT_NUM);
      bitclr(TestMode, TEST_MODE_USART_ISR_BIT_NUM);
    }
    else if (1 == paramater_value)
    {
      bitset(TestMode, TEST_MODE_USART_ISR_BIT_NUM);
      bitclr(TestMode, TEST_MODE_USART_ISR_FULL_BIT_NUM);
      Open1USART(
        USART_TX_INT_OFF &
        USART_RX_INT_OFF &
        USART_ASYNCH_MODE &
        USART_EIGHT_BIT &
        USART_CONT_RX &
        USART_BRGH_HIGH &
        USART_ADDEN_OFF,
        2                   // At 48 MHz, this creates 1 Mbaud output
      );
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  // CU,252,1 or CU,252,0 to turn on/off every ISR tick values printing (on RC6)
  else if (252u == parameter_number)
  {
    if (0 == paramater_value)
    {
      bitclr(TestMode, TEST_MODE_USART_ISR_FULL_BIT_NUM);
      bitclr(TestMode, TEST_MODE_USART_ISR_BIT_NUM);
    }
    else if (1 == paramater_value)
    {
      bitset(TestMode, TEST_MODE_USART_ISR_FULL_BIT_NUM);
      bitset(TestMode, TEST_MODE_USART_ISR_BIT_NUM);
      Open1USART(
        USART_TX_INT_OFF &
        USART_RX_INT_OFF &
        USART_ASYNCH_MODE &
        USART_EIGHT_BIT &
        USART_CONT_RX &
        USART_BRGH_HIGH &
        USART_ADDEN_OFF,
        2                   // At 48 MHz, this creates 1 Mbaud output
      );
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  // CU,253,1 or CU,253,0 to turn on/off move command extra debug printing (on RC6)
  else if (253u == parameter_number)
  {
    if (0 == paramater_value)
    {
      bitclr(TestMode, TEST_MODE_USART_COMMAND_BIT_NUM);
    }
    else if (1 == paramater_value)
    {
      bitset(TestMode, TEST_MODE_USART_COMMAND_BIT_NUM);
      Open1USART(
        USART_TX_INT_OFF &
        USART_RX_INT_OFF &
        USART_ASYNCH_MODE &
        USART_EIGHT_BIT &
        USART_CONT_RX &
        USART_BRGH_HIGH &
        USART_ADDEN_OFF,
        2                   // At 48 MHz, this creates 1 Mbaud output
      );
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  // CU,254 turns on 'lock up mode' for measuring true ISR timing by cycling
  // I/O pin on and off as fast as possible, then breaks in that I/O toggle
  // can be seen for the ISR and measured.
  else if (254u == parameter_number)
  {
    bitset(TestMode, TEST_MODE_GPIO_BIT_NUM);
    TRISDbits.TRISD1 = 0;   // D1 high when in ISR
    TRISDbits.TRISD0 = 0;   // D0 high when loading next command
    TRISAbits.TRISA1 = 0;   // A1 when FIFO empty
    while(1)
    {
      _asm
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
        BCF 0x8c,0x0,0x0
        BSF 0x8c,0x0,0x0
      _endasm
    }
  }
  // CU,255,1 or CU,255,0 to turn on/off command parsing debug printing on USB
  else if (255u == parameter_number)
  {
    if (0 == paramater_value)
    {
      bitclr(TestMode, TEST_MODE_DEBUG_COMMAND_BIT_NUM);
    }
    else if (1 == paramater_value)
    {
      bitset(TestMode, TEST_MODE_DEBUG_COMMAND_BIT_NUM);
      Open1USART(
        USART_TX_INT_OFF &
        USART_RX_INT_OFF &
        USART_ASYNCH_MODE &
        USART_EIGHT_BIT &
        USART_CONT_RX &
        USART_BRGH_HIGH &
        USART_ADDEN_OFF,
        2                   // At 48 MHz, this creates 1 Mbaud output
      );
    }
    else
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }
  }
  else
  {
    // parameter_number is not understood
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
  }
  print_line_ending(kLE_OK_NORM);
}

// QU is "Query Utility" and provides a simple mechanism for the PC reading 
// certain values from the EBB.
// "QU,<parameter_number><CR>"
// Returns: Some value(s), dependant on what parameter_number is.
// <parameter_number> <return_packet>
// 1   QU,1,XX  where XX is a value from 00 to FF, representing the contents of 
//              the PortB pins at the time of the last limit switch trigger
// 2   QU,2,ddd to read back the maximum supported FIFO length for this version
// 3   QU,3,ddd to read back the current FIFO length
// 4   QU,4,XXX prints out stack high water value (as 3 digit hex value)
// 5   QU,5,XXX prints out stack high water value (as 3 digit hex value) and resets it to zero
// 60  QU,60,dddd prints out current value of g_PowerMonitorThresholdADC
// 61  QU,61,dddddd prints out current value of g_StepperDisableTimeoutS
// 200 QU,200,dddddddddd,dddddddddd prints out the current value of acc_union[0] and acc_union[1] (the accumulators)

void parse_QU_packet(void)
{
  UINT8 parameter_number;

  print_command(TRUE, TRUE);

  extract_number(kUCHAR, &parameter_number, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // QU,1 to read back current value of gLimitSwitchPortB
  // Returns "QU,1,XX" where XX is two digit hex value from 00 to FF
  if (1u == parameter_number)
  {
    ebb_print_uint(parameter_number);
    ebb_print_char(',');
    ebb_print_hex(gLimitSwitchPortB, 2);
    print_line_ending(kLE_NORM);
  }
  // QU,2 to read back the maximum supported FIFO length for this version
  // Returns "QU,2,ddd" where ddd is one to three digit decimal value from 0 to 255
  else if (2u == parameter_number)
  {
    ebb_print_uint(parameter_number);
    ebb_print_char(',');
    ebb_print_uint(COMMAND_FIFO_MAX_LENGTH);
    print_line_ending(kLE_NORM);
  }
  // QU,3 to read back the current FIFO length
  // Returns "QU,3,ddd" where ddd is one to three digit decimal value from 0 to 255
  else if (3u == parameter_number)
  {
    ebb_print_uint(parameter_number);
    ebb_print_char(',');
    ebb_print_uint(gCurrentFIFOLength);
    print_line_ending(kLE_NORM);
  }
  // QU,4 prints out current stack high water value
  else if (4u == parameter_number)
  {
    check_high_water();
    ebb_print_uint(parameter_number);
    ebb_print_char(',');
    ebb_print_hex(gStackHighWater, 3);
    print_line_ending(kLE_NORM);
  }
  // CU,5 prints out current stack high water value and resets it to zero
  else if (5u == parameter_number)
  {
    ebb_print_uint(parameter_number);
    ebb_print_char(',');
    ebb_print_hex(gStackHighWater, 3);
    print_line_ending(kLE_NORM);
    INTCONbits.GIEL = 0;  // Turn low priority interrupts off
    gStackHighWater = 0;
    INTCONbits.GIEL = 1;  // Turn low priority interrupts on
  }  
  // 60  QU,60,dddd prints out current value of g_PowerMonitorThresholdADC
  else if (60u == parameter_number)
  {
    ebb_print_uint(parameter_number);
    ebb_print_char(',');
    ebb_print_uint(g_PowerMonitorThresholdADC);
    print_line_ending(kLE_NORM);
  }
  // 61  QU,61,dddddd prints out current value of g_StepperDisableTimeoutS
  else if (61u == parameter_number)
  {
    ebb_print_uint(parameter_number);
    ebb_print_char(',');
    ebb_print_uint(g_StepperDisableTimeoutS);
    print_line_ending(kLE_NORM);
  }
// 200 QU,200,dddddddddd,dddddddddd prints out the current value of acc_union[0] and acc_union[1] (the accumulators)
  else if (200u == parameter_number)
  {
    ebb_print_uint(parameter_number);
    ebb_print_char(',');
    INTCONbits.GIEH = 0;    // Turn high priority interrupts off
    ebb_print_uint(acc_union[0].value);
    ebb_print_char(',');
    ebb_print_uint(acc_union[1].value);
    INTCONbits.GIEH = 1;    // Turn high priority interrupts on
    print_line_ending(kLE_NORM);
  }
  else
  {
    // parameter_number is not understood
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
  }

  print_line_ending(kLE_OK_NORM);
}

#if PC_PG_T_COMMANDS_ENABLED
// "T" Packet
// Causes PIC to sample digital or analog inputs at a regular interval and send
// I (or A) packets back at that interval.
// Send T,0,0<CR> to stop I (or A) packets
// FORMAT: T,<TIME_BETWEEN_UPDATES_IN_MS>,<MODE><CR>
// <MODE> is 0 for digital (I packets) and 1 for analog (A packets)
// EXAMPLE: "T,4000,0<CR>" to send an I packet back every 4 seconds.
// EXAMPLE: "T,2000,1<CR>" to send an A packet back every 2 seconds.
void parse_T_packet(void)
{
  unsigned int value;
  unsigned char mode = 0;

  print_command(FALSE, FALSE);

  // Extract the <TIME_BETWEEN_UPDATES_IN_MS> value
  extract_number(kUINT, (void *)&time_between_updates, kREQUIRED);
  // Extract the <MODE> value
  extract_number(kUCHAR, &mode, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Now start up the timer at the right rate or shut 
  // it down.
  if (0u == mode)
  {
    if (0u == time_between_updates)
    {
      // Turn off sending of I packets.
      ISR_D_RepeatRate = 0;
    }
    else
    {
      T4CONbits.TMR4ON = 1;
    
      // Eventually guard this section from interrupts
      ISR_D_RepeatRate = time_between_updates;
    }
  }
  else
  {
    if (0u == time_between_updates)
    {
      // Turn off sending of A packets.
      ISR_A_RepeatRate = 0;
    }
    else
    {
      T4CONbits.TMR4ON = 1;
    
      // Eventually guard this section from interrupts
      ISR_A_RepeatRate = time_between_updates;
    }
  }

  print_line_ending(kLE_OK_NORM);
}
#endif

// IMPORTANT: As of EBB v2.2.3 firmware, this command is different from the
// UBW version. The analog config value is eliminated, replaced with the "AC"
// command.
// FORMAT: C,<portA_IO>,<portB_IO>,<portC_IO>,<portD_IO>,<portE_IO><CR>
// EXAMPLE: "C,255,0,4,0,0,0<CR>"
// <portX_IO> is the byte sent to the Data Direction (DDR) register for
// each port. A 1 in a bit location means input, a 0 means output.
//
// NOTE: it is up to the user to tell the proper port direction bits to be
// inputs for the analog channels they wish to use.
void parse_C_packet(void)
{
  unsigned char PA, PB, PC, PD, PE;

  print_command(FALSE, FALSE);

  // Extract each of the four values.
  extract_number(kUCHAR, &PA, kREQUIRED);
  extract_number(kUCHAR, &PB, kREQUIRED);
  extract_number(kUCHAR, &PC, kREQUIRED);
  extract_number(kUCHAR, &PD, kREQUIRED);
  extract_number(kUCHAR, &PE, kREQUIRED);
    
  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Now write those values to the data direction registers.
  TRISA = PA;
  TRISB = PB;
  TRISC = PC;
  TRISD = PD;
  TRISE = PE;

  print_line_ending(kLE_OK_NORM);
}

// This function turns on or off an analog channel
// It is called from other pieces of code, not the user
void AnalogConfigure(unsigned char Channel, unsigned char Enable)
{
  if (Channel > 16u)
  {
    Channel = 16;
  }

  if (Enable)
  {
    AnalogEnabledChannels |= ((unsigned int)0x0001 << Channel);
    // Make sure to turn this analog input on
    if (Channel < 8u)
    {
      // Clear the right bit in ANCON0
      ANCON0 &= ~(1 << Channel);
    }
    else
    {
      if (Channel <= 12u)
      {
        // Clear the right bit in ANCON1
        ANCON1 &= ~(1 << (Channel-8));
      }
    }
  }
  else
  {
    AnalogEnabledChannels &= ~((unsigned int)0x0001 << Channel);
    // Make sure to turn this analog input off
    if (Channel < 8u)
    {
      // Set the right bit in ANCON0
      ANCON0 |= (1 << Channel);
    }
    else
    {
      if (Channel <= 12u)
      {
        // Set the right bit in ANCON1
        ANCON1 |= (1 << (Channel-8));
      }
    }
  }
}

// Analog Configure
// "AC,<channel>,<enable><CR>"
// <channel> is one of the 16 possible analog channels, from 0 through 15
// <enable> is 0 to disable, or 1 to enable
// To turn on a particular analog channel, use the AC command to enable it.
// To turn off a particular analog channel, use the AC command to disable it.
// Once enabled, that channel will be converted at the normal ADC conversion
// rate and will show up in A packets.
void parse_AC_packet(void)
{
  unsigned char Channel, Enable;

  print_command(FALSE, FALSE);

  // Extract each of the two values.
  extract_number(kUCHAR, &Channel, kREQUIRED);
  extract_number(kUCHAR, &Enable, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  AnalogConfigure(Channel, Enable);
  
  print_line_ending(kLE_OK_NORM);
}

// Outputs values to the ports pins that are set up as outputs.
// Example "O,121,224,002<CR>"
void parse_O_packet(void)
{
  unsigned char Value;
  ExtractReturnType RetVal;

  print_command(FALSE, FALSE);

  // Extract each of the values.
  RetVal = extract_number(kUCHAR,  &Value, kREQUIRED);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATA = Value;
  }
  RetVal = extract_number(kUCHAR,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATB = Value;
  }
  RetVal = extract_number(kUCHAR,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATC = Value;
  }
  RetVal = extract_number(kUCHAR,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATD = Value;
  }
  RetVal = extract_number(kUCHAR,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATE = Value;
  }

  print_line_ending(kLE_OK_NORM);
}

// Read in the three I/O ports (A,B,C) and create
// a packet to send back with all of values.
// Example: "I,143,221,010<CR>"
// Remember that on UBW 28 pin boards, we only have
// Port A bits 0 through 5
// Port B bits 0 through 7
// Port C bits 0,1,2 and 6,7
// And that Port C bits 0,1,2 are used for
// User1 LED, User2 LED and Program switch respectively.
// The rest will be read in as zeros.
void parse_I_packet(void)
{
  print_command(TRUE, TRUE);

  ebb_print_uint(PORTA);
  ebb_print_char(',');
  ebb_print_uint(PORTB);
  ebb_print_char(',');
  ebb_print_uint(PORTC);
  ebb_print_char(',');
  ebb_print_uint(PORTD);
  ebb_print_char(',');
  ebb_print_uint(PORTE);
  print_line_ending(kLE_NORM);
}

// All we do here is just print out our version number
void parse_V_packet(void)
{
  print_command(FALSE, TRUE);
  
  ebb_print((far rom char *)st_version);
  print_line_ending(kLE_NORM);
}

// A is for read Analog inputs
// Just print out the analog values for each of the
// enabled channels.
// Returned packet will look like 
// "A,2:421,5:891,9:3921<CR>" if channels 2, 5 and 9
// are enabled.
void parse_A_packet(void)
{
  char channel = 0;
  unsigned int ChannelBit = 0x0001;

  print_command(TRUE, FALSE);

  // Sit and spin, waiting for one set of analog conversions to complete
  while (PIE1bits.ADIE);

  // Now print each analog value
  for (channel = 0; channel < 16; channel++)
  {
    if (ChannelBit & AnalogEnabledChannels)
    {
      ebb_print_char(',');
      ebb_print_uint(channel);
      ebb_print_char(':');
      ebb_print_uint(ISR_A_FIFO[channel]);
    }
    ChannelBit = ChannelBit << 1;
  }

  // Add \r\n (for line ending legacy mode : note this is backwards from how
  // the rest of the code does legacy line endings) or just \n for new line 
  // ending mode.
  print_line_ending(kLE_REV);
}

// MW is for Memory Write
// "MW,<location>,<value><CR>"
// <location> is a decimal value between 0 and 4096 indicating the RAM address to write to 
// <value> is a decimal value between 0 and 255 that is the value to write
void parse_MW_packet(void)
{
  unsigned int location;
  unsigned char value;

  print_command(FALSE, FALSE);

  extract_number(kUINT, &location, kREQUIRED);
  extract_number(kUCHAR, &value, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }
  // Limit check the address and write the byte in
  if (location < 4096u)
  {
    *((unsigned char *)location) = value;
  }

  print_line_ending(kLE_OK_NORM);
}


// MR is for Memory Read
// "MW,<location><CR>"
// <location> is a decimal value between 0 and 4096 indicating the RAM address to read from 
// The UBW will then send a "MR,<value><CR>" packet back to the PC
// where <value> is the byte value read from the address
void parse_MR_packet(void)
{
  unsigned int location;
  unsigned char value;

  print_command(TRUE, TRUE);

  extract_number(kUINT, &location, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check the address and write the byte in
  if (location < 4096u)
  {
    value = *((unsigned char *)location);
  }

  // Now send back the MR packet
  ebb_print_uint(value);
  print_line_ending(kLE_NORM);
}

// PD is for Pin Direction
// "PD,<port>,<pin>,<direction><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to change direction on
// <direction> is "1" for input, "0" for output
void parse_PD_packet(void)
{
  unsigned char port;
  unsigned char pin;
  unsigned char direction;

  print_command(FALSE, FALSE);

  extract_number(kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number(kUCHAR, &pin, kREQUIRED);
  extract_number(kUCHAR, &direction, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check the parameters
  if (direction > 1u)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }
  if (pin > 7u)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }
  if ('A' == port)
  {
    if (0u == direction)
    {
      bitclr(TRISA, pin);
    }
    else
    {
      bitset(TRISA, pin);
    }
  }
  else if ('B' == port)
  {
    if (0u == direction)
    {
      bitclr(TRISB, pin);
    }
    else
    {
      bitset(TRISB, pin);
    }
  }
  else if ('C' == port)
  {
    if (0u == direction)
    {
      bitclr(TRISC, pin);
    }
    else
    {
      bitset(TRISC, pin);
    }
  }
  else if ('D' == port)
  {
    if (0u == direction)
    {
      bitclr(TRISD, pin);
    }
    else
    {
      bitset(TRISD, pin);
    }
  }
  else if ('E' == port)
  {
    if (0u == direction)
    {
      bitclr(TRISE, pin);
    }
    else
    {
      bitset(TRISE, pin);
    }
  }
  else
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  print_line_ending(kLE_OK_NORM);
}

// PI is for Pin Input
// "PI,<port>,<pin><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to read
// The command returns a "PI,<value><CR>" packet,
// where <value> is the value (0 or 1 for digital)
// value for that pin.
void parse_PI_packet(void)
{
  UINT8 port;
  UINT8 pin;
  UINT8 value = 0;

  print_command(TRUE, TRUE);

  extract_number(kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number(kUCHAR, &pin, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check the parameters
  if (pin > 7u)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  // Then test the bit in question based upon port
  if ('A' == port)
  {
    value = bittst(PORTA, pin);
  }
  else if ('B' == port)
  {
    value = bittst(PORTB, pin);
  }
  else if ('C' == port)
  {
    value = bittst(PORTC, pin);
  }
  else if ('D' == port)
  {
    value = bittst(PORTD, pin);
  }
  else if ('E' == port)
  {
    value = bittst(PORTE, pin);
  }
  else
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  // Convert to just a binary 1 or 0
  if (value)
  {
    value = 1;
  }

  // Now send back our response
  ebb_print_uint(value);
  print_line_ending(kLE_NORM);
}

// PO is for Pin Output
// "PO,<port>,<pin>,<value><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to write out the value to
// <value> is "1" or "0" and indicates the state to change the pin to
void parse_PO_packet(void)
{
  unsigned char port;
  unsigned char pin;
  unsigned char value;

  print_command(FALSE, FALSE);

  extract_number(kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number(kUCHAR, &pin, kREQUIRED);
  extract_number(kUCHAR, &value, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check the parameters
  if (value > 1u)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }
  if (pin > 7u)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }
  if ('A' == port)
  {
    if (0u == value)
    {
      bitclr(LATA, pin);
    }
    else
    {
      bitset(LATA, pin);
    }
  }
  else if ('B' == port)
  {
    if (0u == value)
    {
      bitclr(LATB, pin);
    }
    else
    {
      bitset(LATB, pin);
    }
  }
  else if ('C' == port)
  {
    if (0u == value)
    {
      bitclr(LATC, pin);
    }
    else
    {
      bitset(LATC, pin);
    }
  }
  else if ('D' == port)
  {
    if (0u == value)
    {
      bitclr(LATD, pin);
    }
    else
    {
      bitset(LATD, pin);
    }
  }
  else if ('E' == port)
  {
    if (0u == value)
    {
      bitclr(LATE, pin);
    }
    else
    {
      bitset(LATE, pin);
    }
  }
  else
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  print_line_ending(kLE_OK_NORM);
}

#if PC_PG_T_COMMANDS_ENABLED
// PC Pulse Configure
// Pulses will be generated on PortB, bits 0 through 3
// Pulses are in 1ms units, and can be from 0 (off) through 65535
// Each pin has a pulse length, and a repetition rate.
// The repetition rate can be from 0 through 65535, but must be more than the pulse length
// Only the first set of parameters (for RB0) are required, the rest are optional
//
// Usage:
// PC,<RB0_Len>,<RB0_Rate>,<RB1_Len>,<RB1_Rate>,...,<RB3_Len>,<RB3_Rate><CR>
void parse_PC_packet(void)
{
  unsigned int Length, Rate;
  unsigned char i;
  ExtractReturnType RetVal1, RetVal2;

  print_command(FALSE, FALSE);

  extract_number(kUINT, &Length, kREQUIRED);
  extract_number(kUINT, &Rate, kREQUIRED);
  if (error_byte)
  { 
    return;
  }

  // Handle loading things up for RB0
  gPulseLen[0] = Length;
  gPulseRate[0] = Rate;

  // And now loop for the other 3
  for (i = 0; i < 3u; i++)
  {
    RetVal1 = extract_number(kUINT, &Length, kOPTIONAL);
    RetVal2 = extract_number(kUINT, &Rate, kOPTIONAL);
    if (error_byte) 
    { 
      return;
    }
    if (RetVal1 != kEXTRACT_OK || RetVal2 != kEXTRACT_OK)
    {
      break;
    }
    // Handle loading things up for RB1 through RB3
    gPulseLen[i+1] = Length;
    gPulseRate[i+1] = Rate;
  }

  print_line_ending(kLE_OK_NORM);
}

// PG Pulse Go Command
// Starts a set of pulses (as configured by the PC command)
// going. If a new set of parameters were sent by the PC command,
// PG will cause them all to take effect at the next 1ms
// interval.
//
// Usage:
// PG,1<CR>   Start pulses, or load latest set of parameters and use them
// PG,0<CR>   Stop pulses
void parse_PG_packet(void)
{
  unsigned char Value;

  print_command(FALSE, FALSE);

  extract_number(kUCHAR, &Value, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (Value)
  {
    // Set lower four bits of PortB to outputs
    TRISB = TRISB & 0xF0;

    // Set global flag that turns pulses on
    gPulsesOn = TRUE;
  }
  else
  {
    // Clear global flag that turns pulses on
    gPulsesOn = FALSE;
  }

  print_line_ending(kLE_OK_NORM);
}
#endif

void LongDelay(void)
{
  unsigned char i;
  // A basic for() loop decrementing a 16 bit number would be simpler, but seems to take more code space for
  // a given delay.  So do this instead:
  for(i = 0; i < 0xFF; i++)
  {
    WREG = 0xFF;
    while(WREG)
    {
      WREG--;
      _asm
      bra 0 // Equivalent to bra $+2, which takes half as much code as 2 nop instructions
      bra 0 // Equivalent to bra $+2, which takes half as much code as 2 nop instructions
      bra 0 // Equivalent to bra $+2, which takes half as much code as 2 nop instructions
      _endasm
    }
  }
  // Delay is ~59.8ms at 48MHz.
}

// BL command : simply jump to the bootloader
// Example: "BL<CR>"
void parse_BL_packet(void)
{
  // First, kill interrupts though
  INTCONbits.GIEH = 0;    // Turn high priority interrupts off
  INTCONbits.GIEL = 0;    // Turn low priority interrupts off

  UCONbits.SUSPND = 0;    // Disable USB module
  UCON = 0x00;            // Disable USB module
  // And wait awhile for the USB cable capacitance to discharge down to disconnected (SE0) state. 
  // Otherwise host might not realize we disconnected/reconnected when we do the reset.
  LongDelay();
  _asm goto 0x00001E _endasm
}

// RB ReBoot command : simply jump to the reset vector
// Example: "RB<CR>"
void parse_RB_packet(void)
{
  // First, kill interrupts though
  INTCONbits.GIEH = 0;    // Turn high priority interrupts off
  INTCONbits.GIEL = 0;    // Turn low priority interrupts off

  UCONbits.SUSPND = 0;    // Disable USB module
  UCON = 0x00;            // Disable USB module
  // And wait awhile for the USB cable capacitance to discharge down to disconnected (SE0) state. 
  // Otherwise host might not realize we disconnected/reconnected when we do the reset.
  LongDelay();
  Reset();
}

// QR Query RC Servo power state command
// Example: "RR<CR>"
// Returns "0<CR><LF>OK<CR><LF>" or "1<CR><LF>OK<CR><LF>" 
// 0 = power to RC servo off
// 1 = power to RC servo on
void parse_QR_packet(void)
{
  print_command(FALSE, TRUE);

  ebb_print_uint(RCServoPowerIO_PORT);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_REV);
  }
  print_line_ending(kLE_OK_NORM);
}

// SR Set RC Servo power timeout
// Example: "SR,<new_time_ms>,<new_power_state><CR><LF>"
// Returns "OK<CR><LF>"
// <new_time_ms> is a 32-bit unsigned integer, representing the new RC servo 
// poweroff timeout in milliseconds. This value is not saved across reboots.
// It is the length of time the system will wait after any command that uses
// the motors or servo before killing power to the RC servo.
// Use a value of 0 for <new_time_ms> to completely disable the poweroff timer.
// <new_power_state> is an optional parameter of either 0 or 1. It will
// immediately affect the servo's power state, where 0 turns it off and 1 
// turns it on.
void parse_SR_packet(void)
{
  unsigned long Value;
  UINT8 State;
  ExtractReturnType GotState;

  print_command(FALSE, FALSE);

  extract_number(kULONG, &Value, kREQUIRED);
  GotState = extract_number(kUCHAR, &State, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  gRCServoPoweroffCounterReloadMS = Value;
  
  // Check to see if <new_power_state> is there
  if (GotState == kEXTRACT_OK)
  {
    // Yup, so set new power state
    if (State)
    {
      RCServoPowerIO = RCSERVO_POWER_ON;
    }
    else
    {
      RCServoPowerIO = RCSERVO_POWER_OFF;
    }
  }
  
  print_line_ending(kLE_OK_NORM);
}

// Just used for testing/debugging the packet parsing routines
void parse_CK_packet(void)
{
  unsigned char UByte;
  signed char SByte;
  unsigned int UInt;
  signed int SInt;
  unsigned long ULong;
  signed long SLong;
  unsigned char UChar;
  unsigned char UCaseChar;

  print_command(FALSE, FALSE);
  print_line_ending(kLE_NORM);

  extract_number(kCHAR, &SByte, kREQUIRED);
  extract_number(kUCHAR, &UByte, kREQUIRED);
  extract_number(kINT, &SInt, kREQUIRED);
  extract_number(kUINT, &UInt, kREQUIRED);
  extract_number(kLONG, &SLong, kREQUIRED);
  extract_number(kULONG, &ULong, kREQUIRED);
  extract_number(kASCII_CHAR, &UChar, kREQUIRED);
  extract_number(kUCASE_ASCII_CHAR, &UCaseChar, kREQUIRED);

  ebb_print((rom char far *)"Param1=");
  ebb_print_int(SByte);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param2=");
  ebb_print_uint(UByte);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param3=");
  ebb_print_int(SInt);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param4=");
  ebb_print_uint(UInt);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param5=");
  ebb_print_int(SLong);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param6=");
  ebb_print_uint(ULong);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param7=");
  ebb_print_char(UChar);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param8=");
  ebb_print_char(UCaseChar);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param6=");
  ebb_print_hex(ULong, 8);
  print_line_ending(kLE_NORM);
  ebb_print((rom char far *)"Param6=");
  ebb_print_hex(ULong, 0);
  print_line_ending(kLE_NORM);

  print_line_ending(kLE_OK_NORM);
}

void populateDeviceStringWithName(void)
{
  extern BYTE * USB_SD_Ptr[];

  UINT8 i;

  // Clear out our name array
  for (i=0; i < FLASH_NAME_LENGTH+1; i++)
  {
    gDeviceStringName[i] = 0x00;
  }
  
  // We always read 16, knowing that any unused bytes will be set to zero
  ReadFlash(FLASH_NAME_ADDRESS, FLASH_NAME_LENGTH, gDeviceStringName);

  // The EEB's name is now in the 'name' local variable as a straight string
  // of bytes. We need to move it to the proper locations in the sd002
  // USB string descriptor (which is in RAM now). But it needs to be a 
  // unicode string, so we've got to skip every other byte.
  // Since the FLASH copy of 'name' is padded with zeros and is always 16
  // bytes long, we are safe to always copy 16 bytes over to the string
  // descriptor. 
  // Because sd002 is an anonymous structure without any names for its
  // members, we are totally going to just hack this bad boy and jump
  // into a known offset from the beginning of the structure.
  // As of 2.5.5, we now not only update the Product string, but also the
  // serial number string.
  for (i=0; i < FLASH_NAME_LENGTH; i++)
  {
    // Only copy over valid ASCII characters. On the first invalid
    // one, bail out.
    if (gDeviceStringName[i] <= 128u && gDeviceStringName[i] >= 32u)
    {
      *(USB_SD_Ptr[2] + 24 + (i*2)) = gDeviceStringName[i];
      *(USB_SD_Ptr[3] + 2 + (i*2)) = gDeviceStringName[i];
    }
    else
    {
      break;
    }
  }
  // Now update the string descriptor lengths based on how many characters
  // we copied over from Flash
  *(USB_SD_Ptr[2]) = 24 + (i * 2);
  *(USB_SD_Ptr[3]) = 2 + (i * 2);
}

// ST command : Set Tag
// "ST,<new name><CR>"
// <new name> is a 0 to 16 character ASCII string.
// This string gets saved in FLASH, and is returned by the "QT" command, as
// well as being appended to the USB name that shows up in the OS
void parse_ST_packet(void)
{
  UINT8 bytes = 0;
  UINT8 i;
  
  print_command(FALSE, FALSE);

  // Clear out our name array
  for (i=0; i < FLASH_NAME_LENGTH+1; i++)
  {
    gDeviceStringName[i] = 0x00;
  }
  
  bytes = extract_string(gDeviceStringName, FLASH_NAME_LENGTH);
  
  // We have reserved FLASH addresses 0xF800 to 0xFBFF (1024 bytes) for
  // storing persistent variables like the EEB's name. Note that no wear-leveling
  // is done, so it's not a good idea to change these values more than 10K times. :-)
  
  EraseFlash(FLASH_NAME_ADDRESS, FLASH_NAME_ADDRESS + 0x3FF);
  
  WriteBytesFlash(FLASH_NAME_ADDRESS, FLASH_NAME_LENGTH, gDeviceStringName);

  print_line_ending(kLE_OK_NORM);
}

// QT command : Query Tag
// "QT<CR>"
// Prints out the 'tag' that was set with the "ST" command previously, if any

/// TODO: Optimize this by simply pointing ebb_print() at the string in FLASH?
/// We could save 16 bytes of RAM that way and make the code simpler.

void parse_QT_packet(void)
{
  UINT8 i;
  
  print_command(FALSE, TRUE);

  // Clear out our name array
  for (i=0; i < FLASH_NAME_LENGTH+1; i++)
  {
    gDeviceStringName[i] = 0x00;
  }
  
  // We always read 16, knowing that any unused bytes will be set to zero
  ReadFlash(FLASH_NAME_ADDRESS, FLASH_NAME_LENGTH, gDeviceStringName);
  
  // Only print it out if the first character is printable ASCII
  if (gDeviceStringName[0] < 128u && gDeviceStringName[0] > 32u)
  {
    ebb_print_ram((char *)gDeviceStringName);
  }
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_NORM);
  }
  print_line_ending(kLE_OK_NORM);
}

// Look at the string in g_RX_buf[]
// Copy over all bytes from g_RX_buf_out into ReturnValue until you hit
// a comma or a CR or you've copied over MaxBytes characters. 
// Return the number of bytes copied. Advance g_RX_buf_out as you go.
UINT8 extract_string (
  unsigned char * ReturnValue, 
  UINT8 MaxBytes
)
{
  UINT8 bytes = 0;

  // Always terminate the string
  *ReturnValue = 0x00;
    
  // Check to see if we're already at the end
  if (kCR == g_RX_buf[g_RX_buf_out])
  {
    bitset(error_byte, kERROR_BYTE_MISSING_PARAMETER);
    return(0);
  }

  // Check for comma where ptr points
  if (g_RX_buf[g_RX_buf_out] != ',')
  {
    ebb_print((rom char far *)"!5 Err: Need comma next, found: '");
    ebb_print_char(g_RX_buf[g_RX_buf_out]);
    ebb_print_char(0x27); // The ' character
    print_line_ending(kLE_NORM);
    bitset(error_byte, kERROR_BYTE_PRINTED_ERROR);
    return(0);
  }

  // Move to the next character
  advance_RX_buf_out();

  while(1)
  {
    // Check to see if we're already at the end
    if (kCR == g_RX_buf[g_RX_buf_out] || (BYTE)',' == g_RX_buf[g_RX_buf_out] || bytes >= MaxBytes)
    {
      return (bytes);
    }

    // Copy over a byte
    *ReturnValue = g_RX_buf[g_RX_buf_out];
    
    // Move to the next character
    advance_RX_buf_out();
    
    // Count this one
    bytes++;
    ReturnValue++;
  }
  
  return(bytes);
}


// Look at the string pointed to by g_RX_buf[g_RX_buf_out]
// There should be a comma where g_RX_buf[g_RX_buf_out] points to upon entry.
// If not, throw a comma error.
// If so, then look for up to like a ton of bytes after the
// comma for numbers, and put them all into one
// unsigned long accumulator. 
// Advance the pointer to the byte after the last number
// and return.
ExtractReturnType extract_number(
  ExtractType Type, 
  void * ReturnValue, 
  unsigned char Required
)
{
  unsigned long ULAccumulator;
  signed long Accumulator;
  BOOL Negative = FALSE;

  // Check to see if we're already at the end
  if (kCR == g_RX_buf[g_RX_buf_out])
  {
    if (0u == Required)
    {
      bitset(error_byte, kERROR_BYTE_MISSING_PARAMETER);
    }
    return(kEXTRACT_MISSING_PARAMETER);
  }

  // Check for comma where ptr points
  if (g_RX_buf[g_RX_buf_out] != ',')
  {
    if (0u == Required)
    {
      ebb_print((rom char far *)"!5 Err: Need comma next, found: '");
      ebb_print_char(g_RX_buf[g_RX_buf_out]);
      ebb_print_char(0x27);     // The ' character
      print_line_ending(kLE_NORM);
      bitset (error_byte, kERROR_BYTE_PRINTED_ERROR);
    }
    return(kEXTRACT_COMMA_MISSING);
  }

  // Move to the next character
  advance_RX_buf_out();

  // Check for end of command
  if (kCR == g_RX_buf[g_RX_buf_out])
  {
    if (0u == Required)
    {
      bitset(error_byte, kERROR_BYTE_MISSING_PARAMETER);
    }
    return(kEXTRACT_MISSING_PARAMETER);
  }

  // Now check for a sign character if we're not looking for ASCII chars
  if (
    ('-' == g_RX_buf[g_RX_buf_out]) 
    && 
    (
      (kASCII_CHAR != Type)
      &&
      (kUCASE_ASCII_CHAR != Type)
    )
  )
  {
    // It's an error if we see a negative sign on an unsigned value
    if (
      (kUCHAR == Type)
      ||
      (kUINT == Type)
      ||
      (kULONG == Type)
    )
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
      return(kEXTRACT_PARAMETER_OUTSIDE_LIMIT);
    }
    else
    {
      Negative = TRUE;
      // Move to the next character
      advance_RX_buf_out();
    }
  }

  // If we need to get a digit, go do that
  if (
    (kASCII_CHAR != Type)
    &&
    (kUCASE_ASCII_CHAR != Type)
  )
  {
    extract_digit(&ULAccumulator, 10);
  }
  else
  {
    // Otherwise just copy the byte
    ULAccumulator = g_RX_buf[g_RX_buf_out];

    // Force uppercase if that's what type we have
    if (kUCASE_ASCII_CHAR == Type)
    {
      ULAccumulator = toupper(ULAccumulator);
    }
    
    // Move to the next character
    advance_RX_buf_out();
  }

  // Range check absolute values
  if (Negative)
  {
    if (
      (
        kCHAR == Type
        &&
        (ULAccumulator > (unsigned long)128)
      )
      ||
      (
        kINT == Type
        &&
        (ULAccumulator > (unsigned long)32768)
      )
      ||
      (
        kLONG == Type
        &&
        (ULAccumulator > (unsigned long)0x80000000L)
      )
    )
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
      return(kEXTRACT_PARAMETER_OUTSIDE_LIMIT);
    }

    Accumulator = ULAccumulator;
    // Then apply the negative if that's the right thing to do
    if (Negative)
    {
      Accumulator = -Accumulator;
    }
  }
  else
  {
    if (
      (
        kCHAR == Type
        &&
        (ULAccumulator > (unsigned long)127)
      )
      ||
      (
        kUCHAR == Type
        &&
        (ULAccumulator > (unsigned long)255)
      )
      ||
      (
        kINT == Type
        &&
        (ULAccumulator > (unsigned long)32767)
      )
      ||
      (
        kUINT == Type
        &&
        (ULAccumulator > (unsigned long)65535)
      )
      ||
      (
        kLONG == Type
        &&
        (ULAccumulator > (unsigned long)0x7FFFFFFFL)
      )
    )
    {
      bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
      return(kEXTRACT_PARAMETER_OUTSIDE_LIMIT);
    }

    if (kULONG != Type)
    {
      Accumulator = ULAccumulator;
    }
  }

  // If all went well, then copy the result
  switch (Type)
  {
    case kCHAR:
      *(signed char *)ReturnValue = (signed char)Accumulator;
      break;
    case kUCHAR:
    case kASCII_CHAR:
    case kUCASE_ASCII_CHAR:
      *(unsigned char *)ReturnValue = (unsigned char)Accumulator;
      break;
    case kINT:
      *(signed int *)ReturnValue = (signed int)Accumulator;
      break;
    case kUINT:
      *(unsigned int *)ReturnValue = (unsigned int)Accumulator;
      break;
    case kLONG:
      *(signed long *)ReturnValue = Accumulator;
      break;
    case kULONG:
      *(unsigned long *)ReturnValue = ULAccumulator;
      break;
    default:
      return(kEXTRACT_INVALID_TYPE);
  }
  return(kEXTRACT_OK);
}

// Loop 'digits' number of times, looking at the
// byte in input_buffer index *ptr, and if it is
// a digit, adding it to acc. Take care of 
// powers of ten as well. If you hit a non-numerical
// char, then return FALSE, otherwise return TRUE.
// Store result as you go in *acc.
signed char extract_digit(unsigned long * acc, unsigned char digits)
{
  unsigned char val;
  unsigned char digit_cnt;

  *acc = 0;

  for (digit_cnt = 0; digit_cnt < digits; digit_cnt++)
  {
    val = g_RX_buf[g_RX_buf_out];
    if ((val >= 48u) && (val <= 57u))
    {
      *acc = (*acc * 10) + (val - 48);
      // Move to the next character
      advance_RX_buf_out();
    }
    else
    {
      return(FALSE);
    }
  }
  return(TRUE);
}


// For debugging, this command will spit out a bunch of values.
void print_status(void)
{
  ebb_print((far rom char*)"Status=");
  ebb_print_uint(ISR_D_FIFO_length);
  print_line_ending(kLE_NORM);
}

/******************************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs corresponding to
 *                  the USB device state.
 *
 * Note:            mLED macros can be found in io_cfg.h
 *                  usb_device_state is declared in usbmmap.c and is modified
 *                  in usbdrv.c, usbctrltrf.c, and usb9.c
 *****************************************************************************/
void BlinkUSBStatus(void)
{
  static WORD LEDCount = 0;
  static unsigned char LEDState = 0;

  if (
    USBDeviceState == DETACHED_STATE
    ||
    1u == USBSuspendControl
  )
  {
    LEDCount--;
    if (0u == LEDState)
    {
      if (0u == LEDCount)
      {
        mLED_1_On();
        LEDCount = 4000U;
        LEDState = 1;
      }
    }
    else
    {
      if (0u == LEDCount)
      {
        mLED_1_Off();
        LEDCount = 4000U;
        LEDState = 0;
      }
    }
  }
  else if (
    USBDeviceState == ATTACHED_STATE
    ||
    USBDeviceState == POWERED_STATE
    ||
    USBDeviceState == DEFAULT_STATE
    ||
    USBDeviceState == ADDRESS_STATE
  )
  {
    LEDCount--;
    if (0u == LEDState)
    {
      if (0u == LEDCount)
      {
        mLED_1_On();
        LEDCount = 20000U;
        LEDState = 1;
      }
    }
    else
    {
      if (0u == LEDCount)
      {
        mLED_1_Off();
        LEDCount = 20000U;
        LEDState = 0;
      }
    }
  }
  else if (USBDeviceState == CONFIGURED_STATE)
  {
    LEDCount--;
    if (0u == LEDState)
    {
      if (0u == LEDCount)
      {
        mLED_1_On();
        LEDCount = 10000U;
        LEDState = 1;
      }
    }
    else if (1u == LEDState)
    {
      if (0u == LEDCount)
      {
        mLED_1_Off();
        LEDCount = 10000U;
        LEDState = 2;
      }
    }
    else if (2u == LEDState)
    {
      if (0u == LEDCount)
      {
        mLED_1_On();
        LEDCount = 100000U;
        LEDState = 3;
      }
    }
    else
    {
      if (0u == LEDCount)
      {
        mLED_1_Off();
        LEDCount = 10000U;
        LEDState = 0;
      }
    }
  }
}

volatile near unsigned char * rom RPnTRISPort[25] = {
  &TRISA,     // RP0
  &TRISA,     // RP1
  &TRISA,     // RP2
  &TRISB,     // RP3
  &TRISB,     // RP4
  &TRISB,     // RP5
  &TRISB,     // RP6
  &TRISB,     // RP7
  &TRISB,     // RP8
  &TRISB,     // RP9
  &TRISB,     // RP10
  &TRISC,     // RP11
  &TRISC,     // RP12
  &TRISC,     // RP13
  &TRISC,     // RP14
  &TRISC,     // RP15
  &TRISC,     // RP16
  &TRISC,     // RP17
  &TRISC,     // RP18
  &TRISD,     // RP19
  &TRISD,     // RP20
  &TRISD,     // RP21
  &TRISD,     // RP22
  &TRISD,     // RP23
  &TRISD,     // RP24
};

volatile near unsigned char * rom RPnLATPort[25] = {
  &LATA,      // RP0
  &LATA,      // RP1
  &LATA,      // RP2
  &LATB,      // RP3
  &LATB,      // RP4
  &LATB,      // RP5
  &LATB,      // RP6
  &LATB,      // RP7
  &LATB,      // RP8
  &LATB,      // RP9
  &LATB,      // RP10
  &LATC,      // RP11
  &LATC,      // RP12
  &LATC,      // RP13
  &LATC,      // RP14
  &LATC,      // RP15
  &LATC,      // RP16
  &LATC,      // RP17
  &LATC,      // RP18
  &LATD,      // RP19
  &LATD,      // RP20
  &LATD,      // RP21
  &LATD,      // RP22
  &LATD,      // RP23
  &LATD,      // RP24
};

const char rom RPnBit[25] = {
  0,          // RP0
  1,          // RP1
  5,          // RP2
  0,          // RP3
  1,          // RP4
  2,          // RP5
  3,          // RP6
  4,          // RP7
  5,          // RP8
  6,          // RP9
  7,          // RP10
  0,          // RP11
  1,          // RP12
  2,          // RP13
  3,          // RP14
  4,          // RP15
  5,          // RP16
  6,          // RP17
  7,          // RP18
  2,          // RP19
  3,          // RP20
  4,          // RP21
  5,          // RP22
  6,          // RP23
  7,          // RP24
};

// From RPn (Pin) number, set LAT value for that pin
void SetPinLATFromRPn(char Pin, char State)
{
  if (Pin > 25)
  {
    return;
  }

  if (State)
  {
    bitset(*RPnLATPort[Pin], RPnBit[Pin]);
  }
  else
  {
    bitclr(*RPnLATPort[Pin], RPnBit[Pin]);
  }
}

// From RPn (Pin) number, set TRIS value for that pin
void SetPinTRISFromRPn(char Pin, char State)
{
  if (Pin > 25)
  {
    return;
  }

  if (OUTPUT_PIN == State)
  {
    bitclr (*RPnTRISPort[Pin], RPnBit[Pin]);
  }
  else
  {
    bitset (*RPnTRISPort[Pin], RPnBit[Pin]);
  }
}

/** EOF user.c ***************************************************************/
