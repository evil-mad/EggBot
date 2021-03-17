
#include <p18cxxx.h>
#include <usart.h>
#include <stdio.h>
#include <ctype.h>
#include <delays.h>
#include <math.h>
#include "usb_config.h"
#include "Usb\usb.h"
#include "Usb\usb_function_cdc.h"
#include "HardwareProfile.h"
#include "ebb.h"
#include "delays.h"
#include "ebb_demo.h"
#include "servo.h"
#include "fifo.h"
#include "parse.h"
#include "utility.h"
#include "stepper.h"


#pragma udata

static unsigned char i;

unsigned int DemoModeActive;
unsigned int comd_counter;

DriverConfigurationType DriverConfiguration;


unsigned long NodeCount;
char Layer;
BOOL ButtonPushed;
BOOL UseAltPause;
UINT StoredEngraverPower;

// Stepper (mode) Configure command
// SC,1,0<CR> will disable the solenoid (RB4) output for pen up/down
// SC,1,1<CR> will enable the solenoid (RB4) output for pen up/down (default)
// SC,2,0<CR> will make PIC control drivers (default)
// SC,2,1<CR> will make PIC control external drivers using these pins
//    ENABLE1 = RD1
//    ENABLE2 = RA1
//    STEP1 = RC6
//    DIR1 = RC2
//    STEP2 = RA5
//    DIR2 = RA2
// SC,2,2<CR> will disconnect PIC from drivers and allow external step/dir source
// FOR EBB
// SC,4,<gPenMinPosition><CR> will set the minimum value for the pen servo (1 to 11890)
// SC,5,<gPenMaxPosition><CR> will set the maximum value for the pen servo (1 to 11890)
// FOR 3BB
// SC,4,<gPenMinPosition><CR> will set the minimum value for the pen stepper (1 to 32767 in microsteps)
// SC,5,<gPenMaxPosition><CR> will set the maximum value for the pen stepper (1 to 32767 in microsteps)
// SC,8,<servo2_slots><CR> sets the number of slots for the servo2 system (1 to 24)
// SC,9,<servo2_slotMS><CR> sets the number of ms in duration for each slot (1 to 6)
// SC,10,<gPenMoveDuration><CR> set the new global default pen move duration in ms (16 bit uint)
// SC,13,1<CR> enables RB3 as parallel input to PRG button for pause detection
// SC,13,0<CR> disables RB3 as parallel input to PRG button for pause detection
void parseSCCommand(void)
{
  unsigned char Para1 = 0;
  unsigned int Para2 = 0;

  // Extract each of the values.
  extract_number (kUINT8, &Para1, kREQUIRED);
  extract_number (kUINT16, &Para2, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Check for command to enable/disable solenoid output (RB4) for pen up/down
  if (Para1 == 1)
  {
    // Disable use of solenoid output
    if (Para2 == 0)
    {
      gUseSolenoid = FALSE;
      /// TOTO: Turn off solenoid output here
    }
    else
    {
      gUseSolenoid = TRUE;
      /// TODO: Turn on solenoid output here (make sure RB4 is an output)
    }
  }
#if defined(BOARD_EBB)
  // Check for command to switch between built-in drivers and external drivers
  else if (Para1 == 2)
  {
    if (Para2 == 0)
    {
      DriverConfiguration = PIC_CONTROLS_DRIVERS;
      // Connections to drivers become outputs
      Enable1IO_TRIS = OUTPUT_PIN;
      Enable2IO_TRIS = OUTPUT_PIN;
      Step1IO_TRIS = OUTPUT_PIN;
      Dir1IO_TRIS = OUTPUT_PIN;
      Step2IO_TRIS = OUTPUT_PIN;
      Dir2IO_TRIS = OUTPUT_PIN;
      // Alternate I/O pins become inputs
      Dir1AltIO_TRIS = INPUT_PIN;
      Dir2AltIO_TRIS = INPUT_PIN;
      Step1AltIO_TRIS = INPUT_PIN;
      Step2AltIO_TRIS = INPUT_PIN;
      Enable1AltIO_TRIS = INPUT_PIN;
      Enable2AltIO_TRIS = INPUT_PIN;
    }
    else if (Para2 == 1)
    {
      DriverConfiguration = PIC_CONTROLS_EXTERNAL;
      // Connections to drivers become inputs
      Enable1IO_TRIS = INPUT_PIN;
      Enable2IO_TRIS = INPUT_PIN;
      Step1IO_TRIS = INPUT_PIN;
      Dir1IO_TRIS = INPUT_PIN;
      Step2IO_TRIS = INPUT_PIN;
      Dir2IO_TRIS = INPUT_PIN;
      // Alternate I/O pins become outputs
      Dir1AltIO_TRIS = OUTPUT_PIN;
      Dir2AltIO_TRIS = OUTPUT_PIN;
      Step1AltIO_TRIS = OUTPUT_PIN;
      Step2AltIO_TRIS = OUTPUT_PIN;
      Enable1AltIO_TRIS = OUTPUT_PIN;
      Enable2AltIO_TRIS = OUTPUT_PIN;
    }
    else if (Para2 == 2)
    {
      DriverConfiguration = EXTERNAL_CONTROLS_DRIVERS;
      // Connections to drivers become inputs
      Enable1IO_TRIS = INPUT_PIN;
      Enable2IO_TRIS = INPUT_PIN;
      Step1IO_TRIS = INPUT_PIN;
      Dir1IO_TRIS = INPUT_PIN;
      Step2IO_TRIS = INPUT_PIN;
      Dir2IO_TRIS = INPUT_PIN;
      // Alternate I/O pins become inputs
      Dir1AltIO_TRIS = INPUT_PIN;
      Dir2AltIO_TRIS = INPUT_PIN;
      Step1AltIO_TRIS = INPUT_PIN;
      Step2AltIO_TRIS = INPUT_PIN;
      Enable1AltIO_TRIS = INPUT_PIN;
      Enable2AltIO_TRIS = INPUT_PIN;
     }
  }
#endif
  else if (Para1 == 4)
  {
    // Set minimum position for pen
    gPenMinPosition = (INT16)Para2;
  }
  else if (Para1 == 5)
  {
    // Set maximum position for pen
    gPenMaxPosition = (INT16)Para2;
  }
  // Set <gRC2Slots>
  else if (Para1 == 8)
  {
    if (Para2 > MAX_RC2_SERVOS)
    {
      Para2 = MAX_RC2_SERVOS;
    }
    gRC2Slots = Para2;
  }
  else if (Para1 == 9)
  {
    if (Para2 > 6)
    {
      Para2 = 6;
    }
    gRC2SlotMS = Para2;
  }
  else if (Para1 == 10)
  {
    gPenMoveDuration = Para2;
  }
  else if (Para1 == 13)
  {
    if (Para2)
    {
      UseAltPause = TRUE;
    }
    else
    {
      UseAltPause = FALSE;
    }
  }
  print_ack();
}

#if 0
void fprint(float f)
{
  float pf = 0;

  if (f > 2147483648.0)
  {
    printf((far rom char *)"f too big\n);
  }
  else
  {
    if (f < -2147483648.0)
    {
      printf((far rom char *)"f too small\n");
    }
    else
    {
      if (f < 0.0)
      {
        pf = 0.0 - f;
      }
      else
      {
        pf = f;
      }
      printf((far rom char *)"%ld.%04lu\n", (INT32)f, (UINT32)((pf - (float)((INT32)pf)) * 10000));
    }
  }
}
#endif





// Query Pen
// Usage: QP<CR>
// Returns: 0 for down, 1 for up, then OK<CR>
void parseQPCommand(void)
{
  printf((far rom char *)"%d\n", gPenStateActual);

  print_ack();
}

// Node counter increment
// Usage: NI<CR>
void parseNICommand(void)
{
  if (NodeCount < 0xFFFFFFFEL)
  {
    NodeCount++;
  }
  print_ack();
}

// Node counter Decrement
// Usage: ND<CR>
void parseNDCommand(void)
{
  if (NodeCount)
  {
    NodeCount--;
  }
  print_ack();
}

// Set Node counter
// Usage: SN,<value><CR>
// <value> is a 4 byte unsigned value
void parseSNCommand(void)
{
  unsigned long Temp;
  ExtractReturnType RetVal;

  RetVal = extract_number (kUINT32, &Temp, kREQUIRED);
  if (kEXTRACT_OK == RetVal)
  {
    NodeCount = Temp;
  }
  print_ack();
}

// Query Node counter
// Usage: QN<CR>
// Returns: <NodeCount><CR>
// OK<CR>
void parseQNCommand(void)
{
  printf ((far rom char*)"%010lu\n", NodeCount);

  print_ack();
}

// Set Layer
// Usage: SL,<NewLayer><CR>
void parseSLCommand(void)
{
  // Extract each of the values.
  extract_number (kUINT8, &Layer, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  print_ack();
}

// Query Layer
// Usage: QL<CR>
// Returns: <Layer><CR>
// OK<CR>
void parseQLCommand(void)
{
  printf ((far rom char*)"%03i\n", Layer);

  print_ack();
}

// Query Button
// Usage: QB<CR>
// Returns: <HasButtonBeenPushedSinceLastQB><CR> (0 or 1)
// OK<CR>
void parseQBCommand(void)
{
  printf ((far rom char*)"%1i\n", ButtonPushed);
  if (ButtonPushed)
  {
    ButtonPushed = FALSE;
  }
  print_ack();
}

// Query Current
// Usage: QC<CR>
// Returns: <voltage_on_REF_RA0_net>,<voltage_on_v+_net><CR>
// Both values have a range of 0 to 1023 (10-bit ADC)
// The ADC is set up with 0V = 0 and 3.3V = 1023
// For the REF_RA0 (current adjustment pot) a value of 0V-ADC (0 counts) = 46mA
// and a value of 2.58V-ADC (800 counts) = 1.35A
// For the V+ net a value of 0V-ADC (0 counts) = 0V on V+
// and a value of 2.79V-ADC (870 counts) = 31.4V on V+
// REF_RA0 comes in on AN0 (RA0)
// V+ comes in on AN11 (RC2)
void parseQCCommand(void)
{
  // Print out our results
  /// TODO: update this for EBB for analogConvert())
//  printf ((far rom char*)"%04i,%04i\n", ISR_A_FIFO[0], ISR_A_FIFO[11]);

  print_ack();
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
void parseQGCommand(void)
{
  UINT8 result = process_QM();
  UINT8 param = 0;
  
  extract_number (kUINT8, &param, kOPTIONAL);

  // process_QM() gives us the low 4 bits of our output result.
  result = result & 0x0F;

  if (gPenStateActual)
  {
    result = result | (1 << 4);
  }
  if (ButtonPushed)
  {
    result = result | (1 << 5);
  }
  if (PORTBbits.RB2)
  {
    result = result | (1 << 6);
  }
  if (PORTBbits.RB5)
  {
    result = result | (1 << 7);
  }

  if (param == 1)
  {
    printf ((far rom char*)"%02X,0\n", result);
  }
  else if (param == 2)
  {
    printf ((far rom char*)"%02X,0,%u\n", result, FIFODepth);
  }
  else
  {
    printf ((far rom char*)"%02X\n", result);
  }
    
  // Reset the button pushed flag
  if (ButtonPushed)
  {
    ButtonPushed = FALSE;
  }
}

// Set Engraver
// Usage: SE,<state>,<power>,<use_motion_queue><CR>
// <state> is 0 for off and 1 for on (required)
// <power> is 10 bit PWM power level (optional). 0 = 0%, 1023 = 100%
// <use_motion_queue> if 1 then put this command in motion queue (optional))
// We boot up with <power> at 0
// The engraver motor is always assumed to be on RB3
// So our init routine will map ECCP1
//
// Timer0 is RC command
// Timer1 is stepper
// Timer2 and ECCP1 is engraver PWM
// Timer3 and ECCP2 is RC servo2 output
// Timer4 is 1ms ISR

void parseSECommand(void)
{
  UINT8 State = 0;
  UINT16 Power = 0;
  UINT8 SEUseMotionQueue = FALSE;
  ExtractReturnType PowerExtract;

  // Extract each of the values.
  extract_number (kUINT8, &State, kREQUIRED);
  PowerExtract = extract_number (kUINT16, &Power, kOPTIONAL);
  extract_number (kUINT8, &SEUseMotionQueue, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check
  if (Power > 1023)
  {
    Power = 1023;
  }
  if (State > 1)
  {
    State = 1;
  }
  if (SEUseMotionQueue > 1)
  {
    SEUseMotionQueue = 1;
  }
    
  // Set to %50 if no Power parameter specified, otherwise use parameter
  if (State == 1 && PowerExtract == kEXTRACT_MISSING_PARAMETER)
  {
    StoredEngraverPower = 512;
  }
  else
  {
    StoredEngraverPower = Power;
  }

  // If we're not on, then turn us on
  if (T2CONbits.TMR2ON != 1)
  {
    // Set up PWM for Engraver control
    // We will use ECCP1 and Timer2 for the engraver PWM output on RB3
    // Our PWM will operate at about 40Khz.

    // Set our reload value
    PR2 = 0xFF;

    // Initialize Timer2

    // The prescaler will be at 1
    T2CONbits.T2CKPS = 0b00;

    // Do not generate an interrupt
    PIE1bits.TMR2IE = 0;

    TCLKCONbits.T3CCP1 = 1;       // ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4
    TCLKCONbits.T3CCP2 = 0;       // ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4

    CCP1CONbits.CCP1M = 0b1100;   // Set EECP1 as PWM mode
    CCP1CONbits.P1M = 0b00;       // Enhanced PWM mode: single output

    // Set up output routing to go to RB3 (RP6)
    RPOR6 = 14;                   // 14 is CCP1/P1A - ECCP1 PWM Output Channel A

    T2CONbits.TMR2ON = 1;         // Turn it on
  }

  // Acting on the state is only done if the SE command is not put on the motion queue
  if (!SEUseMotionQueue)
  {
    // Now act on the State
    if (State)
    {
      // Set RB3 to StoredEngraverPower
      CCPR1L = StoredEngraverPower >> 2;
      CCP1CON = (CCP1CON & 0b11001111) | ((StoredEngraverPower << 4) & 0b00110000);
    }
    else
    {
      // Set RB3 to low by setting PWM duty cycle to zero
      CCPR1L = 0;
      CCP1CON = (CCP1CON & 0b11001111);
    }
  }
  else
  {
    // Trial: Spin here until there's space in the fifo
    WaitForRoomInQueue();

    // Set up the motion queue command
    FIFO_G1[FIFOIn].SEPower = StoredEngraverPower;
    FIFO_G5[FIFOIn].DelayCounter = 0;
    FIFO_G2[FIFOIn].SEState = State;
    FIFO_Command[FIFOIn] = COMMAND_SE;

    fifo_Inc();
  }
    
  print_ack();
}

