/*********************************************************************
 *
 *                ThreeBotBoard Firmware
 *
 *********************************************************************
 * FileName:        commands.c
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

/************** INCLUDES ******************************************************/

#include <stdio.h>
#include "stm32g4xx_hal.h"
#include "parse.h"
//#include "fifo.h"
//#include "main.h"
#include "servo.h"
//#include "analog.h"
#include "utility.h"
//#include "ebb.h"
//#include "init.h"
#include "commands.h"

/************** MODULE DEFINES ************************************************/

// How many 'addresses' of 32-bit RAM variables should the MR/MW commands have
#define NUMBER_OF_RAM_COOKIES   512

/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// Set to TRUE to turn Pulse Mode on
uint8_t gPulsesOn = false;
// For Pulse Mode, how long should each pulse be on for in ms?
uint16_t gPulseLen[4] = {0,0,0,0};
// For Pulse Mode, how many ms between rising edges of pulses?
uint16_t gPulseRate[4] = {0,0,0,0};
// For Pulse Mode, counters keeping track of where we are
uint16_t gPulseCounters[4] = {0,0,0,0};

// For MR and MW commands, array to store RAM cookies
static uint32_t Cookies[NUMBER_OF_RAM_COOKIES];

/************** PRIVATE FUNCTION PROTOTYPES ***********************************/

/************** PRIVATE FUNCTIONS *********************************************/

/************** PUBLIC FUNCTIONS **********************************************/

/*
 * commands_SCCommand()
 * Stepper/Servo (mode) Configure command
 * SC,1,0<CR> will disable the solenoid (RB4) output for pen up/down
 * SC,1,1<CR> will enable the solenoid (RB4) output for pen up/down (default)
 * SC,2,0<CR> will make PIC control drivers (default)
 * SC,2,1<CR> will make PIC control external drivers using these pins
 *    ENABLE1 = RD1
 *    ENABLE2 = RA1
 *    STEP1 = RC6
 *    DIR1 = RC2
 *    STEP2 = RA5
 *    DIR2 = RA2
 * SC,2,2<CR> will disconnect PIC from drivers and allow external step/dir source
 * FOR EBB
 * SC,4,<gPenMinPosition><CR> will set the minimum value for the pen servo (1 to 11890)
 * SC,5,<gPenMaxPosition><CR> will set the maximum value for the pen servo (1 to 11890)
 * FOR 3BB
 * SC,4,<gPenMinPosition><CR> will set the minimum value for the pen stepper (1 to 32767 in microsteps)
 * SC,5,<gPenMaxPosition><CR> will set the maximum value for the pen stepper (1 to 32767 in microsteps)
 * SC,8,<servo2_slots><CR> sets the number of slots for the servo2 system (1 to 24)
 * SC,9,<servo2_slotMS><CR> sets the number of ms in duration for each slot (1 to 6)
 * SC,10,<gPenMoveDuration><CR> set the new global default pen move duration in ms (16 bit uint)
 * SC,13,1<CR> enables RB3 as parallel input to PRG button for pause detection
 * SC,13,0<CR> disables RB3 as parallel input to PRG button for pause detection
 */
void commands_SCCommand(void)
{
  uint8_t para1 = 0;
  uint16_t para2 = 0;

  // Extract each of the values.
  extract_number (kUINT8, &para1, kREQUIRED);
  extract_number (kUINT16, &para2, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

#if 0
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
#endif

  // Set Pen RC Servo min position (pen up)
  if (para1 == 4)
  {
    servo_SetPenMinPosition(para2);
  }
  // Set Pen RC servo max position (pen down)
  if (para1 == 5)
  {
    servo_SetPenMaxPosition(para2);
  }

#if 0
  // Set <gRC2Slots>
  if (Para1 == 8)
  {
    if (Para2 > MAX_RC2_SERVOS)
    {
      Para2 = MAX_RC2_SERVOS;
    }
    gRC2Slots = Para2;
  }
  if (Para1 == 9)
  {
    if (Para2 > 6)
    {
      Para2 = 6;
    }
    gRC2SlotMS = Para2;
  }
#endif

  // Set Pen RC servo rates - both going up and going down
  if (para1 == 10)
  {
    servo_SetPenRateUp(para2);
    servo_SetPenRateDown(para2);
  }
  // Set Pen RC servo rate - just going up
  if (para1 == 11)
  {
    servo_SetPenRateUp(para2);
  }
  // Set Pen RC servo rate - just going down
  if (para1 == 12)
  {
    servo_SetPenRateDown(para2);
  }

#if 0
  if (Para1 == 13)
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
#endif

  print_ack();
}


#if 0
// Return all I/Os to their default power-on values
void parseRSCommand(void)
{
  UserInit();
  print_ack();
}

// CU is "Configure UBW" and controls system-wide configuration values
// "CU,<parameter_number>,<paramter_value><CR>"
// <paramter_number> <parameter_value>
// 1                  {1|0} turns on or off the 'ack' ("OK" at end of packets)
void parseCUCommand(void)
{
  unsigned char parameter_number = 0;
  signed int parameter_value = 0;

  extract_number (kUINT8, &parameter_number, kREQUIRED);
  extract_number (kINT16, &parameter_value, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  switch(parameter_number)
  {
    case 1:   // Turn on/off ACK ENABLE
      if (0 == parameter_value || 1 == parameter_value)
      {
        g_ack_enable = parameter_value;
      }
      else
      {
        bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
      }
      break;
      
    case 2:   // Turn on/off limit checks
      if (0 == parameter_value || 1 == parameter_value)
      {
        gLimitChecks = parameter_value;
      }
      else
      {
        bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
      }
      break;
      
    case 3:   // Set/read FIFO SIZE
      if (0 != parameter_value)
      {
        // Set the FIFOSize to parameter_value if parameter_value isn't zero
        if (parameter_value > COMMAND_FIFO_LENGTH)
        {
          FIFOSize = COMMAND_FIFO_LENGTH;
        }
        else
        {
          FIFOSize = parameter_value;
        }
      }
      // Always return FIFODepth and max FIFO depth
      printf ((far rom char*)"%i,%i\n", FIFOSize, COMMAND_FIFO_LENGTH);
      break;
      
    default:
      break;
  }
  print_ack();
}

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
void parseCBCommand(void)
{
  unsigned char PA, PB, PC, PD, PE;

  // Extract each of the four values.
  extract_number (kUINT8, &PA, kREQUIRED);
  extract_number (kUINT8, &PB, kREQUIRED);
  extract_number (kUINT8, &PC, kREQUIRED);
  extract_number (kUINT8, &PD, kREQUIRED);
  extract_number (kUINT8, &PE, kREQUIRED);

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

  print_ack ();
}


// Outputs values to the ports pins that are set up as outputs.
// Example "O,121,224,002<CR>"
void parseODCommand(void)
{
  unsigned char Value;
  ExtractReturnType RetVal;

  // Extract each of the values.
  RetVal = extract_number (kUINT8,  &Value, kREQUIRED);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATA = Value;
  }
  RetVal = extract_number (kUINT8,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATB = Value;
  }
  RetVal = extract_number (kUINT8,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATC = Value;
  }
  RetVal = extract_number (kUINT8,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATD = Value;
  }
  RetVal = extract_number (kUINT8,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATE = Value;
  }

  print_ack ();
}

// Read in the three I/O ports (A,B,C) and create
// a packet to send back with all of values.
// Example: "I,143,221,010<CR>"
// Remember that on UBW 28 pin boards, we only have
// Port A bits 0 through 5
// Port B bits 0 through 7
// Port C bits 0,1,2 and 6,7
// And that Port C bits 0,1,2 are used for
//  User1 LED, User2 LED and Program switch respectively.
// The rest will be read in as zeros.
void parseIDCommand(void)
{
  printf (
    (far rom char*)"I,%03i,%03i,%03i,%03i,%03i\n",
    PORTA,
    PORTB,
    PORTC,
    PORTD,
    PORTE
  );
}

#endif

// All we do here is just print out our version number
void commands_VRCommand(void)
{
  printf("%s\n", st_version);
}

// All we do here is just print out our version number
// This is the legacy version which has \r\n at the end
void commands_VCommand(void)
{
  printf("%s\r\n", st_version);
}


// MW is for Memory Write
// "MW,<location>,<value><CR>"
// <location> is a decimal value between 0 and 511 indicating the RAM address to write to
// <value> is am unsigned 32 bit decimal value between 0 and 4294967295 that is the value to write
void commands_MWCommand(void)
{
  uint16_t location;
  uint32_t value;

  extract_number(kUINT16, &location, kREQUIRED);
  extract_number(kUINT32, &value, kREQUIRED);

  // Limit check the address
  if (location >= NUMBER_OF_RAM_COOKIES)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
  }

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }
  Cookies[location] = value;

  print_ack();
}


// MR is for Memory Read
// "MW,<location><CR>"
// <location> is a decimal value between 0 and 4096 indicating the RAM address to read from 
// The UBW will then send a "MR,<value><CR>" packet back to the PC
// where <value> is the byte value read from the address
void commands_MRCommand(void)
{
  uint16_t location;
  uint32_t value;

  extract_number(kUINT32, &location, kREQUIRED);

  // Limit check the address
  if (location >= NUMBER_OF_RAM_COOKIES)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
  }

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  value = Cookies[location];

  // Now send back the MR packet
  printf("MR,%lu\n", value);

  print_ack();
}

#if 0

// PD is for Pin Direction
// "PD,<port>,<pin>,<direction><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to change direction on
// <direction> is "1" for input, "0" for output
void parsePDCommand(void)
{
  unsigned char port;
  unsigned char pin;
  unsigned char direction;

  extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number (kUINT8, &pin, kREQUIRED);
  extract_number (kUINT8, &direction, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check the parameters
  if (direction > 1)
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }
  if (pin > 7)
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }
  if ('A' == port)
  {
    if (0 == direction)
    {
      bitclr (TRISA, pin);
    }
    else
    {
      bitset (TRISA, pin);
    }
  }
  else if ('B' == port)
  {
    if (0 == direction)
    {
      bitclr (TRISB, pin);
    }
    else
    {
      bitset (TRISB, pin);
    }
  }
  else if ('C' == port)
  {
    if (0 == direction)
    {
      bitclr (TRISC, pin);
    }
    else
    {
      bitset (TRISC, pin);
    }
  }
  else if ('D' == port)
  {
    if (0 == direction)
    {
      bitclr (TRISD, pin);
    }
    else
    {
      bitset (TRISD, pin);
    }
  }
  else if ('E' == port)
  {
    if (0 == direction)
    {
      bitclr (TRISE, pin);
    }
    else
    {
      bitset (TRISE, pin);
    }
  }
  else
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  print_ack ();
}

// PI is for Pin Input
// "PI,<port>,<pin><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to read
// The command returns a "PI,<value><CR>" packet,
// where <value> is the value (0 or 1 for digital)
// value for that pin.
void parsePICommand(void)
{
  unsigned char port;
  unsigned char pin;
  unsigned char value = 0;

  extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number (kUINT8, &pin, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check the parameters
  if (pin > 7)
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  // Then test the bit in question based upon port
  if ('A' == port)
  {
    value = bittst (PORTA, pin);
  }
  else if ('B' == port)
  {
    value = bittst (PORTB, pin);
  }
  else if ('C' == port)
  {
    value = bittst (PORTC, pin);
  }
  else if ('D' == port)
  {
    value = bittst (PORTD, pin);
  }
  else if ('E' == port)
  {
    value = bittst (PORTE, pin);
  }
  else
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  // Convert to just a binary 1 or 0
  if (value)
  {
    value = 1;
  }

  // Now send back our response
  printf(
     (far rom char *)"PI,%1u\n"
    ,value
  );
}

// PO is for Pin Output
// "PO,<port>,<pin>,<value><CR>"
// <port> is "A", "B", "C" and indicates the port
// <pin> is a number between 0 and 7 and indicates which pin to write out the value to
// <value> is "1" or "0" and indicates the state to change the pin to
void parsePOCommand(void)
{
  unsigned char port;
  unsigned char pin;
  unsigned char value;

  extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number (kUINT8, &pin, kREQUIRED);
  extract_number (kUINT8, &value, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check the parameters
  if (value > 1)
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }
  if (pin > 7)
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }
  if ('A' == port)
  {
    if (0 == value)
    {
      bitclr (LATA, pin);
    }
    else
    {
      bitset (LATA, pin);
    }
  }
  else if ('B' == port)
  {
    if (0 == value)
    {
      bitclr (LATB, pin);
    }
    else
    {
      bitset (LATB, pin);
    }
  }
  else if ('C' == port)
  {
    if (0 == value)
    {
      bitclr (LATC, pin);
    }
    else
    {
      bitset (LATC, pin);
    }
  }
 else if ('D' == port)
  {
    if (0 == value)
    {
      bitclr (LATD, pin);
    }
    else
    {
      bitset (LATD, pin);
    }
  }
  else if ('E' == port)
  {
    if (0 == value)
    {
      bitclr (LATE, pin);
    }
    else
    {
      bitset (LATE, pin);
    }
  }
  else
  {
    bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  print_ack();
}


// PC Pulse Configure
// Pulses will be generated on PortB, bits 0 through 3
// Pulses are in 1ms units, and can be from 0 (off) through 65535
// Each pin has a pulse length, and a repetition rate.
// The repetition rate can be from 0 through 65535, but must be more than the pulse length
// Only the first set of parameters (for RB0) are required, the rest are optional
//
// Usage:
// PC,<RB0_Len>,<RB0_Rate>,<RB1_Len>,<RB1_Rate>,...,<RB3_Len>,<RB3_Rate><CR>
void parsePCCommand(void)
{
  unsigned int Length, Rate;
  unsigned char i;
  ExtractReturnType RetVal1, RetVal2;

  extract_number(kUINT16, &Length, kREQUIRED);
  extract_number(kUINT16, &Rate, kREQUIRED);
  if (error_byte) 
  { 
    return;
  }

  // Handle loading things up for RB0
  gPulseLen[0] = Length;
  gPulseRate[0] = Rate;

  // And now loop for the other 3
  for (i = 0; i < 3; i++)
  {
    RetVal1 = extract_number(kUINT16, &Length, kOPTIONAL);
    RetVal2 = extract_number(kUINT16, &Rate, kOPTIONAL);
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

  print_ack();
}

// PG Pulse Go Command
// Starts a set of pulses (as configured by the PC command)
// going. If a new set of parameters were sent by the PC command,
// PG will cause them all to take effect at the next 1ms
// interval.
//
// Usage:
// PG,1<CR>   Start pulses, or load latest set of paramters and use them
// PG,0<CR>   Stop pulses
void parsePGCommand(void)
{
  unsigned char Value;

  extract_number(kUINT8, &Value, kREQUIRED);

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

  print_ack();
}

void LongDelay(void)
{
  unsigned char i;
  //A basic for() loop decrementing a 16 bit number would be simpler, but seems to take more code space for
  //a given delay.  So do this instead:
  for(i = 0; i < 0xFF; i++)
  {
    WREG = 0xFF;
    while(WREG)
    {
      WREG--;
      _asm
      bra 0 //Equivalent to bra $+2, which takes half as much code as 2 nop instructions
      bra 0 //Equivalent to bra $+2, which takes half as much code as 2 nop instructions
      bra 0 //Equivalent to bra $+2, which takes half as much code as 2 nop instructions
      _endasm
    }
  }
  //Delay is ~59.8ms at 48MHz.
}

// BL command : simply jump to the bootloader
// Example: "BL<CR>"
void parseBLCommand()
{
  // First, kill interrupts though
  INTCONbits.GIEH = 0;  // Turn high priority interrupts on
  INTCONbits.GIEL = 0;  // Turn low priority interrupts on

  UCONbits.SUSPND = 0;  //Disable USB module
  UCON = 0x00;          //Disable USB module
  //And wait awhile for the USB cable capacitance to discharge down to disconnected (SE0) state. 
  //Otherwise host might not realize we disconnected/reconnected when we do the reset.
  LongDelay();
  _asm goto 0x00001E _endasm
}

// RB ReBoot command : simply jump to the reset vector
// Example: "RB<CR>"
void parseRBCommand()
{
  // First, kill interrupts though
  INTCONbits.GIEH = 0;  // Turn high priority interrupts on
  INTCONbits.GIEL = 0;  // Turn low priority interrupts on

  UCONbits.SUSPND = 0;  //Disable USB module
  UCON = 0x00;          //Disable USB module
  //And wait awhile for the USB cable capacitance to discharge down to disconnected (SE0) state. 
  //Otherwise host might not realize we disconnected/reconnected when we do the reset.
  LongDelay();
  Reset();
}


/*
 * T1 - For testing that parameter input routines work properly
 * 
 * This function (along with T2) reads in one parameter for every type of input 
 * that the extract_number() function can read in. Use it to test that all 
 * values are properly parsed and returned.
 * 
 * Sample test lines:
 * T1,1,1,1,1,1,1,1,1
 * T1,0,0,0,0,0,0,0,0
 * T1,127,255,FF,32767,65535,FFFF,G,g
 * T1,-127,255,FF,-32767,65535,FFFF,!,!
 * 
 */
/// TODO: To save some flash space, maybe not include this function in production builds?
void parseT1Command()
{
  UINT8  UInt8;
  INT8   SInt8;
  UINT8  HInt8;
  UINT16 UInt16;
  INT16  SInt16;
  UINT16 HInt16;
  UINT8  UChar;
  UINT8  UCaseChar;

  extract_number(kINT8, &SInt8, kREQUIRED);
  extract_number(kUINT8, &UInt8, kREQUIRED);
  extract_number(kHEX8, &HInt8, kREQUIRED);
  extract_number(kINT16, &SInt16, kREQUIRED);
  extract_number(kUINT16, &UInt16, kREQUIRED);
  extract_number(kHEX16, &HInt16, kREQUIRED);
  extract_number(kASCII_CHAR, &UChar, kREQUIRED);
  extract_number(kUCASE_ASCII_CHAR, &UCaseChar, kREQUIRED);

  printf ((rom char far *)"kINT8   =%d\n", SInt8);
  printf ((rom char far *)"kUINT8  =%u\n", UInt8);
  printf ((rom char far *)"kHEX8   =%X\n", HInt8);
  printf ((rom char far *)"kINT16  =%d\n", SInt16);
  printf ((rom char far *)"kUINT16 =%u\n", UInt16);
  printf ((rom char far *)"kHEX16  =%X\n", HInt16);
  printf ((rom char far *)"kASCII_CHAR=%c\n", UChar);
  printf ((rom char far *)"kUCASE_ASCII_CHAR=%c\n", UCaseChar);

  print_ack();
}

/*
 * T2 - For testing that parameter input routines work properly
 * 
 * This function (along with T1) reads in one parameter for every type of input 
 * that the extract_number() function can read in. Use it to test that all 
 * values are properly parsed and returned.
 * 
 * Sample test lines:
 * T2,1,1,1,1,1
 * T2,0,0,0,0,0
 * T2,16777215,FFFFFF,2147483647,4294967295,FFFFFFFF
 * T2,16777215,FFFFFF,-2147483647,4294967295,FFFFFFFF
 * 
 */
/// TODO: To save some flash space, maybe not include this function in production builds?
void parseT2Command()
{
  UINT24 UInt24;
  UINT24 HInt24;
  UINT32 UInt32;
  INT32  SInt32;
  UINT32 HInt32;

  extract_number(kUINT24, &UInt24, kREQUIRED);
  extract_number(kHEX24, &HInt24, kREQUIRED);  
  extract_number(kINT32, &SInt32, kREQUIRED);
  extract_number(kUINT32, &UInt32, kREQUIRED);
  extract_number(kHEX32, &HInt32, kREQUIRED);

  printf ((rom char far *)"kUINT24 =%Hu\n", UInt24);
  printf ((rom char far *)"kHEX24  =%HX\n", HInt24);
  printf ((rom char far *)"kINT32  =%ld\n", SInt32);
  printf ((rom char far *)"kUINT32 =%lu\n", UInt32);
  printf ((rom char far *)"kHEX32  =%lXk\n", HInt32);

  print_ack();
}
#endif
