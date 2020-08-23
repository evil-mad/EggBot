#include <p18cxxx.h>
#include "parse.h"
#include "utility.h"
#include "fifo.h"
#include "main.h"
#include <stdio.h>
#include "servo.h"
#include "analog.h"
#include "utility.h"
#include "ebb.h"
#include "init.h"

/** D E F I N E S ********************************************************/


/** V A R I A B L E S ********************************************************/

  
// Set to TRUE to turn Pulse Mode on
unsigned char gPulsesOn = FALSE;
// For Pulse Mode, how long should each pulse be on for in ms?
unsigned int gPulseLen[4] = {0,0,0,0};
// For Pulse Mode, how many ms between rising edges of pulses?
unsigned int gPulseRate[4] = {0,0,0,0};
// For Pulse Mode, counters keeping track of where we are
unsigned int gPulseCounters[4] = {0,0,0,0};

/** P R I V A T E  P R O T O T Y P E S ***************************************/



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
      printf ((far rom char*)"%i,%i\r\n", FIFOSize, COMMAND_FIFO_LENGTH);
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
    (far rom char*)"I,%03i,%03i,%03i,%03i,%03i\r\n", 
    PORTA,
    PORTB,
    PORTC,
    PORTD,
    PORTE
  );
}

// All we do here is just print out our version number
void parseVRCommand(void)
{
  printf ((far rom char *)st_version);
}

// MW is for Memory Write
// "MW,<location>,<value><CR>"
// <location> is a decimal value between 0 and 4096 indicating the RAM address to write to 
// <value> is a decimal value between 0 and 255 that is the value to write
void parseMWCommand(void)
{
  unsigned int location;
  unsigned char value;

  extract_number (kUINT16, &location, kREQUIRED);
  extract_number (kUINT8, &value, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }
  // Limit check the address and write the byte in
  if (location < 4096)
  {
    *((unsigned char *)location) = value;
  }

  print_ack ();
}


// MR is for Memory Read
// "MW,<location><CR>"
// <location> is a decimal value between 0 and 4096 indicating the RAM address to read from 
// The UBW will then send a "MR,<value><CR>" packet back to the PC
// where <value> is the byte value read from the address
void parseMRCommand(void)
{
  unsigned int location;
  unsigned char value;

  extract_number (kUINT16, &location, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  // Limit check the address and write the byte in
  if (location < 4096)
  {
    value = *((unsigned char *)location);
  }

  // Now send back the MR packet
  printf (
    (far rom char *)"MR,%03u\r\n" 
    ,value
  );
}

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
     (far rom char *)"PI,%1u\r\n" 
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

#if defined(BOARD_EBB)
// QR Query RC Servo power state command
// Example: "RR<CR>"
// Returns "0<CR><LF>OK<CR><LF>" or "1<CR><LF>OK<CR><LF>" 
// 0 = power to RC servo off
// 1 = power to RC servo on
void parseQRCommand()
{
  printf ((far rom char *)"%1u\r\n", RCServoPowerIO_PORT);
  print_ack();
}
#endif

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
#if defined(BOARD_EBB)
void parseSRCommand(void)
{
  unsigned long Value;
  UINT8 State;
  ExtractReturnType GotState;

  extract_number(kUINT32, &Value, kREQUIRED);
  GotState = extract_number(kUINT8, &State, kOPTIONAL);

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

  print_ack();
}
#endif

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

  printf ((rom char far *)"kINT8   =%d\r\n", SInt8);
  printf ((rom char far *)"kUINT8  =%u\r\n", UInt8);
  printf ((rom char far *)"kHEX8   =%X\r\n", HInt8);
  printf ((rom char far *)"kINT16  =%d\r\n", SInt16);
  printf ((rom char far *)"kUINT16 =%u\r\n", UInt16);
  printf ((rom char far *)"kHEX16  =%X\r\n", HInt16);
  printf ((rom char far *)"kASCII_CHAR=%c\r\n", UChar);
  printf ((rom char far *)"kUCASE_ASCII_CHAR=%c\r\n", UCaseChar);

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

  printf ((rom char far *)"kUINT24 =%Hu\r\n", UInt24);
  printf ((rom char far *)"kHEX24  =%HX\r\n", HInt24);
  printf ((rom char far *)"kINT32  =%ld\r\n", SInt32);
  printf ((rom char far *)"kUINT32 =%lu\r\n", UInt32);
  printf ((rom char far *)"kHEX32  =%lX\r\n", HInt32);

  print_ack();
}
