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
void parse_R_packet(void)
{
  UserInit();
  print_ack();
}

// CU is "Configure UBW" and controls system-wide configuration values
// "CU,<parameter_number>,<paramter_value><CR>"
// <paramter_number> <parameter_value>
// 1                  {1|0} turns on or off the 'ack' ("OK" at end of packets)
void parse_CU_packet(void)
{
  unsigned char parameter_number = 0;
  signed int parameter_value = 0;

  extract_number (kUCHAR, &parameter_number, kREQUIRED);
  extract_number (kINT, &parameter_value, kOPTIONAL);

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
void parse_C_packet(void)
{
  unsigned char PA, PB, PC, PD, PE;

  // Extract each of the four values.
  extract_number (kUCHAR, &PA, kREQUIRED);
  extract_number (kUCHAR, &PB, kREQUIRED);
  extract_number (kUCHAR, &PC, kREQUIRED);
  extract_number (kUCHAR, &PD, kREQUIRED);
  extract_number (kUCHAR, &PE, kREQUIRED);

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
void parse_O_packet(void)
{
  unsigned char Value;
  ExtractReturnType RetVal;

  // Extract each of the values.
  RetVal = extract_number (kUCHAR,  &Value, kREQUIRED);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATA = Value;
  }
  RetVal = extract_number (kUCHAR,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATB = Value;
  }
  RetVal = extract_number (kUCHAR,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATC = Value;
  }
  RetVal = extract_number (kUCHAR,  &Value, kOPTIONAL);
  if (error_byte) return;
  if (kEXTRACT_OK == RetVal)
  {
    LATD = Value;
  }
  RetVal = extract_number (kUCHAR,  &Value, kOPTIONAL);
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
void parse_I_packet(void)
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
void parse_V_packet(void)
{
  printf ((far rom char *)st_version);
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

  // Put the beginning of the packet in place
  printf ((far rom char *)"A");

  // Sit and spin, waiting for one set of analog conversions to complete
  while (PIE1bits.ADIE);

  // Now print each analog value
  for (channel = 0; channel < 16; channel++)
  {
    if (ChannelBit & AnalogEnabledChannels)
    {
      printf(
        (far rom char *)",%02u:%04u"
        ,channel
        ,ISR_A_FIFO[channel]
      );
    }
    ChannelBit = ChannelBit << 1;
  }

  print_ack ();
}

// MW is for Memory Write
// "MW,<location>,<value><CR>"
// <location> is a decimal value between 0 and 4096 indicating the RAM address to write to 
// <value> is a decimal value between 0 and 255 that is the value to write
void parse_MW_packet(void)
{
  unsigned int location;
  unsigned char value;

  extract_number (kUINT, &location, kREQUIRED);
  extract_number (kUCHAR, &value, kREQUIRED);

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
void parse_MR_packet(void)
{
  unsigned int location;
  unsigned char value;

  extract_number (kUINT, &location, kREQUIRED);

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
void parse_PD_packet(void)
{
  unsigned char port;
  unsigned char pin;
  unsigned char direction;

  extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number (kUCHAR, &pin, kREQUIRED);
  extract_number (kUCHAR, &direction, kREQUIRED);

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
void parse_PI_packet(void)
{
  unsigned char port;
  unsigned char pin;
  unsigned char value = 0;

  extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number (kUCHAR, &pin, kREQUIRED);

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
void parse_PO_packet(void)
{
  unsigned char port;
  unsigned char pin;
  unsigned char value;

  extract_number (kUCASE_ASCII_CHAR, &port, kREQUIRED);
  extract_number (kUCHAR, &pin, kREQUIRED);
  extract_number (kUCHAR, &value, kREQUIRED);

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
void parse_PC_packet (void)
{
  unsigned int Length, Rate;
  unsigned char i;
  ExtractReturnType RetVal1, RetVal2;

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
  for (i = 0; i < 3; i++)
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
void parse_PG_packet (void)
{
  unsigned char Value;

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
void parse_BL_packet()
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
void parse_RB_packet()
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

// QR Query RC Servo power state command
// Example: "RR<CR>"
// Returns "0<CR><LF>OK<CR><LF>" or "1<CR><LF>OK<CR><LF>" 
// 0 = power to RC servo off
// 1 = power to RC servo on
void parse_QR_packet()
{
  printf ((far rom char *)"%1u\r\n", RCServoPowerIO_PORT);
  print_ack();
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

  print_ack();
}

// Just used for testing/debugging the packet parsing routines
void parse_CK_packet()
{
  unsigned char UByte;
  signed char SByte;
  unsigned int UInt;
  signed int SInt;
  unsigned long ULong;
  signed long SLong;
  unsigned char UChar;
  unsigned char UCaseChar;

  extract_number(kCHAR, &SByte, kREQUIRED);
  extract_number(kUCHAR, &UByte, kREQUIRED);
  extract_number(kINT, &SInt, kREQUIRED);
  extract_number(kUINT, &UInt, kREQUIRED);
  extract_number(kLONG, &SLong, kREQUIRED);
  extract_number(kULONG, &ULong, kREQUIRED);
  extract_number(kASCII_CHAR, &UChar, kREQUIRED);
  extract_number(kUCASE_ASCII_CHAR, &UCaseChar, kREQUIRED);

  printf ((rom char far *)"Param1=%d\r\n", SByte);
  printf ((rom char far *)"Param2=%d\r\n", UByte);
  printf ((rom char far *)"Param3=%d\r\n", SInt);
  printf ((rom char far *)"Param4=%u\r\n", UInt);
  printf ((rom char far *)"Param5=%ld\r\n", SLong);
  printf ((rom char far *)"Param6=%lu\r\n", ULong);
  printf ((rom char far *)"Param7=%c\r\n", UChar);
  printf ((rom char far *)"Param8=%c\r\n", UCaseChar);

  print_ack();
}



