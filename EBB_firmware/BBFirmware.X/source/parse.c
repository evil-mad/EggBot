
#include <p18cxxx.h>
#include <GenericTypeDefs.h>
#include "parse.h"
#include "usbser.h"
#include "utility.h"
#include <ctype.h>
#include "stepper.h"
#include <stdio.h>
#include "commands.h"
#include "servo.h"
#include "ebb.h"
#include "analog.h"
#include "serial.h"

// This byte has each of its bits used as a separate error flag
unsigned char error_byte;

// Normally set to TRUE. Able to set FALSE to not send "OK" message after packet reception
BOOL g_ack_enable;


static INT8 extractHexDigits(UINT32 * acc, UINT8 digits);
static INT8 extractDecDigits(UINT32 * acc, UINT8 digits);

// Look at the new packet, see what command it is, and 
// route it appropriately. We come in knowing that
// our packet is in g_RX_buf[], and that the beginning
// of the packet is at g_RX_buf_out, and the end (CR) is at
// g_RX_buf_in. Note that because of buffer wrapping,
// g_RX_buf_in may be less than g_RX_buf_out.
void parse_packet(void)
{
  unsigned int command = 0;
  unsigned char cmd1 = 0;
  unsigned char cmd2 = 0;

  // Always grab the first character (which is the first byte of the command)
  cmd1 = toupper(g_RX_buf[g_RX_buf_out]);
  advance_RX_buf_out();
  command = cmd1;

  // Only grab second one if it is not a comma
  if (g_RX_buf[g_RX_buf_out] != ',' && g_RX_buf[g_RX_buf_out] != kCR)
  {
    cmd2 = toupper (g_RX_buf[g_RX_buf_out]);
    advance_RX_buf_out();
    command = ((unsigned int)(cmd1) << 8) + cmd2;
  }

  // Now 'command' is equal to one or two bytes of our command
  switch (command)
  {
    case ('L' * 256) + 'M':
    {
      // Low Level Move
      parse_LM_packet();
      break;
    }
    case 'R':
    {
      // Reset command (resets everything to power-on state)
      parse_R_packet ();
      break;
    }
    case 'C':
    {
      // Configure command (configure ports for Input or Output)
      parse_C_packet ();
      break;
    }
    case ('C' * 256) + 'U':
    {
      // For configuring UBW
      parse_CU_packet ();
      break;
    }
    case 'O':
    {
      // Output command (tell the ports to output something)
      parse_O_packet ();
      break;
    }
    case 'I':
    {
      // Input command (return the current status of the ports)
      parse_I_packet ();
      break;
    }
    case 'V':
    {
      // Version command
      parse_V_packet ();
      break;
    }
    case ('A' * 256) + 'R':
    {
      // Analog Read
      parseARPacket();
      break;
    }
    case ('P' * 256) + 'I':
    {
      // PI for reading a single pin
      parse_PI_packet ();
      break;
    }
    case ('P' * 256) + 'O':
    {
      // PO for setting a single pin
      parse_PO_packet ();
      break;
    }

    case ('P' * 256) + 'D':
    {
      // PD for setting a pin's direction
      parse_PD_packet ();
      break;
    }
    case ('M' * 256) + 'R':
    {
      // MR for Memory Read
      parse_MR_packet ();
      break;
    }
    case ('M' * 256) + 'W':
    {
      // MW for Memory Write
      parse_MW_packet ();
      break;
    }
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
    case ('S' * 256) + 'M':
    {
      // SM for stepper motor
      parse_SM_packet ();
      break;
    }
    case ('A' * 256) + 'M':
    {
      // AM for Accelerated Motion
      parse_AM_packet ();
      break;
    }
    case ('S' * 256) + 'P':
    {
      // SP for set pen
      parse_SP_packet ();
      break;
    }
    case ('T' * 256) + 'P':
    {
      // TP for toggle pen
      parse_TP_packet ();
      break;
    }
    case ('Q' * 256) + 'P':
    {
      // QP for query pen
      parse_QP_packet ();
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
    case ('T' * 256) + '1':
    {
      // T1 for test
      parseT1Packet();
      break;
    }
    case ('T' * 256) + '2':
    {
      // T2 for test
      parseT2Packet();
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
      servo_S2_command();
      break;
    }
    case ('R' * 256) + 'M':
    {
      // RM for Run Motor
      parse_RM_packet();
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
      parseACPacket();
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
#if defined(BOARD_EBB)
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
#endif
    case ('H' * 256) + 'M':
    {
      // HM is for Home Motor
      parse_HM_packet();
      break;
    }
    case ('D' * 256) + 'R':
    {
      // HM is for Read Serial
      parseDRPacket();
      break;
    }
    case ('D' * 256) + 'W':
    {
      // WS is for Write Serial
      parseDWPacket();
      break;
    }
    
    
    default:
    {
      if (0 == cmd2)
      {
        // Send back 'unknown command' error
        printf (
           (far rom char *)"!8 Err: Unknown command '%c:%2X'\r\n"
          ,cmd1
          ,cmd1
        );
      }
      else
      {
        // Send back 'unknown command' error
        printf (
           (far rom char *)"!8 Err: Unknown command '%c%c:%2X%2X'\r\n"
          ,cmd1
          ,cmd2
          ,cmd1
          ,cmd2
        );
      }
      break;
    }
  }

  // Double check that our output pointer is now at the ending <CR>
  // If it is not, this indicates that there were extra characters that
  // the command parsing routine didn't eat. This would be an error and needs
  // to be reported. (Ignore for Reset command because FIFO pointers get cleared.)
  if (
    (g_RX_buf[g_RX_buf_out] != kCR && 0 == error_byte)
    &&
    ('R' != command)
  )
  {
    bitset (error_byte, kERROR_BYTE_EXTRA_CHARACTERS);
  }

  // Clean up by skipping over any bytes we haven't eaten
  // This is safe since we parse each packet as we get a <CR>
  // (i.e. g_RX_buf_in doesn't move while we are in this routine)
  g_RX_buf_out = g_RX_buf_in;
}

// Print out the positive acknowledgment that the packet was received
// if we have acks turned on.
void print_ack(void)
{
  if (g_ack_enable)
  {
    printf ((far rom char *)st_OK);
  }
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
    bitset (error_byte, kERROR_BYTE_MISSING_PARAMETER);
    return (0);
  }

  // Check for comma where ptr points
  if (g_RX_buf[g_RX_buf_out] != ',')
  {
    printf ((rom char far *)"!5 Err: Need comma next, found: '%c'\r\n", g_RX_buf[g_RX_buf_out]);
    bitset (error_byte, kERROR_BYTE_PRINTED_ERROR);
    return (0);
  }

  // Move to the next character
  advance_RX_buf_out();

  while(1)
  {
    // Check to see if we're already at the end
    if (kCR == g_RX_buf[g_RX_buf_out] || ',' == g_RX_buf[g_RX_buf_out] || bytes >= MaxBytes)
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


// Look at the string pointed to by ptr
// There should be a comma where ptr points to upon entry.
// If not, throw a comma error.
// If so, then look for up to like a ton of bytes after the
// comma for numbers, and put them all into one
// unsigned long accumulator. 
// Advance the pointer to the byte after the last number
// and return.
ExtractReturnType extract_number(
  ExtractType Type, 
  void * ReturnValue, 
  UINT8 Required
)
{
  UINT32 ULAccumulator;
  INT32 Accumulator;
  BOOL Negative = FALSE;

  // Check to see if we're already at the end
  if (kCR == g_RX_buf[g_RX_buf_out])
  {
    if (0 == Required)
    {
      bitset (error_byte, kERROR_BYTE_MISSING_PARAMETER);
    }
    return (kEXTRACT_MISSING_PARAMETER);
  }

  // Check for comma where ptr points
  if (g_RX_buf[g_RX_buf_out] != ',')
  {
    if (0 == Required)
    {
      printf ((rom char far *)"!5 Err: Need comma next, found: '%c'\r\n", g_RX_buf[g_RX_buf_out]);
      bitset (error_byte, kERROR_BYTE_PRINTED_ERROR);
    }
    return (kEXTRACT_COMMA_MISSING);
  }

  // Move to the next character
  advance_RX_buf_out();

  // Check for end of command
  if (kCR == g_RX_buf[g_RX_buf_out])
  {
    if (0 == Required)
    {
      bitset (error_byte, kERROR_BYTE_MISSING_PARAMETER);
    }
    return (kEXTRACT_MISSING_PARAMETER);
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
      (kUINT8 == Type)
      ||
      (kUINT16 == Type)
      ||
      (kUINT32 == Type)
      ||
      (kHEX32 == Type)
    )
    {
      bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
      return (kEXTRACT_PARAMETER_OUTSIDE_LIMIT);
    }
    else
    {
      Negative = TRUE;
      // Move to the next character
      advance_RX_buf_out();
    }
  }

  // Read in the digits of the number
  switch(Type)
  {
    case kINT8:
    case kUINT8:
      extractDecDigits(&ULAccumulator, 3);
      break;
      
    case kINT16:
    case kUINT16:
      extractDecDigits(&ULAccumulator, 5);
      break;
    
    case kUINT24:
      extractDecDigits(&ULAccumulator, 8);
      break;

    case kINT32:
    case kUINT32:
      extractDecDigits(&ULAccumulator, 10);
      break;
    
    case kHEX8:
      extractHexDigits(&ULAccumulator, 2);
      break;
      
    case kHEX16:
      extractHexDigits(&ULAccumulator, 4);
      break;

    case kHEX24:
      extractHexDigits(&ULAccumulator, 6);
      break;

    case kHEX32:
      extractHexDigits(&ULAccumulator, 8);
      break;

    case kASCII_CHAR:
    case kUCASE_ASCII_CHAR:
      // just copy the byte
      ULAccumulator = g_RX_buf[g_RX_buf_out];

      // Force uppercase if that's what type we have
      if (kUCASE_ASCII_CHAR == Type)
      {
        ULAccumulator = toupper(ULAccumulator);
      }

      // Move to the next character
      advance_RX_buf_out();
      break;
   
    default:
      break;
  }

  // Range check absolute values
  if (Negative)
  {
    if (
      (
        kINT8 == Type
        &&
        (ULAccumulator > (UINT32)0x80)
      )
      ||
      (
        kINT16 == Type
        &&
        (ULAccumulator > (UINT32)0x8000)
      )
      ||
      (
        kINT32 == Type
        &&
        (ULAccumulator > (UINT32)0x80000000L)
      )
    )
    {
      bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
      return (kEXTRACT_PARAMETER_OUTSIDE_LIMIT);
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
        kINT8 == Type
        &&
        (ULAccumulator > (UINT32)0x7F)
      )
      ||
      (
        kUINT8 == Type
        &&
        (ULAccumulator > (UINT32)0xFF)
      )
      ||
      (
        kINT16 == Type
        &&
        (ULAccumulator > (UINT32)0x7FFF)
      )
      ||
      (
        kUINT16 == Type
        &&
        (ULAccumulator > (UINT32)0xFFFF)
      )
      ||
      (
        kUINT24 == Type
        &&
        (ULAccumulator > (UINT32)0xFFFFFFL)
      )
      ||
      (
        kINT32 == Type
        &&
        (ULAccumulator > (UINT32)0x7FFFFFFFL)
      )
    )
    {
      bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
      return (kEXTRACT_PARAMETER_OUTSIDE_LIMIT);
    }

    if (kUINT32 != Type)
    {
      Accumulator = ULAccumulator;
    }
  }

  // If all went well, then copy the result
  switch (Type)
  {
    case kINT8:
      *(INT8 *)ReturnValue = (INT8)Accumulator;
      break;
    case kUINT8:
    case kHEX8:
    case kASCII_CHAR:
    case kUCASE_ASCII_CHAR:
      *(UINT8 *)ReturnValue = (UINT8)Accumulator;
      break;
    case kINT16:
      *(INT16 *)ReturnValue = (INT16)Accumulator;
      break;
    case kUINT16:
    case kHEX16:
      *(UINT16 *)ReturnValue = (UINT16)Accumulator;
      break;
    case kUINT24:
    case kHEX24:
      *(UINT24 *)ReturnValue = (UINT24)Accumulator;
      break;
    case kINT32:
      *(INT32 *)ReturnValue = Accumulator;
      break;
    case kUINT32:
    case kHEX32:
      *(UINT32 *)ReturnValue = ULAccumulator;
      break;
    default:
      return (kEXTRACT_INVALID_TYPE);
  }
  return(kEXTRACT_OK);
}

// Loop 'digits' number of times, looking at the
// byte in input_buffer index *ptr, and if it is
// a digit, adding it to acc. Take care of 
// powers of ten as well. If you hit a non-numerical
// char, then return FALSE, otherwise return TRUE.
// Store result as you go in *acc.
static INT8 extractDecDigits(UINT32 * acc, UINT8 digits)
{
  UINT8 val;
  UINT32 result = 0;
  BOOL returnVal = TRUE;

  while(digits)
  {
    val = g_RX_buf[g_RX_buf_out];
    if ((val >= 48) && (val <= 57))
    {
      result = (result * 10) + (val - 48);
      // Move to the next character
      advance_RX_buf_out();
    }
    else
    {
      returnVal = FALSE;
      break;
    }
    digits--;
  }
  *acc = result;
  return(returnVal);
}

// Loop 'digits' number of times, looking at the
// byte in input_buffer index *ptr, and if it is
// a hex digit, adding it to acc. If you hit a non-hex
// character, then return FALSE, otherwise return TRUE.
// Store result as you go in *acc.
static INT8 extractHexDigits(UINT32 * acc, UINT8 digits)
{
  UINT8 val;
  UINT32 result = 0;
  BOOL returnVal = TRUE;
  
  while(digits)
  {
    val = g_RX_buf[g_RX_buf_out];
    // Convert from lowercase 'a-f' to uppercase 'A-F'
    if ((val >= 97) && (val <= 102))
    {
      val -= 32;
    }
    // Is it a number from 0-9?
    if ((val >= '0') && (val <= '9'))
    {
      result = (result << 4) | (val - '0');
      // Move to the next character
      advance_RX_buf_out();
    }
    else if ((val >= 'A') && (val <= 'F'))
    {
      result = (result << 4) | (val - 55);
      // Move to the next character
      advance_RX_buf_out();
    }
    else
    {
      returnVal = FALSE;
      break;
    }
    digits--;
  }
  *acc = result;
  return(returnVal);
}


