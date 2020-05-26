/* 
 * File:   parse.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 5:38 PM
 */

#ifndef PARSE_H
#define	PARSE_H

#define advance_RX_buf_out()              \
{                                         \
  g_RX_buf_out++;                         \
  if (kRX_BUF_SIZE == g_RX_buf_out)       \
  {                                       \
    g_RX_buf_out = 0;                     \
  }                                       \
}

// defines for the error_byte byte - each bit has a meaning
#define kERROR_BYTE_STEPS_TO_FAST           1 // If you ask us to step more than 25 steps/ms
#define kERROR_BYTE_TX_BUF_OVERRUN          2
#define kERROR_BYTE_RX_BUFFER_OVERRUN       3
#define kERROR_BYTE_MISSING_PARAMETER       4
#define kERROR_BYTE_PRINTED_ERROR           5 // We've already printed out an error
#define kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT 6
#define kERROR_BYTE_EXTRA_CHARACTERS        7
#define kERROR_BYTE_UNKNOWN_COMMAND         8 // Part of command parser, not error handler

#define kREQUIRED FALSE
#define kOPTIONAL TRUE

// Enum for extract_num() function parameter
typedef enum {
  kCHAR          // One byte, signed
 ,kUCHAR         // One byte, unsigned
 ,kINT           // Two bytes, signed
 ,kUINT          // Two bytes, unsigned
 ,kASCII_CHAR    // ASCII character, read in as byte
 ,kUCASE_ASCII_CHAR  // ASCII character, must be uppercase
 ,kLONG          // Four bytes, signed
 ,kULONG         // Four bytes, unsigned
} ExtractType;

typedef enum {
  kEXTRACT_OK = 0
 ,kEXTRACT_PARAMETER_OUTSIDE_LIMIT
 ,kEXTRACT_COMMA_MISSING
 ,kEXTRACT_MISSING_PARAMETER
 ,kEXTRACT_INVALID_TYPE
} ExtractReturnType;

extern unsigned char error_byte;
extern BOOL g_ack_enable;


ExtractReturnType extract_number (ExtractType Type, void * ReturnValue, unsigned char Required);
UINT8 extract_string (unsigned char * ReturnValue, UINT8 MaxBytes);
void print_ack (void);

#endif	/* PARSE_H */

