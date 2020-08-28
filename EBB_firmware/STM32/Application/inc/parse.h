/* 
 * File:   parse.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 5:38 PM
 */

#ifndef PARSE_H
#define	PARSE_H

#include <stdbool.h>

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
// NOTE: C18 supports UINT24 type, but not INT24, so it does not appear here
typedef enum {
  kINT8,           // One byte, signed
  kUINT8,          // One byte, unsigned
  kHEX8,           // One byte (two characters) of hex
  kINT16,          // Two bytes, signed
  kUINT16,         // Two bytes, unsigned
  kHEX16,          // Two bytes (four characters) of hex
  kUINT24,         // Three bytes, unsigned
  kHEX24,          // Three bytes (six characters) of hex
  kINT32,          // Four bytes, signed
  kUINT32,         // Four bytes, unsigned
  kHEX32,          // Four bytes (eight characters) of hex
  kASCII_CHAR,     // ASCII character, read in as byte
  kUCASE_ASCII_CHAR  // ASCII character, must be uppercase
} ExtractType;

typedef enum {
  kEXTRACT_OK = 0
 ,kEXTRACT_PARAMETER_OUTSIDE_LIMIT
 ,kEXTRACT_COMMA_MISSING
 ,kEXTRACT_MISSING_PARAMETER
 ,kEXTRACT_INVALID_TYPE
} ExtractReturnType;

extern uint8_t error_byte;
extern bool g_ack_enable;


ExtractReturnType extract_number(ExtractType Type, void * ReturnValue, unsigned char Required);
uint8_t extract_string(unsigned char * ReturnValue, uint8_t MaxBytes);
void print_ack(void);

#endif	/* PARSE_H */

