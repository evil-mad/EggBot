/*********************************************************************
 *
 *                UBW Firmware
 *
 *********************************************************************
 * FileName:        UBW.h
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Based on original files by Microchip Inc. in MAL USB example.
 *
 * Software License Agreement
 *
 * Copyright (c) 2014, Brian Schmalz of Schmalz Haus LLC
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

#ifndef UBW_H
#define UBW_H
#include "GenericTypeDefs.h"
#include "Compiler.h"

#define kTX_BUF_SIZE 			64				// In bytes
#define kRX_BUF_SIZE			256				// In bytes
#define kRX_COMMAND_BUF_SIZE	64				// In bytes

#define kREQUIRED	FALSE
#define kOPTIONAL	TRUE

#define INPUT_PIN	1
#define OUTPUT_PIN  0

#define bitset(var,bitno) ((var) |= (1 << (bitno)))
#define bitclr(var,bitno) ((var) &= ~(1 << (bitno)))
#define bittst(var,bitno) (var & (1 << bitno))


// defines for the error_byte byte - each bit has a meaning
#define kERROR_BYTE_STEPS_TO_FAST			1			// If you ask us to step more than 25 steps/ms
#define kERROR_BYTE_TX_BUF_OVERRUN			2
#define kERROR_BYTE_RX_BUFFER_OVERRUN		3
#define kERROR_BYTE_MISSING_PARAMETER		4
#define kERROR_BYTE_PRINTED_ERROR			5			// We've already printed out an error
#define kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT	6
#define kERROR_BYTE_EXTRA_CHARACTERS 		7
#define kERROR_BYTE_UNKNOWN_COMMAND			8			// Part of command parser, not error handler

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

#define advance_RX_buf_out()						\
{ 													\
	g_RX_buf_out++;									\
	if (kRX_BUF_SIZE == g_RX_buf_out)				\
	{												\
		g_RX_buf_out = 0;							\
	}												\
}

// For the RC command, we define a little data structure that holds the 
// values assoicated with a particular servo connection
// It's port, pin, value (position) and state (INACTIVE, PRIMED or TIMING)
// Later on we make an array of these (19 elements long - 19 pins) to track
// the values of all of the servos.
typedef enum {
	 kOFF = 1
	,kWAITING
	,kPRIMED
	,kTIMING
} tRC_state;

#define kRC_DATA_SIZE			24				// In structs, since there are 3 ports of 8 bits each

extern unsigned char g_RX_buf[kRX_BUF_SIZE];
extern unsigned char g_TX_buf_out;
extern volatile unsigned int ISR_A_FIFO[16];                       // Stores the most recent analog conversions

extern unsigned char error_byte;
//extern unsigned char error_byte;
extern BOOL	g_ack_enable;

extern volatile unsigned int g_RC_value[kRC_DATA_SIZE];			// Stores reload values for TMR0
extern volatile tRC_state g_RC_state[kRC_DATA_SIZE];

extern volatile unsigned long int gRCServoPoweroffCounterMS;
extern volatile unsigned long int gRCServoPoweroffCounterReloadMS;

/** P U B L I C  P R O T O T Y P E S *****************************************/
void UserInit (void);
void ProcessIO (void);
void low_ISR (void);
void high_ISR (void);
ExtractReturnType extract_number (ExtractType Type, void * ReturnValue, unsigned char Required);
UINT8 extract_string (unsigned char * ReturnValue, UINT8 MaxBytes);
void print_ack (void);
void SetPinTRISFromRPn (char Pin, char State);
void SetPinLATFromRPn (char Pin, char State);
void AnalogConfigure (unsigned char Channel, unsigned char Enable);
void populateDeviceStringWithName(void);

#endif //UBW_H
