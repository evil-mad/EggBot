/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        user.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 2.30.01+
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       11/19/04    Original.
 ********************************************************************/

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
#define bittst(var,bitno) (var & ((1 << (bitno + 1)) >> 1))


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
	 kCHAR
	,kUCHAR
	,kINT
	,kUINT
	,kASCII_CHAR
	,kUCASE_ASCII_CHAR
	,kLONG
	,kULONG
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
extern near unsigned char error_byte;
extern volatile unsigned int g_RC_value[kRC_DATA_SIZE];			// Stores reload values for TMR0
extern volatile tRC_state g_RC_state[kRC_DATA_SIZE];

/** P U B L I C  P R O T O T Y P E S *****************************************/
void UserInit(void);
void ProcessIO(void);
void low_ISR(void);
void high_ISR(void);
ExtractReturnType extract_number(ExtractType Type, void * ReturnValue, unsigned char Required);
void print_ack (void);
void SetPinTRISFromRPn(char Pin, char State);
void SetPinLATFromRPn(char Pin, char State);


#endif //UBW_H
