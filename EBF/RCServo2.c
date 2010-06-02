/*
 * This module implements the RC Servo outputs, method 2
 * 
 * Started by Brian Schmalz (www.schmalzhaus.com) on 8/22/09
 * For the Egg Bot Firmware
 *
 * There are several parts of this file. It is written in a modular
 * way to ease incorporation into different UBW (www.schmalzhaus.com/UBW) hardware builds.
 * Because we don't want to have any function calls in the ISR, the part of the ISR that
 * this module contributes is implemented as a big macro. There is a section for 
 * the API calls (for other modules) to turn on/off RC method 2 and to set values and
 * configure outputs. Then there is the user command section that handles parsing 
 * user input. This module (the .c and .h files) should be able to be dropped into
 * any UBW firmware that is 'module aware' with minimial changes. 
 * There is also an init section that gets called on bootup.
 *
 * MODULE THEORY
 *
 * The idea is that we want to generate between zero and four RC servo outputs. We want
 * a maximum of resolution in their timing, and a minimum of CPU and ISR overhead. We want
 * maximum flexibility with respect to which pins receive the output pulses.
 * This method 2 only works with PICs that have the PPS (Perhipheral Pin Select) hardware
 * Using this method, we will be able to generate servo output pulses (positive going)
 * on up to four output pins (selectable using PPS), with times from 0ms to 3ms, at a
 * repitition rate of 18ms. The resolution of the pulse will be 83ns (Fosc/4). So a
 * value of 0 will represent 0ms pulse, and 36000 will represent 3ms.
 *
 * On UBW firmware, TIMER1 generates an interrupt every 1ms. This interrupt is used
 * to schedule the four RC servo outputs. Every 3 fires of this ISR, we check to see
 * if we need to set the next RC output pin. If so, we select the proper pin using
 * the PPS, set it high, record what state
 * we are now in, and then set up TMR3 and ECCP2 to fire after X 83ns ticks of TMR3.
 * ECCP2 will wait until TMR3 fires, then will set the output pin low. This method
 * allows us to choose any of the PPS pins as outputs, and only requires CPU time every
 * 3 ms. to start off the pulses.
 */

#include <p18cxxx.h>
#include <stdio.h>
#include <ctype.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "ebb.h"
#include "UBW.h"
#include "RCServo2.h"

BOOL gUseRCServo2;
unsigned char gRC2msCounter;
unsigned int gRC2Value[MAX_RC2_SERVOS];
unsigned char gRC2Pin[MAX_RC2_SERVOS];
unsigned int gRC2Target[MAX_RC2_SERVOS];
unsigned int gRC2Rate[MAX_RC2_SERVOS];
unsigned char gRC2Ptr;
unsigned char gRC2Slots;
unsigned char gRC2SlotMS;
far ram unsigned char * gRC2RPORPtr; 
unsigned int g_servo2_max;
unsigned int g_servo2_min;
unsigned int g_servo2_rate;

/*
The idea with RCServo2 is to use the ECCP2 module and timer 3.
We divide time into 21ms periods. Inside each 21ms period, we
can fire up to 7 RC servo's pulses (slots). Each pulse can be between
0ms and 3ms long, controlled entierly by the ECCP2 hardware,
so there is no jitter in the high time of the pulse.

We want to go from 0ms to 3ms so we can accomodate RC servos
who need really short or really long pulses to reach the
physical extremes of its motion.

This RCServo2 method will only be available on the 18F45J50 based
EggBotBoards, because it requires the PPS (perhipheral pin select)
facility to be practical.

Timer3 will be configured to clock at Fosc/4 = 12MHz.
At this rate, a 1ms high pulse would need a CCPR2 value of 12,000.

Variables:

gUseRCServo2 - set TRUE if this service is turned on
gRC2msCounter - counts from 0 to 2, in ms, to know when to fire each servo
gRC2Ptr - index into gRC2Value and gRC2Pin arrays = next RC servo to fire
gRC2Value - uint array of values (times) to load into Timer3
gRC2Pin - what PPS pin is affected by this RC servo slot.
gRC2RPORPtr - a RAM pointer into the RPORxx register

If a slot's gRC2Value[] = 0, then that slot is disabled and its pin will be low.
The value in gRC2Pin is the PPS RPx pin number that this slot controls

/*
 * Module Init Function
 *
 * Put a call to this function inside the UserInit() call in UBW.c
 */
void RCServo2_Init(void)
{
	unsigned char i;

	gUseRCServo2 = FALSE;
	gRC2msCounter = 0;
	gRC2Ptr = 0;

	for (i=0; i < MAX_RC2_SERVOS; i++)
	{
		gRC2Value[i] = 0;
		gRC2Pin[i] = 0;
		gRC2Target[i] = 0;
		gRC2Rate[i] = 0;
	}
	// Initalize the RPOR pointer
	gRC2RPORPtr = &RPOR0;

	// Set up TIMER3
	T3CONbits.TMR3CS = 0b00;  	// Use Fosc/4 as input
	T3CONbits.T3CKPS = 0b00;  	// Prescale is 1:1
	T3CONbits.RD16 = 1;			// Enable 16 bit mode
	TMR3H = 0;
	TMR3L = 0;
	T3CONbits.TMR3ON = 0;		// Keep timer off for now
	
	TCLKCONbits.T3CCP1 = 1;		// EECP1 uses Timer1/2 and EECP2 uses Timer3/4
	TCLKCONbits.T3CCP2 = 0;		// EECP1 uses Timer1/2 and EECP2 uses Timer3/4

	CCP2CONbits.CCP2M = 0b1001;	// Set EECP2 as compare, clear output on match

	// We start out with 7 slots because that is good for RC servos (3ms * 7 = 21ms)
	gRC2Slots = 7;

	// We start out with 3ms slot duration because it's good for RC servos
	gRC2SlotMS = 3;

	g_servo2_min = 12000;
	g_servo2_max = 16000;

	gUseRCServo1 = FALSE;
	TRISBbits.TRISB1 = 0; 	// RB1 needs to be an output
	gUseRCServo2 = TRUE;
	g_servo2_rate = 400;
	Process_S2(1, g_servo2_min, 4, g_servo2_rate);
	process_SP(1, 0);			// Start servo up 

}

// Servo method 2 enable command
// S2,0<CR> will turn off RC Servo method 2 support
// S2,<channel>,<duration>,<output_pin><CR> will set RC output <channel> for <duration> on output pin <output_pin>
//	<channel> can be 0 through 7, with 0 meaning turn off.
//	<duration> can be 0 (output off) to 32,000 (3ms on time)
//	<output_pin> is an RPx pin number (0 through 24)

void RCServo2_S2_command (void)
{
	unsigned char Channel = 0;
	unsigned int Duration = 0;
	unsigned char Pin = 0;
	unsigned char Rate = 0;

	// Extract each of the values.
	extract_number (kUCHAR, &Channel, kREQUIRED);
	extract_number (kUINT, &Duration, kOPTIONAL);
	extract_number (kUCHAR, &Pin, kOPTIONAL);
	extract_number (kUINT, &Rate, kOPTIONAL);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	if (Channel > MAX_RC2_SERVOS)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}

	if (Pin > 24)
	{
		bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
		return;
	}
	
	Process_S2(Channel, Duration, Pin, Rate);

	print_ack();
}

void Process_S2(
	unsigned char Channel, 
	unsigned int Duration, 
	unsigned char Pin,
	unsigned int Rate
)
{
	if (0 == Channel)
	{
		// Turn things off
		gUseRCServo2 = FALSE;
	}
	else
	{
		if (Channel <= MAX_RC2_SERVOS && Pin <= 24)
		{
			gUseRCServo2 = TRUE;
			gRC2Rate[Channel - 1] = Rate;
			gRC2Target[Channel - 1] = Duration;
			gRC2Pin[Channel - 1] = Pin;
			if (gRC2Value[Channel - 1] == 0)
			{
				gRC2Value[Channel - 1] = Duration;
			}
		}
	}
}