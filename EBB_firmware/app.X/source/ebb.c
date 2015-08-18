/*********************************************************************
 *
 *                EiBotBoard Firmware
 *
 *********************************************************************
 * FileName:        ebb.c
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
// Versions:
// 1.8 - 
// 1.8.1 5/19/10 - Only change is to recompile with Microchip USB Stack v2.7
// 1.8.2 5/31/10 - Only change is to change name in USB enumeration string to Ei
//                  Bot Board - using new PID for SchmalzHaus
// 1.9   6/11/10 - Added two commands:
//					SQ - Solenoid Query - returns 0 or 1 for down and up
//					ST - Solenoid Toggle - toggles state of the servo/solenoid
// 1.9.2 6/15/10 - Added commands:
//					SC,11 sets pen up speed
//					SC,12 sets pen down speed
//					SL - sets the current layer
//					QL - queries the current layer
//					SN - sets move (node) count
//					QN - Query node count
//					QB - Query Button command
// 1.9.3 6/16/10 - Replaced SN with CL (Clear Node) command
// 1.9.4 6/22/10 - Node Count now incremented on pauses (SM with zero step size)
//                  as well
// 1.9.5 7/2/10 - Node count no longer incrimented at all except for NI command
//					NI - Node count Incriment
//					ND - Node count Decriment
//					SN - Set Node count (with 8 byte variable)
//					BL - With latest bootloader, will jumpt to Boot Load mode
// 1.9.6 7/3/10 - Removed extra vectors below 0x1000 for easier merging of HEX
//                      files
//					- use c018i_HID_BL.o now
// 2.0.0 9/9/10 - Add in
//					QC - Query Current - reads voltage of current adjustment pot
//						NOTE: This is NOT done the 'right way'. Instead, we set
//                      up the pin for analog input at boot, then when the QC
//                      comes in, we activate the ADC and take one reading and
//                      then shut it down. Eventually, we should re-write the
//                      'UBW' ADC routines to work with the much more flexible
//                      ADC in the 46J50 part and then just use that generic
//                      code for reading the value of the pot.
//					SC,13,{0,1} - enables/disables RB0 as another PRG button for
//                      pause detection
// 2.0.1 9/13/10 - Bug fix - on v1.1 EBB hardware, need to disable RB0 alt pause
//                      button.
//					switched it to RB2 on v1.1 hardware
// 2.0.2 10/3/10 - Bug fix - QC command not returning proper results - added
//                      cast and now works OK
// 2.1.0 10/21/10- Added in
//					SE - Set Engraver - turns engraver (on RB3) on or off, or
//                      set to PWM power level
// 				   Added code in init to pre-charge RC7 (USB_SENSE_IO) high
//                      before running rest of code
//					to get around wrong resistor value on hardware.
// 2.1.1 11/21/10- Removed Microchip USB stack v2.7, replaced it with v2.8 from 
//                  MAL 2010_10_19.
//					Also using generic Microchip folder now rather than re-named
//                      one (simpler to update).
//				   Updated code in main.c (and others) to match updates from 
//                      latest MAL CDC example.
// 2.1.1cTest1 01/17/11 - Added third parameter to SP command to use any PortB 
//                      pin for servo output.
//                 For this version only - used PortB2 as standard servo output
// 2.1.1d 02/11/11 - Reverted back to RB1 for servo output
//                 - Updated check_and_send_TX_data() to allow unlimited data to
//                      go out without overrunning the output buffer, same as
//                      UBW 1.4.7.
// 2.1.2 11/04/11 - Fixed PI command to return just a 0 or a 1
//                - Updated to USB stack 2.9a
//                - Created MPLAB X project for this firmware
//                - Added SC,14,<state> to enable/disable solenoid output on RB4
//                - Fixed bug with S2 command and solenoid command interaction -
//                      we now turn off solenoid output on RB4 if user uses S2
//                      command to use RB4 for RC servo output.
//                - Fixed bug with S2 command where a duration of 0 would not
//                      shut off the PWM channel
//                - Fixed bug in S2 command where <rate> variable was not being
//                      used correctly
//                - Switched default number of S2 channels to 8 (from 7 before)
// 2.1.3 12/12/11 - RB3 now defaults to digital I/O on boot, can still use SE
//                      command to do PWM later if you want
//                - Compiled with latest UBW stack - 2.9b from MAL 2011-10-18
// 2.1.4 12/14/11 - RB3 now defaults to OFF, rather than ON, at boot.
// 2.1.5 12/15/11 - Fixed problem with pen servo (RB1) being inverted on boot
// 2.2.0 11/07/12 - Fixed problem with SP command not working properly with 
//                      ports other than RB1 because we don't properly use S2
//                      commands for SP up/down within ISR. Tested on all PortB.
// 2.2.1 09/19/13 - Expanded internal delay counter to 32 bits so we can have
//                      delays longer than 2.1s. Now up to 64K ms.
//                - Fixed bug with all <duration> parameters, SP, TP
//                      commands. Now in ms, defaults to 500ms, and actually
//                      works up to 64Kms.
//                - Fixed uninitialized data bug with command FIFO. We were seeing
//                      very long random delays first time SM,<delay>,0,0 was
//                      used.
//                - SP command was executing servo move at end of <duration>. It
//                      now starts servo move and <duration> delay at same time.
//                - Updated USB stack to Microchip MAL USB v2.9j
// 2.2.2 10/15/13 - Fixed bug with SE command that was preventing anything other
//                  than 50% duty cycle from working.
//                - Updated SC,2,{0,1,2} to control PIC and drivers connection
//                  0 = PIC controls built in drivers
//                  1 = PIC controls external step/dir/en drives
//                  2 = external step/dir/en controls built-in drivers
//                - Also updated the pins that are used for driving external
//                  step/dir/en drives. (See documentation)
//                - Updated SC,1,{0,1,2} documentation to match code and removed
//                  SC,14 which is not needed.
//                - Updated logic for EM command to use state of SC,2,{0,1,2}
//                  properly.
//                - Added SC,14,{0,1} to switch between default of 1/25Khz units
//                  for SP and TP command <duration> parameters, and 1ms units
// 2.2.3 01/11/14 - Rewrote analog system so we don't have problems with QC
//                  commands anymore. New command AC.
// 2.2.4 04/26/14 - Fixed bug where 0 for duration in SM would cause problems
//                - Found bug where the <servo_min> and <servo_max> values were
//                  reversed in the SP command. This has now been fixed so that
//                  SP commands will operate the same as 2.0.1 version.
//                - Set initial  'position' of main servo to be 1mS to mimic
//                  behavior of v2.0.1 firmware.
//                - Updated license to BSD 3-clause
//                - Changed SP and TP commands so that if no <duration> parameter
//                  is used, it does not default to 500mS delay, but rather 0mS.
//                  This should now work exactly as 2.0.1 when no parameter is
//                  used.
//                - Tested 2.2.4 against 2.0.1 with Seleae Logic analyzer. Looked
//                  at several Inkscape plots. Confirmed that timing of steppers
//                  and servo are the same. Confirmed that all RB0 through RB7
//                  outputs are the same between the two versions.
// 2.2.5 04/29/14 - Added 'long' arguments to SM for <move_duration> and <axis1>
//                  and <axis2>. All can be 3 bytes now. Also added checks in
//                  SM command to make sure that arguments don't result in a
//                  step speed that's too low (<0.76Hz).
// 2.2.6 01/01/15 - Added 'QM' command - Query Motor, tells PC what is moving.
// 2.2.7 08/13/15 - Added 'ES' command, which will immediately abort any SM command
//                  but leave the motors energized. (E-stop) It returns "1" if
//                  it aborted a move in progress, "0" otherwise. It will also 
//                  delete any pending SM command in the FIFO.
// 2.2.8 08/14/15 - Corrected error checking in SM command to accurately reflect
//                  too fast and too slow requests. (>25K or <1.31 steps per
//                  second). Also added type cast that now allows for full range
//                  of step values to work properly.
// 2.2.9 08/18/15 - Added extra values to output of ES command to indicate how 
//                  many steps were aborted.
//

#include <p18cxxx.h>
#include <usart.h>
#include <stdio.h>
#include <ctype.h>
#include <delays.h>
#include "Usb\usb.h"
#include "Usb\usb_function_cdc.h"
#include "usb_config.h"
#include "HardwareProfile.h"
#include "ubw.h"
#include "ebb.h"
#include "delays.h"
#include "ebb_demo.h"
#include "RCServo2.h"

// This is the value that gets multiplied by Steps/Duration to compute
// the StepAdd values.
#define OVERFLOW_MUL	(0x8000 / HIGH_ISR_TICKS_PER_MS)

#define MAX_RC_DURATION 11890

// Maximum number of elements in the command FIFO
#define COMMAND_FIFO_LENGTH     4


typedef enum
{
	SOLENOID_OFF = 0,
	SOLENOID_ON,
	SOLENOID_PWM
} SolenoidStateType;

static void process_SM(
	UINT32 Duration,
	INT32 A1Stp,
	INT32 A2Stp
);

typedef enum
{
	PIC_CONTROLS_DRIVERS = 0,
	PIC_CONTROLS_EXTERNAL,
    EXTERNAL_CONTROLS_DRIVERS
} DriverConfigurationType;

#pragma udata access fast_vars
// Working registers
static near MoveCommandType CurrentCommand;
static near unsigned int StepAcc[NUMBER_OF_STEPPERS];
near BOOL FIFOEmpty;

#pragma udata
static unsigned char OutByte;
static unsigned char TookStep;
static unsigned char AllDone;
static unsigned char i;
MoveCommandType CommandFIFO[COMMAND_FIFO_LENGTH];

unsigned int DemoModeActive;
unsigned int comd_counter;
static SolenoidStateType SolenoidState;
static unsigned int SolenoidDelay;
static DriverConfigurationType DriverConfiguration;

// track the latest state of the pen
static PenStateType PenState;

static unsigned long NodeCount;
static char Layer;
static BOOL ButtonPushed;
static BOOL UseAltPause;
unsigned char QC_ms_timer;
static UINT StoredEngraverPower;
// Set TRUE to enable solenoid output for pen up/down
BOOL gUseSolenoid;
// Set TRUE to enable RC Servo output for pen up/down
BOOL gUseRCPenServo;

// ISR
#pragma interrupt high_ISR
void high_ISR(void)
{
	//Check which interrupt flag caused the interrupt.
	//Service the interrupt
	//Clear the interrupt flag
	//Etc.
	#if defined(USB_INTERRUPT)
		USBDeviceTasks();
	#endif

    // 25KHz ISR fire
	if (PIR1bits.TMR1IF)
	{
		// Clear the interrupt 
		PIR1bits.TMR1IF = 0;
		TMR1L = TIMER1_L_RELOAD;	// Set to 120 for 25KHz ISR fire
		TMR1H = TIMER1_H_RELOAD;	//

		OutByte = CurrentCommand.DirBits;
		TookStep = FALSE;
		AllDone = TRUE;

        // Note, you don't even need a command to delay. Any command can have
        // a delay associated with it, if DelayCounter is != 0.
        if (CurrentCommand.DelayCounter)
        {
            // Double check that things aren't way too big
            if (CurrentCommand.DelayCounter > HIGH_ISR_TICKS_PER_MS * (UINT32)0x10000)
            {
                CurrentCommand.DelayCounter = 0;
            }
            else {
                CurrentCommand.DelayCounter--;
            }            
        }

        if (CurrentCommand.DelayCounter)
        {
            AllDone = FALSE;
        }

        PORTDbits.RD1 = 0;

        // Note: by not making this an else-if, we have our DelayCounter
        // counting done at the same time as our motor move or servo move.
        // This allows the delay time to start counting at the beginning of the
        // command execution.
        if (CurrentCommand.Command == COMMAND_MOTOR_MOVE)
		{
            // Only output DIR bits if we are actually doing something
			if (CurrentCommand.StepsCounter[0] || CurrentCommand.StepsCounter[1])
            {
				if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
				{
					if (CurrentCommand.DirBits & DIR1_BIT)
					{
						Dir1IO = 1;
					}
					else
					{
						Dir1IO = 0;
					}	
					if (CurrentCommand.DirBits & DIR2_BIT)
					{
						Dir2IO = 1;
					}
					else
					{
						Dir2IO = 0;
					}	
				}
                else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
				{
					if (CurrentCommand.DirBits & DIR1_BIT)
					{
						Dir1AltIO = 1;
					}
					else
					{
						Dir1AltIO = 0;
					}	
					if (CurrentCommand.DirBits & DIR2_BIT)
					{
						Dir2AltIO = 1;
					}
					else
					{
						Dir2AltIO = 0;
					}	
				}

				// Only do this if there are steps left to take
				if (CurrentCommand.StepsCounter[0])
				{
					StepAcc[0] = StepAcc[0] + CurrentCommand.StepAdd[0];
					if (StepAcc[0] > 0x8000)
					{
						StepAcc[0] = StepAcc[0] - 0x8000;
						OutByte = OutByte | STEP1_BIT;
						TookStep = TRUE;
						CurrentCommand.StepsCounter[0]--;
					}
					AllDone = FALSE;
				}
				if (CurrentCommand.StepsCounter[1])
				{
					StepAcc[1] = StepAcc[1] + CurrentCommand.StepAdd[1];
					if (StepAcc[1] > 0x8000)
					{
						StepAcc[1] = StepAcc[1] - 0x8000;
						OutByte = OutByte | STEP2_BIT;
						TookStep = TRUE;
						CurrentCommand.StepsCounter[1]--;
					}
					AllDone = FALSE;
				}

				if (TookStep)
				{
					if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
					{
						if (OutByte & STEP1_BIT)
						{
							Step1IO = 1;
						}
						if (OutByte & STEP2_BIT)
						{
							Step2IO = 1;
						}
					}
                    else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
					{
						if (OutByte & STEP1_BIT)
						{
							Step1AltIO = 1;
						}
						if (OutByte & STEP2_BIT)
						{
							Step2AltIO = 1;
						}
					}
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					Delay1TCY();
					if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
					{
						Step1IO = 0;
						Step2IO = 0;
					}
                    else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
					{
						Step1AltIO = 0;
						Step2AltIO = 0;
					}
				}
			}
		}
        // Check to see if we should change the state of the pen
		else if (CurrentCommand.Command == COMMAND_SERVO_MOVE)
		{
            if (gUseRCPenServo)
            {
                // Precompute the channel, since we use it all over the place
                UINT8 Channel = CurrentCommand.ServoChannel - 1;

                // This code below is the meat of the RCServo2_Move() function
                // We have to manually write it in here rather than calling
                // the function because a real function inside the ISR
                // causes the compiler to generate enormous amounts of setup/teardown
                // code and things run way too slowly.

                // If the user is trying to turn off this channel's RC servo output
                if (0 == CurrentCommand.ServoPosition)
                {
                    // Turn off the PPS routing to the pin
                    *(gRC2RPORPtr + gRC2RPn[Channel]) = 0;
                    // Clear everything else out for this channel
                    gRC2Rate[Channel] = 0;
                    gRC2Target[Channel] = 0;
                    gRC2RPn[Channel] = 0;
                    gRC2Value[Channel] = 0;
                }
                else
                {
                    // Otherwise, set all of the values that start this RC servo moving
                    gRC2Rate[Channel] = CurrentCommand.ServoRate;
                    gRC2Target[Channel] = CurrentCommand.ServoPosition;
                    gRC2RPn[Channel] = CurrentCommand.ServoRPn;
                    if (gRC2Value[Channel] == 0)
                    {
                        gRC2Value[Channel] = CurrentCommand.ServoPosition;
                    }
                }
            }
            
            // If this servo is the pen servo (on g_servo2_RPn)
            if (CurrentCommand.ServoRPn == g_servo2_RPn)
            {
                // Then set its new state based on the new position
                if (CurrentCommand.ServoPosition == g_servo2_min)
                {
                    PenState = PEN_UP;
                    SolenoidState = SOLENOID_OFF;
                    if (gUseSolenoid)
                    {
                        PenUpDownIO = 0;
                    }
                }
                else
                {
                    PenState = PEN_DOWN;
                    SolenoidState = SOLENOID_ON;
                    if (gUseSolenoid)
                    {
                        PenUpDownIO = 1;
                    }
                }
            }
		}
	
		// If we're done with our current command, load in the next one
		if (AllDone && CurrentCommand.DelayCounter == 0)
		{
			CurrentCommand.Command = COMMAND_NONE;
			if (!FIFOEmpty)
			{
                CurrentCommand = CommandFIFO[0];
                // Zero out command in FIFO
                CommandFIFO[0].Command = COMMAND_NONE;
                CommandFIFO[0].StepAdd[0] = 0;
                CommandFIFO[0].StepAdd[1] = 0;
                CommandFIFO[0].StepsCounter[0] = 0;
                CommandFIFO[0].StepsCounter[1] = 0;
                CommandFIFO[0].DirBits = 0;
                CommandFIFO[0].DelayCounter = 0;
                CommandFIFO[0].ServoPosition = 0;
                CommandFIFO[0].ServoRPn = 0;
                CommandFIFO[0].ServoChannel = 0;
                CommandFIFO[0].ServoRate = 0;
				FIFOEmpty = TRUE;
			}
            else {
                CurrentCommand.DelayCounter = 0;
            }
		}
		
		// Check for button being pushed
		if (
			(!swProgram)
			||
			(
				UseAltPause
				&&
				!PORTBbits.RB0
			)
		)
		{
			ButtonPushed = TRUE;
		}
	}
}

// Init code
void EBB_Init(void)
{
    char i;

    // Initalize all Current Command values
    for (i = 0; i < NUMBER_OF_STEPPERS; i++)
    {
        CurrentCommand.StepAdd[i] = 1;
        CurrentCommand.StepsCounter[i] = 0;
    }
    CurrentCommand.Command = COMMAND_NONE;
    CurrentCommand.DirBits = 0;
    CurrentCommand.DelayCounter = 0;
    CurrentCommand.ServoPosition = 0;
    CurrentCommand.ServoRPn = 0;
    CurrentCommand.ServoChannel = 0;
    CurrentCommand.ServoRate = 0;

    FIFOEmpty = TRUE;

	// Set up TMR1 for our 25KHz High ISR for stepping
	T1CONbits.RD16 = 0; 	// Set 8 bit mode
	T1CONbits.TMR1CS1 = 0; 	// System clocked from Fosc/4
	T1CONbits.TMR1CS0 = 0;
	T1CONbits.T1CKPS1 = 1; 	// Use 1:4 Prescale value
	T1CONbits.T1CKPS0 = 0;
	T1CONbits.T1OSCEN = 0; 	// Don't use external osc
	T1CONbits.T1SYNC = 0;
	TMR1L = TIMER1_L_RELOAD;	// Set to 120 for 25KHz ISR fire
	TMR1H = TIMER1_H_RELOAD;	// 

	T1CONbits.TMR1ON = 1; // Turn the timer on

	IPR1bits.TMR1IP = 1;	// Use high priority interrupt
	PIR1bits.TMR1IF = 0;	// Clear the interrupt
	PIE1bits.TMR1IE = 1;	// Turn on the interrupt

//	PORTA = 0;
	RefRA0_IO_TRIS = INPUT_PIN;
//	PORTB = 0;
//	INTCON2bits.RBPU = 0;	// Turn on weak-pull ups for port B
//	PORTC = 0;		// Start out low
//	TRISC = 0x80;	// Make portC output execpt for PortC bit 7, USB bus sense
//	PORTD = 0;
//	TRISD = 0;
//	PORTE = 0;
//	TRISE = 0;

    // And make sure to always use low priority for ADC
    IPR1bits.ADIP = 0;
    
    // Turn on AN0 (RA0) as analog input
    AnalogConfigure(0,1);
    // Turn on AN11 (V+) as analog input
    AnalogConfigure(11,1);

	MS1_IO = 1;
	MS1_IO_TRIS = OUTPUT_PIN;
	MS2_IO = 1;
	MS2_IO_TRIS = OUTPUT_PIN;
	MS3_IO	= 1;
	MS3_IO_TRIS = OUTPUT_PIN;

	Enable1IO = 1;	
	Enable1IO_TRIS = OUTPUT_PIN;	
	Enable2IO = 1;
	Enable2IO_TRIS = OUTPUT_PIN;

	Step1IO	= 0;
	Step1IO_TRIS = OUTPUT_PIN;
	Dir1IO = 0;
	Dir1IO_TRIS = OUTPUT_PIN;
	Step2IO	= 0;	
	Step2IO_TRIS = OUTPUT_PIN;	
	Dir2IO = 0;	
	Dir2IO_TRIS = OUTPUT_PIN;

	// For bug in VUSB divider resistor, set RC7 as output and set high
	// Wait a little while to charge up
	// Then set back as an input
	// The idea here is to get the schmidt trigger input RC7 high before
	// we make it an input, thus getting it above the 2.65V ST threshold
	// And allowing VUSB to keep the logic level on the pin high at 2.5V
    #if defined(USE_USB_BUS_SENSE_IO)
	    tris_usb_bus_sense = OUTPUT_PIN; // See HardwareProfile.h
    	USB_BUS_SENSE = 1;
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		Delay1TCY();
		tris_usb_bus_sense = INPUT_PIN;
		USB_BUS_SENSE = 0;
	#endif
    gUseSolenoid = TRUE;
    gUseRCPenServo = TRUE;

    // Set up pen up/down direction as output
	PenUpDownIO = 0;
	PenUpDownIO_TRIS = OUTPUT_PIN;

	SolenoidState = SOLENOID_ON;
	DriverConfiguration = PIC_CONTROLS_DRIVERS;
	PenState = PEN_UP;
	Layer = 0;
	NodeCount = 0;
	ButtonPushed = FALSE;
	// Default RB0 to be an input, with the pull-up enabled, for use as alternate
	// PAUSE button (just like PRG)
	// Except for v1.1 hardware, use RB2
	TRISBbits.TRISB0 = 1;
	INTCON2bits.RBPU = 0;	// Turn on all of PortB pull-ups
	UseAltPause = TRUE;

	TRISBbits.TRISB3 = 0;		// Make RB3 an output (for engraver)
	PORTBbits.RB3 = 0;          // And make sure it starts out off
}

// Stepper (mode) Configure command
// SC,1,0<CR> will use just solenoid output for pen up/down
// SC,1,1<CR> will use servo on RB1 for pen up/down
// SC,1,2<CR> will use servo on RB1 for pen up/down, but with ECCP2 (PWM) in hardware (default)
// SC,2,0<CR> will make PIC control drivers (default)
// SC,2,1<CR> will make PIC control external drivers using these pins
//		ENABLE1 = RD1
//		ENABLE2 = RA1
//		STEP1 = RC6
//		DIR1 = RC2
//		STEP2 = RA5
//		DIR2 = RA2
// SC,2,2<CR> will disconnect PIC from drivers and allow external step/dir source
// SC,4,<servo2_min><CR> will set <servo2_min> as the minimum value for the servo (1 to 65535)
// SC,5,<servo2_max><CR> will set <servo2_max> as the maximum value for the servo (1 to 65535)
// SC,6,<servo_min><CR> will set <servo_min> as the minimum value for the servo (1 to 11890)
// SC,7,<servo_max><CR> will set <servo_max> as the maximum value for the servo (1 to 11890)
// SC,8,<servo2_slots><CR> sets the number of slots for the servo2 system (1 to 24)
// SC,9,<servo2_slotMS><CR> sets the number of ms in duration for each slot (1 to 6)
// SC,10,<servo2_rate><CR> sets the rate of change for the servo (both up and down)
// SC,11,<servo2_rate><CR> sets the pen up speed
// SC,12,<servo2_rate><CR> sets the pen down speed
// SC,13,1<CR> enables RB3 as parallel input to PRG button for pause detection
// SC,13,0<CR> disables RB3 as parallel input to PRG button for pause detection


void parse_SC_packet (void)
{
	unsigned char Para1 = 0;
	unsigned int Para2 = 0;

	// Extract each of the values.
	extract_number (kUCHAR, &Para1, kREQUIRED);
	extract_number (kUINT, &Para2, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Check for command to select which (solenoid/servo) gets used for pen
	if (Para1 == 1)
	{
        // Use just solenoid
		if (Para2 == 0)
		{
            gUseSolenoid = TRUE;
            gUseRCPenServo = FALSE;
            // Turn off RC signal on Pen Servo output
            RCServo2_Move(0, g_servo2_RPn, 0, 0);
        }
        // Use just RC servo
		else if (Para2 == 1)
		{
            gUseSolenoid = FALSE;
            gUseRCPenServo = TRUE;
		}
        // Use solenoid AND servo (default)
		else
		{
            gUseSolenoid = TRUE;
            gUseRCPenServo = TRUE;
		}
        // Send a new command to set the state of the servo/solenoid
		process_SP(PenState, 0);
	}
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
	// Set <min_servo> for Servo2 method
	else if (Para1 == 4)
	{
		g_servo2_min = Para2;
	}
	// Set <max_servo> for Servo2
	else if (Para1 == 5)
	{
		g_servo2_max = Para2;
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
		g_servo2_rate_up = Para2;
		g_servo2_rate_down = Para2;
	}
	else if (Para1 == 11)
	{
		g_servo2_rate_up = Para2;
	}
	else if (Para1 == 12)
	{
		g_servo2_rate_down = Para2;
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

// The Stepper Motor command
// Usage: SM,<move_duration>,<axis1_steps>,<axis2_steps><CR>
// <move_duration> is a number from 1 to 65535, indicating the number of milliseconds this move should take
// <axisX_steps> is a signed 16 bit number indicating how many steps (and what direction) the axis should take
// NOTE1: <axis2_steps> is optional and can be left off
// If the EBB can not make the move in the specified time, it will take as long as it needs to at max speed
// i.e. SM,1,1000 will not produce 1000steps in 1ms. Instead, it will take 40ms (25KHz max step rate)
// NOTE2: If you specify zero steps for the axis, then you effectively create a delay. Use for small
// pauses before raising or lowering the pen, for example.
void parse_SM_packet (void)
{
	UINT32 Duration = 0;
	INT32 A1Steps = 0, A2Steps = 0;
    INT32 Steps = 0;

	// Extract each of the values.
	extract_number (kULONG, &Duration, kREQUIRED);
	extract_number (kLONG, &A1Steps, kREQUIRED);
	extract_number (kLONG, &A2Steps, kOPTIONAL);

    // Check for invalid duration
    if (Duration == 0) {
    	bitset (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    }


    // Check for too-fast step request (>25KHz)
    // First get absolute value of steps, then check if it's asking for >25KHz
    if (A1Steps > 0) {
        Steps = A1Steps;
    }
    else {
        Steps = -A1Steps;
    }
    // Limit each parameter to just 3 bytes
    if (Duration > 0xFFFFFF) {
       printf((far rom char *)"!0 Err: <move_duration> larger than 16777215 ms.\n\r");
       return;
    }
    if (Steps > 0xFFFFFF) {
       printf((far rom char *)"!0 Err: <axis1> larger than 16777215 steps.\n\r");
       return;
    }
    // Check for too fast
    if ((Steps/Duration) > HIGH_ISR_TICKS_PER_MS) {
       printf((far rom char *)"!0 Err: <axis1> step rate > 25K steps/second.\n\r");
       return;
    }
    // And check for too slow
    if ((Duration/1311) >= Steps && Steps != 0) {
       printf((far rom char *)"!0 Err: <axis1> step rate < 1.31Hz.\n\r");
       return;
    }

    if (A2Steps > 0) {
        Steps = A2Steps;
    }
    else {
        Steps = -A2Steps;
    }    
    if (Steps > 0xFFFFFF) {
       printf((far rom char *)"!0 Err: <axis2> larger than 16777215 steps.\n\r");
       return;
    }
    if ((Steps/Duration) > HIGH_ISR_TICKS_PER_MS) {
       printf((far rom char *)"!0 Err: <axis2> step rate > 25K steps/second.\n\r");
       return;
    }
    if ((Duration/1311) >= Steps && Steps != 0) {
       printf((far rom char *)"!0 Err: <axis2> step rate < 1.31Hz.\n\r");
       return;
    }

    // Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

    // If we get here, we know that step rate for both A1 and A2 is
    // between 25KHz and 1.31Hz which are the limits of what EBB can do.
  	process_SM(Duration, A1Steps, A2Steps);

	print_ack();
}

// Main stepper move function. This is the reason EBB exists.
// <Duration> is a 3 byte unsigned int, the number of mS that the move should take
// <A1Stp> and <A2Stp> are the Axis 1 and Axis 2 number of steps to take in
//  <Duration> mS, as 3 byte signed values, where the sign determines the motor
//  direction.
// This function waits until there is room in the 1-deep FIFO before placing
// the data in the FIFO. The ISR then sees this data when it is done with its
// current move, and starts this new move.
//
// In the future, making the FIFO more elements deep may be cool.
// 
static void process_SM(
	UINT32 Duration,
	INT32 A1Stp,
	INT32 A2Stp
)
{
    UINT32 temp = 0;

	// Trial: Spin here until there's space in the fifo
	while(!FIFOEmpty)
	;

	// Check for delay
	if (A1Stp == 0 && A2Stp == 0)
	{
		CommandFIFO[0].Command = COMMAND_DELAY;
        // This is OK because we only need to multiply the 3 byte Duration by
        // 25, so it fits in 4 bytes OK.
  		CommandFIFO[0].DelayCounter = HIGH_ISR_TICKS_PER_MS * Duration;
	}
	else
	{
		CommandFIFO[0].DelayCounter = 0; // No delay for motor moves
		CommandFIFO[0].DirBits = 0;
		
		// Always enable both motors when we want to move them
		Enable1IO = ENABLE_MOTOR;
		Enable2IO = ENABLE_MOTOR;

		// First, set the direction bits
		if (A1Stp < 0)
		{
			CommandFIFO[0].DirBits = CommandFIFO[0].DirBits | DIR1_BIT;
			A1Stp = -A1Stp;
		}
		if (A2Stp < 0)
		{
			CommandFIFO[0].DirBits = CommandFIFO[0].DirBits | DIR2_BIT;
			A2Stp = -A2Stp;
		}
        // To compute StepAdd values from Duration.
        // A1Stp is from 0x000001 to 0xFFFFFF.
        // HIGH_ISR_TICKS_PER_MS = 25
        // Duration is from 0x000001 to 0xFFFFFF.
        // temp needs to be from 0x0001 to 0x7FFF.
        // Temp is added to accumulator every 25KHz. So slowest step rate
        // we can do is 1 step every 25KHz / 0x7FFF or 1 every 763mS. 
        // Fastest step rate is obviously 25KHz.
        // If A1Stp is 1, then duration must be 763 or less.
        // If A1Stp is 2, then duration must be 763 * 2 or less.
        // If A1Stp is 0xFFFFFF, then duration must be at least 671088.
        
        // First check for duration to large.
//        if (A1Stp < (0xFFFFFF/763)) {
//            if (duration > (A1Stp * 763)) {
//                printf((far rom char *)"Major malfunction Axis1 duration too long : %lu\n\r", duration);
//                temp = 0;
//                A1Stp = 0;
//            }
//        }
        if (A1Stp < 0x1FFFF) {
            temp = ((((UINT32)0x8000 * A1Stp)/(UINT32)HIGH_ISR_TICKS_PER_MS)/Duration);
        }
        else {
            temp = (((A1Stp/Duration) * (UINT32)0x8000)/(UINT32)HIGH_ISR_TICKS_PER_MS);
        }
        if (temp > 0x8000) {
            printf((far rom char *)"Major malfunction Axis1 StepCounter too high : %lu\n\r", temp);
            temp = 0x8000;
        }
        if (temp == 0 && A1Stp != 0) {
            printf((far rom char *)"Major malfunction Axis1 StepCounter zero\n\r");
            temp = 1;
        }
        CommandFIFO[0].StepAdd[0] = temp;
        CommandFIFO[0].StepsCounter[0] = A1Stp;

        if (A2Stp < 0x1FFFF) {
            temp = ((((UINT32)0x8000 * A2Stp)/(UINT32)HIGH_ISR_TICKS_PER_MS)/Duration);
        }
        else {
            temp = (((A2Stp/Duration) * (UINT32)0x8000)/(UINT32)HIGH_ISR_TICKS_PER_MS);
        }
        if (temp > 0x8000) {
            printf((far rom char *)"Major malfunction Axis2 StepCounter too high : %lu\n\r", temp);
            temp = 0x8000;
        }
        if (temp == 0 && A2Stp != 0) {
            printf((far rom char *)"Major malfunction Axis2 StepCounter zero\n\r");
            temp = 1;
        }
        CommandFIFO[0].StepAdd[1] = temp;
        CommandFIFO[0].StepsCounter[1] = A2Stp;
        CommandFIFO[0].Command = COMMAND_MOTOR_MOVE;
	}
		
	FIFOEmpty = FALSE;
}

// E-Stop
// Usage: ES<CR>
// Returns: <command_interrupted>,<fifo_steps1>,<fifo_steps2>,<steps_remaining1>,<steps_remaining2><CR>OK<CR>
// This command will abort any in-progress motor move (SM) command.
// It will also clear out any pending command(s) in the FIFO.
// <command_interrupted> = 0 if no FIFO or in-progress move commands were interrupted,
//                         1 if a motor move command was in progress or in the FIFO
// <fifo_steps1> and <fifo_steps1> = 24 bit unsigned integers with the number of steps
//                         in any SM command sitting in the fifo for axis1 and axis2.
// <steps_remaining1> and <steps_remaining2> = 24 bit unsigned integers with the number of
//                         steps left in the currently executing SM command (if any) for
//                         axis1 and axis2.
// It will return 0,0,00,0 if no SM command was executing at the time, and no SM
// command was in the FIFO.
void parse_ES_packet(void)
{
    UINT8 command_interrupted = 0;
    UINT32 remaining_steps1 = 0;
    UINT32 remaining_steps2 = 0;
    UINT32 fifo_steps1 = 0;
    UINT32 fifo_steps2 = 0;
   
    
    // If there is a command waiting in the FIFO and it is a move command
    // or the current command is a move command, then remember that for later.
    if (
        (!FIFOEmpty && CommandFIFO[0].Command == COMMAND_MOTOR_MOVE)
        || 
        CurrentCommand.Command == COMMAND_MOTOR_MOVE
    )
    {
        command_interrupted = 1;
    }
    
    // If the FIFO has a move command in it, remove it.
    if (CommandFIFO[0].Command == COMMAND_MOTOR_MOVE)
    {
        CommandFIFO[0].Command = COMMAND_NONE;
        fifo_steps1 = CommandFIFO[0].StepsCounter[0];
        fifo_steps2 = CommandFIFO[0].StepsCounter[1];
        CommandFIFO[0].StepsCounter[0] = 0;
        CommandFIFO[0].StepsCounter[1] = 0;
        FIFOEmpty = TRUE;
    }

    // If the current command is a move command, then stop the move.
    if (CurrentCommand.Command == COMMAND_MOTOR_MOVE)
    {
    	CurrentCommand.Command = COMMAND_NONE;
        remaining_steps1 = CurrentCommand.StepsCounter[0];
        remaining_steps2 = CurrentCommand.StepsCounter[1];
        CurrentCommand.StepsCounter[0] = 0;
        CurrentCommand.StepsCounter[1] = 0;
    }
                
    printf((far rom char *)"%d,%lu,%lu,%lu,%lu\n\r", 
            command_interrupted,
            fifo_steps1,
            fifo_steps2,
            remaining_steps1,
            remaining_steps2
        );
	print_ack();
}

// Query Pen
// Usage: QP<CR>
// Returns: 0 for down, 1 for up, then OK<CR>
void parse_QP_packet(void)
{
	printf((far rom char *)"%d\n\r", PenState);

	print_ack();
}

// Toggle Pen
// Usage: TP,<duration><CR>
// Returns: OK<CR>
// <duration> is optional, and defaults to 0mS
// Just toggles state of pen arm, then delays for the optional <duration>
// Duration is in units of 1ms
void parse_TP_packet(void)
{
	UINT16 CommandDuration = 0;

	// Extract each of the values.
	extract_number (kUINT, &CommandDuration, kOPTIONAL);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	if (PenState == PEN_UP)
	{
		process_SP(PEN_DOWN, CommandDuration);
	}
	else
	{
		process_SP(PEN_UP, CommandDuration);
	}

	print_ack();
}

// Set Pen
// Usage: SP,<State>,<Duration>,<PortB_Pin><CR>
// <State> is 0 (for goto servo_max) or 1 (for goto servo_min)
// <Duration> is how long to wait before the next command in the motion control 
//      FIFO should start. (defaults to 0mS)
//      Note that the units of this parameter is either 1ms
// <PortB_Pin> Is a value from 0 to 7 and allows you to re-assign the Pen
//      RC Servo output to different PortB pins.
// This is a command that the user can send from the PC to set the pen state.
// Note that there is only one pen RC servo output - if you use the <PortB_Pin>
// parameter, then that new pin becomes the pen RC servo output. This command
// does not allow for mulitple servo signals at the same time from port B pins.
// Use the S2 command for that.
//
// This function will use the values for <serv_min>, <servo_max>,
// <servo_rate_up> and <servo_rate_down> (SC,4 SC,5, SC,11, SC,10 commands)
// when it schedules the servo command.
// 
// Internally, the parse_SP_packet() function makes a call to
// process_SP() function to actually make the change in the servo output.
//
void parse_SP_packet(void)
{
	UINT8 State = 0;
	UINT16 CommandDuration = 0;
	UINT8 Pin = DEFAULT_EBB_SERVO_PORTB_PIN;
    ExtractReturnType Ret;

	// Extract each of the values.
	extract_number (kUCHAR, &State, kREQUIRED);
	extract_number (kUINT, &CommandDuration, kOPTIONAL);
	Ret = extract_number (kUCHAR, &Pin, kOPTIONAL);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

    // Error check
	if (Pin > 7)
	{
		Pin = DEFAULT_EBB_SERVO_PORTB_PIN;
	}

    if (State > 1)
    {
        State = 1;
    }

    // Set the PRn of the Pen Servo output
    // Add 3 to get from PORTB pin number to RPn number
    if (g_servo2_RPn != (Pin + 3))
    {
        // if we are changing which pin the pen servo is on, we need to cancel
        // the servo output on the old channel first
        RCServo2_Move(0, g_servo2_RPn, 0, 0);
        // Now record the new RPn
        g_servo2_RPn = Pin + 3;
    }

    // Execute the servo state change
	process_SP(State, CommandDuration);

	print_ack();
}

// Interal use function -
// Perform a state change on the pen RC servo output. Move it up or move it down
// <NewState> is either PEN_UP or PEN_DOWN.
// <CommandDuration> is the number of milliseconds to wait before executing the
//      next command in the motion control FIFO
//
// This function uses the g_servo2_min, max, rate_up, rate_down variables
// to schedule an RC Servo change with the RCServo2_Move() function.
//
void process_SP(PenStateType NewState, UINT16 CommandDuration)
{	
    UINT16 Position;
    UINT16 Rate;

	if (NewState == PEN_UP)
	{
        Position = g_servo2_min;
        Rate = g_servo2_rate_up;
	}
	else
	{
        Position = g_servo2_max;
        Rate = g_servo2_rate_down;
	}

    // Now schedule the movement with the RCServo2 function
    RCServo2_Move(Position, g_servo2_RPn, Rate, CommandDuration);
}

// Enable Motor
// Usage: EM,<EnableAxis1>,<EnableAxis2><CR>
// Everything afer EnableAxis1 is optional
// Each parameter can have a value of
//		0 to disable that motor driver
// FOR OLD DRIVER CHIP (A3967) - can do separate enables for each axis
//		1 to enable the driver in 1/8th step mode
//		2 to enable the driver in 1/4 step mode
//		3 to enable the driver in 1/2 step mode
//		4 to enable the driver in full step mode
// FOR NEW DRIVER CHIP (A4988/A4983)
// (only first parameter applies, and it applies to both drivers)
//		1 to enable the driver in 1/16th step mode
//		2 to enable the driver in 1/8 step mode
//		3 to enable the driver in 1/4 step mode
//		4 to enable the driver in 1/2 step mode
//		5 to enable the driver in full step mode
// If you disable a motor, it goes 'limp' (we clear the ENABLE pin on that motor's
// driver chip)
// Note that when using 0 or 1 for a paraemter, you can use both axis even
// on a 'new' driver chip board. (i.e. EM,0,1 will disable motor 1 and enable 2)
// Note that the MSx lines do not come to any headers, so even when an external
// source is controlling the drivers, the PIC still needs to control the
// MSx lines.
void parse_EM_packet(void)
{
	unsigned char EA1, EA2;
	ExtractReturnType RetVal;

	// Extract each of the values.
	RetVal = extract_number (kUCHAR, &EA1, kREQUIRED);
	if (kEXTRACT_OK == RetVal)
	{
		// Bail if we got a conversion error
		if (error_byte)
		{
			return;
		}
		if (
            (DriverConfiguration == PIC_CONTROLS_DRIVERS)
            ||
            (DriverConfiguration == EXTERNAL_CONTROLS_DRIVERS)
        )
		{
			if (EA1 > 0)
			{
				if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
                {
                    Enable1IO = ENABLE_MOTOR;
                }
				if (EA1 == 1)
				{
					MS1_IO = 1;
					MS2_IO = 1;
					MS3_IO = 1;
				}
				if (EA1 == 2)
				{
					MS1_IO = 1;
					MS2_IO = 1;
					MS3_IO = 0;
				}
				if (EA1 == 3)
				{
					MS1_IO = 0;
					MS2_IO = 1;
					MS3_IO = 0;
				}
				if (EA1 == 4)
				{
					MS1_IO = 1;
					MS2_IO = 0;
					MS3_IO = 0;
				}				
				if (EA1 == 5)
				{
					MS1_IO = 0;
					MS2_IO = 0;
					MS3_IO = 0;
				}				
			}
			else
			{
				if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
                {
    				Enable1IO = DISABLE_MOTOR;
                }
			}
		}
        else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
		{
			if (EA1 > 0)
			{
				Enable1AltIO = ENABLE_MOTOR;
			}
			else
			{
				Enable1AltIO = DISABLE_MOTOR;
			}
		}
	}

	RetVal = extract_number (kUCHAR, &EA2, kOPTIONAL);
	if (kEXTRACT_OK == RetVal)
	{
		// Bail if we got a conversion error
		if (error_byte)
		{
			return;
		}
		if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
		{
			if (EA2 > 0)
			{
				Enable2IO = ENABLE_MOTOR;
			}
			else
			{
				Enable2IO = DISABLE_MOTOR;
			}
		}
        else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
		{
			if (EA2 > 0)
			{
				Enable2AltIO = ENABLE_MOTOR;
			}
			else
			{
				Enable2AltIO = DISABLE_MOTOR;
			}
		}
	}

    print_ack();
}

// Node counter Incriment
// Usage: NI<CR>
void parse_NI_packet(void)
{
	if (NodeCount < 0xFFFFFFFEL)
	{
		NodeCount++;
	}
	print_ack();
}

// Node counter Deccriment
// Usage: ND<CR>
void parse_ND_packet(void)
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
void parse_SN_packet(void)
{
	unsigned long Temp;
	ExtractReturnType RetVal;
	
	RetVal = extract_number (kULONG, &Temp, kREQUIRED);
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
void parse_QN_packet(void)
{
	printf ((far rom char*)"%010lu\r\n", NodeCount);

	print_ack();
}

// Set Layer
// Usage: SL,<NewLayer><CR>
void parse_SL_packet(void)
{
	// Extract each of the values.
	extract_number (kUCHAR, &Layer, kREQUIRED);

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
void parse_QL_packet(void)
{
	printf ((far rom char*)"%03i\r\n", Layer);

	print_ack();
}

// Query Button
// Usage: QB<CR>
// Returns: <HasButtonBeenPushedSinceLastQB><CR> (0 or 1)
// OK<CR>
void parse_QB_packet(void)
{
	printf ((far rom char*)"%1i\r\n", ButtonPushed);
	ButtonPushed = FALSE;

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
void parse_QC_packet(void)
{
    // Since access to ISR_A_FIFO[] is not protected in any way from ISR and
    // mainline code accessing at the same time, we will just wait for
    // the cycle of ADC readings to finish before we spit out our value.
    while (PIE1bits.ADIE);

	// Print out our results
	printf ((far rom char*)"%04i,%04i\r\n", ISR_A_FIFO[0], ISR_A_FIFO[11]);

	print_ack();
}	

// Set Engraver
// Usage: SE,<state>,<power><CR>
// <state> is 0 for off and 1 for on (required)
// <power> is 10 bit PWM power level (optional)
// We boot up with <power> at 1023 (full on)
// The engraver motor is always assumed to be on RB3
// So our init routine will map ECCP1
//
// Timer0 is RC command
// Timer1 is stepper
// Timer2 and ECCP1 is engraver PWM
// Timer3 and ECCP2 is RC servo2 output
// Timer4 is 1ms ISR

void parse_SE_packet(void)
{
	UINT8 State = 0;
	UINT16 Power = 0;
	
	// Extract each of the values.
	extract_number (kUCHAR, &State, kREQUIRED);
	extract_number (kUINT, &Power, kOPTIONAL);

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

    // Set to %50 if no Power parameter specified, otherwise use parameter
    if (Power == 0 && State == 1)
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

        // Initalize Timer2

        // The prescaler will be at 1
        T2CONbits.T2CKPS = 0b00;

        // Do not generate an interrupt
        PIE1bits.TMR2IE = 0;

        TCLKCONbits.T3CCP1 = 1;		// ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4
        TCLKCONbits.T3CCP2 = 0;		// ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4

        CCP1CONbits.CCP1M = 0b1100;	// Set EECP1 as PWM mode
        CCP1CONbits.P1M = 0b00;		// Enhanged PWM mode: single ouptut

        // Set up output routing to go to RB3 (RP6)
        RPOR6 = 14;	// 14 is CCP1/P1A - ECCP1 PWM Output Channel A

        T2CONbits.TMR2ON = 1;		// Turn it on
    }

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

	print_ack();
}

// RM command
// For Run Motor - allows completely independant running of the two stepper motors
void parse_RM_packet(void)
{
	
	
}

// QM command
// For Query Motor - returns the current status of each motor
// QM takes no parameters, so usage is just QM<CR>
// QM returns:
// QM,<CommandExecutingStatus>,<Motor1Satus>,<Motor2Status><CR>
// where:
//   <CommandExecutingStatus>: 0 if no 'motion command' is excuting, > 0 if some 'motion command' is executing
//   <Motor1Status>: 0 if motor 1 is idle, 1 if motor is moving
//   <Motor2Status>: 0 if motor 2 is idle, 1 if motor is moving
void parse_QM_packet(void)
{
    UINT8 CommandExecuting = 0;
    UINT8 Motor1Running = 0;
    UINT8 Motor2Running = 0;
    MoveCommandType LocalCommand;

    // Need to turn off high priorioty interrupts breifly here to read out value that ISR uses
    INTCONbits.GIEH = 0;	// Turn high priority interrupts off

    // Make a local copy of the things we care about
    LocalCommand = CurrentCommand;

    // Re-enable interrupts
    INTCONbits.GIEH = 1;	// Turn high priority interrupts on

    // Create our output values to print back to the PC
    if (LocalCommand.DelayCounter != 0) {
        CommandExecuting = 1;
    }
    if (LocalCommand.Command != COMMAND_NONE) {
        CommandExecuting = 1;
    }
    if (FIFOEmpty == FALSE) {
        CommandExecuting = 1;
    }
    if (CommandExecuting && LocalCommand.StepsCounter[0] != 0) {
        Motor1Running = 1;
    }
    if (CommandExecuting && LocalCommand.StepsCounter[1] != 0) {
        Motor2Running = 1;
    }

	printf((far ROM char *)"QM,%i,%i,%i\n\r", CommandExecuting, Motor1Running, Motor2Running);
}
