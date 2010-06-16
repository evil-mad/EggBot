// Versions:
// 1.8 - 
// 1.8.1 5/19/10 - Only change is to recompile with Microchip USB Stack v2.7
// 1.8.2 5/31/10 - Only change is to change name in USB enumeration string to Ei Bot Board - using new PID for SchmalzHaus
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

#include <p18cxxx.h>
#include <usart.h>
#include <stdio.h>
#include <ctype.h>
#include <delays.h>
#include "Usb\usb.h"
#include "Usb\usb_function_cdc.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "Usb\usb_device.h"
#include "HardwareProfile.h"
#include "ubw.h"
#include "ebb.h"
#include "delays.h"
#include "ebb_demo.h"
/// TODO: Fix this based upon type of CPU
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
	#include "RCServo2.h"
#endif

// Reload value for TIMER1
// We need a 25KHz ISR to fire, so we take Fosc (48Mhz), devide by 4
// (normal CPU instruction rate of Fosc/4), then use the TIMER1 prescaler
// to divide by 4 again. Then we use a reload value of 120 to give us
// a rate of 48MHz/4/4/120 = 25KHz.
#define TIMER1_L_RELOAD (255 - 113)
#define TIMER1_H_RELOAD (255)

// This is the value that gets multiplied by Steps/Duration to compute
// the StepAdd values.
#define OVERFLOW_MUL	(0x8000 / 25)

#define MAX_RC_DURATION 11890

#if defined(BOARD_EBB_V10)
	#define DIR1_BIT	(0x80)
	#define STEP1_BIT	(0x40)
	#define DIR2_BIT	(0x20)
	#define STEP2_BIT	(0x10)
	#define DIR3_BIT	(0x08)
	#define STEP3_BIT	(0x04)
	#define DIR4_BIT	(0x02)
	#define STEP4_BIT	(0x01)
#elif defined(BOARD_EBB_V11)
	#define STEP1_BIT	(0x01)
	#define DIR1_BIT	(0x02)
	#define STEP2_BIT	(0x04)
	#define DIR2_BIT	(0x08)
#elif defined(BOARD_EBB_V12)
	#define STEP1_BIT	(0x01)
	#define DIR1_BIT	(0x02)
	#define STEP2_BIT	(0x04)
	#define DIR2_BIT	(0x08)
#elif defined(BOARD_EBB_V13) 
/// TODO: Edit these
	#define STEP1_BIT	(0x01)
	#define DIR1_BIT	(0x02)
	#define STEP2_BIT	(0x04)
	#define DIR2_BIT	(0x08)
#elif defined(BOARD_UBW)
	#define DIR1_BIT	(0x02)
	#define STEP1_BIT	(0x01)
	#define DIR2_BIT	(0x08)
	#define STEP2_BIT	(0x04)
	#define DIR3_BIT	(0x20)
	#define STEP3_BIT	(0x10)
	#define DIR4_BIT	(0x80)
	#define STEP4_BIT	(0x40)
#endif

typedef enum
{
	COMMAND_NONE = 0,
	COMMAND_MOVE,
	COMMAND_DELAY,
	COMMAND_PEN_UP,
	COMMAND_PEN_DOWN
} CommandType;

typedef enum
{
	SOLENOID_OFF = 0,
	SOLENOID_ON,
	SOLENOID_PWM
} SolenoidStateType;

// LOCAL FUNCTIONS
static void process_SM(
	unsigned int Duration, 
	signed int A1Stp, 
	signed int A2Stp, 
	signed int A3Stp, 
	signed int A4Stp
);

#pragma udata access fast_vars
// Working registers
static near unsigned int StepAcc[4];
static near signed int StepAdd[4];
static near unsigned int StepsCounter[4];
static near unsigned char DirBits;
static near unsigned char OutByte;
static near unsigned char TookStep;
static near unsigned char AllDone;
static near unsigned char i;
near unsigned char NextReady;
static near CommandType Command;

#pragma udata
// ToLoad registers
static signed int ToLoadStepAdd[4];
static unsigned int ToLoadStepsCounter[4];
static unsigned char ToLoadDirBits;
static CommandType ToLoadCommand;
static unsigned short ToLoadDelayCounter;
static unsigned short DelayCounter;
unsigned int DemoModeActive;
unsigned int comd_counter;
static SolenoidStateType SolenoidState;
static unsigned int SolenoidDelay;
static unsigned char UseBuiltInDrivers;
static unsigned char UseServoForUpDown;
static unsigned int g_servo_max;
static unsigned int g_servo_min;
static PenStateType PenState;
static unsigned long NodeCount;
static char Layer;
static BOOL ButtonPushed;

// ISR
// PORTB is the step and direction port 
// RB0 = Step4
// RB1 = Dir4
// RB2 = Step3
// RB3 = Dir3
// RB4 = Step2
// RB5 = Dir2
// RB6 = Step1
// RB7 = Dir1
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

	if (PIR1bits.TMR1IF)
	{
		// Clear the interrupt 
		PIR1bits.TMR1IF = 0;
		TMR1L = TIMER1_L_RELOAD;	// Set to 120 for 25KHz ISR fire
		TMR1H = TIMER1_H_RELOAD;	//

		OutByte = DirBits;
		TookStep = FALSE;
		AllDone = TRUE;

		if (Command == COMMAND_DELAY)
		{
			if (DelayCounter)
			{
				DelayCounter--;
			}
			if (DelayCounter)
			{
				AllDone = FALSE;
			}
		}
		else if (Command == COMMAND_MOVE)
		{
			// Only output DIR bits if we are actually doing something
			if (StepsCounter[0] || StepsCounter[1] || StepsCounter[2] || StepsCounter[3])
			{
				// Always output direction bits early so they're ready when we step
#if defined(BOARD_UBW) || defined(BOARD_EBB_V10)
				if (UseBuiltInDrivers)
				{
					PORTB = DirBits;
				}
				else
				{
					PORTC = DirBits;
				}
#elif defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
				if (UseBuiltInDrivers)
				{
					if (DirBits & DIR1_BIT)
					{
						Dir1IO = 1;
					}
					else
					{
						Dir1IO = 0;
					}	
					if (DirBits & DIR2_BIT)
					{
						Dir2IO = 1;
					}
					else
					{
						Dir2IO = 0;
					}	
				}
				else
				{
					if (DirBits & DIR1_BIT)
					{
						Dir1AltIO = 1;
					}
					else
					{
						Dir1AltIO = 0;
					}	
					if (DirBits & DIR2_BIT)
					{
						Dir2AltIO = 1;
					}
					else
					{
						Dir2AltIO = 0;
					}	
				}
#endif

				// Only do this if there are steps left to take
				if (StepsCounter[0])
				{
					StepAcc[0] = StepAcc[0] + StepAdd[0];
					if (StepAcc[0] > 0x8000)
					{
						StepAcc[0] = StepAcc[0] - 0x8000;
						OutByte = OutByte | STEP1_BIT;
						TookStep = TRUE;
						StepsCounter[0]--;
					}
					AllDone = FALSE;
				}
				if (StepsCounter[1])
				{
					StepAcc[1] = StepAcc[1] + StepAdd[1];
					if (StepAcc[1] > 0x8000)
					{
						StepAcc[1] = StepAcc[1] - 0x8000;
						OutByte = OutByte | STEP2_BIT;
						TookStep = TRUE;
						StepsCounter[1]--;
					}
					AllDone = FALSE;
				}
#if defined(BOARD_UBW) || defined(BOARD_EBB_V10)
				if (StepsCounter[2])
				{
					StepAcc[2] = StepAcc[2] + StepAdd[2];
					if (StepAcc[2] > 0x8000)
					{
						StepAcc[2] = StepAcc[2] - 0x8000;
						OutByte = OutByte | STEP3_BIT;
						TookStep = TRUE;
						StepsCounter[2]--;
					}
					AllDone = FALSE;
				}
				if (StepsCounter[3])
				{
					StepAcc[3] = StepAcc[3] + StepAdd[3];
					if (StepAcc[3] > 0x8000)
					{
						StepAcc[3] = StepAcc[3] - 0x8000;
						OutByte = OutByte | STEP4_BIT;
						TookStep = TRUE;
						StepsCounter[3]--;
					}
					AllDone = FALSE;
				}
#endif	

				if (TookStep)
				{
#if defined(BOARD_UBW) || defined(BOARD_EBB_V10)
					if (UseBuiltInDrivers)
					{
						PORTB = OutByte;
					}
					else
					{
						PORTC = OutByte;
					}
#elif defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
					if (UseBuiltInDrivers)
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
					else
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
#endif
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
#if defined(BOARD_UBW) || defined(BOARD_EBB_V10)
					if (UseBuiltInDrivers)
					{
						PORTB = DirBits;
					}
					else
					{
						PORTC = DirBits;
					}
#elif defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
					if (UseBuiltInDrivers)
					{
						Step1IO = 0;
						Step2IO = 0;
					}
					else
					{
						Step1AltIO = 0;
						Step2AltIO = 0;
					}
#endif
				}
			}
		}
		// Check to see if we should change the state of the pen
		else if (Command == COMMAND_PEN_UP)
		{
			if (gUseRCServo1)
			{
				g_RC_value[9] = g_servo_min;
			}
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
			else if (gUseRCServo2)
			{
				// This code below is the meat of the Process_S2() function
				// We have to manually write it in here rather than calling
				// the function because a real function inside the ISR 
				// causes the compiler to generate enormous amounts of setup/teardown
				// code and things run way too slowly.
				// Process_S2(1, g_servo2_min, 4, g_servo2_rate_up);
				gRC2Rate[0] = g_servo2_rate_up;
				gRC2Target[0] = g_servo2_min;
				gRC2Pin[0] = 4;
				if (gRC2Value[0] == 0)
				{
					gRC2Value[0] = g_servo2_min;
				}
			}
#endif
			else
			{
				SolenoidState = SOLENOID_OFF;
			}		
			PenUpDownIO = 0;

			if (DelayCounter)
			{
				DelayCounter--;
			}
			if (DelayCounter)
			{
				AllDone = FALSE;
			}
			PenState = PEN_UP;
		}
		else if (Command == COMMAND_PEN_DOWN)
		{
			if (gUseRCServo1)
			{
				g_RC_value[9] = g_servo_max;
			}
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
			else if (gUseRCServo2)
			{
				// This code below is the meat of the Process_S2() function
				// We have to manually write it in here rather than calling
				// the function because a real function inside the ISR 
				// causes the compiler to generate enormous amounts of setup/teardown
				// code and things run way too slowly.
				// Process_S2(1, g_servo2_max, 4, g_servo2_rate_down);
				gRC2Rate[0] = g_servo2_rate_down;
				gRC2Target[0] = g_servo2_max;
				gRC2Pin[0] = 4;
				if (gRC2Value[0] == 0)
				{
					gRC2Value[0] = g_servo2_max;
				}
			}
#endif
			else
			{
				SolenoidState = SOLENOID_ON;
			}
			PenUpDownIO = 1;

			if (DelayCounter)
			{
				DelayCounter--;
			}
			if (DelayCounter)
			{
				AllDone = FALSE;
			}
			PenState = PEN_DOWN;
		}
		else
		{
			
		}
	
		// Load the next move set in
		if (AllDone)
		{
			if (Command == COMMAND_MOVE)
			{
				NodeCount++;
			}
			Command = COMMAND_NONE;
			if (NextReady)
			{
				for (i=0; i<4; i++)
				{
					StepAdd[i] = ToLoadStepAdd[i];
					StepsCounter[i] = ToLoadStepsCounter[i];
				}
				DirBits = ToLoadDirBits;
				Command = ToLoadCommand;
				DelayCounter = ToLoadDelayCounter;
				NextReady = FALSE;
			}
		}
		

		// Check for button being pushed
		if (!swProgram)
		{
			ButtonPushed = TRUE;
		}
	}
}

// Init code
void EBB_Init(void)
{
	StepAdd[0] = 1;
	StepAdd[1] = 1;
	StepAdd[2] = 1;
	StepAdd[3] = 1;
	StepsCounter[0] = 0;
	StepsCounter[1] = 0;
	StepsCounter[2] = 0;
	StepsCounter[3] = 0;
	NextReady = FALSE;

#if defined(BOARD_EBB_V10)
	// Allow access to our bits in T1CON
	WDTCONbits.ADSHR = 0;
#endif

#if defined(BOARD_EBB_V10) || defined(BOARD_UBW)
	// Set up TMR1 for our 25KHz High ISR for stepping
	T1CONbits.RD16 = 0; 	// Set 8 bit mode
	T1CONbits.T1RUN = 0; 	// System clocked from other than T1
	T1CONbits.T1CKPS1 = 1; 	// Use 1:4 Prescale value
	T1CONbits.T1CKPS0 = 0;
	T1CONbits.T1OSCEN = 0; 	// Don't use external osc
	T1CONbits.T1SYNC = 0;
	T1CONbits.TMR1CS = 0; 	// Use Fosc/4 to clock timer
#elif defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
	// Set up TMR1 for our 25KHz High ISR for stepping
	T1CONbits.RD16 = 0; 	// Set 8 bit mode
	T1CONbits.TMR1CS1 = 0; 	// System clocked from Fosc/4
	T1CONbits.TMR1CS0 = 0;
	T1CONbits.T1CKPS1 = 1; 	// Use 1:4 Prescale value
	T1CONbits.T1CKPS0 = 0;
	T1CONbits.T1OSCEN = 0; 	// Don't use external osc
	T1CONbits.T1SYNC = 0;
#endif
	TMR1L = TIMER1_L_RELOAD;	// Set to 120 for 25KHz ISR fire
	TMR1H = TIMER1_H_RELOAD;	// 

	T1CONbits.TMR1ON = 1; // Turn the timer on

	IPR1bits.TMR1IP = 1;	// Use high priority interrupt
	PIR1bits.TMR1IF = 0;	// Clear the interrupt
	PIE1bits.TMR1IE = 1;	// Turn on the interrupt

	// For debugging
#if defined(BOARD_EBB_V10)
	PORTA = 0;
	TRISA = 0;
	PORTB = 0;
	TRISB = 0;
	PORTC = 0;		// Start out low
	TRISC = 0;		// Make portC
	PORTD = 0;
	TRISD = 0;
	PORTE = 0x16;
	TRISE = 0;
	PORTF = 0xA4;
	TRISF = 0x40;	// RF6 needs to be an input
	PORTG = 0;
	TRISG = 0;	
	PORTH = 0;
	TRISH = 0;
	PORTJ = 0;
	TRISJ = 0;

	Enable1IO = ENABLE_MOTOR;
	Enable1IO_TRIS = OUTPUT_PIN;
	Enable2IO = ENABLE_MOTOR;
	Enable2IO_TRIS = OUTPUT_PIN;
	Enable3IO = ENABLE_MOTOR;
	Enable3IO_TRIS = OUTPUT_PIN;
	Enable4IO = ENABLE_MOTOR;
	Enable4IO_TRIS = OUTPUT_PIN;

	Sleep1IO = 1;
	Sleep2IO = 1;
	Sleep3IO = 1;
	Sleep4IO = 1;

	MS1_1IO = 1;
	MS2_1IO = 1;
	MS1_2IO = 1;
	MS2_2IO = 1;
	MS1_3IO = 1;
	MS2_3IO = 1;
	MS1_4IO = 1;
	MS2_4IO = 1;

#elif defined(BOARD_EBB_V11)
	PORTA = 0;
	TRISA = 0x81;	// Bit0 and Bit7 needs to be an input (RA0 is REF analog input)
	PORTB = 0;
	TRISB = 2;		// Bit1 is our StartDemo switch
	INTCON2bits.RBPU = 0;	// Turn on weak-pull ups for port B
	PORTC = 0;		// Start out low
	TRISC = 0x80;	// Make portC output execpt for PortC bit 7, USB bus sense
	PORTD = 0;
	TRISD = 0;
	PORTE = 0;
	TRISE = 0;	
	ANCON0 = 0xFE;	// Let AN0 (RA0) be an analog input
	ANCON1 = 0x1F;	// Set all the rest to digital I/O

	Enable1IO = ENABLE_MOTOR;
	Enable2IO = ENABLE_MOTOR;
	MS1_1IO = 1;
	MS2_1IO = 1;
	MS1_2IO	= 1;
	MS2_2IO	= 1;
	Sleep1IO = 1;	
	Sleep2IO = 1;
	Step1IO	= 0;
	Dir1IO = 0;
	Step2IO	= 0;	
	Dir2IO = 0;	

#elif defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
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
	ANCON0 = 0xFE;	// Let AN0 (RA0) be an analog input
	ANCON1 = 0x1F;	// Set all the rest to digital I/O

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

#elif defined(BOARD_UBW)
	PORTA = 0;
	TRISA = 0;
	PORTB = 0;
	TRISB = 0;
	PORTC = 0;		// Start out low
	TRISC = 0;		// Make portC outputs
#endif

	// Set up pen up/down direction as output
	PenUpDownIO = 0;
	PenUpDownIO_TRIS = OUTPUT_PIN;

	SolenoidState = SOLENOID_ON;
	UseBuiltInDrivers = TRUE;
	gUseRCServo1 = FALSE;
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
	gUseRCServo2 = TRUE;
#endif
	PenState = PEN_UP;
	Layer = 0;
	NodeCount = 0;
	ButtonPushed = FALSE;
}

// Stepper (mode) Configure command
// SC,1,0<CR> will use solenoid output for pen up/down (default)
// SC,1,1<CR> will use servo on RB1 for pen up/down
// SC,1,2<CR> will use servo on RB1 for pen up/down, but with ECCP2 (PWM) in hardware
// SC,2,0<CR> will use built-in stepper driver chips (default)
// SC,2,1<CR> will use the following pins for stepper driver outputs (EBB_V11)
//		ENABLE1 = RA5
//		ENABLE2 = RB5
//		STEP1 = RD1
//		DIR1 = RD0
//		STEP2 = RC2
//		DIR2 = RC0
// SC,4,<servo2_min><CR> will set <servo2_min> as the minimum value for the servo (1 to 65535)
// SC,5,<servo2_max><CR> will set <servo2_max> as the maximum value for the servo (1 to 65535)
// SC,6,<servo_min><CR> will set <servo_min> as the minimum value for the servo (1 to 11890)
// SC,7,<servo_max><CR> will set <servo_max> as the maximum value for the servo (1 to 11890)
// SC,8,<servo2_slots><CR> sets the number of slots for the servo2 system (1 to 24)
// SC,9,<servo2_slotMS><CR> sets the number of ms in duration for each slot (1 to 6)
// SC,10,<servo2_rate><CR> sets the rate of change for the servo (both up and down)
// SC,11,<servo2_rate><CR> sets the pen up speed
// SC,12,<servo2_rate><CR> sets the pen down speed
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

	// Check for command to use servo rather than solenoid (we'll leave
	// the solenoid on too)
	if (Para1 == 1)
	{
		if (Para2 == 0)
		{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
			gUseRCServo2 = FALSE;
#endif
			gUseRCServo1 = FALSE;
			// Turn off RC Servo pulses on RB1
			g_RC_value[9] = 0;
		}
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)					// NOTE: Only VBB V1.1 and above have this RC Servo option
		else if (Para2 == 1)
		{
			gUseRCServo1 = TRUE;
			gUseRCServo2 = FALSE;
			TRISBbits.TRISB1 = 0; 	// RB1 needs to be an output
			
			// We're going to do the work here of an 'RC' command, and set the RC servo to one
			// of it limits.
			// Store the new RC time value
//			g_RC_value[9] = (65535 - (g_servo_min + 45));			
			// Only set this state if we are off - if we are already running on 
			// this pin, then the new value will be picked up next time around (19ms)
			if (kOFF == g_RC_state[9])
			{
				g_RC_state[9] = kWAITING;
			}
		}
		else
		{
			gUseRCServo1 = FALSE;
			TRISBbits.TRISB1 = 0; 	// RB1 needs to be an output
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
			Process_S2(1, g_servo2_min, 4, g_servo2_rate_up);
#endif
			process_SP(PEN_UP, 0);			// Start servo up 
		}
#endif
	}
	// Check for command to switch between built-in drivers and external drivers
	else if (Para1 == 2)
	{
		if (Para2 == 0)
		{
			UseBuiltInDrivers = TRUE;
			// Initalize the alternate driver I/O ports
			Dir1AltIO_TRIS = 0;
			Dir2AltIO_TRIS = 0;
			Step1AltIO_TRIS = 0;
			Step2AltIO_TRIS = 0;
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
			Enable1AltIO_TRIS = 0;
			Enable2AltIO_TRIS = 0;
#endif
		}
		else
		{
			UseBuiltInDrivers = FALSE;
		}
	}
	// Set <min_servo> for Servo2 method
	else if (Para1 == 4)
	{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		g_servo2_min = Para2;
#endif
	}
	// Set <max_servo> for Servo2
	else if (Para1 == 5)
	{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		g_servo2_max = Para2;
#endif
	}
	// Set <min_servo>
	else if (Para1 == 6)
	{
		if (Para2 > MAX_RC_DURATION)
		{
			Para2 = MAX_RC_DURATION;
		}
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		g_servo_min = Para2;
#endif
	}
	// Set <max_servo>
	else if (Para1 == 7)
	{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		if (Para2 > MAX_RC_DURATION)
		{
			Para2 = MAX_RC_DURATION;
		}
		g_servo_max = Para2;
#endif
	}
	// Set <gRC2Slots>
	else if (Para1 == 8)
	{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		if (Para2 > MAX_RC2_SERVOS)
		{
			Para2 = MAX_RC2_SERVOS;
		}
		gRC2Slots = Para2;
#endif
	}
	else if (Para1 == 9)
	{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		if (Para2 > 6)
		{
			Para2 = 6;
		}
		gRC2SlotMS = Para2;
#endif
	}
	else if (Para1 == 10)
	{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		g_servo2_rate_up = Para2;
		g_servo2_rate_down = Para2;
#endif
	}
	else if (Para1 == 11)
	{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		g_servo2_rate_up = Para2;
#endif
	}
	else if (Para1 == 12)
	{
#if defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		g_servo2_rate_down = Para2;
#endif
	}
	print_ack();
}

// The Stepper Motor command
// Usage: SM,<move_duration>,<axis1_steps>,<axis2_steps>,<axis3_steps>,<axis4_steps><CR>
// <move_duration> is a number from 1 to 65535, indiciating the number of milliseconds this move should take
// <axisX_steps> is a signed 16 bit number indicating how many steps (and what direction) the axis should take
// NOTE1: <axis2_steps>, <axis3_steps> and <axis4_steps> are optional and can be left off
// If the EBB can not make the move in the speicified time, it will take as long as it needs to at max speed
// i.e. SM,1,1000 will not produce 1000steps in 1ms. Instead, it will take 40ms (25KHz max step rate)
// NOTE2: If you specify zero steps for the axies, then you effectively create a delay. Use for small
// pauses before raising or lowering the pen, for example.
void parse_SM_packet (void)
{
	unsigned int Duration;
	signed int A1Steps = 0, A2Steps = 0, A3Steps = 0, A4Steps = 0;

	// Extract each of the values.
	extract_number (kUINT, &Duration, kREQUIRED);
	extract_number (kINT, &A1Steps, kREQUIRED);
	extract_number (kINT, &A2Steps, kOPTIONAL);
#if defined(BOARD_UBW) || defined(BOARD_EBB_V10)
	extract_number (kINT, &A3Steps, kOPTIONAL);
	extract_number (kINT, &A4Steps, kOPTIONAL);
#endif
	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}
	process_SM(Duration, A1Steps, A2Steps, A3Steps, A4Steps);

	print_ack();
}

static void process_SM(
	unsigned int Duration, 
	signed int A1Stp, 
	signed int A2Stp, 
	signed int A3Stp, 
	signed int A4Stp
)
{
	// Trial: Spin here until there's space in the fifo
	while(NextReady)
	;

	// Check for delay
	if (A1Stp == 0 && A2Stp == 0 && A3Stp == 0 && A4Stp == 0)
	{
		ToLoadCommand = COMMAND_DELAY;
		ToLoadDelayCounter = 25 * Duration;
	}
	else
	{
		ToLoadDelayCounter = 1;
		ToLoadDirBits = 0;
		
		// Always enable both motors when we want to move them
#if defined(BOARD_EBB_V10) || defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
		Enable1IO = ENABLE_MOTOR;
		Enable2IO = ENABLE_MOTOR;
#if defined(BOARD_EBB_V10)
		Enable3IO = ENABLE_MOTOR;
		Enable4IO = ENABLE_MOTOR;
#endif
#endif

		// First, set the direction bits
		if (A1Stp < 0)
		{
			ToLoadDirBits = ToLoadDirBits | DIR1_BIT;
			A1Stp = -A1Stp;
		}
		if (A2Stp < 0)
		{
			ToLoadDirBits = ToLoadDirBits | DIR2_BIT;
			A2Stp = -A2Stp;
		}
#if defined(BOARD_UBW) || defined(BOARD_EBB_V10)
		if (A3Stp < 0)
		{
			ToLoadDirBits = ToLoadDirBits | DIR3_BIT;
			A3Stp = -A3Stp;
		}
		if (A4Stp < 0)
		{
			ToLoadDirBits = ToLoadDirBits | DIR4_BIT;
			A4Stp = -A4Stp;
		}
#endif	
		// Range check Steps/Duration
//		if (
//			(A1Steps / Duration < 25)
//			||
//			(A2Steps / Duration < 25)
//			||
//			(A3Steps / Duration < 25)
//			||
//			(A4Steps / Duration < 25)
//		)
//		{
//				bitset (error_byte, kERROR_BYTE_STEPS_TO_FAST);
//				return;			
//		}	
//		else
//		{
			// To compute StepAdd values from Duration,
			ToLoadStepAdd[0] = (unsigned int)
									(
										((unsigned long)0x8000 * (unsigned long)A1Stp)
										/
										((unsigned long)25 * (unsigned long)Duration)
									) + 1;
			ToLoadStepsCounter[0] = A1Stp;
			ToLoadStepAdd[1] = (unsigned int)
									(
										((unsigned long)0x8000 * (unsigned long)A2Stp)
										/
										((unsigned long)25 * (unsigned long)Duration)
									) + 1;
			ToLoadStepsCounter[1] = A2Stp;
#if defined(BOARD_UBW) || defined(BOARD_EBB_V10)
			ToLoadStepAdd[2] = (unsigned int)
									(
										((unsigned long)0x8000 * (unsigned long)A3Stp)
										/
										((unsigned long)25 * (unsigned long)Duration)
									) + 1;
			ToLoadStepsCounter[2] = A3Stp;
			ToLoadStepAdd[3] = (unsigned int)
									(
										((unsigned long)0x8000 * (unsigned long)A4Stp)
										/
										((unsigned long)25 * (unsigned long)Duration)
									) + 1;
			ToLoadStepsCounter[3] = A4Stp;
#endif
			ToLoadCommand = COMMAND_MOVE;

//printf("SA:%5d S:%4d SA:%5d S:%4d\n\r", ToLoadStepAdd[0], ToLoadStepsCounter[0], ToLoadStepAdd[1], ToLoadStepsCounter[1]);

//		}
	}
		
	NextReady = TRUE;

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
// Usage: TP<CR>
// Returns: OK<CR>
// Just toggles state of pen arm
void parse_TP_packet(void)
{
	unsigned short CommandDuration = 500;

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
// Usage: SP,<1,0>,<Duration><CR>
void parse_SP_packet(void)
{
	unsigned char State = 0;
	unsigned short CommandDuration = 0;

	// Extract each of the values.
	extract_number (kUCHAR, &State, kREQUIRED);
	extract_number (kUINT, &CommandDuration, kOPTIONAL);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	process_SP(State, CommandDuration);

	print_ack();
}

void process_SP(SolenoidStateType NewState, unsigned short CommandDuration)
{	
	// Trial: Spin here until there's space in the fifo
	while(NextReady)
	;

	if (NewState == PEN_UP)
	{
		ToLoadCommand = COMMAND_PEN_UP;
	}
	else
	{
		ToLoadCommand = COMMAND_PEN_DOWN;
	}
	ToLoadDelayCounter = CommandDuration * 25;

	NextReady = TRUE;	
}

// Enable Motor
// Usage: EM,<EnableAxis1>,<EnableAxis2>,<EnableAxis3>,<EnableAxis4><CR>
// Everything afer EnableAxis1 is optional
// Each parameter can have a value of
//		0 to disable that motor driver
// FOR OLD DRIVER CHIP
//		1 to enable the driver in 1/8th step mode
//		2 to enable the driver in 1/4 step mode
//		3 to enable the driver in 1/2 step mode
//		4 to enable the driver in full step mode
// FOR NEW DRIVER CHIP (only first parameter applies, and it applies to both drivers)
//		1 to enable the driver in 1/16th step mode
//		2 to enable the driver in 1/8 step mode
//		3 to enable the driver in 1/4 step mode
//		4 to enable the driver in 1/2 step mode
//		5 to enable the driver in full step mode
// If you disable a motor, it goes 'limp' (we clear the ENABLE pin on that motor's
// driver chip)
void parse_EM_packet(void)
{
#if defined(BOARD_EBB_V10) || defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
	unsigned char EA1, EA2, EA3, EA4;
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
		if (UseBuiltInDrivers)
		{
			if (EA1 > 0)
			{
				Enable1IO = ENABLE_MOTOR;
#if defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13)
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
#else
				if (EA1 == 1)
				{
					MS1_1IO = 1;
					MS2_1IO = 1;
				}
				if (EA1 == 2)
				{
					MS1_1IO = 0;
					MS2_1IO = 1;
				}
				if (EA1 == 3)
				{
					MS1_1IO = 1;
					MS2_1IO = 0;
				}
				if (EA1 == 4)
				{
					MS1_1IO = 0;
					MS2_1IO = 0;
				}
#endif
			}
			else
			{
				Enable1IO = DISABLE_MOTOR;
			}
		}
		else
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
		if (UseBuiltInDrivers)
		{
			if (EA2 > 0)
			{
				Enable2IO = ENABLE_MOTOR;
#if !(defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13))
/// TODO: fix this based upon type of driver chip
				if (EA2 == 1)
				{
					MS1_2IO = 1;
					MS2_2IO = 1;
				}
				if (EA2 == 2)
				{
					MS1_2IO = 0;
					MS2_2IO = 1;
				}
				if (EA2 == 3)
				{
					MS1_2IO = 1;
					MS2_2IO = 0;
				}
				if (EA2 == 4)
				{
					MS1_2IO = 0;
					MS2_2IO = 0;
				}				
#endif
			}
			else
			{
				Enable2IO = DISABLE_MOTOR;
			}
		}
		else
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
#if defined(BOARD_EBB_V10)
	RetVal = extract_number (kUCHAR, &EA3, kOPTIONAL);
	if (kEXTRACT_OK == RetVal)
	{
		// Bail if we got a conversion error
		if (error_byte)
		{
			return;
		}
		if (EA3 > 0)
		{
			Enable3IO = ENABLE_MOTOR;
		}
		else
		{
			Enable3IO = DISABLE_MOTOR;
		}
	}
	RetVal = extract_number (kUCHAR, &EA4, kOPTIONAL);
	if (kEXTRACT_OK == RetVal)
	{
		// Bail if we got a conversion error
		if (error_byte)
		{
			return;
		}
		if (EA4 > 0)
		{
			Enable4IO = ENABLE_MOTOR;
		}
		else
		{
			Enable4IO = DISABLE_MOTOR;
		}
	}
#endif
#endif
	print_ack();
}

// Set Node counter
// Usage: SN,<NewNodeCount><CR>
void parse_SN_packet(void)
{
	unsigned int NewNodeCount = 0;

	// Extract each of the values.
	extract_number (kUINT, &NewNodeCount, kREQUIRED);

	// Bail if we got a conversion error
	if (error_byte)
	{
		return;
	}

	// Just copy it over
	NodeCount = NewNodeCount;

	print_ack();
}

// Query Node counter
// Usage: QN<CR>
// Returns: <NodeCount><CR>
// OK<CR>
void parse_QN_packet(void)
{
	printf ((far rom char*)"%020li\r\n", NodeCount);

	print_ack();
}

// Set Layer
// Usage: SL,<NewLayer><CR>
void parse_SL_packet(void)
{
	// Extract each of the values.
	extract_number (kUINT, &Layer, kREQUIRED);

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
