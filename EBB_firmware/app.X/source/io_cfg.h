/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        io_cfg.h
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
 * Rawin Rojvanit       11/19/04     Original.
 * Brian Schmalz		03/15/06	 Updated for UBW boards.
 ********************************************************************/

/******************************************************************************
 * -io_cfg.h-
 * I/O Configuration File
 * The purpose of this file is to provide a mapping mechanism between
 * pin functions and pin assignments. This provides a layer of abstraction
 * for the firmware code and eases the migration process from one target
 * board design to another.
 *
 *****************************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

/** I N C L U D E S *************************************************/
#include "autofiles\usbcfg.h"

/** T R I S *********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0

/** U S B ***********************************************************/
#define self_power          1

#if defined(BOARD_EBB_V10)
	#define usb_bus_sense       1

	/** L E D ***********************************************************/
	/* On EBB, LED1 = RD0, LED2 = RD1, SW = RD2		 					*/
	#define mInitAllLEDs()      LATD &= 0xFC; TRISD &= 0xFC;
	#define mLED_1              LATDbits.LATD0
	#define mLED_2              LATDbits.LATD1

	/** S W I T C H *****************************************************/
	#define mInitSwitch()		TRISDbits.TRISD2 = 1;
	#define swProgram			PORTDbits.RD2

	/** P E N   U P  D O W N *******************************************/
	#define PenUpDownIO			LATGbits.LATG0
	#define PenUpDownIO_TRIS	TRISGbits.TRISG0

	/** D R I V E R   E N A B L E **************************************/
	#define Enable1IO			LATEbits.LATE0
	#define Enable1IO_TRIS		TRISEbits.TRISE0
	#define Enable2IO			LATEbits.LATE5
	#define Enable2IO_TRIS		TRISEbits.TRISE5
	#define Enable3IO			LATJbits.LATJ2
	#define Enable3IO_TRIS		TRISJbits.TRISJ2
	#define Enable4IO			LATJbits.LATJ4
	#define Enable4IO_TRIS		TRISJbits.TRISJ4
	#define Enable1AltIO		LATCbits.LATC0
	#define Enable1AltIO_TRIS	TRISCbits.TRISC0
	#define Enable2AltIO		LATCbits.LATC1
	#define Enable2AltIO_TRIS	TRISCbits.TRISC1

	/** D R I V E R   M I C R O S T E P ********************************/
	#define MS1_1IO				LATFbits.LATF2
	#define MS1_1IO_TRIS		TRISFbits.TRISF2
	#define MS2_1IO				LATFbits.LATF7
	#define MS2_1IO_TRIS		TRISFbits.TRISF7
	#define MS1_2IO				LATEbits.LATE1
	#define MS1_2IO_TRIS		TRISEbits.TRISE1
	#define MS2_2IO				LATEbits.LATE4
	#define MS2_2IO_TRIS		TRISEbits.TRISE4
	#define MS1_3IO				LATEbits.LATE6
	#define MS1_3IO_TRIS		TRISEbits.TRISE6
	#define MS2_3IO				LATJbits.LATJ1
	#define MS2_3IO_TRIS		TRISJbits.TRISJ1
	#define MS1_4IO				LATJbits.LATJ3
	#define MS1_4IO_TRIS		TRISJbits.TRISJ3
	#define MS2_4IO				LATJbits.LATJ5
	#define MS2_4IO_TRIS		TRISJbits.TRISJ5

	/** D R I V E R   S L E E P ****************************************/
	#define Sleep1IO			LATFbits.LATF5
	#define Sleep2IO			LATEbits.LATE2
	#define Sleep3IO			LATEbits.LATE7
 	#define Sleep4IO			LATJbits.LATJ7

	/** S T E P  A N D  D I R ******************************************/
	#define Step1IO				LATBbits.LATB6
	#define Dir1IO				LATBbits.LATB7
	#define Step2IO				LATBbits.LATB4
	#define Dir2IO				LATBbits.LATB5
	#define Step3IO				LATBbits.LATB2
	#define Dir3IO				LATBbits.LATB3
	#define Step4IO				LATBbits.LATB0
	#define Dir4IO				LATBbits.LATB1

	#define Step1AltIO			LATHbits.LATH6
	#define Step1AltIO_TRIS		TRISHbits.TRISH6
	#define Dir1AltIO			LATHbits.LATH7
	#define Dir1AltIO_TRIS		TRISHbits.TRISH7
	#define Step2AltIO			LATHbits.LATH4
	#define Step2AltIO_TRIS		TRISHbits.TRISH4
	#define Dir2AltIO			LATHbits.LATH5
	#define Dir2AltIO_TRIS		TRISHbits.TRISH5
	#define Step3AltIO			LATHbits.LATH2
	#define Step3AltIO_TRIS		TRISHbits.TRISH2
	#define Dir3AltIO			LATHbits.LATH3
	#define Dir3AltIO_TRIS		TRISHbits.TRISH3
	#define Step4AltIO			LATHbits.LATH0
	#define Step4AltIO_TRIS		TRISHbits.TRISH0
	#define Dir4AltIO			LATHbits.LATH1
	#define Dir4AltIO_TRIS		TRISHbits.TRISH1

#elif defined(BOARD_EBB_V11)
	#define usb_bus_sense       PORTCbits.RC7

	/** L E D ***********************************************************/
	/* On EBB v11, LED1 = RD3, LED2 = RD2, SW = RA7	 					*/
	#define mInitAllLEDs()      LATDbits.LATD3 = 0; LATDbits.LATD2 = 0; TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;
	#define mLED_1              LATDbits.LATD3
	#define mLED_2              LATDbits.LATD2

	/** S W I T C H *****************************************************/
	#define mInitSwitch()		TRISAbits.TRISA7 = 1;
	#define swProgram			PORTAbits.RA7
	#define swStartDemo			PORTBbits.RB1

	/** P E N   U P  D O W N *******************************************/
	#define PenUpDownIO			LATAbits.LATA6
	#define PenUpDownIO_TRIS	TRISAbits.TRISA6

	/** D R I V E R   E N A B L E **************************************/
	#define Enable1IO			LATAbits.LATA2
	#define Enable2IO			LATDbits.LATD4
	#define Enable1AltIO		LATAbits.LATA5
	#define Enable1AltIO_TRIS	TRISAbits.TRISA5
	#define Enable2AltIO		LATBbits.LATB5
	#define Enable2AltIO_TRIS	TRISBbits.TRISB5

	/** D R I V E R   M I C R O S T E P ********************************/
	#define MS1_1IO				LATEbits.LATE1
	#define MS2_1IO				LATAbits.LATA1
	#define MS1_2IO				LATDbits.LATD6
	#define MS2_2IO				LATDbits.LATD5

	/** D R I V E R   S L E E P ****************************************/
	#define Sleep1IO			LATEbits.LATE2
	#define Sleep2IO			LATCbits.LATC1

	/** S T E P  A N D  D I R ******************************************/
	#define Step1IO				LATAbits.LATA3
	#define Dir1IO				LATEbits.LATE0
	#define Step2IO				LATBbits.LATB0
	#define Dir2IO				LATDbits.LATD7

	#define Step1AltIO			LATDbits.LATD1
	#define Step1AltIO_TRIS		TRISDbits.TRISD1
	#define Dir1AltIO			LATDbits.LATD0
	#define Dir1AltIO_TRIS		TRISDbits.TRISD0
	#define Step2AltIO			LATCbits.LATC2
	#define Step2AltIO_TRIS		TRISCbits.TRISC2
	#define Dir2AltIO			LATCbits.LATC0
	#define Dir2AltIO_TRIS		TRISCbits.TRISC0

#elif defined(BOARD_EBB_V12)
	#define usb_bus_sense       PORTCbits.RC7

	/** L E D ***********************************************************/
	/* On EBB v11, LED1 = RD3, LED2 = RD2, SW = RA7		 				*/
	#define mInitAllLEDs()      LATDbits.LATD3 = 0; LATDbits.LATD2 = 0; TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;
	#define mLED_1              LATDbits.LATD3
	#define mLED_2              LATDbits.LATD2

	/** S W I T C H *****************************************************/
	#define mInitSwitch()		TRISAbits.TRISA7 = INPUT_PIN;
	#define swProgram			PORTAbits.RA7
	#define swStartDemo			PORTBbits.RB1

	/** P E N   U P  D O W N *******************************************/
	#define PenUpDownIO			LATBbits.LATB4
	#define PenUpDownIO_TRIS	TRISBbits.TRISB4

	/** D R I V E R   E N A B L E **************************************/
	#define Enable1IO			LATEbits.LATE0
	#define Enable1IO_TRIS		TRISEbits.TRISE0
	#define Enable2IO			LATDbits.LATD6
	#define Enable2IO_TRIS		TRISDbits.TRISD6

	#define Enable1AltIO		LATEbits.LATE0
	#define Enable1AltIO_TRIS	TRISEbits.TRISE0
	#define Enable2AltIO		LATDbits.LATD6
	#define Enable2AltIO_TRIS	TRISDbits.TRISD6

	/** D R I V E R   M I C R O S T E P ********************************/
	#define MS1_IO				LATEbits.LATE2
	#define MS1_IO_TRIS			TRISEbits.TRISE2
	#define MS2_IO				LATEbits.LATE1
	#define MS2_IO_TRIS			TRISEbits.TRISE1
	#define MS3_IO				LATAbits.LATA6
	#define MS3_IO_TRIS			TRISAbits.TRISA6

	/** S T E P  A N D  D I R ******************************************/
	#define Step1IO				LATCbits.LATC1
	#define Step1IO_TRIS		TRISCbits.TRISC1
	#define Dir1IO				LATDbits.LATD7
	#define Dir1IO_TRIS			TRISDbits.TRISD7
	#define Step2IO				LATDbits.LATD4
	#define Step2IO_TRIS		TRISDbits.TRISD4
	#define Dir2IO				LATDbits.LATD5
	#define Dir2IO_TRIS			TRISDbits.TRISD5

	#define Step1AltIO			LATCbits.LATC1
	#define Step1AltIO_TRIS		TRISCbits.TRISC1
	#define Dir1AltIO			LATDbits.LATD7
	#define Dir1AltIO_TRIS		TRISDbits.TRISD7
	#define Step2AltIO			LATDbits.LATD4
	#define Step2AltIO_TRIS		TRISDbits.TRISD4
	#define Dir2AltIO			LATDbits.LATD5
	#define Dir2AltIO_TRIS		TRISDbits.TRISD5

#elif defined(BOARD_EBB_V13)
	#define usb_bus_sense       PORTCbits.RC7

	/** L E D ***********************************************************/
	/* On EBB v13, LED1 (USB) = RD3, LED2 (USR) = RD2, SW = RA7			*/
	#define mInitAllLEDs()      LATDbits.LATD3 = 0; LATDbits.LATD2 = 0; TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;
	#define mLED_1              LATDbits.LATD3
	#define mLED_2              LATDbits.LATD2

	/** S W I T C H *****************************************************/
	#define mInitSwitch()		TRISAbits.TRISA7 = INPUT_PIN;
	#define swProgram			PORTAbits.RA7

	/** P E N   U P  D O W N *******************************************/
	#define PenUpDownIO			LATBbits.LATB4
	#define PenUpDownIO_TRIS	TRISBbits.TRISB4

	/** D R I V E R   E N A B L E **************************************/
	#define Enable1IO			LATEbits.LATE0
	#define Enable1IO_TRIS		TRISEbits.TRISE0
	#define Enable2IO			LATCbits.LATC1
	#define Enable2IO_TRIS		TRISCbits.TRISC1
/// TODO: Where should these go?
	#define Enable1AltIO		LATEbits.LATE0
	#define Enable1AltIO_TRIS	TRISEbits.TRISE0
	#define Enable2AltIO		LATDbits.LATD6
	#define Enable2AltIO_TRIS	TRISDbits.TRISD6

	/** D R I V E R   M I C R O S T E P ********************************/
	#define MS1_IO				LATEbits.LATE2
	#define MS1_IO_TRIS			TRISEbits.TRISE2
	#define MS2_IO				LATEbits.LATE1
	#define MS2_IO_TRIS			TRISEbits.TRISE1
	#define MS3_IO				LATAbits.LATA6
	#define MS3_IO_TRIS			TRISAbits.TRISA6

	/** S T E P  A N D  D I R ******************************************/
	#define Step1IO				LATDbits.LATD6
	#define Step1IO_TRIS		TRISDbits.TRISD6
	#define Dir1IO				LATDbits.LATD7
	#define Dir1IO_TRIS			TRISDbits.TRISD7
	#define Step2IO				LATDbits.LATD4
	#define Step2IO_TRIS		TRISDbits.TRISD4
	#define Dir2IO				LATDbits.LATD5
	#define Dir2IO_TRIS			TRISDbits.TRISD5
/// TODO: What should these be?
	#define Step1AltIO			LATCbits.LATC1
	#define Step1AltIO_TRIS		TRISCbits.TRISC1
	#define Dir1AltIO			LATDbits.LATD7
	#define Dir1AltIO_TRIS		TRISDbits.TRISD7
	#define Step2AltIO			LATDbits.LATD4
	#define Step2AltIO_TRIS		TRISDbits.TRISD4
	#define Dir2AltIO			LATDbits.LATD5
	#define Dir2AltIO_TRIS		TRISDbits.TRISD5

#elif defined(BOARD_UBW)
	#define usb_bus_sense       1

	/** L E D ***********************************************************/
	/* On EBB, LED1 = RD0, LED2 = RD1, SW = RD2		 					*/
	#define mInitAllLEDs()      LATC &= 0xFC; TRISC &= 0xFC;
	#define mLED_1              LATCbits.LATC0
	#define mLED_2              LATCbits.LATC1

	/** S W I T C H *****************************************************/
	#define mInitSwitch()		TRISCbits.TRISC2 = 1;
	#define swProgram			PORTCbits.RC2

	/** P E N   U P  D O W N *******************************************/
	#define PenUpDownIO			LATAbits.LATA0
	#define PenUpDownIO_TRIS	TRISAbits.TRISA0

	/** S T E P  A N D  D I R ******************************************/
	#define Step1IO				LATBbits.LATB0
	#define Dir1IO				LATBbits.LATB1
	#define Step2IO				LATBbits.LATB2
	#define Dir2IO				LATBbits.LATB3
	#define Step1AltIO			LATBbits.LATB0
	#define Step1AltTRIS		TRISBbits.TRISB0
	#define Dir1AltIO			LATBbits.LATB1
	#define Dir1AltTRIS			TRISBbits.TRISB1
	#define Step2AltIO			LATBbits.LATB2
	#define Step2AltTRIS		TRISBbits.TRISB2
	#define Dir2AltIO			LATBbits.LATB3
	#define Dir2AltTRIS			TRISBbits.TRISB3

#else
	#error Please define a BOARD_xxx type.
#endif

/** G E N E R I C ***************************************************/

#define mLED_USB_Toggle()	mLED_1 = !mLED_1;

#define mLED_1_On()         mLED_1 = 1;
#define mLED_2_On()         mLED_2 = 1;

#define mLED_1_Off()        mLED_1 = 0;
#define mLED_2_Off()        mLED_2 = 0;

#define mLED_1_Toggle()     mLED_1 = !mLED_1;
#define mLED_2_Toggle()     mLED_2 = !mLED_2;

#define mLED_Both_Off()     {mLED_1_Off(); mLED_2_Off();}
#define mLED_Both_On()      {mLED_1_On(); mLED_2_On();}
#define mLED_Only_1_On()    {mLED_1_On(); mLED_2_Off();}
#define mLED_Only_2_On()    {mLED_1_Off(); mLED_2_On();}

#endif //IO_CFG_H
