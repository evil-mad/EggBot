/********************************************************************
 FileName:     	HardwareProfile - PICDEM FSUSB.h
 Dependencies:  See INCLUDES section
 Processor:     PIC18 USB Microcontrollers
 Hardware:      PICDEM FSUSB
 Compiler:      Microchip C18
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
********************************************************************/

#ifndef HARDWARE_PROFILE_EBB_V10_H
#define HARDWARE_PROFILE_EBB_V10_H

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //The PICDEM FS USB Demo Board platform supports the USE_SELF_POWER_SENSE_IO
    //and USE_USB_BUS_SENSE_IO features.  Uncomment the below line(s) if
    //it is desireable to use one or both of the features.
    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTAbits.RA2
    #else
    #define self_power          1
    #endif

    #define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISCbits.TRISC7    // Input
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTCbits.RC7
    #else
    #define USB_BUS_SENSE       1
    #endif


    //Uncomment the following line to make the output HEX of this  
    //  project work with the MCHPUSB Bootloader    
    //#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER
	
    //Uncomment the following line to make the output HEX of this 
    //  project work with the HID Bootloader
    #define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER		

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    #define CLOCK_FREQ 48000000

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
	
#endif  //HARDWARE_PROFILE_EBB_V10_H
