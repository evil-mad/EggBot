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

#ifndef HARDWARE_PROFILE_EBB_V12_H
#define HARDWARE_PROFILE_EBB_V12_H

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

    //#define USE_USB_BUS_SENSE_IO
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
	/* On EBB v12, LED1 (USB) = RD3, LED2 (USR) = RD2, SW = RA7			*/
	#define mInitAllLEDs()      LATDbits.LATD3 = 0; LATDbits.LATD2 = 0; TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;
	#define mLED_1              LATDbits.LATD3
	#define mLED_2              LATDbits.LATD2

	/** S W I T C H *****************************************************/
	#define mInitSwitch()		TRISAbits.TRISA7 = INPUT_PIN;
	#define swProgram			PORTAbits.RA7

	/** R E F   A N A L O G   I N P U T *********************************/
	#define RefRA0_IO_TRIS		TRISAbits.TRISA0

	/** P E N   U P  D O W N ********************************************/
	#define PenUpDownIO			LATBbits.LATB4
	#define PenUpDownIO_TRIS	TRISBbits.TRISB4

	/** D R I V E R   E N A B L E ***************************************/
	#define Enable1IO			LATEbits.LATE0
	#define Enable1IO_TRIS		TRISEbits.TRISE0
	#define Enable2IO			LATDbits.LATD6
	#define Enable2IO_TRIS		TRISDbits.TRISD6
/// TODO: Where should these go?
	#define Enable1AltIO		LATEbits.LATE0
	#define Enable1AltIO_TRIS	TRISEbits.TRISE0
	#define Enable2AltIO		LATDbits.LATD6
	#define Enable2AltIO_TRIS	TRISDbits.TRISD6

	/** D R I V E R   M I C R O S T E P *********************************/
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
/// TODO: What should these be?
	#define Step1AltIO			LATCbits.LATC1
	#define Step1AltIO_TRIS		TRISCbits.TRISC1
	#define Dir1AltIO			LATDbits.LATD7
	#define Dir1AltIO_TRIS		TRISDbits.TRISD7
	#define Step2AltIO			LATDbits.LATD4
	#define Step2AltIO_TRIS		TRISDbits.TRISD4
	#define Dir2AltIO			LATDbits.LATD5
	#define Dir2AltIO_TRIS		TRISDbits.TRISD5
    
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
	
#endif  //HARDWARE_PROFILE_EBB_V12_H
