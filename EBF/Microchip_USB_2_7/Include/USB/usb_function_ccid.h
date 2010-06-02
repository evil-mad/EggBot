/*******************************************************************************
  File Information:
    FileName:     	usb_function_ccid.h
    Dependencies:	See INCLUDES section
    Processor:		PIC18 or PIC24 USB Microcontrollers
    Hardware:		The code is natively intended to be used on the following
    				hardware platforms: PICDEM™ FS USB Demo Board,
    				PIC18F87J50 FS USB Plug-In Module, or
    				Explorer 16 + PIC24 USB PIM.  The firmware may be
    				modified for use on other USB platforms by editing the
    				HardwareProfile.h file.
    Complier:  		Microchip C18 (for PIC18) or C30 (for PIC24)
    Company:		Microchip Technology, Inc.

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

    Change History:
     Rev   Date         Description
     0.1   2/June/2009	Draft

  Summary:
    This file contains all of functions, macros, definitions, variables,
    datatypes, etc. that are required for usage with the AUDIO function
    driver. This file should be included in projects that use the Audio
    \function driver.  This file should also be included into the
    usb_descriptors.c file and any other user file that requires access to the
    HID interface.



    This file is located in the "\<Install Directory\>\\Microchip\\Include\\USB"
    directory.

  Description:
    USB AUDIO Function Driver File

    This file contains all of functions, macros, definitions, variables,
    datatypes, etc. that are required for usage with the AUDIO function
    driver. This file should be included in projects that use the AUDIO
    \function driver.  This file should also be included into the
    usb_descriptors.c file and any other user file that requires access to the
    AUDIO interface.

    This file is located in the "\<Install Directory\>\\Microchip\\Include\\USB"
    directory.

    When including this file in a new project, this file can either be
    referenced from the directory in which it was installed or copied
    directly into the user application folder. If the first method is
    chosen to keep the file located in the folder in which it is installed
    then include paths need to be added so that the library and the
    application both know where to reference each others files. If the
    application folder is located in the same folder as the Microchip
    folder (like the current demo folders), then the following include
    paths need to be added to the application's project:

    .

    ..\\..\\Microchip\\Include

    If a different directory structure is used, modify the paths as
    required. An example using absolute paths instead of relative paths
    would be the following:

    C:\\Microchip Solutions\\Microchip\\Include

    C:\\Microchip Solutions\\My Demo Application
*******************************************************************/

/********************************************************************
 Change History:
  Rev    Description
  ----   -----------
  2.6    Initial Release

********************************************************************/

#ifndef CCID_H
	#define CCID_H

/** I N C L U D E S *******************************************************/


/** DEFINITIONS ****************************************************/

/****** CCID Class-Specific Request Codes ************/
#define ABORT  					0x01
#define GET_CLOCK_FREQUENCIES   0x02
#define GET_DATA_RATES  		0x03

// CCID Commands/response
#define PC_to_RDR_IccPowerOn	        0x62
#define PC_to_RDR_IccPowerOff	        0x63
#define PC_to_RDR_GetSlotStatus         0x65
#define PC_to_RDR_XfrBlock		        0x6F
#define PC_to_RDR_GetParameters	        0x6C
#define PC_to_RDR_ResetParameters	    0x6D
#define PC_to_RDR_SetParameters	        0x61
#define PC_to_RDR_Escape                0x6B
#define PC_to_RDR_IccClock              0x6E
#define PC_to_RDR_T0APDU                0x6A
#define PC_to_RDR_Secure                0x69
#define PC_to_RDR_Mechanical            0x71
#define PC_to_RDR_Abort                 0x72
#define PC_to_RDR_SetDataRateAndClockFrequency  0x73

#define RDR_to_PC_DataBlock		    0x80
#define RDR_to_PC_SlotStatus        0x81
#define RDR_to_PC_Parameters	    0x82
#define RDR_to_PC_Escape            0x83
#define RDR_to_PC_DataRateAndClockFrequency 0x84

//CCID Errors
#define CMD_ABORTED                 0xFF
#define ICC_MUTE                    0xFE
#define XFR_PARITY_ERROR            0xFD
#define XFR_OVERRUN                 0xFC
#define HW_ERROR                    0xFB
#define BAD_ATR_TS                  0xF8
#define BAD_ATR_TCK                 0xF7
#define ICC_PROTOCOL_NOT_SUPPORTED  0xF6
#define ICC_CLASS_NOT_SUPPORTED     0xF5
#define PROCEDURE_BYTE_CONFLICT     0xF4
#define DEACTIVATED_PROTOCOL        0xF3
#define BUSY_WITH_AUTO_SEQUENCE     0xF2
#define PIN_TIMEOUT                 0xF0
#define PIN_CANCELLED               0xEF
#define CMD_SLOT_BUSY               0xE0
#define CMD_NOT_SUPPORTED           0x00


/** E X T E R N S ************************************************************/
extern USB_HANDLE lastTransmission;
extern volatile CTRL_TRF_SETUP SetupPkt;
extern ROM BYTE configDescriptor1[];
extern volatile BYTE CtrlTrfData[USB_EP0_BUFF_SIZE];



/** Section: PUBLIC PROTOTYPES **********************************************/
void USBCheckCCIDRequest(void);
#endif //CCID_H
