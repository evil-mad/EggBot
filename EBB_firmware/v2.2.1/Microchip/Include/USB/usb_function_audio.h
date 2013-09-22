/*******************************************************************************
  File Information:
    FileName:     	usb_function_audio.h
    Dependencies:	See INCLUDES section
    Processor:		PIC18, PIC24 or PIC32
    Complier:  		C18, C30 or C32
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

  Summary:
    This file contains all of functions, macros, definitions, variables,
    datatypes, etc. that are required for usage with the USB Device AUDIO 
    function driver. This file should be included in projects that use the Audio
    \function driver.  This file should also be included into the
    usb_descriptors.c file and any other user file that requires access to the
    USB Device Audio interface.



    This file is located in the "\<Install Directory\>\\Microchip\\Include\\USB"
    directory.

  Description:
    USB Device Audio Function Driver File

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

#ifndef AUDIO_H
	#define AUDIO_H

/** I N C L U D E S *******************************************************/


/** DEFINITIONS ****************************************************/
/******Audio Interface Class Code**********/
#define AUDIO_DEVICE 0x01

/******* Audio Interface Subclass Codes**********/
#define AUDIOCONTROL 0x01
#define AUDIOSTREAMING 0x02
#define MIDISTREAMING 0x03

/******* Audio Class-Specific Descriptor Types**********/
#define CS_UNDEFINED 0x20
#define CS_DEVICE 0x21
#define CS_CONFIGURATION 0x22
#define CS_STRING 0x23
#define CS_INTERFACE 0x24
#define CS_ENDPOINT 0x25

/******* Audio Class-Specific AC Interface Descriptor Subtypes**********/
#define AC_DESCRIPTOR_UNDEFINED 0x00
#define HEADER 0x01
#define INPUT_TERMINAL 0x02
#define OUTPUT_TERMINAL 0x03
#define MIXER_UNIT 0x04
#define SELECTOR_UNIT 0x05
#define FEATURE_UNIT 0x06
#define PROCESSING_UNIT 0x07
#define EXTENSION_UNIT 0x08

/******* Audio Class-Specific AS Interface Descriptor Subtypes**********/
#define AS_DESCRIPTOR_UNDEFINED 0x00
#define AS_GENERAL 0x01
#define FORMAT_TYPE 0x02
#define FORMAT_SPECIFIC 0x03

/*******  Processing Unit Process Types **********/
#define PROCESS_UNDEFINED 0x00
#define UP_DOWNMIX_PROCESS 0x01
#define DOLBY_PROLOGIC_PROCESS 0x02
#define THREE_D_STEREO_EXTENDER_PROCESS 0x03
#define REVERBERATION_PROCESS 0x04
#define CHORUS_PROCESS 0x05
#define DYN_RANGE_COMP_PROCESS 0x06

/******* Audio Class-Specific Endpoint Descriptor Subtypes**********/
#define DESCRIPTOR_UNDEFINED 0x00
#define EP_GENERAL 0x01

/****** Audio Class-Specific Request Codes ************/
#define REQUEST_CODE_UNDEFINED 0x00
#define SET_CUR  0x01
#define SET_MIN  0x02
#define SET_MAX  0x03
#define SET_RES  0x04
#define SET_MEM  0x05
#define GET_CUR  0x81
#define GET_MIN  0x82
#define GET_MAX  0x83
#define GET_RES  0x84
#define GET_MEM  0x85
#define GET_STAT  0xFF

/************ Terminal Control Selectors ******/
#define TE_CONTROL_UNDEFINED 0x00
#define COPY_PROTECT_CONTROL 0x01

/************ Feature Unit Control Selectors ****/
#define FU_CONTROL_UNDEFINED 0x00
#define MUTE_CONTROL 0x01
#define VOLUME_CONTROL 0x02
#define BASS_CONTROL 0x03
#define MID_CONTROL 0x04
#define TREBLE_CONTROL 0x05
#define GRAPHIC_EQUALIZER_CONTROL 0x06
#define AUTOMATIC_GAIN_CONTROL 0x07
#define DELAY_CONTROL 0x08
#define BASS_BOOST_CONTROL 0x09
#define LOUDNESS_CONTROL 0x0A


/************  Processing Unit Control Selectors ****/
/*  Up/Down-mix Processing Unit Control Selectors */

#define UD_CONTROL_UNDEFINED 0x00
#define UD_ENABLE_CONTROL 0x01
#define UD_MODE_SELECT_CONTROL 0x02

/* Dolby Prologic(TM) Processing Unit Control Selectors */
#define DP_CONTROL_UNDEFINED 0x00
#define DP_ENABLE_CONTROL 0x01
#define DP_MODE_SELECT_CONTROL 0x02

/* Stereo Extender Processing Unit Control Selectors */
#define THREE_D_CONTROL_UNDEFINED 0x00
#define THREE_D_ENABLE_CONTROL 0x01
#define SPACIOUSNESS_CONTROL 0x03

/* Reverberation Processing Unit Control Selectors */
#define RV_CONTROL_UNDEFINED 0x00
#define RV_ENABLE_CONTROL 0x01
#define REVERB_LEVEL_CONTROL 0x02
#define REVERB_TIME_CONTROL 0x03
#define REVERB_FEEDBACK_CONTROL 0x04

/* Chorus Processing Unit Control Selectors */
#define CH_CONTROL_UNDEFINED 0x00
#define CH_ENABLE_CONTROL 0x01
#define CHORUS_LEVEL_CONTROL 0x02
#define CHORUS_RATE_CONTROL 0x03
#define CHORUS_DEPTH_CONTROL 0x04

/* Dynamic Range Compressor Processing Unit Control Selectors */
#define DR_CONTROL_UNDEFINED 0x00
#define DR_ENABLE_CONTROL 0x01
#define COMPRESSION_RATE_CONTROL 0x02
#define MAXAMPL_CONTROL 0x03
#define THRESHOLD_CONTROL 0x04
#define ATTACK_TIME 0x05
#define RELEASE_TIME 0x06


/************ Extension Unit Control Selectors **********/
#define XU_CONTROL_UNDEFINED 0x00
#define XU_ENABLE_CONTROL 0x01

/************ Endpoint Control Selectors **********/
#define EP_CONTROL_UNDEFINED 0x00
#define SAMPLING_FREQ_CONTROL 0x01
#define PITCH_CONTROL 0x02

/*********** Terminal Types***********************/ 
/*A complete list of Terminal Type codes is provided in 
the document USB Audio Terminal Types */ 
#define USB_STREAMING 0x01, 0x01
#define MICROPHONE 0x01,0x02 
#define SPEAKER 0x01,0x03
#define HEADPHONES 0x02,0x03


/** E X T E R N S ************************************************************/
extern USB_HANDLE lastTransmission;
extern volatile CTRL_TRF_SETUP SetupPkt;
extern ROM BYTE configDescriptor1[];
extern volatile BYTE CtrlTrfData[USB_EP0_BUFF_SIZE];



/********************************************************************
    Function:
 		void USBCheckAudioRequest(void)
        
    Summary:
 		This routine checks the setup data packet to see if it
 		knows how to handle it
        
    Description:
 		This routine checks the setup data packet to see if it
 		knows how to handle it

    PreCondition:
        None
        
    Parameters:
		None
        
    Return Values:
		None
        
    Remarks:
        None
  
 *******************************************************************/
void USBCheckAudioRequest(void);
#endif //AUDIO_H
