/*********************************************************************
  File Information:
    FileName:        usb_function_msd.h
    Dependencies:    See INCLUDES section below
    Processor:       PIC18, PIC24, or PIC32
    Compiler:        C18, C30, or C32
    Company:         Microchip Technology, Inc.

    Software License Agreement

    The software supplied herewith by Microchip Technology Incorporated
    (the “Company”) for its PICmicro® Microcontroller is intended and
    supplied to you, the Company’s customer, for use solely and
    exclusively on Microchip PICmicro Microcontroller products. The
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
    This file contains functions, macros, definitions, variables,
    datatypes, etc. that are required for use of the MSD function
    driver. This file should be included in projects that use the MSD
    \function driver.
    
    
    
    This file is located in the "\<Install Directory\>\\Microchip\\USB\\MSD
    Device Driver" directory.

  Description:
    USB MSD Function Driver File
    
    This file contains functions, macros, definitions, variables,
    datatypes, etc. that are required for use of the MSD function
    driver. This file should be included in projects that use the MSD
    \function driver.
    
    This file is located in the "\<Install Directory\>\\Microchip\\USB\\MSD
    Device Driver" directory.
    
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

 Change History:
   Rev    Description
   ----   ------------------------------------------
   2.6    No Change
   2.7b   Added a couple of additional sense key definitions.

 ********************************************************************/
#ifndef MSD_H
#define MSD_H

/** I N C L U D E S **********************************************************/
#include "Compiler.h"
#include "GenericTypeDefs.h"
#include "MDD File System/FSDefs.h"
//#include "SD Card/sdcard.h"

/** D E F I N I T I O N S ****************************************************/

/* MSD Interface Class Code */
#define MSD_INTF                    0x08

/* MSD Interface Class SubClass Codes */
//Options - from usb_msc_overview_1[1].2.pdf
//  Supported
#define SCSI_TRANSPARENT    0x06
//  Not-Supported
#define RBC                 0x01    // Reduced Block Commands (RBC) T10 Project 1240-D
#define SSF_8020i           0x02    // C/DVD devices typically use SSF-8020i or MMC-2
#define MMC_2               0x02
#define QIC_157             0x03    // Tape drives typically use QIC-157 command blocks
#define UFI                 0x04    // Typically a floppy disk drive (FDD) device
#define SSF_8070i           0x05    // Typically a floppy disk drive uses SSF-8070i commands

#define MSD_INTF_SUBCLASS          SCSI_TRANSPARENT
//#define MSD_INTF_SUBCLASS          RBC

/* MSD Interface Class Protocol Codes */
#define MSD_PROTOCOL           0x50

/* Class Commands */
#define MSD_RESET 0xff
#define GET_MAX_LUN 0xfe

#define BLOCKLEN_512                0x0200

#define STMSDTRIS TRISD0
#define STRUNTRIS TRISD1
#define STMSDLED LATDbits.LATD0
#define STRUNLED LATDbits.LATD1
#define ToggleRUNLED() STRUNLED = !STRUNLED;

    //**********************************************************DOM-IGNORE-BEGIN
    //Various States of Mass Storage Firmware (MSDTasks)
    //**********************************************************DOM-IGNORE-END

    //MSD_WAIT is when the MSD state machine is idle (returned by MSDTasks())
    #define MSD_WAIT                            0x00
    //MSD_DATA_IN is when the device is sending data (returned by MSDTasks())
    #define MSD_DATA_IN                         0x01
    //MSD_DATA_OUT is when the device is receiving data (returned by MSDTasks())
    #define MSD_DATA_OUT                        0x02
    //MSD_SEND_CSW is when the device is waiting to send the CSW (returned by MSDTasks())
    #define MSD_SEND_CSW                        0x03
    
    //States of the MSDProcessCommand state machine
    #define MSD_COMMAND_WAIT                    0xFF
    #define MSD_COMMAND_ERROR                   0xFE
    #define MSD_COMMAND_RESPONSE                0xFD
    #define MSD_COMMAND_RESPONSE_SEND           0xFC
    #define MSD_COMMAND_STALL                   0xFB
    
    /* SCSI Transparent Command Set Sub-class code */
    #define MSD_INQUIRY 						0x12
    #define MSD_READ_FORMAT_CAPACITY 			0x23			 
    #define MSD_READ_CAPACITY 					0x25
    #define MSD_READ_10 						0x28
    #define MSD_WRITE_10 						0x2a
    #define MSD_REQUEST_SENSE 					0x03
    #define MSD_MODE_SENSE 						0x1a
    #define MSD_PREVENT_ALLOW_MEDIUM_REMOVAL 	0x1e
    #define MSD_TEST_UNIT_READY 				0x00
    #define MSD_VERIFY 							0x2f
    #define MSD_STOP_START 						0x1b
    
    #define MSD_READ10_WAIT                     0x00
    #define MSD_READ10_BLOCK                    0x01
    #define MSD_READ10_SECTOR                   0x02
    #define MSD_READ10_TX_SECTOR                0x03
    #define MSD_READ10_TX_PACKET                0x04
    
    #define MSD_WRITE10_WAIT                    0x00
    #define MSD_WRITE10_BLOCK                   0x01
    #define MSD_WRITE10_SECTOR                  0x02
    #define MSD_WRITE10_RX_SECTOR               0x03
    #define MSD_WRITE10_RX_PACKET               0x04

//Define MSD_USE_BLOCKING in order to block the code in an 
//attempt to get better throughput.
//#define MSD_USE_BLOCKING

#define MSD_CSW_SIZE 0x0d	// 10 bytes CSW data
#define MSD_CBW_SIZE 0x1f	// 31 bytes CBW data

#define INVALID_CBW 1
#define VALID_CBW !INVALID_CBW

/* Sense Key Error Codes */

#define S_NO_SENSE 0x0
#define S_RECOVERED_ERROR 0x1
#define S_NOT_READY 0x2
#define S_MEDIUM_ERROR 0x3
#define S_HARDWARE_ERROR 0X4
#define S_ILLEGAL_REQUEST 0x5
#define S_UNIT_ATTENTION 0x6
#define S_DATA_PROTECT 0x7
#define S_BLANK_CHECK 0x8
#define S_VENDOR_SPECIFIC 0x9
#define S_COPY_ABORTED 0xa
#define S_ABORTED_COMMAND 0xb
#define S_OBSOLETE 0xc
#define S_VOLUME_OVERFLOW 0xd
#define S_MISCOMPARE 0xe

#define S_CURRENT 0x70
#define S_DEFERRED 0x71

/* ASC ASCQ Codes for Sense Data (only those that we plan to use) */
// with sense key Illegal request for a command not supported
#define ASC_NO_ADDITIONAL_SENSE_INFORMATION 0x00
#define ASCQ_NO_ADDITIONAL_SENSE_INFORMATION 0x00

#define ASC_INVALID_COMMAND_OPCODE 0x20
#define ASCQ_INVALID_COMMAND_OPCODE 0x00

// from SPC-3 Table 185
// with sense key Illegal Request for test unit ready
#define ASC_LOGICAL_UNIT_NOT_SUPPORTED 0x25
#define ASCQ_LOGICAL_UNIT_NOT_SUPPORTED 0x00

// with sense key Not ready
#define ASC_LOGICAL_UNIT_DOES_NOT_RESPOND 0x05
#define ASCQ_LOGICAL_UNIT_DOES_NOT_RESPOND 0x00

#define ASC_MEDIUM_NOT_PRESENT 0x3a
#define ASCQ_MEDIUM_NOT_PRESENT 0x00

#define ASC_LOGICAL_UNIT_NOT_READY_CAUSE_NOT_REPORTABLE 0x04
#define ASCQ_LOGICAL_UNIT_NOT_READY_CAUSE_NOT_REPORTABLE 0x00

#define ASC_LOGICAL_UNIT_IN_PROCESS 0x04
#define ASCQ_LOGICAL_UNIT_IN_PROCESS 0x01

#define ASC_LOGICAL_UNIT_NOT_READY_INIT_REQD 0x04
#define ASCQ_LOGICAL_UNIT_NOT_READY_INIT_REQD 0x02

#define ASC_LOGICAL_UNIT_NOT_READY_INTERVENTION_REQD 0x04
#define ASCQ_LOGICAL_UNIT_NOT_READY_INTERVENTION_REQD 0x03

#define ASC_LOGICAL_UNIT_NOT_READY_FORMATTING 0x04
#define ASCQ_LOGICAL_UNIT_NOT_READY_FORMATTING 0x04

#define ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE 0x21
#define ASCQ_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE 0x00

#define ASC_WRITE_PROTECTED 0x27
#define ASCQ_WRITE_PROTECTED 0x00

/** S T R U C T U R E S ******************************************************/
/********************** ******************************************************/
 
typedef struct _USB_MSD_CBW 		//31 bytes total Command Block Wrapper
{
    DWORD dCBWSignature;			// 55 53 42 43h
    DWORD dCBWTag;					// sent by host, device echos this value in CSW (associated a CSW with a CBW)
    DWORD dCBWDataTransferLength; 	// number of bytes of data host expects to transfer
    BYTE bCBWFlags; 				// CBW flags, bit 7 = 0-data out from host to device, 
    								//					= 1-device to host, rest bits 0
    BYTE bCBWLUN;					// Most Significant 4bits are always zero, 0 in our case as only one logical unit
    BYTE bCBWCBLength;				// Here most significant 3bits are zero
	BYTE CBWCB[16];		            // Command block to be executed by the device
} USB_MSD_CBW;

typedef struct { 					// Command Block for Read 10 (0x28)& Write 10 (0x2a)commands
			BYTE Opcode;				
			BYTE Flags;						// b7-b5 RDProtect, b4 DPO, b3 FUA, b2 Reserved, b1 FUA_NV, b0 Obsolete
			DWORD_VAL LBA;						// 
			BYTE GroupNumber;				// b4-b0 is Group Number rest are reserved
			WORD_VAL TransferLength;
			BYTE Control;
} ReadWriteCB;
			
typedef struct {					// Inquiry command format
	BYTE Opcode;
	BYTE EVPD;				// only b0 is enable vital product data
	BYTE PageCode;
	WORD AllocationLength;
	BYTE Control;
} InquiryCB;

typedef struct {					// Read Capacity 10
	BYTE Opcode;
	BYTE Reserved1;
	DWORD LBA;				// Logical Block Address
	WORD Reserved2;
	BYTE PMI;				// Partial medium Indicator b0 only
	BYTE Control; 
} ReadCapacityCB;

typedef struct {					// Request Sense 0x03		
	BYTE Opcode;
	BYTE Desc;
	WORD Reserved;
	BYTE AllocationLength;
	BYTE Control;
} RequestSenseCB;	

typedef struct {					// Mode Sense 0x1a
	BYTE Opcode;
	BYTE DBD;				// actually only b3 is used as disable block descriptor
	BYTE PageCode;			// b7,b6 PC=Page Control, b5-b0 PageCode
									// Page Control bits 00=> CurrentValue, 01=>Changeable Values,10=>Default Value, 11=>Saved Values
	BYTE SubPageCode;
	BYTE AllocationLength;
	BYTE Control;		
} ModeSenseCB;

typedef struct {					// PREVENT_ALLOW_MEDIUM_REMOVAL 0x1e
	BYTE Opcode;
	BYTE Reserved[3];
	BYTE Prevent;			// only b1-b0 is prevent, rest reserved
	BYTE Control;
} PreventAllowMediumRemovalCB;

typedef struct {					// TEST_UNIT_READY 0x00
	BYTE Opcode;
	DWORD Reserved;
	BYTE Control;			
} TestUnitReadyCB;

typedef struct {					// VERIFY 10 Command 0x2f
	BYTE Opcode;
	BYTE VRProtect;			// b7-b5 VRProtect, b4 DPO, b3-b2,Reserved, b1 BYTCHK, b0 Obsolete
	DWORD LBA;
	BYTE GroupNumber;		// b4-b0 Group Number, rest reserved
	WORD VerificationLength;	
	BYTE Control;
} VerifyCB;
	
typedef struct {					// STOP_START 0x1b
	BYTE Opcode;
	BYTE Immed;
	WORD Reserved;
	BYTE Start;				// b7-b4 PowerCondition, b3-b2reserved, b1 LOEJ, b0 Start
	BYTE Control; 	
} StopStartCB;	


typedef struct _USB_MSD_CSW			// Command Status Wrapper
{
	DWORD dCSWSignature;			// 55 53 42 53h Signature of a CSW packet
	DWORD dCSWTag;					// echo the dCBWTag of the CBW packet
	DWORD dCSWDataResidue;			// difference in data expected (dCBWDataTransferLength) and actual amount processed/sent
	BYTE bCSWStatus;				// 00h Command Passed, 01h Command Failed, 02h Phase Error, rest obsolete/reserved
} USB_MSD_CSW;

typedef struct 
{
 	BYTE Peripheral; 					// Peripheral_Qualifier:3; Peripheral_DevType:5;
	BYTE Removble;						// removable medium bit7 = 0 means non removable, rest reserved
	BYTE Version;						// version
	BYTE Response_Data_Format;		// b7,b6 Obsolete, b5 Access control co-ordinator, b4 hierarchical addressing support 
										// b3:0 response data format 2 indicates response is in format defined by spec
	BYTE AdditionalLength;				// length in bytes of remaining in standard inquiry data
	BYTE Sccstp; 						// b7 SCCS, b6 ACC, b5-b4 TGPS, b3 3PC, b2-b1 Reserved, b0 Protected 
	BYTE bqueetc;						// b7 bque, b6- EncServ, b5-VS, b4-MultiP, b3-MChngr, b2-b1 Obsolete, b0-Addr16	
    BYTE CmdQue;                        // b7-b6 Obsolete, b5-WBUS, b4-Sync, b3-Linked, b2 Obsolete,b1 Cmdque, b0-VS
	char vendorID[8];	
	char productID[16];
	char productRev[4];
} InquiryResponse;

typedef struct {
	BYTE ModeDataLen;
	BYTE MediumType;
	unsigned Resv:4;
	unsigned DPOFUA:1;					// 0 indicates DPO and FUA bits not supported
	unsigned notused:2;
	unsigned WP:1;						// 0 indicates not write protected		
	BYTE BlockDscLen;					// Block Descriptor Length
} tModeParamHdr;

/* Short LBA mode block descriptor (see Page 1009, SBC-2) */
typedef struct {
	BYTE NumBlocks[4];
	BYTE Resv;							// reserved
	BYTE BlockLen[3];
} tBlockDescriptor;

/* Page_0 mode page format */
typedef struct {
	
	unsigned PageCode:6;				// SPC-3 7.4.5
	unsigned SPF:1;						// SubPageFormat=0 means Page_0 format
	unsigned PS:1;						// Parameters Saveable

	BYTE PageLength;					// if 2..n bytes of mode parameters PageLength = n-1
	BYTE ModeParam[];					// mode parameters
} tModePage;	

typedef struct {
	tModeParamHdr Header;
	tBlockDescriptor BlockDsc;
	tModePage modePage;
} ModeSenseResponse;


/* Fixed format if Desc bit of request sense cbw is 0 */
typedef union __attribute__((packed)){
	struct
    {
        BYTE _byte[18];
    };
	struct __attribute__((packed)){
		unsigned ResponseCode:7;			// b6-b0 is Response Code Fixed or descriptor format
		unsigned VALID:1;					// Set to 1 to indicate information field is a valid value
	
		BYTE Obsolete;
	
		unsigned SenseKey:4;				// Refer SPC-3 Section 4.5.6
		unsigned Resv:1;					
		unsigned ILI:1;						// Incorrect Length Indicator
		unsigned EOM:1;						// End of Medium
		unsigned FILEMARK:1; 				// for READ and SPACE commands
	
		BYTE InformationB0;					// device type or command specific (SPC-33.1.18)
        BYTE InformationB1;					// device type or command specific (SPC-33.1.18)
        BYTE InformationB2;					// device type or command specific (SPC-33.1.18)
        BYTE InformationB3;					// device type or command specific (SPC-33.1.18)
		BYTE AddSenseLen;					// number of additional sense bytes that follow <=244
		DWORD_VAL CmdSpecificInfo;				// depends on command on which exception occured
		BYTE ASC;							// additional sense code 
		BYTE ASCQ;							// additional sense code qualifier Section 4.5.2.1 SPC-3
		BYTE FRUC;							// Field Replaceable Unit Code 4.5.2.5 SPC-3
	
		BYTE SenseKeySpecific[3];			// msb is SKSV sense-key specific valied field set=> valid SKS
											// 18-n additional sense bytes can be defined later
											// 18 Bytes Request Sense Fixed Format
	};
} RequestSenseResponse;

/**************************************************************************
  Summary:
    LUN_FUNCTIONS is a structure of function pointers that tells the stack
    where to find each of the physical layer functions it is looking for.
    This structure needs to be defined for any project for PIC24F or PIC32.
  Description:
    LUN_FUNCTIONS is a structure of function pointers that tells the stack
    where to find each of the physical layer functions it is looking for.
    This structure needs to be defined for any project for PIC24F or PIC32.
    
    Typical Usage:
    <code>
        LUN_FUNCTIONS LUN[MAX_LUN + 1] =
        {
            {
                &amp;MDD_SDSPI_MediaInitialize,
                &amp;MDD_SDSPI_ReadCapacity,
                &amp;MDD_SDSPI_ReadSectorSize,
                &amp;MDD_SDSPI_MediaDetect,
                &amp;MDD_SDSPI_SectorRead,
                &amp;MDD_SDSPI_WriteProtectState,
                &amp;MDD_SDSPI_SectorWrite
            }
        };
    </code>
    
    In the above code we are passing the address of the SDSPI functions to
    the corresponding member of the LUN_FUNCTIONS structure. In the above
    case we have created an array of LUN_FUNCTIONS structures so that it is
    possible to have multiple physical layers by merely increasing the
    MAX_LUN variable and by adding one more set of entries in the array.
    Please take caution to insure that each function is in the the correct
    location in the structure. Incorrect alignment will cause the USB stack
    to call the incorrect function for a given command.
    
    See the MDD File System Library for additional information about the
    available physical media, their requirements, and how to use their
    associated functions.                                                  
  **************************************************************************/
typedef struct
{
    //Function pointer to the MediaInitialize() function of the physical media 
    //  being used. 
    MEDIA_INFORMATION* (*MediaInitialize)();
    //Function pointer to the ReadCapacity() function of the physical media 
    //  being used.
    DWORD (*ReadCapacity)();
    //Function pointer to the ReadSectorSize() function of the physical media 
    //  being used.
    WORD  (*ReadSectorSize)();
    //Function pointer to the MediaDetect() function of the physical media 
    //  being used.
    BYTE  (*MediaDetect)();
    //Function pointer to the SectorRead() function of the physical media being 
    //  used.
    BYTE  (*SectorRead)(DWORD sector_addr, BYTE* buffer);
    //Function pointer to the WriteProtectState() function of the physical 
    //  media being used.
    BYTE  (*WriteProtectState)();
    //Function pointer to the SectorWrite() function of the physical media 
    //  being used.
    BYTE  (*SectorWrite)(DWORD sector_addr, BYTE* buffer, BYTE allowWriteToZero);
} LUN_FUNCTIONS;

/** Section: Externs *********************************************************/
extern volatile USB_MSD_CBW msd_cbw;
extern volatile USB_MSD_CSW msd_csw;
extern volatile char msd_buffer[512];
extern BOOL SoftDetach[MAX_LUN + 1];
extern volatile CTRL_TRF_SETUP SetupPkt;
extern volatile BYTE CtrlTrfData[USB_EP0_BUFF_SIZE];


/** Section: Public Prototypes ***********************************************/
void USBCheckMSDRequest(void);
BYTE MSDTasks(void);
void USBMSDInit(void);

/**************************************************************************
    Function:
    void LUNSoftDetach(BYTE LUN)
    
    Summary:
    
    Description:
    
    Parameters:
        LUN - logical unit number to detach
    
    Return Values:
        None

    Remarks:
        Once a soft detached is initiated a soft attached, LUNSoftAttach(),
        on the same LUN must be performed before the device will re-attach
                    
  **************************************************************************/
#define LUNSoftDetach(LUN) SoftDetach[LUN]=TRUE;

/**************************************************************************
    Function:
    void LUNSoftAttach(BYTE LUN)
    
    Summary:
    
    Description:
    
    Parameters:
        LUN - logical unit number to detach
    
    Return Values:
        None

    Remarks:
        Once a soft detached is initiated a soft attached, LUNSoftAttach(),
        on the same LUN must be performed before the device will re-attach
                    
  **************************************************************************/
#define LUNSoftAttach(LUN) SoftDetach[LUN]=FALSE;


#endif
