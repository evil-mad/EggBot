/******************************************************************************

    USB PIC18-Specific Header

This file defines PIC18-specific items.


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

Author          Date    Comments
--------------------------------------------------------------------------------
KO/BC       2-20-2008   Initial Creation

*******************************************************************************/



// To Do:  Put all PIC18-specific USB HW definitions here,

#define KVA_TO_PA(v) 	v
#define PA_TO_KVA0(pa)  pa
#define PA_TO_KVA1(pa)	pa
#define GET_PHYSICAL_ADDRESS(v) (v)


/* translate betwwen KSEG0 and KSEG1 virtual addresses */
#define KVA0_TO_KVA1(v)	((v) | 0x20000000)
#define KVA1_TO_KVA0(v)	((v) & ~0x20000000)


/********************************************************************
 * USB - PIC Endpoint Definitions
 * PIC Endpoint Address Format: X:EP3:EP2:EP1:EP0:DIR:PPBI:X
 * This is used when checking the value read from USTAT
 *
 * NOTE: These definitions are not used in the descriptors.
 * EP addresses used in the descriptors have different format.
 *******************************************************************/
#define USTAT_EP0_PP_MASK   ~0x02
#define USTAT_EP_MASK       0x7E
#define USTAT_EP0_OUT       0x00
#define USTAT_EP0_OUT_EVEN  0x00
#define USTAT_EP0_OUT_ODD   0x02

#define USTAT_EP0_IN        0x04
#define USTAT_EP0_IN_EVEN   0x04
#define USTAT_EP0_IN_ODD    0x06


//******************************************************************************
// USB Endpoint Control Registers
//
// In USB Host mode, only EP0 control registers are used.  The other registers
// should be disabled.
//******************************************************************************
typedef union
{
    BYTE UEP[16];
} _UEP;

#define UEP_STALL 0x0002


/********************************************************************
 * Buffer Descriptor Status Register
 *******************************************************************/
    
/* Buffer Descriptor Status Register Initialization Parameters */
#define _BSTALL     0x04        //Buffer Stall enable
#define _DTSEN      0x08        //Data Toggle Synch enable
#define _INCDIS     0x10        //Address increment disable
#define _KEN        0x20        //SIE keeps buff descriptors enable
#define _DAT0       0x00        //DATA0 packet expected next
#define _DAT1       0x40        //DATA1 packet expected next
#define _DTSMASK    0x40        //DTS Mask
#define _USIE       0x80        //SIE owns buffer
#define _UCPU       0x00        //CPU owns buffer

#define _STAT_MASK  0xFF

/* BDT entry structure definition */
typedef union _BD_STAT
{
    BYTE Val;
    struct{
        //If the CPU owns the buffer then these are the values
        unsigned BC8:1;         //bit 8 of the byte count
        unsigned BC9:1;         //bit 9 of the byte count
        unsigned BSTALL:1;      //Buffer Stall Enable
        unsigned DTSEN:1;       //Data Toggle Synch Enable
        unsigned INCDIS:1;      //Address Increment Disable
        unsigned KEN:1;         //BD Keep Enable
        unsigned DTS:1;         //Data Toggle Synch Value
        unsigned UOWN:1;        //USB Ownership
    };
    struct{
        //if the USB module owns the buffer then these are
        // the values
        unsigned BC8:1;         //bit 8 of the byte count
        unsigned BC9:1;         //bit 9 of the byte count
        unsigned PID0:1;        //Packet Identifier
        unsigned PID1:1;
        unsigned PID2:1;
        unsigned PID3:1;
        unsigned :1;
        unsigned UOWN:1;        //USB Ownership
    };
    struct{
        unsigned :2;
        unsigned PID:4;         //Packet Identifier
        unsigned :2;
    };
} BD_STAT;                      //Buffer Descriptor Status Register


/********************************************************************
 * Buffer Descriptor Table Mapping
 *******************************************************************/

// BDT Entry Layout
typedef union __BDT
{
    struct
    {
        BD_STAT STAT;
        BYTE CNT;
        BYTE ADRL;                      //Buffer Address Low
        BYTE ADRH;                      //Buffer Address High
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        BYTE* ADR;                      //Buffer Address
    };
    DWORD Val;
    BYTE v[4];
} BDT_ENTRY;


/****************************************************************
  	Function:
  		void USBPowerModule(void)
  		
  	Description:
  		This macro is used to power up the USB module if required<br>
  		PIC18: defines as nothing<br>
  		PIC24: defines as U1PWRCbits.USBPWR = 1;<br>
  	
  	Precondition:
  		None
  		
  	Parameters:
  		None
  		
  	Return Values:
  		None
  		
  	Remarks:
  		None
  	
  ****************************************************************/
#define USBPowerModule()

/****************************************************************
  	Function:
  		USBSetBDTAddress(addr)
  		
  	Description:
  		This macro is used to power up the USB module if required
  		
  	Precondition:
  		None
  		
  	Parameters:
  		None
  		
  	Return Values:
  		None
  		
  	Remarks:
  		None
  	 
  ****************************************************************/
#define USBSetBDTAddress(addr)
#define USBPingPongBufferReset UCONbits.PPBRST

#define USBTransactionCompleteIE UIEbits.TRNIE
#define USBTransactionCompleteIF UIRbits.TRNIF
#define USBTransactionCompleteIFReg (BYTE*)&UIR
#define USBTransactionCompleteIFBitNum 3

#define USBResetIE  UIEbits.URSTIE
#define USBResetIF  UIRbits.URSTIF
#define USBResetIFReg (BYTE*)&UIR
#define USBResetIFBitNum 0

#define USBIdleIE UIEbits.IDLEIE
#define USBIdleIF UIRbits.IDLEIF
#define USBIdleIFReg (BYTE*)&UIR
#define USBIdleIFBitNum 4

#define USBActivityIE UIEbits.ACTVIE
#define USBActivityIF UIRbits.ACTVIF
#define USBActivityIFReg (BYTE*)&UIR
#define USBActivityIFBitNum 2

#define USBSOFIE UIEbits.SOFIE
#define USBSOFIF UIRbits.SOFIF
#define USBSOFIFReg (BYTE*)&UIR
#define USBSOFIFBitNum 6

#define USBStallIE UIEbits.STALLIE
#define USBStallIF UIRbits.STALLIF
#define USBStallIFReg (BYTE*)&UIR
#define USBStallIFBitNum 5

#define USBErrorIE UIEbits.UERRIE
#define USBErrorIF UIRbits.UERRIF
#define USBErrorIFReg (BYTE*)&UIR
#define USBErrorIFBitNum 1

#define USBSE0Event UCONbits.SE0
#define USBSuspendControl UCONbits.SUSPND
#define USBPacketDisable UCONbits.PKTDIS
#define USBResumeControl UCONbits.RESUME



