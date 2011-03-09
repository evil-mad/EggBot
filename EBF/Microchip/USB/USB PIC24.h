
/******************************************************************************

    USB PIC24-Specific Header

This file defines PIC24-specific items.


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

// To Do:  Put all PIC24-specific USB HW definitions here,

#if defined(USB_SUPPORT_HOST) && !defined(USB_SUPPORT_OTG)
//#error
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
#define USTAT_EP0_PP_MASK   ~0x04
#define USTAT_EP_MASK       0xFC
#define USTAT_EP0_OUT       0x00
#define USTAT_EP0_OUT_EVEN  0x00
#define USTAT_EP0_OUT_ODD   0x04

#define USTAT_EP0_IN        0x08
#define USTAT_EP0_IN_EVEN   0x08
#define USTAT_EP0_IN_ODD    0x0C


//******************************************************************************
// USB Endpoint Control Registers
//
// In USB Host mode, only EP0 control registers are used.  The other registers
// should be disabled.
//******************************************************************************
/*typedef union
{
    WORD UEP[16];
} _UEP;*/

#define UEP_STALL 0x0002


/********************************************************************
 * Buffer Descriptor Status Register
 *******************************************************************/
    
/* Buffer Descriptor Status Register Initialization Parameters */

#if !defined(USB_SUPPORT_DEVICE)
//The _BSTALL definition is changed from 0x04 to 0x00 to
// fix a difference in the PIC18 and PIC24 definitions of this
// bit.  This should be changed back once the definitions are
// synced.
#define _BSTALL     0x04        //Buffer Stall enable
#define _DTSEN      0x08        //Data Toggle Synch enable
#define _DAT0       0x00        //DATA0 packet expected next
#define _DAT1       0x40        //DATA1 packet expected next
#define _DTSMASK    0x40        //DTS Mask
#define _USIE       0x80        //SIE owns buffer
#define _UCPU       0x00        //CPU owns buffer

#define _STAT_MASK  0xFC

// Buffer Descriptor Status Register layout.
typedef union _BD_STAT
{
    struct{
        unsigned            :2;      //Byte count
        unsigned    BSTALL  :1;     //Buffer Stall Enable
        unsigned    DTSEN   :1;     //Data Toggle Synch Enable
        unsigned            :2;     //Reserved - write as 00
        unsigned    DTS     :1;     //Data Toggle Synch Value
        unsigned    UOWN    :1;     //USB Ownership
    };
    struct{
        unsigned            :2;
        unsigned    PID0    :1;
        unsigned    PID1    :1;
        unsigned    PID2    :1;
        unsigned    PID3    :1;
    };
    struct{
        unsigned            :2;
        unsigned    PID     :4;     // Packet Identifier
    };
    BYTE            Val;
} BD_STAT;                      //Buffer Descriptor Status Register

/********************************************************************
 * Buffer Descriptor Table Mapping
 *******************************************************************/
// BDT Entry Layout
typedef union __BDT
{
    union
    {
        struct
        {
            BYTE CNT         __attribute__ ((packed));
            BD_STAT     STAT __attribute__ ((packed));
        };
        struct
        {
            WORD        count:10;   //test
            BYTE        :6;
            BYTE*       ADR; //Buffer Address
        };
    };
    DWORD           Val;
    WORD            v[2];
} BDT_ENTRY;
#endif


/* Register Abstractions
 *************************************************************************
 */

#define USBSetBDTAddress(addr)         U1BDTP1 = (((unsigned int)addr)/256);
#define USBPowerModule() U1PWRCbits.USBPWR = 1;
#define USBPingPongBufferReset U1CONbits.PPBRST

//#define USBTransactionCompleteIE U1IEbits.TRNIE
//#define USBTransactionCompleteIF U1IRbits.TRNIF
//#define USBTransactionCompleteIFReg (BYTE*)&U1IR
//#define USBTransactionCompleteIFBitNum 3

#define USBResetIE  U1IEbits.URSTIE
#define USBResetIF  U1IRbits.URSTIF
#define USBResetIFReg (BYTE*)&U1IR
#define USBResetIFBitNum 0

#define USBIdleIE U1IEbits.IDLEIE
#define USBIdleIF U1IRbits.IDLEIF
#define USBIdleIFReg (BYTE*)&U1IR
#define USBIdleIFBitNum 4

#define USBActivityIE U1OTGIEbits.ACTVIE
#define USBActivityIF U1OTGIRbits.ACTVIF
#define USBActivityIFReg (BYTE*)&U1OTGIR
#define USBActivityIFBitNum 4

#define USBSOFIE U1IEbits.SOFIE
#define USBSOFIF U1IRbits.SOFIF
#define USBSOFIFReg (BYTE*)&U1IR
#define USBSOFIFBitNum 2

#define USBStallIE U1IEbits.STALLIE
#define USBStallIF U1IRbits.STALLIF
#define USBStallIFReg (BYTE*)&U1IR
#define USBStallIFBitNum 7

#define USBErrorIE U1IEbits.UERRIE
#define USBErrorIF U1IRbits.UERRIF
#define USBErrorIFReg (BYTE*)&U1IR
#define USBErrorIFBitNum 1

//#define USBSE0Event U1CONbits.SE0
#define USBSuspendControl U1PWRCbits.USUSPEND
#define USBPacketDisable U1CONbits.PKTDIS
#define USBResumeControl U1CONbits.RESUME

#define USBT1MSECIE U1OTGIEbits.T1MSECIE
#define USBT1MSECIF U1OTGIRbits.T1MSECIF
#define USBT1MSECIFReg (BYTE*)&U1OTGIR
#define USBT1MSECIFBitNum   6

#define USBIDIE U1OTGIEbits.IDIE
#define USBIDIF U1OTGIRbits.IDIF
#define USBIDIFReg (BYTE*)&U1OTGIR
#define USBIDIFBitNum   7

#define USB_PING_PONG__ALL_BUT_EP0          0x03    // U1CFG1 - Ping-pong on all endpoints except EP0
#define USB_PING_PONG__FULL_PING_PONG       0x02    // U1CFG1 - Ping-pong on all endpoints
#define USB_PING_PONG__EP0_OUT_ONLY         0x01    // U1CFG1 - Ping-pong on EP 0 out only
#define USB_PING_PONG__NO_PING_PONG         0x00    // U1CFG1 - No ping-pong

#endif
