/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.2
 *
 *********************************************************************
 * FileName:        usbcfg.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 3.11+
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
 ********************************************************************/

#ifndef USBCFG_H
#define USBCFG_H

/** D E F I N I T I O N S *******************************************/
#define MAX_NUM_INT             1   // For tracking Alternate Setting
#define EP0_BUFF_SIZE           8   // Valid Options: 8, 16, 32, or 64 bytes.
									// There is little advantage in using 
									// more than 8 bytes on EP0 IN/OUT in most cases.

/* Parameter definitions are defined in usbdrv.h */
#define MODE_PP                 _PPBM0
#define UCFG_VAL                _PUEN|_TRINT|_FS|MODE_PP


/* Uncomment only the hardware platform that you are using*/
//#define PIC18F4550_PICDEM_FS_USB
//#define PIC18F87J50_FS_USB_PIM
//#define PIC18F46J50_PIM


#if defined(PIC18F4550_PICDEM_FS_USB)
    #define USE_SELF_POWER_SENSE_IO
    #define USE_USB_BUS_SENSE_IO

#elif defined(PIC18F87J50_FS_USB_PIM)
    //#define USE_USB_BUS_SENSE_IO		//JP1 must be in R-U position to use this feature on this board		

/*If using the YOUR_BOARD selection, uncomment below section as appropriate for your hardware*/
#elif defined(EBB_V10)
	//#define USE_SELF_POWER_SENSE_IO	//See MCHPFSUSB Firmware User's Guide
   	//#define USE_USB_BUS_SENSE_IO		//(DS51679) for more details about these features.
#elif defined(EBB_V11)
	//#define USE_SELF_POWER_SENSE_IO	//See MCHPFSUSB Firmware User's Guide
   	//#define USE_USB_BUS_SENSE_IO		//(DS51679) for more details about these features.
#elif defined(THREEBEEBEE_V10)
	//#define USE_SELF_POWER_SENSE_IO	//See MCHPFSUSB Firmware User's Guide
  //#define USE_USB_BUS_SENSE_IO		//(DS51679) for more details about these features.
#elif defined(PIC18F46J50_PIM)
    //#define USE_USB_BUS_SENSE_IO		//JP1 must be in R-U position to use this feature on this board		

/*If using the YOUR_BOARD selection, uncomment below section as appropriate for your hardware*/
//#elif defined(YOUR_BOARD)
	//#define USE_SELF_POWER_SENSE_IO	//See MCHPFSUSB Firmware User's Guide
   	//#define USE_USB_BUS_SENSE_IO		//(DS51679) for more details about these features.

#else
    #error Not a supported board (yet), See __FILE__, line __LINE__, or double click on this text.
//See above commented section.  You need to select the features your hardware will be using.
#endif

/** D E V I C E  C L A S S  U S A G E *******************************/
#define USB_USE_HID

/*
 * MUID = Microchip USB Class ID
 * Used to identify which of the USB classes owns the current
 * session of control transfer over EP0
 */
#define MUID_NULL               0
#define MUID_USB9               1
#define MUID_HID                2
#define MUID_CDC                3

/** E N D P O I N T S  A L L O C A T I O N **************************/
/*
 * See usbmmap.c for an explanation of how the endpoint allocation works
 */

/* HID */
#define HID_INTF_ID             0x00
#define HID_UEP                 UEP1
#define HID_BD_OUT              ep1Bo
#define HID_INT_OUT_EP_SIZE     64
#define HID_BD_IN               ep1Bi
#define HID_INT_IN_EP_SIZE      64
#define HID_NUM_OF_DSC          1		//Just the Report descriptor (no physical descriptor present)
#define HID_RPT01_SIZE          29

/* HID macros */
#define mUSBGetHIDDscAdr(ptr)               \
{                                           \
    if(usb_active_cfg == 1)                 \
        ptr = (rom byte*)&cfg01.hid_i00a00; \
}

#define mUSBGetHIDRptDscAdr(ptr)            \
{                                           \
    if(usb_active_cfg == 1)                 \
        ptr = (rom byte*)&hid_rpt01;        \
}

#define mUSBGetHIDRptDscSize(count)         \
{                                           \
    if(usb_active_cfg == 1)                 \
        count = sizeof(hid_rpt01);          \
}

#define MAX_EP_NUMBER           1           // UEP1

#endif //USBCFG_H
