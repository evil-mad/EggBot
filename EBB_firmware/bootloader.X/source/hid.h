/*********************************************************************
 *
 *             Microchip USB C18 Firmware -  HID Version 1.0
 *
 *********************************************************************
 * FileName:        hid.h
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
 * Rawin Rojvanit       11/19/04    Original.
 ********************************************************************/
#ifndef HID_H
#define HID_H

/** I N C L U D E S **********************************************************/
#include "typedefs.h"

/** D E F I N I T I O N S ****************************************************/

/* Class-Specific Requests */
#define GET_REPORT      0x01
#define GET_IDLE        0x02
#define GET_PROTOCOL    0x03
#define SET_REPORT      0x09
#define SET_IDLE        0x0A
#define SET_PROTOCOL    0x0B

/* Class Descriptor Types */
#define DSC_HID         0x21
#define DSC_RPT         0x22
#define DSC_PHY         0x23

/* Protocol Selection */
#define BOOT_PROTOCOL   0x00
#define RPT_PROTOCOL    0x01


/* HID Interface Class Code */
#define HID_INTF                    0x03

/* HID Interface Class SubClass Codes */
#define BOOT_INTF_SUBCLASS          0x01

/* HID Interface Class Protocol Codes */
#define HID_PROTOCOL_NONE           0x00
#define HID_PROTOCOL_KEYBOAD        0x01
#define HID_PROTOCOL_MOUSE          0x02

/******************************************************************************
 * Macro:           (bit) mHIDRxIsBusy(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This macro is used to check if HID OUT endpoint is
 *                  busy (owned by SIE) or not.
 *                  Typical Usage: if(mHIDRxIsBusy())
 *
 * Note:            None
 *****************************************************************************/
#define mHIDRxIsBusy()              HID_BD_OUT.Stat.UOWN

/******************************************************************************
 * Macro:           (bit) mHIDTxIsBusy(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This macro is used to check if HID IN endpoint is
 *                  busy (owned by SIE) or not.
 *                  Typical Usage: if(mHIDTxIsBusy())
 *
 * Note:            None
 *****************************************************************************/
#define mHIDTxIsBusy()              HID_BD_IN.Stat.UOWN

/******************************************************************************
 * Macro:           byte mHIDGetRptRxLength(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          mHIDGetRptRxLength returns hid_rpt_rx_len
 *
 * Side Effects:    None
 *
 * Overview:        mHIDGetRptRxLength is used to retrieve the number of bytes
 *                  copied to user's buffer by the most recent call to
 *                  HIDRxReport function.
 *
 * Note:            None
 *****************************************************************************/
#define mHIDGetRptRxLength()        hid_rpt_rx_len

/** S T R U C T U R E S ******************************************************/
typedef struct _USB_HID_DSC_HEADER
{
    byte bDscType;
    word wDscLength;
} USB_HID_DSC_HEADER;

typedef struct _USB_HID_DSC
{
    byte bLength;       byte bDscType;      word bcdHID;
    byte bCountryCode;  byte bNumDsc;
    USB_HID_DSC_HEADER hid_dsc_header[HID_NUM_OF_DSC];
    /*
     * HID_NUM_OF_DSC is defined in autofiles\usbcfg.h
     */
} USB_HID_DSC;

/** E X T E R N S ************************************************************/
extern near byte hid_rpt_rx_len;

/** P U B L I C  P R O T O T Y P E S *****************************************/
void HIDInitEP(void);
void USBCheckHIDRequest(void);
void HIDTxReport(char *buffer, byte len);
byte HIDRxReport(char *buffer, byte len);

#endif //HID_H
