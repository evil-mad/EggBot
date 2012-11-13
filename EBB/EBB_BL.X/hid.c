/*********************************************************************
 *
 *             Microchip USB C18 Firmware -  HID Version 1.0
 *
 *********************************************************************
 * FileName:        hid.c
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

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "typedefs.h"
#include "usb.h"

#ifdef USB_USE_HID

/** V A R I A B L E S ********************************************************/
#pragma udata access fast_vars
near byte idle_rate;
near byte active_protocol;               // [0] Boot Protocol [1] Report Protocol
near byte hid_rpt_rx_len;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void HIDGetReportHandler(void);
void HIDSetReportHandler(void);

/** D E C L A R A T I O N S **************************************************/
#pragma code

/** C L A S S  S P E C I F I C  R E Q ****************************************/
/******************************************************************************
 * Function:        void USBCheckHIDRequest(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine checks the setup data packet to see if it
 *                  knows how to handle it
 *
 * Note:            None
 *****************************************************************************/
void USBCheckHIDRequest(void)
{
    if(SetupPkt.Recipient != RCPT_INTF) return;
    if(SetupPkt.bIntfID != HID_INTF_ID) return;
    
    /*
     * There are two standard requests that hid.c may support.
     * 1. GET_DSC(DSC_HID,DSC_RPT,DSC_PHY);
     * 2. SET_DSC(DSC_HID,DSC_RPT,DSC_PHY);
     */
    if(SetupPkt.bRequest == GET_DSC)
    {
        switch(SetupPkt.bDscType)
        {
            case DSC_HID:
                ctrl_trf_session_owner = MUID_HID;
                mUSBGetHIDDscAdr(pSrc.bRom);        // See usbcfg.h
                wCount._word = sizeof(USB_HID_DSC);
                break;
            case DSC_RPT:
                ctrl_trf_session_owner = MUID_HID;
                mUSBGetHIDRptDscAdr(pSrc.bRom);     // See usbcfg.h
                mUSBGetHIDRptDscSize(wCount._word); // See usbcfg.h
                break;
            case DSC_PHY:
                // ctrl_trf_session_owner = MUID_HID;
                break;
        }//end switch(SetupPkt.bDscType)
        usb_stat.ctrl_trf_mem = _ROM;
    }//end if(SetupPkt.bRequest == GET_DSC)
    
    if(SetupPkt.RequestType != CLASS) return;
    switch(SetupPkt.bRequest)
    {
        case GET_REPORT:
            HIDGetReportHandler();
            break;
        case SET_REPORT:
            HIDSetReportHandler();            
            break;
        case GET_IDLE:
            ctrl_trf_session_owner = MUID_HID;
            pSrc.bRam = (byte*)&idle_rate;      // Set source
            usb_stat.ctrl_trf_mem = _RAM;       // Set memory type
            LSB(wCount) = 1;                    // Set data count
            break;
        case SET_IDLE:
            ctrl_trf_session_owner = MUID_HID;
            idle_rate = MSB(SetupPkt.W_Value);
            break;
        case GET_PROTOCOL:
            ctrl_trf_session_owner = MUID_HID;
            pSrc.bRam = (byte*)&active_protocol;// Set source
            usb_stat.ctrl_trf_mem = _RAM;       // Set memory type
            LSB(wCount) = 1;                    // Set data count
            break;
        case SET_PROTOCOL:
            ctrl_trf_session_owner = MUID_HID;
            active_protocol = LSB(SetupPkt.W_Value);
            break;
    }//end switch(SetupPkt.bRequest)

}//end USBCheckHIDRequest

void HIDGetReportHandler(void)
{
    // ctrl_trf_session_owner = MUID_HID;
}//end HIDGetReportHandler

void HIDSetReportHandler(void)
{
    // ctrl_trf_session_owner = MUID_HID;
    // pDst.bRam = (byte*)&hid_report_out;
}//end HIDSetReportHandler

/** U S E R  A P I ***********************************************************/

/******************************************************************************
 * Function:        void HIDInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        HIDInitEP initializes HID endpoints, buffer descriptors,
 *                  internal state-machine, and variables.
 *                  It should be called after the USB host has sent out a
 *                  SET_CONFIGURATION request.
 *                  See USBStdSetCfgHandler() in usb9.c for examples.
 *
 * Note:            None
 *****************************************************************************/
void HIDInitEP(void)
{   
    hid_rpt_rx_len =0;
    
    HID_UEP = EP_OUT_IN|HSHK_EN;                // Enable 2 data pipes
    
    HID_BD_OUT.Cnt = sizeof(hid_report_out);    // Set buffer size
    HID_BD_OUT.ADR = (byte*)&hid_report_out;    // Set buffer address
    HID_BD_OUT.Stat._byte = _USIE|_DAT0|_DTSEN; // Set status

    /*
     * Do not have to init Cnt of IN pipes here.
     * Reason:  Number of bytes to send to the host
     *          varies from one transaction to
     *          another. Cnt should equal the exact
     *          number of bytes to transmit for
     *          a given IN transaction.
     *          This number of bytes will only
     *          be known right before the data is
     *          sent.
     */
    HID_BD_IN.ADR = (byte*)&hid_report_in;      // Set buffer address
    HID_BD_IN.Stat._byte = _UCPU|_DAT1;         // Set status

}//end HIDInitEP

/******************************************************************************
 * Function:        void HIDTxReport(char *buffer, byte len)
 *
 * PreCondition:    mHIDTxIsBusy() must return false.
 *
 *                  Value of 'len' must be equal to or smaller than
 *                  HID_INT_IN_EP_SIZE
 *                  For an interrupt endpoint, the largest buffer size is
 *                  64 bytes.
 *
 * Input:           buffer  : Pointer to the starting location of data bytes
 *                  len     : Number of bytes to be transferred
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Use this macro to transfer data located in data memory.
 *
 *                  Remember: mHIDTxIsBusy() must return false before user
 *                  can call this function.
 *                  Unexpected behavior will occur if this function is called
 *                  when mHIDTxIsBusy() == 0
 *
 *                  Typical Usage:
 *                  if(!mHIDTxIsBusy())
 *                      HIDTxReport(buffer, 3);
 *
 * Note:            None
 *****************************************************************************/
void HIDTxReport(char *buffer, byte len)
{
	byte i;
	
    /*
     * Value of len should be equal to or smaller than HID_INT_IN_EP_SIZE.
     * This check forces the value of len to meet the precondition.
     */
	if(len > HID_INT_IN_EP_SIZE)
	    len = HID_INT_IN_EP_SIZE;

   /*
    * Copy data from user's buffer to dual-ram buffer
    */
    for (i = 0; i < len; i++)
    	hid_report_in[i] = buffer[i];

    HID_BD_IN.Cnt = len;
    mUSBBufferReady(HID_BD_IN);

}//end HIDTxReport

/******************************************************************************
 * Function:        byte HIDRxReport(char *buffer, byte len)
 *
 * PreCondition:    Value of input argument 'len' should be smaller than the
 *                  maximum endpoint size responsible for receiving report
 *                  data from USB host for HID class.
 *                  Input argument 'buffer' should point to a buffer area that
 *                  is bigger or equal to the size specified by 'len'.
 *
 * Input:           buffer  : Pointer to where received bytes are to be stored
 *                  len     : The number of bytes expected.
 *
 * Output:          The number of bytes copied to buffer.
 *
 * Side Effects:    Publicly accessible variable hid_rpt_rx_len is updated
 *                  with the number of bytes copied to buffer.
 *                  Once HIDRxReport is called, subsequent retrieval of
 *                  hid_rpt_rx_len can be done by calling macro
 *                  mHIDGetRptRxLength().
 *
 * Overview:        HIDRxReport copies a string of bytes received through
 *                  USB HID OUT endpoint to a user's specified location. 
 *                  It is a non-blocking function. It does not wait
 *                  for data if there is no data available. Instead it returns
 *                  '0' to notify the caller that there is no data available.
 *
 * Note:            If the actual number of bytes received is larger than the
 *                  number of bytes expected (len), only the expected number
 *                  of bytes specified will be copied to buffer.
 *                  If the actual number of bytes received is smaller than the
 *                  number of bytes expected (len), only the actual number
 *                  of bytes received will be copied to buffer.
 *****************************************************************************/
byte HIDRxReport(char *buffer, byte len)
{
    hid_rpt_rx_len = 0;
    
    if(!mHIDRxIsBusy())
    {
        /*
         * Adjust the expected number of bytes to equal
         * the actual number of bytes received.
         */
        if(len > HID_BD_OUT.Cnt)
            len = HID_BD_OUT.Cnt;
        
        /*
         * Copy data from dual-ram buffer to user's buffer
         */
        for(hid_rpt_rx_len = 0; hid_rpt_rx_len < len; hid_rpt_rx_len++)
            buffer[hid_rpt_rx_len] = hid_report_out[hid_rpt_rx_len];

        /*
         * Prepare dual-ram buffer for next OUT transaction
         */
        HID_BD_OUT.Cnt = sizeof(hid_report_out);
        mUSBBufferReady(HID_BD_OUT);
    }//end if
    
    return hid_rpt_rx_len;
    
}//end HIDRxReport

#endif //def USB_USE_HID

/** EOF hid.c ***************************************************************/
