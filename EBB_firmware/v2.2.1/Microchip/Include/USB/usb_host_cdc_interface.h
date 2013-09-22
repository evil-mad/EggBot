/******************************************************************************

  USB Host Communication Device Class(CDC) Interface header file

This file provides all the interface functions to the CDC host client driver.
This file should be used in a project with usb_host_cdc.h to provide access
to CDC host.

Acronyms/abbreviations used by this class:
    * CDC - Communication Device Class
    * ACM - Abstract Control Module

This file provides interface functions for USB CDC host to the application.
These interface function can be used to set up transfers with the device attached
on the bus. Interface function USBHostCDC_Api_Get_IN_Data can be used to set up
transfer request to receive data from the device.Interface function
USBHostCDC_Api_Send_OUT_Data can be used to set up transfer request to send
data to the device. If USB_ENABLE_TRANSFER_EVENT is defined application will receive
an event on completion of transfer request otherwise application can poll the status
of transfer using function USBHostCDC_ApiTransferIsComplete. This file also provides
interface routine for ACM class specific requests in function USBHostCDC_Api_ACM_Request.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************

* FileName:        usb_host_cdc_interface.h
* Dependencies:    None
* Processor:       PIC24/dsPIC30/dsPIC33/PIC32MX
* Compiler:        C30 v2.01/C32 v0.00.18
* Company:         Microchip Technology, Inc.

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
ADG          9-Apr-2008 First release
********************************************************************************
 Change History:
 Revision     Description
 v2.7         Modified the code to allow connection of USB-RS232 dongles that do
              not fully comply with CDC specifications
              Modified API USBHostCDC_Api_Send_OUT_Data to allow data transfers
              more than 256 bytes
*******************************************************************************/
//DOM-IGNORE-END

//DOM-IGNORE-BEGIN
#ifndef _USB_HOST_CDC_INTERFACE_H_
#define _USB_HOST_CDC_INTERFACE_H_
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
//******************************************************************************
// Data Structures
//******************************************************************************
//******************************************************************************

// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************

/****************************************************************************
  Function:
    BOOL USBHostCDC_Api_Get_IN_Data(BYTE no_of_bytes, BYTE* data)

  Description:
    This function is called by application to receive Input data over DATA
    interface. This function setsup the request to receive data from the device.

  Precondition:
    None

  Parameters:
    BYTE    no_of_bytes - Number of Bytes expected from the device.
    BYTE*   data        - Pointer to application receive data buffer.

  Return Values:
    TRUE    -   Transfer request is placed successfully.
    FALSE   -   Transfer request failed.

  Remarks:
    None
***************************************************************************/
BOOL USBHostCDC_Api_Get_IN_Data(BYTE no_of_bytes, BYTE* data);

/****************************************************************************
  Function:
    BOOL USBHostCDC_Api_Send_OUT_Data(WORD no_of_bytes, BYTE* data)

  Description:
    This function is called by application to transmit out data over DATA
    interface. This function setsup the request to transmit data to the
    device.

  Precondition:
    None

  Parameters:
    BYTE    no_of_bytes - Number of Bytes expected from the device.
    BYTE*   data        - Pointer to application transmit data buffer.


  Return Values:
    TRUE    -   Transfer request is placed successfully.
    FALSE   -   Transfer request failed.

  Remarks:
    None
***************************************************************************/
BOOL USBHostCDC_Api_Send_OUT_Data(WORD no_of_bytes, BYTE* data);

/****************************************************************************
  Function:
    BOOL USBHostCDC_ApiTransferIsComplete(BYTE* errorCodeDriver,BYTE* byteCount)

  Description:
    This function is called by application to poll for transfer status. This
    function returns true in the transfer is over. To check whether the transfer
    was successfull or not , application must check the error code returned by
    reference.

  Precondition:
    None

  Parameters:
    BYTE    *errorCodeDriver - returns.
    BYTE    *byteCount       - Number of bytes transferred.


  Return Values:
    TRUE    -   Transfer is has completed.
    FALSE   -   Transfer is pending.

  Remarks:
    None
***************************************************************************/
BOOL USBHostCDC_ApiTransferIsComplete(BYTE* errorCodeDriver, BYTE* byteCount );

/*******************************************************************************
  Function:
    BOOL USBHostCDC_ApiDeviceDetect( void )

  Description:
    This function determines if a CDC device is attached
    and ready to use.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    TRUE   -  CDC present and ready
    FALSE  -  CDC not present or not ready

  Remarks:
    Since this will often be called in a loop while waiting for
    a device, we'll make sure the tasks are executed.
*******************************************************************************/
BOOL USBHostCDC_ApiDeviceDetect( void );


/*******************************************************************************
  Function:
    BYTE USBHostCDC_Api_ACM_Request(BYTE requestType, BYTE size, BYTE* data)

  Description:
    This function can be used by application code to dynamically access ACM specific
    requests. This function should be used only if apllication intends to modify for
    example the Baudrate from previouly configured rate. Data transmitted/received
    to/from device is a array of bytes. Application must take extra care of understanding
    the data format before using this function.

  Precondition:
    Device must be enumerated and attached successfully.

  Parameters:
    BYTE    requestType     These are the mandatory CDC request supported by the
                            CDC host stack.
                            - USB_CDC_SEND_ENCAPSULATED_COMMAND
                            - USB_CDC_GET_ENCAPSULATED_REQUEST
                            - USB_CDC_SET_LINE_CODING
                            - USB_CDC_SET_CONTROL_LINE_STATE
                            - USB_CDC_SET_CONTROL_LINE_STATE
    BYTE    size            - Number bytes to be transferred.
    BYTE    *data           - Pointer to data being transferred.

  Return Values:
    USB_SUCCESS                 - Request started successfully
    USB_CDC_DEVICE_NOT_FOUND    - No device with specified address
    USB_CDC_DEVICE_BUSY         - Device not in proper state for
                                  performing a transfer
    USB_CDC_COMMAND_FAILED      - Request is not supported.
    USB_CDC_ILLEGAL_REQUEST     - Requested ID is invalid.

  Remarks:
    None
 *******************************************************************************/
BYTE USBHostCDC_Api_ACM_Request(BYTE requestType, BYTE size, BYTE* data);

#endif /* _USB_HOST_CDC_INTERFACE_H_ */
