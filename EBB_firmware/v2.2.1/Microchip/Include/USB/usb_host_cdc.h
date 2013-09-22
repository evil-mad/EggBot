/******************************************************************************

  USB Host Communication Device Class(CDC) driver Header file

Description:
    This is the header file for a USB Embedded Host that is using the Communication
    Device Class.

    This file should be included with usb_host.h to provide the USB hardware
    interface. It must be included after the application-specific usb_config.h
    file and after the USB Embedded Host header file usb_host.h, as definitions
    in those files are required for proper compilation.

Acronyms/abbreviations used by this class:
    * CDC - Communication Device Class
    * ACM - Abstract Control Module

    To interface with usb_host.c, the routine USBHostCDCInitialize() should be
    specified as the Initialize() function, and USBHostCDCEventHandler() should
    be specified as the EventHandler() function in the usbClientDrvTable[] array
    declared in usb_config.h.

    This driver can be configured to use transfer events from usb_host.c.  Transfer
    events require more RAM and ROM than polling, but it cuts down or even
    eliminates the required polling of the various USBxxxTasks functions.  For this
    class, USBHostCDCTasks() is compiled out if transfer events from usb_host.c
    are used.  However, USBHostTasks() still must be called to provide attach,
    enumeration, and detach services.  If transfer events from usb_host.c
    are going to be used, USB_ENABLE_TRANSFER_EVENT should be defined.  If transfer
    status is going to be polled, USB_ENABLE_TRANSFER_EVENT should not be defined.
    CDC client layer transfer events are also enables once USB_ENABLE_TRANSFER_EVENT
    is defined. The application must define USB_HOST_APP_EVENT_HANDLER in file usb_config.h.
    All the CDC client layer event are called back using this function handler. This eliminates
    the need of polling for transfer status by the application, however handing of these events
    should be taken care by the application.

    Transfer of Data Class Interface can be performed with BULK transfers,
    hence USB_SUPPORT_BULK_TRANSFERS must be defined. Data Class Interface can also use
    ISOCHRONOUS transfers,however the CDC client is not tested for ISOCHRONOUS transfers.


*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************

* FileName:        usb_host_cdc.h
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

 Change History:
 Rev     Description
 ----    ------------------------------------------------
 2.6     Changed format of 'bcdCDC' from word to array of byte.

 2.6a    No change

 2.7     Modified the code to allow connection of USB-RS232 dongles that do
         not fully comply with CDC specifications

         Modified API USBHostCDC_Api_Send_OUT_Data to allow data transfers
         more than 256 bytes

 2.9     Fixed bug: changed USB_CDC_CONTROL_LINE_LENGTH to 0x00 (was 0x02).		 
********************************************************************************/
//DOM-IGNORE-END

//DOM-IGNORE-BEGIN
#ifndef _USB_HOST_CDC_H_
#define _USB_HOST_CDC_H_
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// CDC Class Error Codes
// *****************************************************************************

#define USB_CDC_CLASS_ERROR              USB_ERROR_CLASS_DEFINED

#define USB_CDC_COMMAND_PASSED           USB_SUCCESS                 // Command was successful.
#define USB_CDC_COMMAND_FAILED          (USB_CDC_CLASS_ERROR | 0x01) // Command failed at the device.
#define USB_CDC_PHASE_ERROR             (USB_CDC_CLASS_ERROR | 0x02) // Command had a phase error at the device.
#define USB_CDC_DEVICE_NOT_FOUND        (USB_CDC_CLASS_ERROR | 0x03) // Device with the specified address is not available.
#define USB_CDC_DEVICE_BUSY             (USB_CDC_CLASS_ERROR | 0x04) // A transfer is currently in progress.
#define USB_CDC_NO_REPORT_DESCRIPTOR    (USB_CDC_CLASS_ERROR | 0x05) // No report descriptor found
#define USB_CDC_INTERFACE_ERROR         (USB_CDC_CLASS_ERROR | 0x06) // The interface layer cannot support the device.
#define USB_CDC_REPORT_DESCRIPTOR_BAD   (USB_CDC_CLASS_ERROR | 0x05) // Report Descriptor for not proper
#define USB_CDC_RESET_ERROR             (USB_CDC_CLASS_ERROR | 0x0A) // An error occurred while resetting the device.
#define USB_CDC_ILLEGAL_REQUEST         (USB_CDC_CLASS_ERROR | 0x0B) // Cannot perform requested operation.


// *****************************************************************************
// Section: Interface and Protocol Constants
// *****************************************************************************

#define DEVICE_CLASS_CDC             0x02 // CDC Interface Class Code
#define USB_CDC_COMM_INTF            0x02 // Communication Interface Class Code

/* Communication Interface Class SubClass Codes */
#define USB_CDC_DIRECT_LINE_CONTROL_MODEL         0x01 // Direct Line Control Model
#define USB_CDC_ABSTRACT_CONTROL_MODEL            0x02 // Abstract Control Model
#define USB_CDC_TELEPHONE_CONTROL_MODEL           0x03 // Telephone Control Model
#define USB_CDC_MULTI_CHANNEL_CONTROL_MODEL       0x04 // Multi-Channel Control Model
#define USB_CDC_CAPI_CONTROL_MODEL                0x05 // CAPI Control Model
#define USB_CDC_ETHERNET_NETWORKING_CONTROL_MODEL 0x06 // Ethernet Networking Control Model
#define USB_CDC_ATM_NETWORKING_CONTROL_MODEL      0x07 // ATM Networking Control Model
#define USB_CDC_WIRELESS_HANDSET_CONTROL_MODEL    0x08 // Wireless Handset Control Model
#define USB_CDC_DEVICE_MANAGEMENT                 0x09 // Device Management
#define USB_CDC_MOBILE_DIRECT_LINE_MODEL          0x0A // Mobile Direct Line Model
#define USB_CDC_OBEX                              0x0B // OBEX
#define USB_CDC_ETHERNET_EMULATION_MODEL          0x0C // Ethernet Emulation Model

/* Communication Interface Class Control Protocol Codes */
/* First Cut V.250 AT Commands will be supported .. other protocols ??? */
/* For more.... see Table 5 in USB CDC Specification 1.2 */
#define USB_CDC_V25TER                      0x01    // Common AT commands ("Hayes(TM)")

/* Data Interface Class Codes */
#define USB_CDC_DATA_INTF                   0x0A

/* Data Interface Class Protocol Codes */
#define USB_CDC_NO_PROTOCOL                 0x00    // No class specific protocol required
/* For more.... see Table 7 in USB CDC Specification 1.2 */

/* Functional Descriptor Details */
/* Type Values for the bDscType Field */
#define USB_CDC_CS_INTERFACE                0x24
#define USB_CDC_CS_ENDPOINT                 0x25

/* bDscSubType in Functional Descriptors */
#define USB_CDC_DSC_FN_HEADER               0x00
#define USB_CDC_DSC_FN_CALL_MGT             0x01
#define USB_CDC_DSC_FN_ACM                  0x02    // ACM - Abstract Control Management
#define USB_CDC_DSC_FN_DLM                  0x03    // DLM - Direct Line Managment
#define USB_CDC_DSC_FN_TELEPHONE_RINGER     0x04
#define USB_CDC_DSC_FN_RPT_CAPABILITIES     0x05
#define USB_CDC_DSC_FN_UNION                0x06
#define USB_CDC_DSC_FN_COUNTRY_SELECTION    0x07
#define USB_CDC_DSC_FN_TEL_OP_MODES         0x08
#define USB_CDC_DSC_FN_USB_TERMINAL         0x09
/* more.... see Table 13 in USB CDC Specification 1.2 */


/* bRequestCode - Class Specific Request codes */
/* Type Values for the bRequest Field */
#define USB_CDC_SEND_ENCAPSULATED_COMMAND   0x00    // Issues a command in the format of the supported control protocol.
#define USB_CDC_GET_ENCAPSULATED_REQUEST    0x01    // Requests a response in the format of the supported control protocol.
#define USB_CDC_SET_COMM_FEATURE            0x02    // Controls the settings for a particular communications feature.
#define USB_CDC_GET_COMM_FEATURE            0x03    // Returns the current settings for the communications feature.
#define USB_CDC_SET_LINE_CODING             0x20    // Configures DTE rate, stop-bits, parity, and number-of-character bits.
#define USB_CDC_GET_LINE_CODING             0x21    // Requests current DTE rate, stop-bits, parity, and number-of-character bits.
#define USB_CDC_SET_CONTROL_LINE_STATE      0x22    // [V24] signal used to tell the DCE device the DTE device is now present.
#define USB_CDC_SEND_BREAK                  0x23    // Sends special carrier modulation used to specify [V24] style break.


// *****************************************************************************
// Additional return values for USBHostCDCDeviceStatus (see USBHostDeviceStatus also)
// *****************************************************************************

#define USB_CDC_DEVICE_DETACHED             0x50    // Device is detached.
#define USB_CDC_INITIALIZING                0x51    // Device is initializing.
#define USB_CDC_NORMAL_RUNNING              0x53    // Device is running and available for data transfers.
#define USB_CDC_DEVICE_HOLDING              0x54    // Device is holding due to error
#define USB_CDC_RESETTING_DEVICE            0x55    // Device is being reset.

// *****************************************************************************
// Section: CDC Event Definition
// *****************************************************************************

// If the application has not defined an offset for CDC events, set it to 0.
#ifndef EVENT_CDC_OFFSET
    #define EVENT_CDC_OFFSET    0
#endif

//#ifndef EVENT_CDC_BASE
//   #define EVENT_CDC_BASE       400 // need to add base in usb_common.h
//#endif

#define EVENT_CDC_NONE                 EVENT_CDC_BASE + EVENT_CDC_OFFSET + 0   // No event occured (NULL event)
#define EVENT_CDC_ATTACH               EVENT_CDC_BASE + EVENT_CDC_OFFSET + 1   // No event occured (NULL event)
#define EVENT_CDC_COMM_READ_DONE       EVENT_CDC_BASE + EVENT_CDC_OFFSET + 2   // A CDC Communication Read transfer has completed
#define EVENT_CDC_COMM_WRITE_DONE      EVENT_CDC_BASE + EVENT_CDC_OFFSET + 3   // A CDC Communication Write transfer has completed
#define EVENT_CDC_DATA_READ_DONE       EVENT_CDC_BASE + EVENT_CDC_OFFSET + 4   // A CDC Data Read transfer has completed
#define EVENT_CDC_DATA_WRITE_DONE      EVENT_CDC_BASE + EVENT_CDC_OFFSET + 5   // A CDC Data Write transfer has completed
#define EVENT_CDC_RESET                EVENT_CDC_BASE + EVENT_CDC_OFFSET + 6   // CDC reset complete
#define EVENT_CDC_NAK_TIMEOUT          EVENT_CDC_BASE + EVENT_CDC_OFFSET + 7   // CDC device NAK timeout has occurred


#define USB_CDC_LINE_CODING_LENGTH          0x07   // Number of bytes Line Coding transfer
#define USB_CDC_CONTROL_LINE_LENGTH         0x00   // Number of bytes Control line transfer
#define USB_CDC_MAX_PACKET_SIZE             0x200   // Max transfer size is 64 bytes for Full Speed USB
//******************************************************************************
//******************************************************************************
// Data Structures
//******************************************************************************
//******************************************************************************
typedef union _USB_CDC_LINE_CODING
{
    struct
    {
        BYTE _byte[USB_CDC_LINE_CODING_LENGTH];
    };
    struct
    {
        DWORD_VAL   dwDTERate;          // Data terminal rate, in bits per second.
        BYTE    bCharFormat;            // Stop bits 0:1 Stop bit, 1:1.5 Stop bits, 2:2 Stop bits
        BYTE    bParityType;            // Parity 0:None, 1:Odd, 2:Even, 3:Mark, 4:Space
        BYTE    bDataBits;              // Data bits (5, 6, 7, 8 or 16)
    };
} USB_CDC_LINE_CODING;

typedef union _USB_CDC_CONTROL_SIGNAL_BITMAP
{
    BYTE _byte;
    struct
    {
        unsigned DTE_PRESENT:1;       // [0] Not Present  [1] Present
        unsigned CARRIER_CONTROL:1;   // [0] Deactivate   [1] Activate
    };
} USB_CDC_CONTROL_SIGNAL_BITMAP;

/* Functional Descriptor Structure - See CDC Specification 1.2 for details */

/* Header Functional Descriptor */
typedef struct _USB_CDC_HEADER_FN_DSC
{
    BYTE bFNLength;         // Size of this functional descriptor, in bytes.
    BYTE bDscType;          // CS_INTERFACE
    BYTE bDscSubType;       // Header. This is defined in [USBCDC1.2], which defines this as a header.
    BYTE bcdCDC[2];         // USB Class Definitions for Communications Devices Specification release number in binary-coded decimal.
} USB_CDC_HEADER_FN_DSC;

/* Abstract Control Management Functional Descriptor */
typedef struct _USB_CDC_ACM_FN_DSC
{
    BYTE bFNLength;         // Size of this functional descriptor, in bytes.
    BYTE bDscType;          // CS_INTERFACE
    BYTE bDscSubType;       // Abstract Control Management functional descriptor subtype as defined in [USBCDC1.2].
    BYTE bmCapabilities;    // The capabilities that this configuration supports. (A bit value of zero means that the request is not supported.)
} USB_CDC_ACM_FN_DSC;

/* Union Functional Descriptor */
typedef struct _USB_CDC_UNION_FN_DSC
{
    BYTE bFNLength;        // Size of this functional descriptor, in bytes.
    BYTE bDscType;         // CS_INTERFACE
    BYTE bDscSubType;      // Union Descriptor Functional Descriptor subtype as defined in [USBCDC1.2].
    BYTE bMasterIntf;      // Interface number of the control (Communications Class) interface
    BYTE bSaveIntf0;       // Interface number of the subordinate (Data Class) interface
} USB_CDC_UNION_FN_DSC;

/* Call Management Functional Descriptor */
typedef struct _USB_CDC_CALL_MGT_FN_DSC
{
    BYTE bFNLength;         // Size of this functional descriptor, in bytes.
    BYTE bDscType;          // CS_INTERFACE
    BYTE bDscSubType;       // Call Management functional descriptor subtype, as defined in [USBCDC1.2].
    BYTE bmCapabilities;    // The capabilities that this configuration supports:
    BYTE bDataInterface;    // Interface number of Data Class interface optionally used for call management.
} USB_CDC_CALL_MGT_FN_DSC;

/*
   This structure stores communication interface details of the attached CDC device
*/
typedef struct _COMM_INTERFACE_DETAILS
{
    BYTE                            interfaceNum;         // communication interface number
    BYTE                            noOfEndpoints;        // Number endpoints for communication interface
    /* Functional Descriptor Details */
    USB_CDC_HEADER_FN_DSC           Header_Fn_Dsc;        // Header Function Descriptor
    USB_CDC_ACM_FN_DSC              ACM_Fn_Desc;          // Abstract Control Model Function Descriptor
    USB_CDC_UNION_FN_DSC            Union_Fn_Desc;        // Union Function Descriptor
    USB_CDC_CALL_MGT_FN_DSC         Call_Mgt_Fn_Desc;     // Call Management Function Descriptor
    /* Endpoint Descriptor Details*/
    WORD                            endpointMaxDataSize;  // Max data size for a interface.
    WORD                            endpointInDataSize;   // Max data size for a interface.
    WORD                            endpointOutDataSize;  // Max data size for a interface.
    BYTE                            endpointPollInterval; // Polling rate of corresponding interface.
    BYTE                            endpointType;         // Endpoint type - either Isochronous or Bulk
    BYTE                            endpointIN;           // IN endpoint for comm interface.
    BYTE                            endpointOUT;          // IN endpoint for comm interface.

}   COMM_INTERFACE_DETAILS;


/*
   This structure stores data interface details of the attached CDC device
*/
typedef struct _DATA_INTERFACE_DETAILS
{
    BYTE                            interfaceNum;         // Data interface number 
    BYTE                            noOfEndpoints;        // number of endpoints associated with data interface

    /* Endpoint Descriptor Details*/
    WORD                            endpointInDataSize;   // Max data size for a interface.
    WORD                            endpointOutDataSize;  // Max data size for a interface.
    BYTE                            endpointType;         // Endpoint type - either Isochronous or Bulk
    BYTE                            endpointIN;           // IN endpoint for comm interface.
    BYTE                            endpointOUT;          // IN endpoint for comm interface.
}   DATA_INTERFACE_DETAILS;

/*
   This structure is used to hold information about an attached CDC device
*/
typedef struct _USB_CDC_DEVICE_INFO
{
    BYTE*                               userData;              // Data pointer to application buffer.
    WORD                                reportSize;            // Total length of user data
    WORD                                remainingBytes;        // Number bytes remaining to be transferrerd in case user data length is more than 64 bytes
    WORD                                bytesTransferred;      // Number of bytes transferred to/from the user's data buffer.
    union
    {
        struct
        {
            BYTE                        bfDirection          : 1;   // Direction of current transfer (0=OUT, 1=IN).
            BYTE                        bfReset              : 1;   // Flag indicating to perform CDC Reset.
            BYTE                        bfClearDataIN        : 1;   // Flag indicating to clear the IN endpoint.
            BYTE                        bfClearDataOUT       : 1;   // Flag indicating to clear the OUT endpoint.
        };
        BYTE                            val;
    }                                   flags;
    BYTE                                driverSupported;       // If CDC driver supports requested Class,Subclass & Protocol.
    BYTE                                deviceAddress;         // Address of the device on the bus.
    BYTE                                errorCode;             // Error code of last error.
    BYTE                                state;                 // State machine state of the device.
    BYTE                                returnState;           // State to return to after performing error handling.
    BYTE                                noOfInterfaces;        // Total number of interfaces in the device.
    BYTE                                interface;             // Interface number of current transfer.
    BYTE                                endpointDATA;          // Endpoint to use for the current transfer.
    BYTE                                commRequest;           // Current Communication code
    BYTE                                clientDriverID;        // Client driver ID for device requests.
    COMM_INTERFACE_DETAILS              commInterface;         // This structure stores communication interface details.
    DATA_INTERFACE_DETAILS              dataInterface;         // This structure stores data interface details.
} USB_CDC_DEVICE_INFO;


// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************


/*******************************************************************************
  Function:
    USBHostCDCRead_DATA( address,interface,size,data,endpointData)

  Summary:
    This function intiates a read request from a attached CDC device.

  Description:
    This function starts a CDC read transfer.

  Preconditions:
    None

  Parameters:
    address          - Device address
    interface        - interface number of the requested transfer
    size             - Number of bytes to be read from the device
    data             - address of location where received data is to be stored
    endpointDATA     - endpoint details on which the transfer is requested

  Return Values:
    USB_SUCCESS                 - Request started successfully
    USB_CDC_DEVICE_NOT_FOUND    - No device with specified address
    USB_CDC_DEVICE_BUSY         - Device not in proper state for
                                  performing a transfer
  Remarks:
    None
*******************************************************************************/
#define USBHostCDCRead_DATA( address,interface,size,data,endpointData) \
         USBHostCDCTransfer( address,0,1,interface, size,data,endpointData)

/*******************************************************************************
  Function:
    USBHostCDCSend_DATA( address,interface,size,data,endpointData)

  Summary:
    This function intiates a write request to a attached CDC device.

  Description:
    This function starts a CDC write transfer.

  Preconditions:
    None

  Parameters:
    address          - Device address
    interface        - interface number of the requested transfer
    size             - Number of bytes to be transfered to the device
    data             - address of location where the data to be transferred is stored
    endpointDATA     - endpoint details on which the transfer is requested

  Return Values:
    USB_SUCCESS                 - Request started successfully
    USB_CDC_DEVICE_NOT_FOUND    - No device with specified address
    USB_CDC_DEVICE_BUSY         - Device not in proper state for
                                  performing a transfer
  Remarks:
    None
*******************************************************************************/
#define USBHostCDCSend_DATA( address,interface,size,data,endpointData) \
         USBHostCDCTransfer( address,0,0,interface, size,data,endpointData)


/*******************************************************************************
  Function:
    BYTE    USBHostCDCDeviceStatus( BYTE deviceAddress )

  Summary:
    This function determines the status of a CDC device.

  Description:
    This function determines the status of a CDC device.

  Preconditions:  None

  Parameters:
    BYTE deviceAddress - address of device to query

  Return Values:
    USB_CDC_DEVICE_NOT_FOUND           -  Illegal device address, or the
                                          device is not an CDC
    USB_CDC_INITIALIZING               -  CDC is attached and in the
                                          process of initializing
    USB_PROCESSING_REPORT_DESCRIPTOR   -  CDC device is detected and report
                                          descriptor is being parsed
    USB_CDC_NORMAL_RUNNING             -  CDC Device is running normal, 
                                          ready to send and receive reports 
    USB_CDC_DEVICE_HOLDING             -  Device is holding due to error
    USB_CDC_DEVICE_DETACHED            -  CDC detached.

  Remarks:
    None
*******************************************************************************/
BYTE    USBHostCDCDeviceStatus( BYTE deviceAddress );

/*******************************************************************************
  Function:
    BYTE USBHostCDCResetDevice( BYTE deviceAddress )

  Summary:
    This function starts a CDC  reset.

  Description:
    This function starts a CDC reset.  A reset can be
    issued only if the device is attached and not being initialized.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress - Device address

  Return Values:
    USB_SUCCESS                 - Reset started
    USB_MSD_DEVICE_NOT_FOUND    - No device with specified address
    USB_MSD_ILLEGAL_REQUEST     - Device is in an illegal state for reset

  Remarks:
    None
*******************************************************************************/
BYTE    USBHostCDCResetDevice( BYTE deviceAddress );

/*******************************************************************************
  Function:
     void USBHostCDCTasks( void )

  Summary:
    This function performs the maintenance tasks required by CDC class

  Description:
    This function performs the maintenance tasks required by the CDC
    class.  If transfer events from the host layer are not being used, then
    it should be called on a regular basis by the application.  If transfer
    events from the host layer are being used, this function is compiled out,
    and does not need to be called.

  Precondition:
    USBHostCDCInitialize() has been called.

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
*******************************************************************************/
void    USBHostCDCTasks( void );

/*******************************************************************************
  Function:
    USBHostCDCTransfer( BYTE deviceAddress, BYTE direction, BYTE reportid, BYTE size, BYTE *data)

  Summary:
    This function starts a CDC transfer.

  Description:
    This function starts a CDC transfer. A read/write wrapper is provided in application
    interface file to access this function.

  Preconditions:
    None

  Parameters:
    BYTE deviceAddress      - Device address
    BYTE request            - Request type for Communication Interface
    BYTE direction          - 1=read, 0=write
    BYTE interfaceNum       - interface number of the requested transfer
    BYTE size               - Byte size of the data buffer
    BYTE *data              - Pointer to the data buffer
    BYTE endpointDATA       - endpoint details on which the transfer is requested

  Return Values:
    USB_SUCCESS                 - Request started successfully
    USB_CDC_DEVICE_NOT_FOUND    - No device with specified address
    USB_CDC_DEVICE_BUSY         - Device not in proper state for
                                  performing a transfer
  Remarks:
    None
*******************************************************************************/
BYTE USBHostCDCTransfer( BYTE deviceAddress,BYTE request , BYTE direction, BYTE interfaceNum, WORD size, BYTE *data , BYTE endpointDATA);

/*******************************************************************************
  Function:
    BOOL USBHostCDCTransferIsComplete( BYTE deviceAddress,
                        BYTE *errorCode, DWORD *byteCount )

  Summary:
    This function indicates whether or not the last transfer is complete.

  Description:
    This function indicates whether or not the last transfer is complete.
    If the functions returns TRUE, the returned byte count and error
    code are valid. Since only one transfer can be performed at once
    and only one endpoint can be used, we only need to know the
    device address.

  Precondition:
    None

  Parameters:
    BYTE deviceAddress  - Device address
    BYTE *errorCode     - Error code from last transfer
    DWORD *byteCount    - Number of bytes transferred

  Return Values:
    TRUE    - Transfer is complete, errorCode is valid
    FALSE   - Transfer is not complete, errorCode is not valid
*******************************************************************************/
BOOL    USBHostCDCTransferIsComplete( BYTE deviceAddress, BYTE *errorCode, BYTE *byteCount );


// *****************************************************************************
// *****************************************************************************
// USB Host Callback Function Prototypes
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    BOOL USBHostCDCInitialize( BYTE address, DWORD flags, BYTE clientDriverID )

  Summary:
    This function is the initialization routine for this client driver.

  Description:
    This function is the initialization routine for this client driver.  It
    is called by the host layer when the USB device is being enumerated.For a
    CDC device we need to look into CDC descriptor, interface descriptor and
    endpoint descriptor.

  Precondition:
    None

  Parameters:
    BYTE address        - Address of the new device
    DWORD flags          - Initialization flags
    BYTE clientDriverID - Client driver identification for device requests

  Return Values:
    TRUE   - We can support the device.
    FALSE  - We cannot support the device.

  Remarks:
    None
*******************************************************************************/
BOOL USBHostCDCInitialize( BYTE address, DWORD flags, BYTE clientDriverID );

/*******************************************************************************
  Function:
    BOOL USBHostCDCEventHandler( BYTE address, USB_EVENT event,
                        void *data, DWORD size )

  Precondition:
    The device has been initialized.

  Summary:
    This function is the event handler for this client driver.

  Description:
    This function is the event handler for this client driver.  It is called
    by the host layer when various events occur.

  Parameters:
    BYTE address    - Address of the device
    USB_EVENT event - Event that has occurred
    void *data      - Pointer to data pertinent to the event
    DWORD size       - Size of the data

  Return Values:
    TRUE   - Event was handled
    FALSE  - Event was not handled

  Remarks:
    None
*******************************************************************************/
BOOL USBHostCDCEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size );

/*******************************************************************************
  Function:
    BOOL USBHostCDCInitAddress( BYTE address, DWORD flags, BYTE clientDriverID )

  Precondition:
    The device has been enumerated without any errors.

  Summary:
    This function intializes the address of the attached CDC device.

  Description:
    This function intializes the address of the attached CDC device. Once the
    device is enumerated without any errors, the CDC client call this function.
    For all the transfer requesets this address is used to indentify the CDC
    device.


  Parameters:
    BYTE address    -   Address of the new device
    DWORD flags     -   Initialization flags
    BYTE clientDriverID - Client driver identification for device requests

  Return Values:
    TRUE    -   We can support the device.
    FALSE   -   We cannot support the device.

  Remarks:
    None
*******************************************************************************/
BOOL USBHostCDCInitAddress( BYTE address, DWORD flags, BYTE clientDriverID );

#endif
