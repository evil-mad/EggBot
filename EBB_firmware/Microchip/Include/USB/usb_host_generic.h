/*******************************************************************************

    USB Host Generic Client Driver (Header File)

Description:
    This is the Generic client driver file for a USB Embedded Host device.  This
    driver should be used in a project with usb_host.c to provided the USB
    hardware interface.

    To interface with USB Embedded Host layer, the routine USBHostGenericInit()
    should be specified as the Initialize() function, and
    USBHostGenericEventHandler() should be specified as the EventHandler()
    function in the usbClientDrvTable[] array declared in usb_config.c.

    This driver can be configured to either use transfer events from usb_host.c
    or use a polling mechanism.  If USB_ENABLE_TRANSFER_EVENT is defined, this
    driver will utilize transfer events.  Otherwise, this driver will utilize
    polling.

    Since the generic class is performed with interrupt transfers,
    USB_SUPPORT_INTERRUPT_TRANSFERS must be defined.

Summary:
    This is the Generic client driver file for a USB Embedded Host device.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

* FileName:        usb_client_generic.h
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
BC/KO       25-Dec-2007 First release

*******************************************************************************/
#ifndef __USBHOSTGENERIC_H__
#define __USBHOSTGENERIC_H__
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// This is the default Generic Client Driver endpoint number.
#ifndef USB_GENERIC_EP
    #define USB_GENERIC_EP       1
#endif

// *****************************************************************************
// *****************************************************************************
// Section: USB Generic Client Events
// *****************************************************************************
// *****************************************************************************

        // This is an optional offset for the values of the generated events.
        // If necessary, the application can use a non-zero offset for the
        // generic events to resolve conflicts in event number.
#ifndef EVENT_GENERIC_OFFSET
#define EVENT_GENERIC_OFFSET 0
#endif

        // This event indicates that a Generic device has been attached.
        // When USB_HOST_APP_EVENT_HANDLER is called with this event, *data
        // points to a GENERIC_DEVICE_ID structure, and size is the size of the
        // GENERIC_DEVICE_ID structure.
#define EVENT_GENERIC_ATTACH  (EVENT_GENERIC_BASE+EVENT_GENERIC_OFFSET+0)

        // This event indicates that the specified device has been detached
        // from the USB.  When USB_HOST_APP_EVENT_HANDLER is called with this
        // event, *data points to a BYTE that contains the device address, and
        // size is the size of a BYTE.
#define EVENT_GENERIC_DETACH  (EVENT_GENERIC_BASE+EVENT_GENERIC_OFFSET+1)

        // This event indicates that a previous write request has completed.
        // These events are enabled if USB Embedded Host transfer events are
        // enabled (USB_ENABLE_TRANSFER_EVENT is defined).  When
        // USB_HOST_APP_EVENT_HANDLER is called with this event, *data points
        // to the buffer that completed transmission, and size is the actual
        // number of bytes that were written to the device.
#define EVENT_GENERIC_TX_DONE (EVENT_GENERIC_BASE+EVENT_GENERIC_OFFSET+2)

        // This event indicates that a previous read request has completed.
        // These events are enabled if USB Embedded Host transfer events are
        // enabled (USB_ENABLE_TRANSFER_EVENT is defined).  When
        // USB_HOST_APP_EVENT_HANDLER is called with this event, *data points
        // to the receive buffer, and size is the actual number of bytes read
        // from the device.
#define EVENT_GENERIC_RX_DONE (EVENT_GENERIC_BASE+EVENT_GENERIC_OFFSET+3)


// *****************************************************************************
// *****************************************************************************
// Section: USB Data Structures
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Generic Device ID Information

This structure contains identification information about an attached device.
*/
typedef struct _GENERIC_DEVICE_ID
{
    WORD        vid;                    // Vendor ID of the device
    WORD        pid;                    // Product ID of the device
    #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
        WORD   *serialNumber;           // Pointer to the Unicode serial number string
        BYTE    serialNumberLength;     // Length of the serial number string (in Unicode characters)
    #endif
    BYTE        deviceAddress;          // Address of the device on the USB
} GENERIC_DEVICE_ID;


// *****************************************************************************
/* Generic Device Information

This structure contains information about an attached device, including
status flags and device identification.
*/
typedef struct _GENERIC_DEVICE
{
    GENERIC_DEVICE_ID   ID;             // Identification information about the device
    DWORD               rxLength;       // Number of bytes received in the last IN transfer
    BYTE                clientDriverID; // ID to send when issuing a Device Request
    
    #ifndef USB_ENABLE_TRANSFER_EVENT
        BYTE            rxErrorCode;    // Error code of last IN transfer
        BYTE            txErrorCode;    // Error code of last OUT transfer
    #endif

    union
    {
        BYTE val;                       // BYTE representation of device status flags
        struct
        {
            BYTE initialized    : 1;    // Driver has been initialized
            BYTE txBusy         : 1;    // Driver busy transmitting data
            BYTE rxBusy         : 1;    // Driver busy receiving data
            #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
                BYTE serialNumberValid    : 1;    // Serial number is valid
            #endif
        };
    } flags;                            // Generic client driver status flags

} GENERIC_DEVICE;


// *****************************************************************************
// *****************************************************************************
// Section: Global Variables
// *****************************************************************************
// *****************************************************************************

extern GENERIC_DEVICE   gc_DevData; // Information about the attached device.

// *****************************************************************************
// *****************************************************************************
// Section: Host Stack Interface Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BOOL USBHostGenericInit ( BYTE address, DWORD flags, BYTE clientDriverID )

  Summary:
    This function is called by the USB Embedded Host layer when a "generic"
    device attaches.

  Description:
    This routine is a call out from the USB Embedded Host layer to the USB
    generic client driver.  It is called when a "generic" device has been
    connected to the host.  Its purpose is to initialize and activate the USB
    Generic client driver.

  Preconditions:
    The device has been configured.

  Parameters:
    BYTE address    - Device's address on the bus
    DWORD flags     - Initialization flags
    BYTE clientDriverID - ID to send when issuing a Device Request via
                            USBHostIssueDeviceRequest(), USBHostSetDeviceConfiguration(),
                            or USBHostSetDeviceInterface().  

  Return Values:
    TRUE    - Initialization was successful
    FALSE   - Initialization failed

  Remarks:
    Multiple client drivers may be used in a single application.  The USB
    Embedded Host layer will call the initialize routine required for the
    attached device.
  ***************************************************************************/

BOOL USBHostGenericInit ( BYTE address, DWORD flags, BYTE clientDriverID );


/****************************************************************************
  Function:
    BOOL USBHostGenericEventHandler ( BYTE address, USB_EVENT event,
                            void *data, DWORD size )

  Summary:
    This routine is called by the Host layer to notify the general client of
    events that occur.

  Description:
    This routine is called by the Host layer to notify the general client of
    events that occur.  If the event is recognized, it is handled and the
    routine returns TRUE.  Otherwise, it is ignored and the routine returns
    FALSE.

  Preconditions:
    None

  Parameters:
    BYTE address    - Address of device with the event
    USB_EVENT event - The bus event that occured
    void *data      - Pointer to event-specific data
    DWORD size      - Size of the event-specific data

  Return Values:
    TRUE    - The event was handled
    FALSE   - The event was not handled

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostGenericEventHandler ( BYTE address, USB_EVENT event, void *data, DWORD size );


// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes and Macro Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BOOL API_VALID( BYTE address )

  Description:
    This function is used internally to ensure that the requested device is
    attached and initialized before performing an operation.

  Preconditions:
    None

  Parameters:
    BYTE address    - USB address of the device

  Returns:
    TRUE    - A device with the requested address is attached and initialized.
    FALSE   - A device with the requested address is not available, or it
                has not been initialized.

  Remarks:
    None
  ***************************************************************************/

#define API_VALID(a) ( (((a)==gc_DevData.ID.deviceAddress) && gc_DevData.flags.initialized == 1) ? TRUE : FALSE )


/****************************************************************************
  Function:
    BOOL USBHostGenericDeviceDetached( BYTE deviceAddress )

  Description:
    This interface is used to check if the devich has been detached from the
    bus.

  Preconditions:
    None

  Parameters:
    BYTE deviceAddress	- USB Address of the device.

  Return Values:
    TRUE    - The device has been detached, or an invalid deviceAddress is given.
    FALSE   - The device is attached

  Example:
    <code>
    if (USBHostGenericDeviceDetached( deviceAddress ))
    {
        // Handle detach
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

#define USBHostGenericDeviceDetached(a) ( (((a)==gc_DevData.ID.deviceAddress) && gc_DevData.flags.initialized == 1) ? FALSE : TRUE )
//BOOL USBHostGenericDeviceDetached( BYTE deviceAddress );


/****************************************************************************
  Function:
    BOOL USBHostGenericGetDeviceAddress(GENERIC_DEVICE_ID *pDevID)

  Description:
    This interface is used get the address of a specific generic device on
    the USB.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    GENERIC_DEVICE_ID* pDevID  - Pointer to a structure containing the Device ID Info (VID,
                    		 PID, serial number, and device address).

  Return Values:
    TRUE    - The device is connected
    FALSE   - The device is not connected.

  Example:
    <code>
    GENERIC_DEVICE_ID   deviceID;
    WORD                serialNumber[] = { '1', '2', '3', '4', '5', '6' };
    BYTE                deviceAddress;

    deviceID.vid          = 0x1234;
    deviceID.pid          = 0x5678;
    deviceID.serialNumber = &serialNumber;

    if (USBHostGenericGetDeviceAddress(&deviceID))
    {
        deviceAddress = deviceID.deviceAddress;
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostGenericGetDeviceAddress(GENERIC_DEVICE_ID *pDevID);


/****************************************************************************
  Function:
    DWORD USBHostGenericGetRxLength( BYTE deviceAddress )

  Description:
    This function retrieves the number of bytes copied to user's buffer by
    the most recent call to the USBHostGenericRead() function.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    BYTE deviceAddress	- USB Address of the device

  Returns:
    Returns the number of bytes most recently received from the Generic
    device with address deviceAddress.

  Remarks:
    This function can only be called once per transfer.  Subsequent calls will
    return zero until new data has been received.
  ***************************************************************************/

#define USBHostGenericGetRxLength(a) ( (API_VALID(a)) ? gc_DevData.rxLength : 0 )
//DWORD USBHostGenericGetRxLength( BYTE deviceAddress );


/****************************************************************************
  Function:
    void USBHostGenericRead( BYTE deviceAddress, BYTE *buffer, DWORD length )

  Description:
    Use this routine to receive from the device and store it into memory.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    BYTE deviceAddress  - USB Address of the device.
    BYTE *buffer        - Pointer to the data buffer
    DWORD length        - Number of bytes to be transferred

  Return Values:
    USB_SUCCESS         - The Read was started successfully
    (USB error code)    - The Read was not started.  See USBHostRead() for
                            a list of errors.

  Example:
    <code>
    if (!USBHostGenericRxIsBusy( deviceAddress ))
    {
        USBHostGenericRead( deviceAddress, &buffer, sizeof(buffer) );
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BYTE USBHostGenericRead( BYTE deviceAddress, void *buffer, DWORD length);
/* Macro Implementation:
#define USBHostGenericRead(a,b,l)                                                 \
        ( API_VALID(deviceAddress) ? USBHostRead((a),USB_GENERIC_EP,(BYTE *)(b),(l)) : \
                              USB_INVALID_STATE )
*/


/****************************************************************************
  Function:
    BOOL USBHostGenericRxIsBusy( BYTE deviceAddress )

  Summary:
    This interface is used to check if the client driver is currently busy
    receiving data from the device.

  Description:
    This interface is used to check if the client driver is currently busy
    receiving data from the device.  This function is intended for use with
    transfer events.  With polling, the function USBHostGenericRxIsComplete()
    should be used.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    BYTE deviceAddress     - USB Address of the device

  Return Values:
    TRUE    - The device is receiving data or an invalid deviceAddress is
                given.
    FALSE   - The device is not receiving data

  Example:
    <code>
    if (!USBHostGenericRxIsBusy( deviceAddress ))
    {
        USBHostGenericRead( deviceAddress, &buffer, sizeof( buffer ) );
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

#define USBHostGenericRxIsBusy(a) ( (API_VALID(a)) ? ((gc_DevData.flags.rxBusy == 1) ? TRUE : FALSE) : TRUE )
//BOOL USBHostGenericRxIsBusy( BYTE deviceAddress );


/****************************************************************************
  Function:
    BOOL USBHostGenericRxIsComplete( BYTE deviceAddress, BYTE *errorCode,
                DWORD *byteCount )

  Summary:
    This routine indicates whether or not the last IN transfer is complete.

  Description:
    This routine indicates whether or not the last IN transfer is complete.
    If it is, then the returned errorCode and byteCount are valid, and
    reflect the error code and the number of bytes received.

    This function is intended for use with polling.  With transfer events,
    the function USBHostGenericRxIsBusy() should be used.

  Preconditions:
    None

  Parameters:
    BYTE deviceAddress  - Address of the attached peripheral
    BYTE *errorCode     - Error code of the last transfer, if complete
    DWORD *byteCount    - Bytes transferred during the last transfer, if
                            complete

  Return Values:
    TRUE    - The IN transfer is complete.  errorCode and byteCount are valid.
    FALSE   - The IN transfer is not complete.  errorCode and byteCount are
                invalid.

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostGenericRxIsComplete( BYTE deviceAddress,
                                    BYTE *errorCode, DWORD *byteCount );


/****************************************************************************
  Function:
    void USBHostGenericTasks( void )

  Summary:
    This routine is used if transfer events are not utilized. It monitors the
    host status and updates the transmit and receive flags.

  Description:
    This routine is used if transfer events are not utilized. It monitors the
    host status and updates the transmit and receive flags.  If serial
    numbers are supported, then this routine handles the reception of the
    serial number.

  Preconditions:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    This function is compiled only if USB_ENABLE_TRANSFER_EVENT is not
    defined.
  ***************************************************************************/

#ifndef USB_ENABLE_TRANSFER_EVENT
void USBHostGenericTasks( void );
#endif


/****************************************************************************
  Function:
    BOOL USBHostGenericTxIsBusy( BYTE deviceAddress )

  Summary:
    This interface is used to check if the client driver is currently busy
    transmitting data to the device.

  Description:
    This interface is used to check if the client driver is currently busy
    transmitting data to the device.  This function is intended for use with
    transfer events.  With polling, the function USBHostGenericTxIsComplete()
    should be used.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    BYTE deviceAddress	- USB Address of the device

  Return Values:
    TRUE    - The device is transmitting data or an invalid deviceAddress
                is given.
    FALSE   - The device is not transmitting data

  Example:
    <code>
    if (!USBHostGenericTxIsBusy( deviceAddress ) )
    {
        USBHostGenericWrite( deviceAddress, &buffer, sizeof( buffer ) );
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

#define USBHostGenericTxIsBusy(a) ( (API_VALID(a)) ? ((gc_DevData.flags.txBusy == 1) ? TRUE : FALSE) : TRUE )
//BOOL USBHostGenericTxIsBusy( BYTE deviceAddress );


/****************************************************************************
  Function:
    BOOL USBHostGenericTxIsComplete( BYTE deviceAddress, BYTE *errorCode )

  Summary:
    This routine indicates whether or not the last OUT transfer is complete.

  Description:
    This routine indicates whether or not the last OUT transfer is complete.
    If it is, then the returned errorCode is valid, and reflect the error
    code of the transfer.

    This function is intended for use with polling.  With transfer events,
    the function USBHostGenericTxIsBusy() should be used.

  Preconditions:
    None

  Parameters:
    BYTE deviceAddress  - Address of the attached peripheral
    BYTE *errorCode     - Error code of the last transfer, if complete

  Return Values:
    TRUE    - The OUT transfer is complete.  errorCode is valid.
    FALSE   - The OUT transfer is not complete.  errorCode is invalid.

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostGenericTxIsComplete( BYTE deviceAddress, BYTE *errorCode );


/****************************************************************************
  Function:
    void USBHostGenericWrite( BYTE deviceAddress, BYTE *buffer, DWORD length )

  Description:
    Use this routine to transmit data from memory to the device.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    BYTE deviceAddress   - USB Address of the device.
    BYTE *buffer         - Pointer to the data buffer
    DWORD length         - Number of bytes to be transferred

  Return Values:
    USB_SUCCESS         - The Write was started successfully
    (USB error code)    - The Write was not started.  See USBHostWrite() for
                            a list of errors.

  Example:
    <code>
    if (!USBHostGenericTxIsBusy( deviceAddress ))
    {
        USBHostGenericWrite( deviceAddress, &buffer, sizeof(buffer) );
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BYTE USBHostGenericWrite( BYTE deviceAddress, void *buffer, DWORD length);
/* Macro Implementation:
#define USBHostGenericWrite(a,b,l)                                                 \
        ( API_VALID(deviceAddress) ? USBHostWrite((a),USB_GENERIC_EP,(BYTE *)(b),(l)) : \
                              USB_INVALID_STATE )
*/


/*************************************************************************
 * EOF usb_client_generic.h
 */

#endif
