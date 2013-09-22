/*******************************************************************************

    USB Host Charger Client Driver (Header File)

Description:
    This is the Charger client driver file for a USB Embedded Host device.  This
    driver should be used in a project with usb_host.c to provided the USB
    hardware interface.

    To interface with USB Embedded Host layer, the routine USBHostChargerInit()
    should be specified as the Initialize() function, and
    USBHostChargerEventHandler() should be specified as the EventHandler()
    function in the usbClientDrvTable[] array declared in usb_config.c.

    This driver can be used in either the event driven or polling mechanism.

Summary:
    This is the Charger client driver file for a USB Embedded Host device.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

* FileName:        usb_client_charger.h
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
  Rev    Description
  -----  ----------------------------------
  2.6a-  No change
   2.7a
*******************************************************************************/
#ifndef __USBHOSTCHARGER_H__
#define __USBHOSTCHARGER_H__
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Max Number of Supported Devices

This value represents the maximum number of attached devices this client driver
can support.  If the user does not define a value, it will be set to 1.
*/
#ifndef USB_MAX_CHARGING_DEVICES
    #define USB_MAX_CHARGING_DEVICES     1
#endif


// *****************************************************************************
// *****************************************************************************
// Section: USB Charger Client Events
// *****************************************************************************
// *****************************************************************************

        // This is an optional offset for the values of the generated events.
        // If necessary, the application can use a non-zero offset for the
        // generic events to resolve conflicts in event number.
#ifndef EVENT_CHARGER_OFFSET
#define EVENT_CHARGER_OFFSET 0
#endif

        // This event indicates that a device has been attached for charging.
        // When USB_HOST_APP_EVENT_HANDLER is called with this event, *data
        // points to a USB_CHARGING_DEVICE_ID structure, and size is the size of the
        // USB_CHARGING_DEVICE_ID structure.
#define EVENT_CHARGER_ATTACH    (EVENT_CHARGER_BASE+EVENT_CHARGER_OFFSET+0)

        // This event indicates that the specified device has been detached
        // from the USB.  When USB_HOST_APP_EVENT_HANDLER is called with this
        // event, *data points to a BYTE that contains the device address, and
        // size is the size of a BYTE.
#define EVENT_CHARGER_DETACH    (EVENT_CHARGER_BASE+EVENT_CHARGER_OFFSET+1)


// *****************************************************************************
// *****************************************************************************
// Section: USB Data Structures
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Charging Device ID Information

This structure contains identification information about an attached device.
*/
typedef struct
{
    WORD        vid;                    // Vendor ID of the device
    WORD        pid;                    // Product ID of the device
    BYTE        deviceAddress;          // Address of the device on the USB
    BYTE        clientDriverID;         // Client driver ID for device requests.
} USB_CHARGING_DEVICE_ID;


// *****************************************************************************
/* Charging Device Information

This structure contains information about an attached device, including
status flags and device identification.
*/
typedef struct
{
    USB_CHARGING_DEVICE_ID   ID;        // Identification information about the device

    union
    {
        BYTE val;                       // BYTE representation of device status flags
        struct
        {
            BYTE    inUse       : 1;    // Record is being used.  DO NOT MODIFY
        };
    } flags;                            // Generic client driver status flags

} USB_CHARGING_DEVICE;


// *****************************************************************************
// *****************************************************************************
// Section: Global Variables
// *****************************************************************************
// *****************************************************************************

extern USB_CHARGING_DEVICE   usbChargingDevices[USB_MAX_CHARGING_DEVICES]; // Information about the attached device.

// *****************************************************************************
// *****************************************************************************
// Section: Host Stack Interface Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BOOL USBHostChargerInitialize ( BYTE address, DWORD flags, BYTE clientDriverID )

  Summary:
    This function is called by the USB Embedded Host layer when a device
    attaches for charging.

  Description:
    This routine is a call out from the USB Embedded Host layer to the USB
    charger client driver.  It is called when a device that is supported for
    charging only has been connected to the host.  Its purpose is to initialize
    and activate the USB Charger client driver.

  Preconditions:
    The device has been configured.

  Parameters:
    BYTE address    - Device's address on the bus
    DWORD flags     - Initialization flags
    BYTE clientDriverID - ID to send when issuing a Device Request via
                            USBHostIssueDeviceRequest() or USBHostSetDeviceConfiguration().

  Return Values:
    TRUE    - Initialization was successful
    FALSE   - Initialization failed

  Remarks:
    Multiple client drivers may be used in a single application.  The USB
    Embedded Host layer will call the initialize routine required for the
    attached device.
  ***************************************************************************/

BOOL USBHostChargerInitialize( BYTE address, DWORD flags, BYTE clientDriverID );


/****************************************************************************
  Function:
    BOOL USBHostChargerEventHandler ( BYTE address, USB_EVENT event,
                            void *data, DWORD size )

  Summary:
    This routine is called by the Host layer to notify the charger client of
    events that occur.

  Description:
    This routine is called by the Host layer to notify the charger client of
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

BOOL USBHostChargerEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size );


// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes and Macro Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BOOL USBHostChargerDeviceDetached( BYTE deviceAddress )

  Description:
    This interface is used to check if the devich has been detached from the
    bus.

  Preconditions:
    None

  Parameters:
    deviceAddress     - USB Address of the device.

  Return Values:
    TRUE    - The device has been detached, or an invalid deviceAddress is given.
    FALSE   - The device is attached

  Example:
    <code>
    if (USBHostChargerDeviceDetached( deviceAddress ))
    {
        // Handle detach
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostChargerDeviceDetached( BYTE deviceAddress );


/****************************************************************************
  Function:
    BOOL USBHostChargerGetDeviceAddress(USB_CHARGING_DEVICE_ID *pDevID)

  Description:
    This interface is used get the address of a specific generic device on
    the USB.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    pDevID  - Pointer to a structure containing the Device ID Info (VID,
                    PID, and device address).

  Return Values:
    TRUE    - The device is connected
    FALSE   - The device is not connected.

  Example:
    <code>
    USB_CHARGING_DEVICE_ID  deviceID;
    BYTE                    deviceAddress;

    deviceID.vid          = 0x1234;
    deviceID.pid          = 0x5678;

    if (USBHostChargerGetDeviceAddress(&deviceID))
    {
        deviceAddress = deviceID.deviceAddress;
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostChargerGetDeviceAddress(USB_CHARGING_DEVICE_ID *pDevID);


/*************************************************************************
 * EOF
 */

#endif
