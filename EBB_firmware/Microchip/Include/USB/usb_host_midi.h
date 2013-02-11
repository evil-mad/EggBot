/*******************************************************************************

    USB Host MIDI Client Driver (Header File)

Description:
    This is the MIDI client driver file for a USB Embedded Host device.  This
    driver should be used in a project with usb_host.c to provided the USB
    hardware interface.

    To interface with USB Embedded Host layer, the routine USBHostMIDIInit()
    should be specified as the Initialize() function, and
    USBHostMICIEventHandler() should be specified as the EventHandler()
    function in the usbClientDrvTable[] array declared in usb_config.c.

    This driver can be configured to either use transfer events from usb_host.c
    or use a polling mechanism.  If USB_ENABLE_TRANSFER_EVENT is defined, this
    driver will utilize transfer events.  Otherwise, this driver will utilize
    polling.

Since the MIDI class is performed with bulk transfers,
USB_SUPPORT_BULK_TRANSFERS must be defined.

This driver has been tested with the following USB keyboards:
Akai Professional Synth Station 25
Korg Nano Key
eKeys 37
Ion Discover Keyboard USB
M-Audio Oxygen 25
Prodipe MIDI USB Keyboard Controller 25C
Alesis Q49

Summary:
    This is the MIDI client driver file for a USB Embedded Host device.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

* FileName:        usb_host_midi.h
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
TL       17-Oct-2011    Preliminary release

*******************************************************************************/
#ifndef __USBHOSTMIDI_H__
#define __USBHOSTMIDI_H__
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

// The Audio Class, MIDI Subclass, and MIDI Protocol
#define AUDIO_CLASS     1
#define MIDI_SUB_CLASS  3
#define MIDI_PROTOCOL   0

// This is the default MIDI Client Driver endpoint number.
#ifndef USB_MIDI_EP
    #define USB_MIDI_EP       1
#endif

//Each USB-MIDI packet is supposed
//to be exactly 32-bits (4 bytes long)
#define USB_MIDI_PACKET_LENGTH (BYTE)4

//State machine definitions, when receiving a MIDI command on the MIDI In jack
#define STATE_WAIT_STATUS_BYTE  0x00
#define STATE_WAITING_BYTES     0x01

/* Code Index Number (CIN) values */
/*   Table 4-1 of midi10.pdf      */
#define MIDI_CIN_MISC_FUNCTION_RESERVED         0x0
#define MIDI_CIN_CABLE_EVENTS_RESERVED          0x1
#define MIDI_CIN_2_BYTE_MESSAGE                 0x2
#define MIDI_CIN_MTC                            0x2
#define MIDI_CIN_SONG_SELECT                    0x2
#define MIDI_CIN_3_BYTE_MESSAGE                 0x3
#define MIDI_CIN_SSP                            0x3
#define MIDI_CIN_SYSEX_START                    0x4
#define MIDI_CIN_SYSEX_CONTINUE                 0x4
#define MIDI_CIN_1_BYTE_MESSAGE                 0x5
#define MIDI_CIN_SYSEX_ENDS_1                   0x5
#define MIDI_CIN_SYSEX_ENDS_2                   0x6
#define MIDI_CIN_SYSEX_ENDS_3                   0x7
#define MIDI_CIN_NOTE_OFF                       0x8
#define MIDI_CIN_NOTE_ON                        0x9
#define MIDI_CIN_POLY_KEY_PRESS                 0xA
#define MIDI_CIN_CONTROL_CHANGE                 0xB
#define MIDI_CIN_PROGRAM_CHANGE                 0xC
#define MIDI_CIN_CHANNEL_PREASURE               0xD
#define MIDI_CIN_PITCH_BEND_CHANGE              0xE
#define MIDI_CIN_SINGLE_BYTE                    0xF

//Possible system command status bytes
#define MIDI_STATUS_SYSEX_START                 0xF0
#define MIDI_STATUS_SYSEX_END                   0xF7
#define MIDI_STATUS_MTC_QFRAME                  0xF1
#define MIDI_STATUS_SONG_POSITION               0xF2
#define MIDI_STATUS_SONG_SELECT                 0xF3
#define MIDI_STATUS_TUNE_REQUEST                0xF6
#define MIDI_STATUS_MIDI_CLOCK                  0xF8
#define MIDI_STATUS_MIDI_TICK                   0xF9
#define MIDI_STATUS_MIDI_START                  0xFA
#define MIDI_STATUS_MIDI_STOP                   0xFC
#define MIDI_STATUS_MIDI_CONTINUE               0xFB
#define MIDI_STATUS_ACTIVE_SENSE                0xFE
#define MIDI_STATUS_RESET                       0xFF

// *****************************************************************************
// *****************************************************************************
// Section: USB MIDI Client Events
// *****************************************************************************
// *****************************************************************************

        // This is an optional offset for the values of the generated events.
        // If necessary, the application can use a non-zero offset for the
        // MIDI events to resolve conflicts in event number.
#ifndef EVENT_MIDI_OFFSET
#define EVENT_MIDI_OFFSET 0
#endif

        // This event indicates that a MIDI device has been attached.
        // When USB_HOST_APP_EVENT_HANDLER is called with this event, *data
        // points to a MIDI_DEVICE_ID structure, and size is the size of the
        // MIDI_DEVICE_ID structure.
#define EVENT_MIDI_ATTACH  (EVENT_AUDIO_BASE+EVENT_MIDI_OFFSET+0)

        // This event indicates that the specified device has been detached
        // from the USB.  When USB_HOST_APP_EVENT_HANDLER is called with this
        // event, *data points to a BYTE that contains the device address, and
        // size is the size of a BYTE.
#define EVENT_MIDI_DETACH  (EVENT_AUDIO_BASE+EVENT_MIDI_OFFSET+1)

        // This event indicates that a previous write/read request has completed.
        // These events are enabled if USB Embedded Host transfer events are
        // enabled (USB_ENABLE_TRANSFER_EVENT is defined).  When
        // USB_HOST_APP_EVENT_HANDLER is called with this event, *data points
        // to the buffer that completed transmission, and size is the actual
        // number of bytes that were written to the device.
#define EVENT_MIDI_TRANSFER_DONE (EVENT_AUDIO_BASE+EVENT_MIDI_OFFSET+2)


// *****************************************************************************
// *****************************************************************************
// Section: USB Data Structures
// *****************************************************************************
// *****************************************************************************

typedef enum
{
    OUT = 0,
    IN = 1
}ENDPOINT_DIRECTION;    

typedef struct
{
    BOOL busy;                  // Driver busy transmitting or receiving data
    BYTE endpointAddress;       // The endpoint number (1-15, 0 is reserved for control transfers) and direction (0x8X = input, 0x0X = output)
    WORD endpointSize;          // Size of the endpoint (we'll take any size <= 64)
} MIDI_ENDPOINT_DATA;           // MIDI endpoints

// *****************************************************************************
/* MIDI Device Information

This structure contains information about an attached device, including
status flags and device identification.
*/
typedef struct
{
    BYTE deviceAddress;     // Address of the device on the USB
    BYTE clientDriverID;    // ID to send when issuing a Device Request
    
    BYTE numEndpoints;               // Number of OUT endpoints for this interface
    MIDI_ENDPOINT_DATA* endpoints;   // List of OUT endpoints
} MIDI_DEVICE;

// *****************************************************************************
/* MIDI Packet information

This structure contains information contained within a USB MIDI packet
*/
typedef union
{
    DWORD Val;
    BYTE v[4];
    union
    {
        struct
        {
            BYTE CIN :4;
            BYTE CN  :4;
            BYTE MIDI_0;
            BYTE MIDI_1;
            BYTE MIDI_2;
        }; 
        struct
        {
            BYTE CodeIndexNumber :4;
            BYTE CableNumber     :4;
            BYTE DATA_0;
            BYTE DATA_1;
            BYTE DATA_2;    
        };
    };
} USB_AUDIO_MIDI_PACKET;


// *****************************************************************************
// *****************************************************************************
// Section: Global Variables
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Host Stack Interface Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BOOL USBHostMIDIInit ( BYTE address, DWORD flags, BYTE clientDriverID )

  Summary:
    This function is called by the USB Embedded Host layer when a MIDI
    device attaches.

  Description:
    This routine is a call out from the USB Embedded Host layer to the USB
    MIDI client driver.  It is called when a MIDI device has been connected
    to the host.  Its purpose is to initialize and activate the USB
    MIDI client driver.

  Preconditions:
    The device has been configured.

  Parameters:
    BYTE address        - Device's address on the bus
    DWORD flags         - Initialization flags
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

BOOL USBHostMIDIInit ( BYTE address, DWORD flags, BYTE clientDriverID );


/****************************************************************************
  Function:
    BOOL USBHostMIDIEventHandler ( BYTE address, USB_EVENT event,
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

BOOL USBHostMIDIEventHandler ( BYTE address, USB_EVENT event, void *data, DWORD size );


// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes and Macro Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BOOL USBHostMIDIDeviceDetached( void* handle )

  Description:
    This interface is used to check if the device has been detached from the
    bus.

  Preconditions:
    None

  Parameters:
    void* handle - Pointer to a structure containing the Device Info

  Return Values:
    TRUE    - The device has been detached, or an invalid handle is given.
    FALSE   - The device is attached

  Example:
    <code>
    if (USBHostMIDIDeviceDetached( deviceAddress ))
    {
        // Handle detach
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

#define USBHostMIDIDeviceDetached(a) ( (((a)==NULL) ? FALSE : TRUE )
//BOOL USBHostMIDIDeviceDetached( void* handle );


/****************************************************************************
  Function:
    MIDI_ENDPOINT_DIRECTION USBHostMIDIEndpointDirection( void* handle, BYTE endpointIndex )

  Description:
    This function retrieves the endpoint direction of the endpoint at
    endpointIndex for device that's located at handle.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    void* handle       - Pointer to a structure containing the Device Info
    BYTE endpointIndex - the index of the endpoint whose direction is requested

  Returns:
    MIDI_ENDPOINT_DIRECTION - Returns the direction of the endpoint (IN or OUT)

  Remarks:
    None
  ***************************************************************************/
  
#define USBHostMIDIEndpointDirection(a,b) (((MIDI_DEVICE*)a)->endpoints[b].endpointAddress & 0x80) ? IN : OUT
//MIDI_ENDPOINT_DIRECTION USBHostMIDIEndpointDirection( void* handle, BYTE endpointIndex );


/****************************************************************************
  Function:
    DWORD USBHostMIDISizeOfEndpoint( void* handle, BYTE endpointIndex )

  Description:
    This function retrieves the endpoint size of the endpoint at 
    endpointIndex for device that's located at handle.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    void* handle       - Pointer to a structure containing the Device Info
    BYTE endpointIndex - the index of the endpoint whose direction is requested

  Returns:
    DWORD - Returns the number of bytes for the endpoint (4 - 64 bytes per USB spec)

  Remarks:
    None
  ***************************************************************************/

#define USBHostMIDISizeOfEndpoint(a,b) ((MIDI_DEVICE*)a)->endpoints[b].endpointSize
//DWORD USBHostMIDISizeOfEndpoint( void* handle, BYTE endpointNumber );


/****************************************************************************
  Function:
    BYTE USBHostMIDINumberOfEndpoints( void* handle )

  Description:
    This function retrieves the number of endpoints for the device that's
    located at handle.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    void* handle - Pointer to a structure containing the Device Info

  Returns:
    BYTE - Returns the number of endpoints for the device at handle.

  Remarks:
    None
  ***************************************************************************/
  
#define USBHostMIDINumberOfEndpoints(a) ((MIDI_DEVICE*)a)->numEndpoints
//BYTE USBHostMIDINumberOfEndpoints( void* handle );


/****************************************************************************
  Function:
    BYTE USBHostMIDIRead( void* handle, BYTE endpointIndex, void *buffer, WORD length)

  Description:
    This function will attempt to read length number of bytes from the attached MIDI
    device located at handle, and will save the contents to ram located at buffer.

  Preconditions:
    The device must be connected and enumerated. The array at *buffer should have
    at least length number of bytes available.

  Parameters:
    void* handle       - Pointer to a structure containing the Device Info
    BYTE endpointIndex - the index of the endpoint whose direction is requested
    void* buffer       - Pointer to the data buffer
    WORD length        - Number of bytes to be read

  Return Values:
    USB_SUCCESS         - The Read was started successfully
    (USB error code)    - The Read was not started.  See USBHostRead() for
                            a list of errors.

  Example:
    <code>
    if (!USBHostMIDITransferIsBusy( deviceHandle, currentEndpoint )
    {
        USBHostMIDIRead( deviceHandle, currentEndpoint, &buffer, sizeof(buffer));
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BYTE USBHostMIDIRead( void* handle, BYTE endpointIndex, void *buffer, WORD length);


/****************************************************************************
  Function:
    BOOL USBHostMIDITransferIsBusy( void* handle, BYTE endpointIndex )

  Summary:
    This interface is used to check if the client driver is currently busy
    transferring data over endponitIndex for the device at handle.

  Description:
    This interface is used to check if the client driver is currently busy
    receiving or sending data from the device at the endpoint with number
    endpointIndex.  This function is intended for use with transfer events.
    With polling, the function USBHostMIDITransferIsComplete()
    should be used.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    void* handle       - Pointer to a structure containing the Device Info
    BYTE endpointIndex - the index of the endpoint whose direction is requested

  Return Values:
    TRUE    - The device is receiving data or an invalid handle is
                given.
    FALSE   - The device is not receiving data

  Example:
    <code>
    if (!USBHostMIDITransferIsBusy( handle, endpointIndex ))
    {
        USBHostMIDIRead( handle, endpointIndex, &buffer, sizeof( buffer ) );
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

#define USBHostMIDITransferIsBusy(a,b) ((MIDI_DEVICE*)a)->endpoints[b].busy
//BOOL USBHostMIDITransferIsBusy( void* handle, BYTE endpointIndex )


/****************************************************************************
  Function:
    BOOL USBHostMIDITransferIsComplete( void* handle, BYTE endpointIndex,
                                        BYTE *errorCode, DWORD *byteCount );

  Summary:
    This routine indicates whether or not the last transfer over endpointIndex
    is complete.

  Description:
    This routine indicates whether or not the last transfer over endpointIndex
    is complete. If it is, then the returned errorCode and byteCount are valid,
    and reflect the error code and the number of bytes received.

    This function is intended for use with polling.  With transfer events,
    the function USBHostMIDITransferIsBusy() should be used.

  Preconditions:
    None

  Parameters:
    void* handle        - Pointer to a structure containing the Device Info
    BYTE endpointIndex  - index of endpoint in endpoints array
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

#ifndef USB_ENABLE_TRANSFER_EVENT
BOOL USBHostMIDITransferIsComplete( void* handle, BYTE endpointIndex, BYTE *errorCode, DWORD *byteCount );
#endif


/****************************************************************************
  Function:
    BYTE USBHostMIDIWrite(void* handle, BYTE endpointIndex, void *buffer, WORD length)

  Description:
    This function will attempt to write length number of bytes from memory at location
    buffer to the attached MIDI device located at handle.

  Preconditions:
    The device must be connected and enumerated. The array at *buffer should have
    at least length number of bytes available.

  Parameters:
    handle          - Pointer to a structure containing the Device Info
    endpointIndex   - Index of the endpoint
    buffer          - Pointer to the data being transferred
    length          - Size of the data being transferred

  Return Values:
    USB_SUCCESS         - The Write was started successfully
    (USB error code)    - The Write was not started.  See USBHostWrite() for
                            a list of errors.

  Example:
    <code>
    if (!USBHostMIDITransferIsBusy( deviceHandle, currentEndpoint )
    {
        USBHostMIDIWrite( deviceAddress, &buffer, sizeof(buffer) );
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BYTE USBHostMIDIWrite(void* handle, BYTE endpointIndex, void *buffer, WORD length);


/*************************************************************************
 * EOF usb_client_MIDI.h
 */

#endif
