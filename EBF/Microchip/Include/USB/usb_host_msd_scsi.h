/*******************************************************************************

  USB Host Mass Storage Class SCSI Interface Driver (Header File)

Description:
    This is the header file for a USB Embedded Host that is using a SCSI
    interface to the Mass Storage Class.

    This file provides the interface between the file system and the USB Host
    Mass Storage class.  It translates the file system funtionality requirements
    to the appropriate SCSI commands, and sends the SCSI commands via the USB
    Mass Storage class.  This header file should be included with usb_host.h and
    usb_host_msd.h to provide the USB Mass Storage Class interface.  It must
    be included after the application-specific usb_config.h file, the USB Host
    header file usb_host.h, and the USB Host MSD header file usb_host_msd.h,
    as definitions in those files are required for proper compilation.

    The functions in this file are designed to interface the Microchip Memory
    Disk Drive File System library (see Application Note AN1045) to the USB
    Host Mass Storage Class, allowing a PIC application to utilize mass storage
    devices such as USB flash drives.  For ease of integration, this file
    contains macros to allow the File System code to reference the functions in
    this file.

    Currently, the file system layer above this interface layer is limited to
    one LUN (Logical Unit Number) on a single mass storage device.  This layer
    accepts and stores the max LUN from the USB MSD layer, but all sector reads
    and writes are hard-coded to LUN 0, since the layer above does not specify
    a LUN in the sector read and write commands.  Also, to interface with the
    existing file system code, only one attached device is allowed.

Summary:
    This is the header file for a USB Embedded Host that is using a SCSI
    interface to the Mass Storage Class.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************

* FileName:        usb_host_msd_scsi.h
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
  Rev   Description
  ----  --------------------------------------
  2.6a- No change
   2.7
*******************************************************************************/

#ifndef __USBHOSTMSDSCSI_H__
#define __USBHOSTMSDSCSI_H__
//DOM-IGNORE-END

#include "USB/usb.h"
#include "FSConfig.h"
#include "MDD File System/FSDefs.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BYTE USBHostMSDSCSIMediaDetect( void )

  Description:
    This function determines if a mass storage device is attached and ready
    to use.

  Precondition:
    None

  Parameters:
    None - None

  Return Values:
    TRUE    -   MSD present and ready
    FALSE   -   MSD not present or not ready

  Remarks:
    Since this will often be called in a loop while waiting for a device,
    we need to make sure that USB tasks are executed.
  ***************************************************************************/

BYTE    USBHostMSDSCSIMediaDetect( void );


/****************************************************************************
  Function:
    MEDIA_INFORMATION * USBHostMSDSCSIMediaInitialize( void )

  Description:
    This function initializes the media.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    The function returns a pointer to the MEDIA_INFORMATION structure.  The
    errorCode member may contain the following values:
        * MEDIA_NO_ERROR - The media initialized successfully, and the 
                sector size should be valid (confirm using the validityFlags 
                bit). 
        * MEDIA_DEVICE_NOT_PRESENT - The requested device is not attached.
        * MEDIA_CANNOT_INITIALIZE - Cannot initialize the media.

  Remarks:
    This function performs the following SCSI commands:
                        * READ CAPACITY 10
                        * REQUEST SENSE

    The READ CAPACITY 10 command block is as follows:

    <code>
        Byte/Bit    7       6       5       4       3       2       1       0
           0                    Operation Code (0x25)
           1        [                      Reserved                         ]
           2        [ (MSB)
           3                        Logical Block Address
           4
           5                                                          (LSB) ]
           6        [                      Reserved
           7                                                                ]
           8        [                      Reserved                 ] [ PMI ]
           9        [                    Control                            ]
    </code>

    The REQUEST SENSE command block is as follows:

    <code>
        Byte/Bit    7       6       5       4       3       2       1       0
           0                    Operation Code (0x02)
           1        [                      Reserved                 ] [ DESC]
           2        [                      Reserved
           3                                                                ]
           4        [                  Allocation Length                    ]
           5        [                    Control                            ]
    </code>
  ***************************************************************************/

MEDIA_INFORMATION * USBHostMSDSCSIMediaInitialize( void );


/****************************************************************************
  Function:
    BOOL USBHostMSDSCSIMediaReset( void  )

  Summary:
    This function resets the media.

  Description:
    This function resets the media.  It is called if an operation returns an
    error.  Or the application can call it.

  Precondition:
    None

  Parameters:
    None - None

  Return Values:
    USB_SUCCESS                 - Reset successful
    USB_MSD_DEVICE_NOT_FOUND    - No device with specified address
    USB_ILLEGAL_REQUEST         - Device is in an illegal USB state
                                  for reset

  Remarks:
    None
  ***************************************************************************/

BYTE    USBHostMSDSCSIMediaReset( void  );


/****************************************************************************
  Function:
    BYTE USBHostMSDSCSISectorRead( DWORD sectorAddress, BYTE *dataBuffer)

  Summary:
    This function reads one sector.

  Description:
    This function uses the SCSI command READ10 to read one sector.  The size
    of the sector was determined in the USBHostMSDSCSIMediaInitialize()
    function.  The data is stored in the application buffer.

  Precondition:
    None

  Parameters:
    DWORD   sectorAddress   - address of sector to read
    BYTE    *dataBuffer     - buffer to store data

  Return Values:
    TRUE    - read performed successfully
    FALSE   - read was not successful

  Remarks:
    The READ10 command block is as follows:

    <code>
        Byte/Bit    7       6       5       4       3       2       1       0
           0                    Operation Code (0x28)
           1        [    RDPROTECT      ]  DPO     FUA      -     FUA_NV    -
           2        [ (MSB)
           3                        Logical Block Address
           4
           5                                                          (LSB) ]
           6        [         -         ][          Group Number            ]
           7        [ (MSB)         Transfer Length
           8                                                          (LSB) ]
           9        [                    Control                            ]
    </code>
  ***************************************************************************/

BYTE    USBHostMSDSCSISectorRead( DWORD sectorAddress, BYTE *dataBuffer );


/****************************************************************************
  Function:
    BYTE USBHostMSDSCSISectorWrite( DWORD sectorAddress, BYTE *dataBuffer, BYTE allowWriteToZero )

  Summary:
    This function writes one sector.

  Description:
    This function uses the SCSI command WRITE10 to write one sector.  The size
    of the sector was determined in the USBHostMSDSCSIMediaInitialize()
    function.  The data is read from the application buffer.

  Precondition:
    None

  Parameters:
    DWORD   sectorAddress   - address of sector to write
    BYTE    *dataBuffer     - buffer with application data
    BYTE    allowWriteToZero- If a write to sector 0 is allowed.

  Return Values:
    TRUE    - write performed successfully
    FALSE   - write was not successful

  Remarks:
    To follow convention, this function blocks until the write is complete.

    The WRITE10 command block is as follows:

    <code>
        Byte/Bit    7       6       5       4       3       2       1       0
           0                    Operation Code (0x2A)
           1        [    WRPROTECT      ]  DPO     FUA      -     FUA_NV    -
           2        [ (MSB)
           3                        Logical Block Address
           4
           5                                                          (LSB) ]
           6        [         -         ][          Group Number            ]
           7        [ (MSB)         Transfer Length
           8                                                          (LSB) ]
           9        [                    Control                            ]
    </code>
  ***************************************************************************/

BYTE    USBHostMSDSCSISectorWrite( DWORD sectorAddress, BYTE *dataBuffer, BYTE allowWriteToZero);


/****************************************************************************
  Function:
    BYTE USBHostMSDSCSIWriteProtectState( void )

  Description:
    This function returns the write protect status of the device.

  Precondition:
    None

  Parameters:
    None - None

  Return Values:
    0 - not write protected


  Remarks:
    None
  ***************************************************************************/

BYTE    USBHostMSDSCSIWriteProtectState( void );


// *****************************************************************************
// *****************************************************************************
// Section: SCSI Interface Callback Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BOOL USBHostMSDSCSIInitialize( BYTE address, DWORD flags, BYTE clientDriverID )

  Description:
    This function is called when a USB Mass Storage device is being
    enumerated.

  Precondition:
    None

  Parameters:
    BYTE address    -   Address of the new device
    DWORD flags     -   Initialization flags
    BYTE clientDriverID - ID for this layer.  Not used by the media interface layer.

  Return Values:
    TRUE    -   We can support the device.
    FALSE   -   We cannot support the device.

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostMSDSCSIInitialize( BYTE address, DWORD flags, BYTE clientDriverID );


/****************************************************************************
  Function:
    BOOL USBHostMSDSCSIEventHandler( BYTE address, USB_EVENT event,
                        void *data, DWORD size )

  Description:
    This function is called when various events occur in the USB Host Mass
    Storage client driver.

  Precondition:
    The device has been initialized.

  Parameters:
    BYTE address    -   Address of the device
    USB_EVENT event -   Event that has occurred
    void *data      -   Pointer to data pertinent to the event
    DWORD size      -   Size of the data

  Return Values:
    TRUE    -   Event was handled
    FALSE   -   Event was not handled

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostMSDSCSIEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size );


#endif
