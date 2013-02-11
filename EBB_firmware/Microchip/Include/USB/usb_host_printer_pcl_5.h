/******************************************************************************
    PCL 5 Printer Language Support

  Summary:
    This file provides support for the PCL 5 printer language when using the
    USB Embedded Host Printer Client Driver.

  Description:
    This file provides support for the PCL 5 printer language when using the
    USB Embedded Host Printer Client Driver.

    There are several versions of the PCL printer language.  This file is
    targetted to support PCL 5.  Unfortunately, printer language support is not
    always advertised correctly by the printer.  Some printers advertise only
    PCL 6 support when they also support PCL 5.  Therefore, the default value
    for the LANGUAGE_ID_STRING_PCL string used in the routine
    USBHostPrinterLanguagePCL5IsSupported() is set such that the routine will
    return TRUE if any PCL language support is advertised.  It is highly
    recommended to test the target application with the specific printer(s)
    that will be utilized, and, if possible, populate the
    usbPrinterSpecificLanguage[] array in usb_config.c via the configuration
    tool to manually select the printer language and its functional support.

  Notes:
    The PCL 5 coordinate origin is located at the top left corner of the paper.
    The HP-GL/2 coordinate origin, however, is located at the bottom left corner
    of the page.  For consistency for the user, HP-GL/2 coordinate system is
    adjusted to match the PCL coordinate system.   This also matches the
    coordinate system use by the Microchip Graphics library.

    The black and white bit image polarity is 0=white, 1=black, which is
    reversed from the Microchip Graphics Library polarity.  This driver will
    automatically convert the image data to the required format, as long as the
    image data is located in ROM (USB_PRINTER_TRANSFER_FROM_ROM) or it is
    copied from a RAM buffer (USB_PRINTER_TRANSFER_COPY_DATA).  If the data is
    to be sent directly from its original RAM location, the data must already
    be in the format required by the printer language.

    PCL 5 is not compatible with PCL 6; PCL 5 utilizes ASCII input, whereas
    PCL 6 utilizes binary data.  However, some printers that advertise support
    for only PCL 5 do support PCL 6.

    PCL 3 printers utilize many of the PCL 5 commands.  The following
    limitations exist with PCL 3:
        * PCL 3 does not support vector graphics
        * PCL 3 does not support image printing in landscape mode.  The
            page print will fail.
        * Items must be sent to the page in the order that they are to be
            printed from top to bottom.  The printer cannot return to a
            higher position on the page.
        * PCL 3 does not support the USB_PRINTER_EJECT_PAGE command.  Use the
            USB_PRINTER_JOB_STOP and USB_PRINTER_JOB_START commands instead.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

* FileName:        usb_host_printer_pcl_5.h
* Dependencies:    None
* Processor:       PIC24/dsPIC30/dsPIC33/PIC32MX
* Compiler:        C30 v3.10b/C32 v1.02
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
KO         ??-???-2008 First release

*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

        // This is the string that the printer language support determination
        // routine will look for to determine if the printer supports this
        // printer language.  This string is case sensive.  Some printers that
        // report only PCL 6 support also support PCL 5.  So it is recommended
        // to use "PCL" as the search string, rather than "PCL 5", and verify
        // that the correct output is produced by the target printer.
#define LANGUAGE_ID_STRING_PCL      "PCL"
        // These are the support flags that are set for the PCL 3 version of
        // this language.
#define LANGUAGE_SUPPORT_FLAGS_PCL3 0
        // These are the support flags that are set for the PCL 5 version of
        // this language.
#define LANGUAGE_SUPPORT_FLAGS_PCL5 USB_PRINTER_FUNCTION_SUPPORT_VECTOR_GRAPHICS


// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BYTE USBHostPrinterLanguagePCL5( BYTE address,
        USB_PRINTER_COMMAND command, USB_DATA_POINTER data, DWORD size, BYTE transferFlags )

  Summary:
    This function executes printer commands for a PCL 5 printer.

  Description:
    This function executes printer commands for a PCL 5 printer.  When
    the application issues a printer command, the printer client driver
    determines what language to use to communicate with the printer, and
    transfers the command to that language support routine.  As much as
    possible, commands are designed to produce the same output regardless
    of what printer language is used.

    Not all printer commands support data from both RAM and ROM.  Unless
    otherwise noted, the data pointer is assumed to point to RAM, regardless of
    the value of transferFlags.  Refer to the specific command to see if ROM
    data is supported.

  Preconditions:
    None

  Parameters:
    BYTE address                - Device's address on the bus
    USB_PRINTER_COMMAND command - Command to execute.  See the enumeration
                                    USB_PRINTER_COMMAND for the list of
                                    valid commands and their requirements.
    USB_DATA_POINTER data       - Pointer to the required data.  Note that
                                    the caller must set transferFlags
                                    appropriately to indicate if the pointer is
                                    a RAM pointer or a ROM pointer.
    DWORD size                  - Size of the data.  For some commands, this
                                    parameter is used to hold the data itself.
    BYTE transferFlags          - Flags that indicate details about the
                                    transfer operation.  Refer to these flags
                                    * USB_PRINTER_TRANSFER_COPY_DATA
                                    * USB_PRINTER_TRANSFER_STATIC_DATA
                                    * USB_PRINTER_TRANSFER_NOTIFY
                                    * USB_PRINTER_TRANSFER_FROM_ROM
                                    * USB_PRINTER_TRANSFER_FROM_RAM

  Return Values:
    USB_PRINTER_SUCCESS             - The command was executed successfully.
    USB_PRINTER_UNKNOWN_DEVICE      - A printer with the indicated address is not
                                        attached
    USB_PRINTER_TOO_MANY_DEVICES    - The printer status array does not have
                                        space for another printer.
    USB_PRINTER_OUT_OF_MEMORY       - Not enough available heap space to
                                        execute the command.
    other                           - See possible return codes from the
                                        function USBHostPrinterWrite().

  Remarks:
    When developing new commands, keep in mind that the function
    USBHostPrinterCommandReady() will be used before calling this function to
    see if there is space available in the output transfer queue.
    USBHostPrinterCommandReady() will routine TRUE if a single space is
    available in the output queue.  Therefore, each command can generate only
    one output transfer.

    Multiple printer languages may be used in a single application.  The USB
    Embedded Host Printer Client Driver will call the routine required for the
    attached device.
  ***************************************************************************/

BYTE USBHostPrinterLanguagePCL5( BYTE address,
        USB_PRINTER_COMMAND command, USB_DATA_POINTER data, DWORD size, BYTE transferFlags );


/****************************************************************************
  Function:
    BOOL USBHostPrinterLanguagePCL5IsSupported( char *deviceID,
                USB_PRINTER_FUNCTION_SUPPORT *support )

  Summary:
    This function determines if the printer with the given device ID string
    supports the PCL 5 printer language.

  Description:
    This function determines if the printer with the given device ID string
    supports the PCL 5 printer language.

    Unfortunately, printer language support is not always advertised correctly
    by the printer.  Some printers advertise only PCL 6 support when they also
    support PCL 5.  Therefore, this routine will return TRUE if any PCL
    language support is advertised.  It is therefore highly recommended to test
    the target application with the specific printer(s) that will be utilized.

  Preconditions:
    None

  Parameters:
    char *deviceID  - Pointer to the "COMMAND SET:" portion of the device ID
                        string of the attached printer.
    USB_PRINTER_FUNCTION_SUPPORT *support   - Pointer to returned information
                        about what types of functions this printer supports.

  Return Values:
    TRUE    - The printer supports PCL 5.
    FALSE   - The printer does not support PCL 5.

  Remarks:
    The caller must first locate the "COMMAND SET:" section of the device ID
    string.  To ensure that only the "COMMAND SET:" section of the device ID
    string is checked, the ";" at the end of the section should be temporarily
    replaced with a NULL.  Otherwise, this function may find the printer
    language string in the comments or other section, and incorrectly indicate
    that the printer supports the language.

    Device ID strings are case sensitive.
  ***************************************************************************/

BOOL USBHostPrinterLanguagePCL5IsSupported( char *deviceID,
                USB_PRINTER_FUNCTION_SUPPORT *support );

