/******************************************************************************
    ESC/POS Printer Language Support

  Summary:
    This file provides support for the ESC/POS printer language when using the
    USB Embedded Host Printer Client Driver.

  Description:
    This file provides support for the ESC/POS printer language when using the
    USB Embedded Host Printer Client Driver.

    The exact implementation of ESC/POS varies across manufacturers and even
    across different models from the same manufacturer.  Some POS printers use
    specialized Device ID strings (obtained with the GET DEVICE ID class-
    specific request) to indicate the deviations from the strict ESC/POS
    specification.  Also, many printers indicate a custom USB Peripheral Device
    rather than the Printer Class, and therefore do not support the
    GET DEVICE ID device request.  For example:
    <code>
    Printer          Tested VID/PID  Class    Device ID Substring
    -------------------------------------------------------------
    Bixolon SRP-270  0x0419/0x3C01   Printer  ESC
    Epson TM-T88IV   0x04B8/0x0202   Custom   Not supported
    Seiko DPU-V445   0x0619/0x0111   Printer  Not present
    Seiko MPU-L465   0x0619/0x0109   Printer  SIIMPU
    </code>
    Therefore, dynamic language determination is not recommended for POS
    printers.  Instead, create your application to target a either a single
    printer model or a group of printer models with identical requirements,
    and indicate specific VID, PID, and printer language via the USB
    Configuration Tool, and test the application to ensure consistent behavior
    across all supported printer models.

    The ESC/POS language support code provides several #defines that allow the
    language support file to automatically configure itself for different
    printer models.  These #defines can be set using the USB Configuration Tool
    (USBConfig.exe or MPLAB VDI).  Note that they are determined at compile
    time, not run time, so only one type of printer can be utilized by an
    application.  Printer models other that the ones explicitly tested may
    require other modifications to the language support code.

  Notes:
    Currently, only standard mode is supported.

    The black and white bit image polarity is 0=white, 1=black, which is
    reversed from the Microchip Graphics Library polarity.  This driver will
    automatically convert the image data to the required format, as long as the
    image data is located in ROM (USB_PRINTER_TRANSFER_FROM_ROM) or it is
    copied from a RAM buffer (USB_PRINTER_TRANSFER_COPY_DATA).  If the data is
    to be sent directly from its original RAM location, the data must already
    be in the format required by the printer language.


*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

* FileName:        usb_host_printer_esc_pos.h
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
        // printer language.  This string is case sensive.  See the function
        // USBHostPrinterLanguageESCPOSIsSupported() for more information
        // about using or changing this string.  Dynamic language determination
        // is not recommended when using POS printers.
#define LANGUAGE_ID_STRING_ESCPOS       "ESC"
        // These are the support flags that are set for this language.
#define LANGUAGE_SUPPORT_FLAGS_ESCPOS   USB_PRINTER_FUNCTION_SUPPORT_POS


/****************************************************************************
  Function:
    BYTE USBHostPrinterLanguageESCPOS( BYTE address,
        USB_PRINTER_COMMAND command, USB_DATA_POINTER data, DWORD size, BYTE transferFlags )

  Summary:
    This function executes printer commands for an ESC/POS printer.

  Description:
    This function executes printer commands for an ESC/POS printer.  When
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
    USB_DATA_POINTER data    - Pointer to the required data.  Note that
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

BYTE USBHostPrinterLanguageESCPOS( BYTE address,
        USB_PRINTER_COMMAND command, USB_DATA_POINTER data, DWORD size, BYTE transferFlags );


/****************************************************************************
  Function:
    BOOL USBHostPrinterLanguageESCPOSIsSupported( char *deviceID,
                USB_PRINTER_FUNCTION_SUPPORT *support )

  Description:
    This function determines if the printer with the given device ID string
    supports the ESC/POS printer language.

  Preconditions:
    None

  Parameters:
    char *deviceID  - Pointer to the "COMMAND SET:" portion of the device ID
                        string of the attached printer.
    USB_PRINTER_FUNCTION_SUPPORT *support   - Pointer to returned information
                        about what types of functions this printer supports.

  Return Values:
    TRUE    - The printer supports ESC/POS.
    FALSE   - The printer does not support ESC/POS.

  Remarks:
    The caller must first locate the "COMMAND SET:" section of the device ID
    string.  To ensure that only the "COMMAND SET:" section of the device ID
    string is checked, the ";" at the end of the section should be temporarily
    replaced with a NULL.  Otherwise, this function may find the printer
    language string in the comments or other section, and incorrectly indicate
    that the printer supports the language.

    Device ID strings are case sensitive.

    See the file header comments for this file (usb_host_printer_esc_pos.h)
    for cautions regarding dynamic printer language selection with POS
    printers.
  ***************************************************************************/

BOOL USBHostPrinterLanguageESCPOSIsSupported( char *deviceID,
                USB_PRINTER_FUNCTION_SUPPORT *support );


/****************************************************************************
  Function:
    USB_DATA_POINTER USBHostPrinterPOSImageDataFormat( USB_DATA_POINTER image,
        BYTE imageLocation, WORD imageHeight, WORD imageWidth, WORD *currentRow,
        BYTE byteDepth, BYTE *imageData )

  Summary:
    This function formats data for a bitmapped image into the format required
    for sending to a POS printer.

  Description:
    This function formats data for a bitmapped image into the format required
    for sending to a POS printer.  Bitmapped images are stored one row of pixels
    at a time.  Suppose we have an image with vertical black bars, eight pixels
    wide and eight pixels deep.  The image would appear as the following pixels,
    where 0 indicates a black dot and 1 indicates a white dot:
    <code>
    0 1 0 1 0 1 0 1
    0 1 0 1 0 1 0 1
    0 1 0 1 0 1 0 1
    0 1 0 1 0 1 0 1
    0 1 0 1 0 1 0 1
    0 1 0 1 0 1 0 1
    0 1 0 1 0 1 0 1
    0 1 0 1 0 1 0 1
    </code>
    The stored bitmap of the data would contain the data bytes, where each byte
    is one row of data:
    <code>
    0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55
    </code>
    When printing to a full sheet printer, eight separate
    USB_PRINTER_IMAGE_DATA_HEADER / USB_PRINTER_IMAGE_DATA command combinations
    are required to print this image.

    POS printers, however, require image data formated either 8 dots or 24 dots
    deep, depending on the desired (and supported) vertical print density.  For
    a POS printer performing an 8-dot vertical density print, the data needs to
    be in this format:
    <code>
    0x00 0xFF 0x00 0xFF 0x00 0xFF 0x00 0xFF
    </code>
    When printing to a POS printer, only one
    USB_PRINTER_IMAGE_DATA_HEADER / USB_PRINTER_IMAGE_DATA command combination
    is required to print this image.

    This function supports 8-dot and 24-dot vertical densities by specifying
    the byteDepth parameter as either 1 (8-dot) or 3 (24-dot).

  Precondition:
    None

  Parameters:
    USB_DATA_POINTER image  Pointer to the image bitmap data.
    BYTE imageLocation      Location of the image bitmap data.  Valid values
                            are USB_PRINTER_TRANSFER_FROM_ROM and
                            USB_PRINTER_TRANSFER_FROM_RAM.
    WORD imageHeight        Height of the image in pixels.
    WORD imageWidth         Width of the image in pixels.
    WORD *currentRow        The current pixel row.  Upon return, this value is
                            updated to the next pixel row to print.
    BYTE byteDepth          The byte depth of the print.  Valid values are 1
                            (8-dot vertical density) and 3 (24-dot vertical
                            density).
    BYTE *imageData         Pointer to a RAM data buffer that will receive the
                            manipulated data to send to the printer.

  Returns:
    The function returns a pointer to the next byte of image data.

  Example:
    The following example code will send a complete bitmapped image to a POS
    printer.
    <code>
        WORD                    currentRow;
        BYTE                    depthBytes;
        BYTE                    *imageDataPOS;
        USB_PRINTER_IMAGE_INFO  imageInfo;
        BYTE                    returnCode;
        #if defined (__C30__)
            BYTE __prog__       *ptr;
            ptr = (BYTE __prog__ *)logoMCHP.address;
        #elif defined (__PIC32MX__)
            const BYTE          *ptr;
            ptr = (const BYTE *)logoMCHP.address;
        #endif

        imageInfo.densityVertical   = 24;   // 24-dot density
        imageInfo.densityHorizontal = 2;    // Double density

        // Extract the image height and width
        imageInfo.width    = ((WORD)ptr[5] << 8) + ptr[4];
        imageInfo.height   = ((WORD)ptr[3] << 8) + ptr[2];

        depthBytes         = imageInfo.densityVertical / 8;
        imageDataPOS       = (BYTE *)malloc( imageInfo.width *
                                       depthBytes );

        if (imageDataPOS == NULL)
        {
            // Error - not enough heap space
        }

        USBHostPrinterCommandWithReadyWait( &returnCode,
              printerInfo.deviceAddress, USB_PRINTER_IMAGE_START,
              USB_DATA_POINTER_RAM(&imageInfo),
              sizeof(USB_PRINTER_IMAGE_INFO ),
              0 );

        ptr += 10; // skip the header info

        currentRow = 0;
        while (currentRow < imageInfo.height)
        {
            USBHostPrinterCommandWithReadyWait( &returnCode,
              printerInfo.deviceAddress,
              USB_PRINTER_IMAGE_DATA_HEADER, USB_NULL,
              imageInfo.width, 0 );

            ptr = USBHostPrinterPOSImageDataFormat(
              USB_DATA_POINTER_ROM(ptr),
              USB_PRINTER_TRANSFER_FROM_ROM, imageInfo.height,
              imageInfo.width, &currentRow, depthBytes,
              imageDataPOS ).pointerROM;

            USBHostPrinterCommandWithReadyWait( &returnCode,
              printerInfo.deviceAddress, USB_PRINTER_IMAGE_DATA,
              USB_DATA_POINTER_RAM(imageDataPOS), imageInfo.width,
              USB_PRINTER_TRANSFER_COPY_DATA);
        }

        free( imageDataPOS );

        USBHostPrinterCommandWithReadyWait( &returnCode,
              printerInfo.deviceAddress, USB_PRINTER_IMAGE_STOP,
              USB_NULL, 0, 0 );
      </code>

  Remarks:
    This routine currently does not support 36-dot density printing.  Since
    the output for 36-dot vertical density is identical to 24-dot vertical
    density, 24-dot vertical density should be used instead.

    This routine does not yet support reading from external memory.
  ***************************************************************************/

USB_DATA_POINTER USBHostPrinterPOSImageDataFormat( USB_DATA_POINTER image,
        BYTE imageLocation, WORD imageHeight, WORD imageWidth, WORD *currentRow,
        BYTE byteDepth, BYTE *imageData );

