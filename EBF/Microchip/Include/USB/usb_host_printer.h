/*******************************************************************************

  USB Host Printer Client Driver

  Summary:
    This is the Printer client driver file for a USB Embedded Host device.

  Description:
    This is the Printer client driver file for a USB Embedded Host device.
    It allows an embedded application to utilize a USB printer to provide
    printed output.

    USB printers utilize the USB Printer Class to communicate with a USB
    Host.  This class defines the USB transfer type, the endpoint structure,
    a device requests that can be performed.  The actual commands sent to
    the printer, however, are dictated by the printer language used by the
    particular printer.

    Many different printer languages are utilized by the wide variety of
    printers on the market.  Typically, low end printers receive printer-specific
    binary data, utilizing the processing power of the USB Host to perform
    all of the complex calculations required to translate text and graphics to
    a simple binary representation.  This works well when a PC is the USB Host,
    but it is not conducive to an embedded application with limited resources.

    Many printers on the market use a command based printer language, relying
    on the printer itself to interpret commands to produce the desired output.
    Some languages are standardized across printers from a particular
    manufacturer, and some are used across multiple manufacturer.  This method
    lends itself better to embedded applications by allowing the printer to
    take on some of the computational overhead.  Microchip provides support for
    some printer languages, including PostScript and PCL 5 for full sheet
    printers and ESC/POS for receipt and label printers.  Additional printer
    language can be implemented.  Refer to the USB Embedded Host Printer Class
    application notes for more details on implementing printer language support.


  Remarks:
    This driver should be used in a project with usb_host.c to provided the USB
    Embedded Host and hardware interfaces, plus one or more language support
    files.

    To interface with USB Embedded Host layer, the routine USBHostPrinterInitialize()
    should be specified as the Initialize() function, and
    USBHostPrinterEventHandler() should be specified as the EventHandler()
    function in the usbClientDrvTable[] array declared in usb_config.c.

    This driver requires transfer events from usb_host.c, so
    USB_ENABLE_TRANSFER_EVENT must be defined.

    Since the printer class is performed with bulk transfers,
    USB_SUPPORT_BULK_TRANSFERS must be defined.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

* FileName:        usb_host_printer.h
* Dependencies:    None
* Processor:       PIC24/dsPIC30/dsPIC33/PIC32
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

*******************************************************************************/

//DOM-IGNORE-BEGIN
/********************************************************************
 File Description:

 Change History:
  Rev           Description
  ----------    -----------
  2.6 - 2.6A    No chance except stack revision number
  2.7           Minor updates to USBHostPrinterGetStatus() header
                to better describe the function requirements and
                operation.
********************************************************************/
//DOM-IGNORE-END

#ifndef __USBHOSTPRINTER_H__
#define __USBHOSTPRINTER_H__
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Max Number of Supported Devices

This value represents the maximum number of attached devices this class driver
can support.  If the user does not define a value, it will be set to 1.
Currently this must be set to 1, due to limitations in the USB Host layer.
*/
#ifndef USB_MAX_PRINTER_DEVICES
    #define USB_MAX_PRINTER_DEVICES     1
#endif

#if USB_MAX_PRINTER_DEVICES != 1
    #error The Printer client driver supports only one attached device.
#endif


//#define DEBUG_MODE

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

        // For use with POS printers that support Code128 bar codes (extended
        // barcodes).  This is the value of the first data byte of the bar code
        // data, which begins the character code specification.  The next byte
        // must be 'A', 'B', or 'C'.
#define BARCODE_CODE128_CODESET_CHAR            '{'
        // For use with POS printers that support Code128 bar codes (extended
        // barcodes).  This is the value of the first data byte of the bar code
        // data, which begins the character code specification.  The next byte
        // must be 'A', 'B', or 'C'.
#define BARCODE_CODE128_CODESET_STRING          "{"
        // For use with POS printers that support Code128 bar codes (extended
        // barcodes).  This is the value of the second data byte of the bar code
        // data to specify character code set CODE A.  This code set should be
        // used if control characters (0x00-0x1F) are included in the data.
#define BARCODE_CODE128_CODESET_A_CHAR          'A'
        // For use with POS printers that support Code128 bar codes (extended
        // barcodes).  This is the value of the second data byte of the bar code
        // data to specify character code set CODE A.  This code set should be
        // used if control characters (0x00-0x1F) are included in the data.
#define BARCODE_CODE128_CODESET_A_STRING        "A"
        // For use with POS printers that support Code128 bar codes (extended
        // barcodes).  This is the value of the second data byte of the bar code
        // data to specify character code set CODE B.  This code set should be
        // used if lower case letters and higher ASCII characters (0x60-0x7F)
        // are included in the data.
#define BARCODE_CODE128_CODESET_B_CHAR          'B'
        // For use with POS printers that support Code128 bar codes (extended
        // barcodes).  This is the value of the second data byte of the bar code
        // data to specify character code set CODE B.  This code set should be
        // used if lower case letters and higher ASCII characters (0x60-0x7F)
        // are included in the data.
#define BARCODE_CODE128_CODESET_B_STRING        "B"
        // For use with POS printers that support Code128 bar codes (extended
        // barcodes).  This is the value of the second data byte of the bar code
        // data to specify character code set CODE C.  This code set can be
        // used only if the data values are between 0 and 99 (0x00-0x63).
#define BARCODE_CODE128_CODESET_C_CHAR          'C'
        // For use with POS printers that support Code128 bar codes (extended
        // barcodes).  This is the value of the second data byte of the bar code
        // data to specify character code set CODE C.  This code set can be
        // used only if the data values are between 0 and 99 (0x00-0x63).
#define BARCODE_CODE128_CODESET_C_STRING        "C"
        // For use with POS printers.  May not be valid for all printers.  Print
        // the bar code text in 18x36 dot font.  Do not alter this value.
#define BARCODE_TEXT_18x36                      0
        // For use with POS printers.  May not be valid for all printers.  Print
        // the bar code text in 12x24 dot font.  Do not alter this value.
#define BARCODE_TEXT_12x24                      1
        // For use with POS printers.  Do not print readable bar code text.  Do
        // not alter this value.
#define BARCODE_TEXT_OMIT                       0
        // For use with POS printers.  Print readable text above the bar code.
        // Do not alter this value.
#define BARCODE_TEXT_ABOVE                      1
        // For use with POS printers.  Print readable text below the bar code.
        // Do not alter this value.
#define BARCODE_TEXT_BELOW                      2
        // For use with POS printers.  Print readable text above and below the bar
        // code.  Do not alter this value.
#define BARCODE_TEXT_ABOVE_AND_BELOW            3
        // Indicates a black line for drawing graphics objects.
#define PRINTER_COLOR_BLACK                     0
        // Indicates a white line for drawing graphics objects.
#define PRINTER_COLOR_WHITE                     1
        // bRequest value for the GET_DEVICE_ID USB class-specific request.
#define PRINTER_DEVICE_REQUEST_GET_DEVICE_ID    0x00
        // bRequest value for the GET_PORT_STATUS USB class-specific request.
#define PRINTER_DEVICE_REQUEST_GET_PORT_STATUS  0x01
        // bRequest value for the SOFT_RESET USB class-specific request.
#define PRINTER_DEVICE_REQUEST_SOFT_RESET       0x02
        // Indicates a solid color fill for filled graphics objects.
#define PRINTER_FILL_SOLID                      0
        // Indicates a shaded fill for filled graphics objects.  Requires a
        // specified fill percentage.
#define PRINTER_FILL_SHADED                     1
        // Indicates a hatched fill for graphics objects.  Requires a
        // specified line spacing and angle.
#define PRINTER_FILL_HATCHED                    2
        // Indicates a cross-hatched fill for graphics objects.  Requires a
        // specified line spacing and angle.
#define PRINTER_FILL_CROSS_HATCHED              3
        // Indicates a dashed line for drawing graphics objects.
#define PRINTER_LINE_TYPE_DASHED                1
        // Indicates a dotted line for drawing graphics objects.
#define PRINTER_LINE_TYPE_DOTTED                2
        // Indicates a solid line for drawing graphics objects.
#define PRINTER_LINE_TYPE_SOLID                 0
        // Indicates a normal width line for drawing graphics objects.
#define PRINTER_LINE_WIDTH_NORMAL               0
        // Indicates a thick line for drawing graphics objects.
#define PRINTER_LINE_WIDTH_THICK                1
        // Drawn lines will have a butt end.
#define PRINTER_LINE_END_BUTT                   0
        // Drawn lines will have a round end.
#define PRINTER_LINE_END_ROUND                  1
        // Drawn lines will have a square end.
#define PRINTER_LINE_END_SQUARE                 2
        // Drawn lines will be joined with a bevel.
#define PRINTER_LINE_JOIN_BEVEL                 0
        // Drawn lines will be joined with a miter.
#define PRINTER_LINE_JOIN_MITER                 1
        // Drawn lines will be joined with a round.
#define PRINTER_LINE_JOIN_ROUND                 2
        // The height of the page in points when in landscape mode.
#define PRINTER_PAGE_LANDSCAPE_HEIGHT           612
        // The width of the page in points when in landscape mode.
#define PRINTER_PAGE_LANDSCAPE_WIDTH            792
        // The height of the page in points when in portrait mode.
#define PRINTER_PAGE_PORTRAIT_HEIGHT            792
        // The width of the page in points when in portrait mode.
#define PRINTER_PAGE_PORTRAIT_WIDTH             612
        // Image print with double horizontal density.
#define PRINTER_POS_DENSITY_HORIZONTAL_DOUBLE   2
        // Image print with single horizontal density.
#define PRINTER_POS_DENSITY_HORIZONTAL_SINGLE   1
        // Image print with 8-dot vertical density.
#define PRINTER_POS_DENSITY_VERTICAL_8          8
        // Image print with 24-dot vertical density.
#define PRINTER_POS_DENSITY_VERTICAL_24         24
        // POS print direction left to right, starting at the top left corner.
#define PRINTER_POS_LEFT_TO_RIGHT               0
        // POS print direction bottom to top, starting at the bottom left corner.
#define PRINTER_POS_BOTTOM_TO_TOP               1
        // POS print direction right to left, startin at the bottom right corner.
#define PRINTER_POS_RIGHT_TO_LEFT               2
        // POS print direction top to bottom, starting at the top right corner.
#define PRINTER_POS_TOP_TO_BOTTOM               3


        // Constant to use to set the supportsPOS member of the
        // USB_PRINTER_FUNCTION_SUPPORT union.
#define USB_PRINTER_FUNCTION_SUPPORT_POS                0x0002
        // Constant to use to set the supportsVectorGraphics member of the
        // USB_PRINTER_FUNCTION_SUPPORT union.
#define USB_PRINTER_FUNCTION_SUPPORT_VECTOR_GRAPHICS    0x0001


//#define PRINTER_IMAGE_COMPRESSION_NONE          0
//#define PRINTER_IMAGE_BITS_PER_SAMPLE_1         1

// *****************************************************************************
// Section: USB Printer Client Events
// *****************************************************************************

        // This is an optional offset for the values of the generated events.
        // If necessary, the application can use a non-zero offset for the
        // generic events to resolve conflicts in event number.
#ifndef EVENT_PRINTER_OFFSET
#define EVENT_PRINTER_OFFSET 0
#endif

        // This event indicates that a Printer device has been attached.
        // When USB_HOST_APP_EVENT_HANDLER is called with this event, the *data
        // parameter points to a structure of the type
        // USB_PRINTER_DEVICE_ID, which provides important information about
        // the attached printer.  The size parameter is the size of this
        // structure.
#define EVENT_PRINTER_ATTACH        (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+0)

        // This event indicates that the specified device has been detached
        // from the USB.  When USB_HOST_APP_EVENT_HANDLER is called with this
        // event, *data points to a BYTE that contains the device address, and
        // size is the size of a BYTE.
#define EVENT_PRINTER_DETACH        (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+1)

        // This event indicates that a previous write request has completed.
        // When USB_HOST_APP_EVENT_HANDLER is called with this event, *data
        // points to the buffer that completed transmission, and size is the
        // actual number of bytes that were written to the device.
#define EVENT_PRINTER_TX_DONE       (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+2)

        // This event indicates that a previous read request has completed.
        // When USB_HOST_APP_EVENT_HANDLER is called with this event, *data
        // points to the receive buffer, and size is the actual number of bytes
        // read from the device.
#define EVENT_PRINTER_RX_DONE       (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+3)

        // This event indicates that the printer request has completed.
        // These requests occur on endpoint 0 and include getting the printer
        // status and performing a soft reset.
#define EVENT_PRINTER_REQUEST_DONE  (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+4)

        // This event indicates that a bus error occurred while trying to
        // perform a write.  The error code is returned in the size parameter.
        // The data parameter is returned as NULL.
#define EVENT_PRINTER_TX_ERROR      (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+5)

        // This event indicates that a bus error occurred while trying to
        // perform a read.  The error code is returned in the size parameter.
        // The data parameter is returned as NULL.
#define EVENT_PRINTER_RX_ERROR      (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+6)

        // This event indicates that a bus error occurred while trying to
        // perform a device request.  The error code is returned in the size
        // parameter.  The data parameter is returned as NULL.
#define EVENT_PRINTER_REQUEST_ERROR (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+7)

        // This event indicates that a printer has attached for which we do not
        // have printer language support.  Therefore, we cannot talk to this
        // printer.  This event can also occur if there is not enough dynamic
        // memory available to read the device ID string.
#define EVENT_PRINTER_UNSUPPORTED   (EVENT_PRINTER_BASE+EVENT_PRINTER_OFFSET+8)


// *****************************************************************************
// Section: USB Printer Commands, Flags, and Errors
// *****************************************************************************

//-----------------------------------------------------------------------------
/* USB Printer Client Driver Commands

The main interface to the USB Printer Client Driver is through the function
USBHostPrinterCommand().  These are the commands that can be passed to that
function.
*/

typedef enum
{
    // This command is used internally by the printer client driver.
    // Applications do not issue this command.  This command informs the
    // language support code that a new device has attached.
    // Some language support requires the maintenance of certain information about
    // the printering status.  This command, with the USB_PRINTER_DETACHED command,
    // allows the language support information to be maintained properly as printers
    // are attached and detached.  The data and size parameters are not used by
    // this command, and can be passed as USB_NULL and 0 respectively.
    USB_PRINTER_ATTACHED,

    // This command is used internally by the printer client driver.
    // Applications do not issue this command.  This command informs the
    // language support code that a device has detached.
    // Some language support requires the maintenance of certain information about
    // the printering status.  This command, with the USB_PRINTER_ATTACHED command,
    // allows the language support information to be maintained properly as printers
    // are attached and detached.  The data and size parameters are not used by
    // this command, and can be passed as USB_NULL and 0 respectively.
    USB_PRINTER_DETACHED,

    // This command instructs the printer driver to send the buffer directly to
    // the printer, without interpretation by the printer driver.  This is
    // normally used only when debugging new commands.  The data parameter should
    // point to the data to be sent, and size should indicate the number of bytes
    // to send.  This command supports sending data from either RAM or
    // ROM.  If the data is in ROM, be sure to set the
    // USB_PRINTER_TRANSFER_FROM_ROM flag.  If the data is in RAM but the
    // application may overwrite it, set the USB_PRINTER_TRANSFER_COPY_DATA
    // flag to tell the printer client driver to make a local copy of the data,
    // allowing the application to overwrite the original buffer when
    // USBHostPrinterCommand() terminates.
    USB_PRINTER_TRANSPARENT,

    // This command should be issued at the beginning of every print job.  It
    // ensures that the printer is set back to a default state.  The data and
    // size parameters are not used by this command, and can be passed as USB_NULL
    // and 0 respectively.
    USB_PRINTER_JOB_START,

    // This command should be issued at the end of every print job.  It ejects the
    // currently printing page, and ensures that the printer is set back to a
    // default state.  The data and size parameters are not used by this command,
    // and can be passed as USB_NULL and 0 respectively.
    USB_PRINTER_JOB_STOP,

    // This command sets the current page orientation to portrait.  This command
    // must be issued immediately after the USB_PRINTER_JOB_START and
    // USB_PRINTER_EJECT_PAGE commands in order for the command to take effect
    // properly.  Only one orientation command should be sent per page, or the
    // output may not be properly generated.  The default orientation is
    // portrait.  The data and size parameters are not used by this command,
    // and can be passed as USB_NULL and 0 respectively.
    USB_PRINTER_ORIENTATION_PORTRAIT,

    // This command sets the current page orientation to landscape.  This command
    // must be issued immediately after the USB_PRINTER_JOB_START and
    // USB_PRINTER_EJECT_PAGE commands in order for the command to take effect
    // properly.  Only one orientation command should be sent per page, or the
    // output may not be properly generated.  The default orientation is
    // portrait.  The data and size parameters are not used by this command,
    // and can be passed as USB_NULL and 0 respectively.
    USB_PRINTER_ORIENTATION_LANDSCAPE,

    // This command selects the text font.  To make usage easier, the size
    // parameter is used to hold the font name indication.  The data pointer
    // should be passed in as USB_NULL.  Refer to the enums USB_PRINTER_FONTS and
    // USB_PRINTER_FONTS_POS for the valid values for the font name.  With POS
    // printers, the font name also indicates the font size.
    USB_PRINTER_FONT_NAME,

    // (Full sheet printers only.)  This command selects the font size in
    // terms of points.  To make usage
    // easier, the size parameter is used to hold the font size.  The data
    // pointer should be passed in as USB_NULL.  For POS printers, the size is
    // specified as a scale factor.  The value of bits [3:0] plus one is the
    // vertical scale, and the value of bits [7:4] plus one is the horizontal
    // scale.  Each direction can be scaled a maximum of x10.  For example, the
    // value 0x00 is x1 scaling in both directions, and 0x95 is x10 scaling
    // horizontally and x6 scaling vertically.
    USB_PRINTER_FONT_SIZE,

    // This command sets the current font to italic.  The data and size
    // parameters are not used by this command, and can be passed as USB_NULL and
    // 0 respectively.
    USB_PRINTER_FONT_ITALIC,

    // This command sets the current font to upright (not italic).  The data
    // and size parameters are not used by this command, and can be passed as
    // USB_NULL and 0 respectively.
    USB_PRINTER_FONT_UPRIGHT,

    // This command sets the current font to bold.  The data and size
    // parameters are not used by this command, and can be passed as USB_NULL and
    // 0 respectively.
    USB_PRINTER_FONT_BOLD,

    // This command sets the current font to regular weight (not bold).  The
    // data and size parameters are not used by this command, and can be
    // passed as USB_NULL and 0 respectively.
    USB_PRINTER_FONT_MEDIUM,

    // This command ejects the currently printing page.  The command
    // USB_PRINTER_JOB_STOP will also eject the page.  After this command, the
    // selected paper orientation (portrait or landscape) and selected font
    // must be reset.  The data and size parameters are not used
    // by this command, and can be passed as USB_NULL and 0 respectively.
    USB_PRINTER_EJECT_PAGE,

    // This command initiates a text print.  To print text, first issue a
    // USB_PRINTER_TEXT_START command.  Then issue a USB_PRINTER_TEXT command
    // with the text to be printed, setting the transferFlags parameter correctly
    // for the location of the source text (RAM, ROM, or external memory.
    // Finally, use the USB_PRINTER_TEXT_STOP
    // command to terminate the text print.  For best compatibility across
    // printers, do not insert other commands into this sequence.  The
    // data and size parameters are not used by this command, and can be passed
    // as USB_NULL and 0 respectively.
    USB_PRINTER_TEXT_START,

    // This command specifies text to print.  The data parameter should point
    // to the buffer of data to send, and size should indicate how many bytes
    // of data to print.  This command supports printing text from either RAM
    // or ROM.  Be sure to set the transferFlags parameter correctly
    // for the location of the data source.  If the data is in RAM but the
    // application may overwrite it, set the USB_PRINTER_TRANSFER_COPY_DATA
    // flag to tell the printer client driver to make a local copy of the data,
    // allowing the application to overwrite the original buffer when
    // USBHostPrinterCommand() terminates. Refer to the USB_PRINTER_TEXT_START
    // command for the sequence required to print text.
    USB_PRINTER_TEXT,

    // This command terminates a text print.  Refer to the
    // USB_PRINTER_TEXT_START command for the sequence required to print text.
    // The data and size parameters are not used by this command, and can be passed
    // as USB_NULL and 0 respectively.  For POS printers, size is the number of
    // lines to feed after the print.  To get the best result, a minimum of one
    // line is recommended.  The data parameter is not used, and can be
    // passed as USB_NULL.
    USB_PRINTER_TEXT_STOP,

    // This command sets the current printing position on the page. Refer to
    // the documentation for a description of the page coordinates.  Both X and
    // Y coordinates are passed in the size parameter.  The X coordinate is
    // passed in the most significant (upper) WORD, and the Y coordinate is
    // passed in the least significant (lower) WORD.  The macro
    // USBHostPrinterPosition( X, Y ) can be used to create the paramater.  POS
    // printers support specifying the Y-axis position while in page mode only.
    // The data pointer should be passed in as USB_NULL.
    USB_PRINTER_SET_POSITION,

    // This command is used to initialize the printing of a bitmapped image.
    // This command requires a pointer to a variable of type
    // USB_PRINTER_IMAGE_INFO.  To print a bitmapped image, obtain the
    // information required by the USB_PRINTER_IMAGE_INFO structure, and issue
    // this command.  Each row of bitmapped data can now be sent to the
    // printer.  For each row, first issue the USB_PRINTER_IMAGE_DATA_HEADER
    // command.  Then issue the USB_PRINTER_IMAGE_DATA command, with the
    // transferFlags parameter set appropriately for the location of the
    // bitmapped data.  After all rows of data have been sent, terminate the
    // image print with the USB_PRINTER_IMAGE_STOP command.  Be sure that
    // adequate heap space is available, particularly when printing from ROM
    // or external memory, and when printing to a POS printer.  When printing
    // images on POS printers, ensure that the dot density capabilities of the
    // printer are set correctly.  If they are not, the printer will print
    // garbage characters.  Refer to the Printer Client Driver section of the
    // Help file for more information about printing images.
    USB_PRINTER_IMAGE_START,

    // This command is issued before each row of bitmapped image data.  The
    // size parameter is the width of the image in terms of pixels.  The
    // *data parameter is not used and should be passed in as USB_NULL.  Refer to
    // the USB_PRINTER_IMAGE_START command for the sequence required to print
    // an image.  When printing
    // images on POS printers, ensure that the dot density capabilities of the
    // printer are set correctly.  If they are not, the printer will print
    // garbage characters.
    USB_PRINTER_IMAGE_DATA_HEADER,

    // This command is issued for each row of bitmapped image data.  The
    // *data parameter should point to the data, and size should be the
    // number of bits of data to send to the printer, which should match the
    // value passed in the USB_PRINTER_IMAGE_DATA_HEADER command.  This command
    // supports reading image data from either RAM or ROM.  Be sure to
    // set the transferFlags parameter appropriately to indicate the location of the
    // bitmapped data.  If the data is in RAM but the application may overwrite
    // it, set the USB_PRINTER_TRANSFER_COPY_DATA flag to tell the printer
    // client driver to make a local copy of the data, allowing the application
    // to overwrite the original buffer when USBHostPrinterCommand() terminates.
    // Refer to the USB_PRINTER_IMAGE_START command for the sequence
    // required to print an image.  Be sure that
    // adequate heap space is available, particularly when printing from ROM
    // or external memory, and when printing to a POS printer.  When printing
    // images on POS printers, ensure that the dot density capabilities of the
    // printer are set correctly.  If they are not, the printer will print
    // garbage characters.  Refer to the Printer Client Driver section of the
    // Help file for more information about printing images.
    USB_PRINTER_IMAGE_DATA,

    // This command is used to terminate printing a bitmapped image.  Refer to
    // the USB_PRINTER_IMAGE_START command for the sequence required to print
    // an image.
    USB_PRINTER_IMAGE_STOP,

    //--------------------------------------------------------------------------

    // Commands between USB_PRINTER_VECTOR_GRAPHICS_START and
    // USB_PRINTER_VECTOR_GRAPHICS_END are valid only with printers that support
    // vector graphics.  This support is determined by the interface function
    // of the type USB_PRINTER_LANGUAGE_SUPPORTED that is specified in the
    // usb_config.c configuration file.
    USB_PRINTER_VECTOR_GRAPHICS_START,

    // (Vector graphics support required.)
    // This command sets the line type for drawing graphics.  The line type
    // indication is passed in the size parameter.  Valid values are
    // PRINTER_LINE_TYPE_SOLID, PRINTER_LINE_TYPE_DOTTED, and
    // PRINTER_LINE_TYPE_DASHED.  The data pointer parameter is not used and
    // should be set to USB_NULL.
    USB_PRINTER_GRAPHICS_LINE_TYPE,

    // (Vector graphics support required.)
    // This command sets the width of the line for drawing vector graphics.
    // The width indication is passed in the size parameter.  For full sheet
    // printers, valid values are PRINTER_LINE_WIDTH_NORMAL and
    // PRINTER_LINE_WIDTH_THICK.  For POS printers, the size is specified in
    // dots (1-255).  The data pointer parameter is not used and should be set
    // to USB_NULL.
    USB_PRINTER_GRAPHICS_LINE_WIDTH,

    // (Vector graphics support required.)
    // This command sets the style of the end of the lines used for drawing
    // vector graphics.  The style indication is passed in the size
    // parameter.  Valid values are PRINTER_LINE_END_BUTT,
    // PRINTER_LINE_END_ROUND, and PRINTER_LINE_END_SQUARE.  The data
    // pointer parameter is not used and should be set to USB_NULL.
    USB_PRINTER_GRAPHICS_LINE_END,

    // (Vector graphics support required.)
    // This commands sets the style of how lines are joined when drawing vector
    // graphics.  The style indication is passed in the size parameter.  Valid
    // values are PRINTER_LINE_JOIN_BEVEL, PRINTER_LINE_JOIN_MITER, and
    // PRINTER_LINE_JOIN_ROUND.  The data pointer parameter is not used and
    // should be set to USB_NULL.
    USB_PRINTER_GRAPHICS_LINE_JOIN,

    // (Vector graphics support required.)
    // This command sets the fill type for drawing filled vector graphics.
    // The data pointer should point to a data structure that matches the
    // sFillType structure in the USB_PRINTER_GRAPHICS_PARAMETERS union.
    // Valid values for the fillType member are:
    // * PRINTER_FILL_SOLID.  Other structure members are ignored.
    // * PRINTER_FILL_SHADED.  0 <= shading <= 100
    // * PRINTER_FILL_HATCHED.  The spacing is specified in points, angle is
    //    specified in degrees.
    // * PRINTER_FILL_CROSS_HATCHED.  The spacing is specified in points, angle is
    //    specified in degrees.
    // Note: This command is not supported by all printer languages with vector
    // graphics, and not all fill types are supported by all printers.  Refer
    // to the specific language for the supported fill types.
    USB_PRINTER_GRAPHICS_FILL_TYPE,

    // (Vector graphics support required.)
    // This command sets the color of the line for drawing vector graphics.  The
    // color indication is passed in the size parameter.  Valid values
    // are PRINTER_COLOR_BLACK and PRINTER_COLOR_WHITE.  The data pointer
    // parameter is not used and should be set to USB_NULL.
    USB_PRINTER_GRAPHICS_COLOR,

    // (Vector graphics support required.)
    // This command moves the graphics pen to the specified position.  The
    // position is specified in terms of points.  The X-axis position value is
    // passed in the most significant word of the size parameter, and the Y-axis
    // position value is passed in the least significant word of the size
    // parameter.  POS printers support specifying the Y-axis position while in
    // page mode only.  The data pointer parameter is not used and should be set to USB_NULL.
    USB_PRINTER_GRAPHICS_MOVE_TO,

    // (Vector graphics support required.)
    // This command moves the graphics pen to the specified relative position.
    // The change in position is specified in terms of points.  The X-axis
    // position change is passed in the most significant word of the size
    // parameter, and the Y-axis position change is passed in the least
    // significant word of the size parameter.  POS printers do not support
    // specifying the Y-axis position.
    USB_PRINTER_GRAPHICS_MOVE_RELATIVE,

    // (Vector graphics support required.)
    // This command draws a line from one specified x,y position to another
    // specified x,y position.  The data pointer should point to a data
    // structure that matches the sLine structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.
    USB_PRINTER_GRAPHICS_LINE,

    // (Vector graphics support required.)
    // This command draws a line from the current x,y position to the
    // specified x,y position.  The new x,y position is passed in the size
    // parameter.  The X-axis position value is passed in the most significant
    // word of the size parameter, and the Y-axis position value is passed in
    // the least significant word of the size parameter.  The data pointer
    // parameter is not used and should be set to USB_NULL.
    USB_PRINTER_GRAPHICS_LINE_TO,

    // (Vector graphics support required.)
    // This command draws a line from the current x,y position to the
    // x,y position defined by the indicated displacement.  The x and y
    // displacements are passed in the size parameter.  The X-axis displacement
    // is passed in the most significant word of the size parameter, and the
    // Y-axis displacement is passed in the least significant word of the size
    // parameter.  The data pointer parameter is not used and should be set to USB_NULL.
    USB_PRINTER_GRAPHICS_LINE_TO_RELATIVE,

    // (Vector graphics support required.)
    // This command draws an arc, or a piece of a circle.  The data pointer
    // should point to a data structure that matches the sArc structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.  This command can print only one
    // arc of a circle, unlike the Graphics library, which can print multiple
    // separated arcs of the same circle.
    USB_PRINTER_GRAPHICS_ARC,

    // (Vector graphics support required.)
    // This command draws a circle using the current pen color and width.  The
    // inside of the circle is not filled.  To draw a filled circle, use the
    // command USB_PRINTER_GRAPHICS_CIRCLE_FILLED. The data pointer should
    // point to a data structure that matches the sCircle structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.
    USB_PRINTER_GRAPHICS_CIRCLE,

    // (Vector graphics support required.)
    // This command draws a filled circle using the current pen color.  To draw
    // the outline of a circle, use the command
    // USB_PRINTER_GRAPHICS_CIRCLE_FILLED. The data pointer should point to a
    // data structure that matches the sCircle structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.
    USB_PRINTER_GRAPHICS_CIRCLE_FILLED,

    // (Vector graphics support required.)
    // This command draws an outlined bevel (rectangle with rounded corners)
    // using the current pen color and width.  The inside of the bevel is not
    // filled.  To draw a filled bevel, use the command
    // USB_PRINTER_GRAPHICS_BEVEL_FILLED. The data pointer should point to a
    // data structure that matches the sBevel structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.
    USB_PRINTER_GRAPHICS_BEVEL,

    // (Vector graphics support required.)
    // This command draws a filled bevel using the current pen color.  To draw
    // the outline of a bevel, use the command
    // USB_PRINTER_GRAPHICS_BEVEL_FILLED. The data pointer should point to a
    // data structure that matches the sBevel structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.
    USB_PRINTER_GRAPHICS_BEVEL_FILLED,

    // (Vector graphics support required.)
    // This command draws a rectangle using the current pen color and width.  The
    // inside of the rectangle is not filled.  To draw a filled rectangle, use
    // the command USB_PRINTER_GRAPHICS_RECTANGLE_FILLED. The data pointer should
    // point to a data structure that matches the sRectangle structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.
    USB_PRINTER_GRAPHICS_RECTANGLE,

    // (Vector graphics support required.)
    // This command draws a filled rectangle using the current pen color.  To draw
    // the outline of a rectangle, use the command
    // USB_PRINTER_GRAPHICS_RECTANGLE. The data pointer should point to a
    // data structure that matches the sRectangle structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.
    USB_PRINTER_GRAPHICS_RECTANGLE_FILLED,

    // (Vector graphics support required.)
    // This command draws the outline of a polygon with a specified number of
    // sides, using the current pen color and width.  The data pointer should
    // point to a data structure that matches the sPolygon structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.  This structure contains the
    // number of verticies of the polygon and a pointer to an array containing
    // x,y coordinates of the verticies.  Line segments are drawn to each vertex
    // in the order that they appear in the array.
    USB_PRINTER_GRAPHICS_POLYGON,

    // Commands between USB_PRINTER_VECTOR_GRAPHICS_START and
    // USB_PRINTER_VECTOR_GRAPHICS_END are valid only with printers that support
    // vector graphics.  This support is determined by the interface function
    // of the type USB_PRINTER_LANGUAGE_SUPPORTED that is specified in the
    // usb_config.c configuration file.
    USB_PRINTER_VECTOR_GRAPHICS_END,

    //--------------------------------------------------------------------------

#ifdef USB_PRINTER_LANGUAGE_ESCPOS

    // Commands between USB_PRINTER_POS_START and USB_PRINTER_POS_END are valid
    // only with point-of-sale printers.  This support is determined by the
    // interface function of the type USB_PRINTER_LANGUAGE_SUPPORTED that is
    // specified in the usb_config.c configuration file.
    USB_PRINTER_POS_START,

    // (POS support required.)  This command sets the printer into page mode.
    // In this mode, print commands are retained by the printer until it
    // receives the USB_PRINTER_EJECT_PAGE command.  This allows the application
    // to create more sophisticated output.  The data pointer should point to a
    // data struture that matches the sPage structure in the
    // USB_PRINTER_GRAPHICS_PARAMETERS union.  This structure contains the
    // horizontal and vertical starting point, the horizontal and vertical print
    // length, and the print direction and starting point.  Valid values for the
    // print direction and starting point are:
    // * PRINTER_POS_LEFT_TO_RIGHT
    // * PRINTER_POS_BOTTOM_TO_TOP
    // * PRINTER_POS_RIGHT_TO_LEFT
    // * PRINTER_POS_TOP_TO_BOTTOM
    USB_PRINTER_POS_PAGE_MODE,

    // (POS support required.)  This command sets the printer into standard mode.
    // In this mode, print commands are processed and printed immediately.  This
    // is typically the default mode for a POS printer.
    USB_PRINTER_POS_STANDARD_MODE,

    // (POS support required.)  This command feeds the specified number of
    // lines, as dictated by the size parameter (between 0 and 255).  The data
    // parameter is not used, and should be set to USB_NULL.
    USB_PRINTER_POS_FEED,

    // (POS support required.)  This command is a simplified method of printing
    // a text line to a POS printer.  This command prints a single, null
    // terminated string and feeds a specified number of lines after the print.
    // The data pointer must point to a null terminated string located in RAM.
    // Printing strings from ROM is not supported.  The size parameter should
    // be set to the number of lines to feed after the text is printed.
    USB_PRINTER_POS_TEXT_LINE,

    // (POS support required.)  This command cuts the paper completely.  The
    // data parameter is not used, and should be passed as USB_NULL. The least
    // significant byte of the size parameter indicates the number of vertical
    // motion units (printer dependent, typical values are 1/360 inch to
    // 1/144 inch) to feed before the cut.  Not all POS printer models support
    // this command.
    #ifdef USB_PRINTER_POS_CUTTER_SUPPORT
    USB_PRINTER_POS_CUT,
    #endif

    // (POS support required.)  This command cuts the paper, leaving one point
    // uncut.  The data parameter is not used, and should be passed as USB_NULL.
    // The least significant byte of the size parameter indicates the number of
    // vertical motion units (printer dependent, typical values are 1/360 inch
    // to 1/144 inch) to feed before the cut.  Not all POS printer models support
    // this command.
    #ifdef USB_PRINTER_POS_CUTTER_SUPPORT
    USB_PRINTER_POS_CUT_PARTIAL,
    #endif

    // (POS support required.)  This command sets the printing justification to
    // the center of the print area.  The data and size parameters are not used,
    // and should be set to USB_NULL and 0 respectively.
    USB_PRINTER_POS_JUSTIFICATION_CENTER,

    // (POS support required.)  This command sets the printing justification to
    // the left side of the print area.  The data and size parameters are not used,
    // and should be set to USB_NULL and 0 respectively.
    USB_PRINTER_POS_JUSTIFICATION_LEFT,

    // (POS support required.)  This command sets the printing justification to
    // the right side of the print area.  The data and size parameters are not used,
    // and should be set to USB_NULL and 0 respectively.
    USB_PRINTER_POS_JUSTIFICATION_RIGHT,

    // (POS support required.)  This command enables or disables white/black
    // reverse printing of characters.  When enabled, characters are printed in
    // white on a black background, and underlining is not performed.  To enable
    // reverse printing, set the size parameter to 1.  To disable reverse printing,
    // set the size parameter to 0.  (Only the least significant bit of the size
    // parameter is examined.)  The data parameter is not used, and should be set
    // to USB_NULL.  Not all POS printer models support
    // this command.
    #ifdef USB_PRINTER_POS_REVERSE_TEXT_SUPPORT
    USB_PRINTER_POS_FONT_REVERSE,
    #endif

    // (POS support required.)  This command enables or disables underlining.
    // Underlining is not performed if reverse printing is enabled.  To enable
    // underlining, set the size parameter to 1.  To disable underlining, set
    // the size parameter to 0.  (Only the least significant bit of the size
    // parameter is examined.)  The data parameter is not used, and should be set
    // to USB_NULL.
    USB_PRINTER_POS_FONT_UNDERLINE,

    // (POS support required.)  This command changes the print color to black.
    // This command is available only with printers that support two color
    // printing.  The data and size parameters are not used,
    // and should be set to USB_NULL and 0 respectively.
    #ifdef USB_PRINTER_POS_TWO_COLOR_SUPPORT
    USB_PRINTER_POS_COLOR_BLACK,
    #endif

    // (POS support required.)  This command changes the print color to red.
    // This command is available only with printers that support two color
    // printing.  The data and size parameters are not used,
    // and should be set to USB_NULL and 0 respectively.
    #ifdef USB_PRINTER_POS_TWO_COLOR_SUPPORT
    USB_PRINTER_POS_COLOR_RED,
    #endif

    // (POS support required.)  This command prints a bar code.  Not all POS
    // printers provide bar code support, and the types of bar codes supported
    // may vary; check the technical documentation for the desired target
    // printer.  The data pointer should point to a data structure that matches
    // the sBarCode structure in the USB_PRINTER_GRAPHICS_PARAMETERS union.
    // This structure contains the type of bar code (as specified by the
    // USB_PRINTER_POS_BARCODE_FORMAT enumeration) and the bar code data.
    #ifdef USB_PRINTER_POS_BARCODE_SUPPORT
    USB_PRINTER_POS_BARCODE,
    #endif

    // Commands between USB_PRINTER_POS_START and USB_PRINTER_POS_END are valid
    // only with point-of-sale printers.  This support is determined by the
    // interface function of the type USB_PRINTER_LANGUAGE_SUPPORTED that is
    // specified in the usb_config.c configuration file.
    USB_PRINTER_POS_END,

#endif

} USB_PRINTER_COMMAND;


//-----------------------------------------------------------------------------
// Section: Printer Transfer Flags

        // This flag indicates that the printer client driver should make a
        // copy of the data passed to the command.  This allows the application
        // to reuse the data storage immediately instead of waiting until the
        // transfer is sent to the printer.  The client driver will allocate
        // space in the heap for the data copy.  If there is not enough
        // available memory, the command will terminate with a
        // USB_PRINTER_OUT_OF_MEMORY error.  Otherwise, the original data will
        // be copied to the temporary data space.  This temporary data will be
        // freed upon completion, regardless of whether or not the command was
        // performed successfully.  NOTE: If the data is located in ROM, the
        // flag USB_PRINTER_TRANSFER_FROM_ROM must be used instead.
#define USB_PRINTER_TRANSFER_COPY_DATA      0x01

        // This flag indicates that the data will not change in the time
        // between the printer command being issued and the data actually being
        // sent to the printer.
#define USB_PRINTER_TRANSFER_STATIC_DATA    0x00

        // This flag indicates that the application layer wants to receive an
        // event notification when the command completes.
#define USB_PRINTER_TRANSFER_NOTIFY         0x02

        // This flag indicates that the source of the command data is in ROM.
        // The data will be copied to RAM, since the USB Host layer cannot read
        // data from ROM.  If there is not enough available heap space to make
        // a copy of the data, the command will fail.  If using this flag, do
        // not set the USB_PRINTER_TRANSFER_COPY_DATA flag.
#define USB_PRINTER_TRANSFER_FROM_ROM       0x04

        // This flag indicates that the source of the command data is in RAM.
        // The application can then choose whether or not to have the printer
        // client driver make a copy of the data.
#define USB_PRINTER_TRANSFER_FROM_RAM       0x00

        // This flag indicates that the source of the command data is in
        // external memory.  This flag is for use only with the
        // USB_PRINTER_IMAGE_DATA and USB_PRINTER_TEXT commands.
//for future implementation #define USB_PRINTER_TRANSFER_FROM_EXTERNAL  0x08

        // Use this macro to set the USB_PRINTER_TRANSFER_COPY_DATA flag
        // in a variable.
#define USBHOSTPRINTER_SETFLAG_COPY_DATA(x)     {x |= USB_PRINTER_TRANSFER_COPY_DATA;}

        // Use this macro to clear the USB_PRINTER_TRANSFER_COPY_DATA flag
        // in a variable.
#define USBHOSTPRINTER_SETFLAG_STATIC_DATA(x)   {x &= ~USB_PRINTER_TRANSFER_COPY_DATA;}

        // Use this macro to set the USB_PRINTER_TRANSFER_NOTIFY flag in a
        // variable.
#define USBHOSTPRINTER_SETFLAG_NOTIFY(x)        {x |= USB_PRINTER_TRANSFER_NOTIFY;}


//-----------------------------------------------------------------------------
/* Printer Fonts

This enumeration defines the various printer fonts.  If new fonts are added,
they must be added at the end of the list, just before the
USB_PRINTER_FONT_MAX_FONT definition, as the printer language support files
may utilize them for indexing purposes.
*/
typedef enum
{
    USB_PRINTER_FONT_AVANT_GARDE = 0,           // Avant Garde font
    USB_PRINTER_FONT_BOOKMAN,                   // Bookman font
    USB_PRINTER_FONT_COURIER,                   // Courier font
    USB_PRINTER_FONT_HELVETICA,                 // Helvetica font
    USB_PRINTER_FONT_HELVETICA_NARROW,          // Helvetica Narrow font
    USB_PRINTER_FONT_NEW_CENTURY_SCHOOLBOOK,    // New Century Schoolbook font
    USB_PRINTER_FONT_PALATINO,                  // Palatino font
    USB_PRINTER_FONT_TIMES_NEW_ROMAN,           // Times New Roman font
    USB_PRINTER_FONT_MAX_FONT                   // Font out of range
} USB_PRINTER_FONTS;


//-----------------------------------------------------------------------------
/* POS Printer Fonts

This enumeration defines the various printer fonts used by POS printers.  If
new fonts are added, they must be added at the end of the list, just before the
USB_PRINTER_FONT_POS_MAX_FONT definition, as the printer language support files
may utilize them for indexing purposes.
*/
typedef enum
{
    USB_PRINTER_FONT_POS_18x36,         // Character size 18x36
    USB_PRINTER_FONT_POS_18x72,         // Character size 18x36, double height
    USB_PRINTER_FONT_POS_36x36,         // Character size 18x36, double width
    USB_PRINTER_FONT_POS_36x72,         // Character size 18x36, double height and width
    USB_PRINTER_FONT_POS_12x24,         // Character size 12x24
    USB_PRINTER_FONT_POS_12x48,         // Character size 12x24, double height
    USB_PRINTER_FONT_POS_24x24,         // Character size 12x24, double width
    USB_PRINTER_FONT_POS_24x48,         // Character size 12x24, double height and width
    USB_PRINTER_FONT_POS_MAX_FONT       // Font out of range
} USB_PRINTER_FONTS_POS;


//-----------------------------------------------------------------------------
/* Printer Errors

These are errors that can be returned by the printer client driver.  Note that
USB Embedded Host errors can also be returned.
*/
typedef enum
{
        // The command was successful.
    USB_PRINTER_SUCCESS             = 0,

        // The command cannot be performed because the printer client driver's
        // command queue is full.  Use the function USBHostPrinterCommandReady()
        // to determine if there is space available in the queue.
    USB_PRINTER_BUSY                = USB_ERROR_CLASS_DEFINED,

        // An invalid printer command was requested.  Refer to the enumeration
        // USB_PRINTER_COMMAND for the list of valid commands.
    USB_PRINTER_UNKNOWN_COMMAND,

        // A device with the indicated address is not attached or is not
        // a printer.
    USB_PRINTER_UNKNOWN_DEVICE,

        // Not enough free heap space is available to perform the command.
    USB_PRINTER_OUT_OF_MEMORY,

        // The number of attached printers exceeds the maximum specified by
        // USB_MAX_PRINTER_DEVICES.  Refer to the USB configuration tool.
    USB_PRINTER_TOO_MANY_DEVICES,

        // An invalid or out of range parameter was passed.  Run time checking
        // of graphics coordinates must be enabled by defining
        // PRINTER_GRAPHICS_COORDINATE_CHECKING.
    USB_PRINTER_BAD_PARAMETER

} USB_PRINTER_ERRORS;


//-----------------------------------------------------------------------------
/* Bar Code Formats

These are the bar code formats for printing bar codes on POS printers.  They
are used in conjuction with the USB_PRINTER_POS_BARCODE command
(USB_PRINTER_COMMAND).  Bar code information is passed using the sBarCode
structure within the USB_PRINTER_GRAPHICS_PARAMETERS union.  The exact values 
to send for each bar code type can vary for the particular POS printer, and not 
all printers support all bar code types.  Be sure to test the output on the 
target printer, and adjust the values specified in usb_host_printer_esc_pos.c 
if necessary.  Refer to the printer's technical documentation for the required 
values.  Do not alter this enumeration.
*/
#ifdef USB_PRINTER_POS_BARCODE_SUPPORT
typedef enum
{
        // UPC-A bar code format.  Typically used for making products with a
        // unique code,as well as for coupons, periodicals, and paperback books.
        // The data for this bar code must consist
        // of 11 values from '0' to '9' (ASCII), and the data length for this
        // bar code must be 11.  The first digit is the number system character:
        // * 0, 6, 7 Regular UPC codes
        // * 2 Random weight items
        // * 3 National Drug Code and National Health Related Items Code
        // * 4 In-store marking of non-food items
        // * 5 Coupons
        // * 1, 8, 9 Reserved
        // A check digit will be automatically calculated and appended.
        // For more information, refer to the UPC Symbol Specification Manual
        // from the Uniform Code Council.
    USB_PRINTER_POS_BARCODE_UPC_A = 0,
        // UPC-E bar code format.  Similar to UPC-A but with restrictions. Data
        // lengths of 6, 7, or 11 bytes are supported.  Not all printers support
        // the 6 or 7 byte widths; 11 byte data is recommended.  If the data length is
        // not 6, then the first the first digit (the number system character)
        // must be set to '0'.  If 11 data bytes are presented, the printer will
        // generate a shortened 6-digit code.
        // The check digit will be automatically calculated and appended.
    USB_PRINTER_POS_BARCODE_UPC_E,
        // EAN/JAN-13 bar code format.  Similar to UPC-A, but there are 12
        // numeric digits plus a checksum digit.
        // The check digit will be automatically calculated and appended.
    USB_PRINTER_POS_BARCODE_EAN13,
        // EAN/JAN-8 bar code format.  Similar to UPC-E, but there are 7
        // numeric digits, and the first digit (the number system character)
        // must be set to '0'.
        // The check digit will be automatically calculated and appended.
    USB_PRINTER_POS_BARCODE_EAN8,
        // CODE39 bar code format.  Typically used in applications where the
        // data length may change.  This format uses encoded numeric characters,
        // uppercase alphabet characters, and the symbols '-' (dash), '.' (period),
        // ' ' (space), '$' (dollar sign), '/' (forward slash), '+' (plus), and
        // '%' (percent).
        // If the bPrintCheckDigit flag is set, then the check digit will be 
        // automatically calculated and appended.  Otherwise, no check digit 
        // will be printed.
    USB_PRINTER_POS_BARCODE_CODE39,
        // ITF, or Interleaved 2 of 5, bar code format.  Used in
        // applications that have a fixed data length for all items.  Only the
        // digits 0-9 can be encoded, and there must be an even number of
        // digits.
    USB_PRINTER_POS_BARCODE_ITF,
        // Codabar bar code format.  Useful in applications that contain mostly
        // numeric digits and variable data sizes.  This format utilizes the
        // digits 0-9, letters A-D (used as start/stop characters), '-' (dash),
        // '$' (dollar sign), ':' (colon), '/' (forward slash), '.' (period),
        // and '+' (plus).
        // If the bPrintCheckDigit flag is set, then the check digit will be 
        // automatically calculated and appended.  Otherwise, no check digit 
        // will be printed.
    USB_PRINTER_POS_BARCODE_CODABAR,

#ifdef USE_PRINTER_POS_EXTENDED_BARCODE_FORMAT
        // (Available only if the printer supports extended bar code formats.)
        // CODE93 bar code format.  Used in applications that require heavy
        // error checking.  It has a variable data size, and uses 128-bit ASCII
        // characters.  The start code, stop code, and check digits are added
        // automatically.
    USB_PRINTER_POS_BARCODE_CODE93,
        // (Available only if the printer supports extended bar code formats.)
        // Code 128 bar code format.  Used in applications that require a large
        // amount of variable length data and extra error checking.  It uses
        // 128-bit ASCII plus special symbols.  The first two data bytes must
        // be the code set selection character.  The first byte must be
        // BARCODE_CODE128_CODESET (0x7B), and the second byte must be 'A',
        // 'B', or 'C'.  In general Code A should be used if the data contains
        // control characters (0x00 - 0x1F), and Code B should be used if the
        // data contains lower case letters and higher ASCII values (0x60-0x7F).
        // If an ASCII '{' (left brace, 0x7B) is contained in the data, it must
        // be encoded as two bytes with the value 0x7B.
    USB_PRINTER_POS_BARCODE_CODE128,
        // NOT YET SUPPORTED (Available only if the printer supports extended bar code formats.)
        // EAN-128 or UCC-128 bar code format.  Used in shipping applications.
        // Refer to the Application Standard for Shopping Container Codes from
        // the Uniform Code Council.
    USB_PRINTER_POS_BARCODE_EAN128,
#endif

        // Bar code type out of range.
    USB_PRINTER_POS_BARCODE_MAX

} USB_PRINTER_POS_BARCODE_FORMAT;
#endif

//DOM-IGNORE-BEGIN
// For backward compatibility with typo.
#define USB_PRINTER_POS_BARCODE_CODEABAR USB_PRINTER_POS_BARCODE_CODABAR
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: USB Data Structures
// *****************************************************************************
// *****************************************************************************

        // This type is used to represent a generic RAM or ROM pointer when
        // passed to the function USBHostPrinterCommand() or a printer language
        // function of the type
        // USB_PRINTER_LANGUAGE_HANDLER.  Note that the caller must indicate
        // whether the point is actually pointing to RAM or to ROM, so we can
        // tell which pointer is valid.  Not all printer commands can actually
        // use data in ROM.  Refer to the specific printer command in the
        // USB_PRINTER_COMMAND enumeration for more information.
typedef union {
    void            *pointerRAM;    // Pointer to data in RAM.
  #if defined( __C30__ )
    __prog__ void   *pointerROM;    // Pointer to data in ROM.
  #elif defined( __PIC32MX__ )
    const void      *pointerROM;    // Pointer to data in ROM.
  #endif
} USB_DATA_POINTER;

        // Use this definition to cast a pointer being passed to the function
        // USBHostPrinterCommand() that points to data in RAM.
#define USB_DATA_POINTER_RAM(x)  ((USB_DATA_POINTER)(void *)x)
        // Use this definition to cast a pointer being passed to the function
        // USBHostPrinterCommand() that points to data in ROM.
#if defined( __C30__ )
    #define USB_DATA_POINTER_ROM(x)  ((USB_DATA_POINTER)(__prog__ void *)x)
#elif defined( __PIC32MX__ )
    #define USB_DATA_POINTER_ROM(x)  ((USB_DATA_POINTER)(const void *)x)
#endif
        // Use this definition to pass a NULL pointer to the function
        // USBHostPrinterCommand().
#define USB_NULL    (USB_DATA_POINTER)(void *)NULL

// *****************************************************************************
/* Printer Device Function support Information

This structure contains information about the functions that the attached
printer supports.  See the related constants for setting these flags via the
val member:
* USB_PRINTER_FUNCTION_SUPPORT_POS
* USB_PRINTER_FUNCTION_SUPPORT_VECTOR_GRAPHICS
*/

typedef union
{
    WORD    val;                    // The WORD representation of the support flags.
    struct
    {
        WORD    supportsVectorGraphics  : 1;    // The printer supports vector graphics.
        WORD    supportsPOS             : 1;    // The printer is a POS printer.
    } supportFlags;                 // Various printer function support flags.
} USB_PRINTER_FUNCTION_SUPPORT;


// *****************************************************************************
/* Printer Device ID Information

This structure contains identification information about an attached device.
*/
typedef struct _USB_PRINTER_DEVICE_ID
{
    WORD                            vid;            // Vendor ID of the device
    WORD                            pid;            // Product ID of the device
    USB_PRINTER_FUNCTION_SUPPORT    support;        // Function support flags.
    BYTE                            deviceAddress;  // Address of the device on the USB
} USB_PRINTER_DEVICE_ID;


// *****************************************************************************
/* Bitmapped Image Information

This structure contains the information needed to print a bitmapped graphic
image.

When using a full sheet printer, utilize the resolution and the scale members
to specify the size of the image.  Some printer languages (e.g. PostScript) utilize
a scale factor, while others (e.g. PCL 5) utilize a dots-per-inch resolution.
Also, some printers that utilize the resolution specification support only
certain values for the resolution.  For maximum compatibility, specify both
members of this structure.  The following table shows example values that will
generate similarly sized output.
<code>
Resolution (DPI)  Scale
----------------  -----
        75         1.0
       100         0.75
       150         0.5
       200         0.37
       300         0.25
       600         0.13
</code>

When using a POS printer, utilize the densityVertical and densityHorizontal
members to specify the size of the image.  The densityHorizontal can be either
single (1) or double (2).  The valid values for densityVertical are printer
dependent.  Most printers support 8-dot, many support 8 and 24-dot, and a few
support 8, 24, and 36-dot (represented by the values 8, 24, and 36
respectively).  This value affects how the bit image data is sent to the
printer.  The set of allowable values must be configured correctly, since
the image configuration method differs depending on the set of allowed values.
To maintain the aspect ratio, the following selections are recommended:
<code>
Supported Horizontal Densities  densityVertical  densityHorizontal
------------------------------------------------------------------
8-dot                                 8              1 (single)
8 and 24-dot                         24              2 (double)
8, 24, and 36-dot                    24              2 (double)
</code>
The 36-bit density is not recommended, as it requires a great deal of available
heap space, is not supported by the USBHostPrinterPOSImageDataFormat() function,
and produces the same output as the 24-dot density print.
*/
typedef struct
{
    WORD    width;              // The width of the image in pixels.
    WORD    height;             // The height of the image in pixels.
    WORD    positionX;          // The position of the image on the X axis.
    WORD    positionY;          // The position of the image on the Y axis.
    union
    {
        struct
        {
            WORD    resolution;         // (Full sheet printers only.) The resolution
                                        // of the printed image.  This
                                        // parameter is not supported by all printer
                                        // languages.
            float   scale;              // (Full sheet printers only.)  The scaling
                                        // of the printed image.  Both the
                                        // X axis and the Y axis are scaled by this
                                        // amount.  This parameter is not supported by
                                        // all printer languages.
        };
        struct
        {
            BYTE    densityVertical;    // (POS printers only.) The vertical dot
                                        // density of the bit image.  Valid values
                                        // are printer dependent.  See above.
            BYTE    densityHorizontal;  // (POS printers only.)  The horizontal dot
                                        // density of the bit image.  Valid values are
                                        // 1 (single) and 2 (double).  See above.
        };
    };
} USB_PRINTER_IMAGE_INFO;


//-----------------------------------------------------------------------------
/* USB Printer Graphics Parameter Structures

This union can be used to declare a variable that can hold the parameters for
any printer graphics or POS printer command (USB_PRINTER_COMMAND).  The union
allows a single variable to be declared and then reused for any printer
graphics command.
*/

typedef union
{
    // This structure is used by the USB_PRINTER_GRAPHICS_ARC command
    // (USB_PRINTER_COMMAND).
    struct
    {
        WORD    xL;     // X-axis position of the upper left corner.
        WORD    yT;     // Y-axis position of the upper left corner.
        WORD    xR;     // X-axis position of the lower right corner.
        WORD    yB;     // Y-axis position of the lower right corner.
        WORD    r1;     // The inner radius of the two concentric cicles that defines the thickness of the arc.
        WORD    r2;     // The outer of radius the two concentric circles that defines the thickness of the arc.
        WORD    octant; // Bitmask of the octant that will be drawn.  Moving in a clockwise direction from x = 0, y = +radius
                        //     * bit0 : first octant
                        //     * bit1 : second octant
                        //     * bit2 : third octant
                        //     * bit3 : fourth octant
                        //     * bit4 : fifth octant
                        //     * bit5 : sixth octant
                        //     * bit6 : seventh octant
                        //     * bit7 : eigth octant
    } sArc;

    // This structure is used by the USB_PRINTER_POS_BARCODE command (USB_PRINTER_COMMAND).
    struct
    {
        //WORD    x;              // For future implementation - Horizontal location of the lower left corner of the bar code.
        //WORD    y;              // For future implementation - Vertical location of the lower left corner of the bar code.  Readable text will print below this point.
        BYTE    height;         // Bar code height in dots.
        BYTE    type;           // Bar code type.  See the USB_PRINTER_POS_BARCODE_FORMAT enumeration.
        BYTE    textPosition;   // Position of the readable text.  Valid values are BARCODE_TEXT_OMIT, BARCODE_TEXT_ABOVE, BARCODE_TEXT_BELOW, BARCODE_TEXT_ABOVE_AND_BELOW.
        BYTE    textFont;       // Font of the readable text.  Valid values are dependent on the particular POS printer (BARCODE_TEXT_12x24 and BARCODE_TEXT_18x36 for ESC/POS).
        BYTE    *data;          // Pointer to the bar code data.
        BYTE    dataLength;      // Number of bytes of bar code data.
        union
        {
            BYTE    value;
            struct
            {
                BYTE    bPrintCheckDigit    : 1;    // Whether or not to print an optional check digit.  Valid for Code39 (USB_PRINTER_POS_BARCODE_FORMAT USB_PRINTER_POS_BARCODE_CODE39) and CODABAR (USB_PRINTER_POS_BARCODE_FORMAT USB_PRINTER_POS_BARCODE_CODABAR) formats only.
            } bits;
        } flags;        
    } sBarCode;

    // This structure is used by the USB_PRINTER_GRAPHICS_BEVEL and
    // USB_PRINTER_GRAPHICS_BEVEL_FILLED commands (USB_PRINTER_COMMAND).
    struct
    {
        WORD    xL;     // X-axis position of the left side of the bevel.
        WORD    yT;     // Y-axis position of the top of the bevel.
        WORD    xR;     // X-axis position of the right side of the bevel.
        WORD    yB;     // Y-axis position of the bottom of the bevel.
        WORD    r;      // The radius of the cicle that defines the rounded corner
    } sBevel;

    // This structure is used by the USB_PRINTER_GRAPHICS_CIRCLE and
    // USB_PRINTER_GRAPHICS_CIRCLE_FILLED commands (USB_PRINTER_COMMAND).
    struct
    {
        WORD    x;      // X-axis position of the center of the circle.
        WORD    y;      // Y-axis position of the center of the circle.
        WORD    r;      // Radius of the circle.
    } sCircle;

    // This structure is used by the USB_PRINTER_GRAPHICS_FILL_TYPE command (USB_PRINTER_COMMAND).
    struct
    {
        WORD    fillType;   // The type of fill.  See USB_PRINTER_GRAPHICS_FILL_TYPE for valid values.
        WORD    spacing;    // Line spacing for hatched fill (if supported).
        WORD    angle;      // Line angle for hatched fill (if supported).
        WORD    shading;    // Shading level for shaded fill.  Printer support may be limited.
    } sFillType;

    // This structure is used by the USB_PRINTER_GRAPHICS_LINE command (USB_PRINTER_COMMAND).
    struct
    {
        WORD    x1;     // X-axis position of the first point.
        WORD    y1;     // Y-axis position of the first point.
        WORD    x2;     // X-axis position of the second point.
        WORD    y2;     // Y-axis position of the second point.
    } sLine;

    // This structure is used by POS printers and the USB_PRINTER_POS_PAGE_MODE
    // command (USB_PRINTER_COMMAND).
    struct
    {
        WORD    startPointHorizontal;   // The horizontal page starting point.
        WORD    startPointVertical;     // The vertical page starting point.
        WORD    lengthHorizontal;       // The horizontal print length.
        WORD    lengthVertical;         // The vertical print length.
        BYTE    printDirection;         // The print direction and starting point.
    } sPage;

    // This structure is used by the USB_PRINTER_GRAPHICS_POLYGON command (USB_PRINTER_COMMAND).
    struct
    {
        SHORT   numPoints;  // The number of points of the polygon.
        WORD    *points;    // The array of polygon points {x1, y1, x2, y2, ... xn, yn}.
    } sPolygon;

    // This structure is used by the USB_PRINTER_GRAPHICS_RECTANGLE and
    // USB_PRINTER_GRAPHICS_RECTANGLE_FILLED commands (USB_PRINTER_COMMAND).
    struct
    {
        WORD    xL;     // X-axis position of the left side of the rectangle.
        WORD    yT;     // Y-axis position of the top of the rectangle.
        WORD    xR;     // X-axis position of the right side of the rectangle.
        WORD    yB;     // Y-axis position of the bottom of the rectangle.
    } sRectangle;

} USB_PRINTER_GRAPHICS_PARAMETERS;


// *****************************************************************************
// *****************************************************************************
// Section: USB Host Printer - Printer Language Interface
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BYTE (*USB_PRINTER_LANGUAGE_HANDLER) ( BYTE address,
        USB_PRINTER_COMMAND command, USB_DATA_POINTER data, DWORD size, BYTE flags )

  Summary:
    This is a typedef to use when defining a printer language command handler.

  Description:
    This data type defines a pointer to a call-back function that must be
    implemented by a printer language driver.  When the user calls the
    printer interface function, the appropriate language driver with this
    prototype will be called to generate the proper commands for the requested
    operation.

    Not all printer commands support data from both RAM and ROM.  Unless
    otherwise noted, the data pointer is assumed to point to RAM, regardless of
    the value of transferFlags.  Refer to the specific command to see if ROM
    data is supported.

  Precondition:
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
    None
  ***************************************************************************/

typedef BYTE (*USB_PRINTER_LANGUAGE_HANDLER) ( BYTE address,
        USB_PRINTER_COMMAND command, USB_DATA_POINTER data, DWORD size, BYTE flags );


/****************************************************************************
  Function:
    BOOL (*USB_PRINTER_LANGUAGE_SUPPORTED) ( char *deviceID,
                USB_PRINTER_FUNCTION_SUPPORT *support )

  Summary:
    This is a typedef to use when defining a function that determines if the
    printer with the given "COMMAND SET:" portion of the device ID string
    supports the particular printer language.

  Description:
    This data type defines a pointer to a call-back function that must be
    implemented by a printer language driver.  When the user calls a function
    of this type, the language driver will return a BOOL indicating if the
    language driver supports a printer with the indicated "COMMAND SET:"
    portion of the device ID string.  If the printer is supported, this
    function also returns information about the types of operations that
    the printer supports.

  Preconditions:
    None

  Parameters:
    char *deviceID  - Pointer to the "COMMAND SET:" portion of the device ID
                        string of the attached printer.
    USB_PRINTER_FUNCTION_SUPPORT *support   - Pointer to returned information
                        about what types of functions this printer supports.

  Return Values:
    TRUE    - The printer language can be used with the attached printer.
    FALSE   - The printer language cannot be used with the attached printer.

  Remarks:
    The caller must first locate the "COMMAND SET:" section of the device ID
    string.  To ensure that only the "COMMAND SET:" section of the device ID
    string is checked, the ";" at the end of the section should be temporarily
    replaced with a NULL.  Otherwise, this function may find the printer
    language string in the comments or other section, and incorrectly indicate
    that the printer supports the language.

    Device ID strings are case sensitive.
  ***************************************************************************/

typedef BOOL (*USB_PRINTER_LANGUAGE_SUPPORTED) ( char *deviceID,
                USB_PRINTER_FUNCTION_SUPPORT *support );


/****************************************************************************
  Function:
    WORD USB_PRINTER_EXTERNAL_MEMORY_CALLBACK
        (EXTDATA* memory, LONG offset, WORD nCount, void* buffer)

  Summary:
    If printer data is obtained from external memory, then a function must be
    defined by the application to copy requested data from the external memory
    to a RAM buffer.

  Description:
    If printer data is obtained from external memory, then a function must be
    defined by the application to copy requested data from the external memory
    to a RAM buffer.  The label USB_PRINTER_EXTERNAL_MEMORY_CALLBACK must be
    set to the name of this function.

  Precondition:
    None

  Parameters:
    EXTDATA *memory - Pointer to the eternal memory bitmap
    LONG offset     - Byte offset of required data.
    WORD nCount     - Number of data bytes to copy into the buffer.
    void *buffer    - Destination data buffer.

  Returns:
    The function returns the number of data bytes transferred.

  Example:
    <code>
    #define USB_PRINTER_EXTERNAL_MEMORY_CALLBACK ExternalMemoryCallback

    ...

    // If there are several memories in the system they can be selected by IDs.
    // In this example, ID for memory device used is assumed to be 0.
    #define X_MEMORY 0

    WORD ExternalMemoryCallback( EXTDATA *memory, LONG offset, WORD nCount, void *buffer )
    {
        int i;
        long address;

        // Address of the requested data is the starting address of the object
        // referred by EXTDATA structure plus offset
        address = memory->address+offset;

       if (memory->ID == X_MEMORY)
       {
           // MemoryXReadByte() is some function implemented to access external memory.
           // Implementation will be specific to the memory used. In this example
           // it reads byte each time it is called.
           i = 0;
           while (i < nCount)
           {
               (BYTE*)buffer++ = MemoryXReadByte(address++);
               i++;
           }
        }
        // return the actual number of bytes retrieved
        return (i);
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

//for future implementation
//
//#if defined( USB_PRINTER_EXTERNAL_MEMORY_CALLBACK )
//    WORD USB_PRINTER_EXTERNAL_MEMORY_CALLBACK
//                (EXTDATA* memory, LONG offset, WORD nCount, void* buffer);
//#else
//    // If the application does not have an external memory callback, we will
//    // define a stub.
//    #define USB_PRINTER_EXTERNAL_MEMORY_CALLBACK(m,o,c,b) 0
//#endif


// *****************************************************************************
// *****************************************************************************
// Section: USB Data Structures - continued
// *****************************************************************************
// *****************************************************************************

//-----------------------------------------------------------------------------
/* USB Printer Interface Structure

This structure represents the information needed to interface with a printer
language.  An array of these structures must be created in usb_config.c, so
the USB printer client driver can determine what printer language to use to
communicate with the printer.
*/
typedef struct
{
        // Function in the printer language support file that handles all
        // printer commands.
    USB_PRINTER_LANGUAGE_HANDLER    languageCommandHandler;
        // Function in the printer language support file that determines
        // if the printer supports this particular printer language.
    USB_PRINTER_LANGUAGE_SUPPORTED  isLanguageSupported;
} USB_PRINTER_INTERFACE;


//-----------------------------------------------------------------------------
/* USB Printer Specific Interface Structure

This structure is used to explicitly specify what printer language to use for a
particular printer, and what print functions the printer supports.  It can be
used when a printer supports multiple languages with one language preferred over
the others.  It is required for printers that do not support the GET_DEVICE_ID
printer class request.  These printers do not report what printer languages
they support.  Typically, these printers also do not report Printer Class
support in their Interface Descriptors, and must be explicitly supported by
their VID and PID in the TPL.  This structure links the VID and PID of the
printer to the index in the usbPrinterClientLanguages[] array of
USB_PRINTER_INTERFACE structures in usb_config.c that contains the appropriate
printer language functions.
*/
typedef struct
{
    WORD                            vid;            // Printer vendor ID.
    WORD                            pid;            // Printer product ID.
    WORD                            languageIndex;  // Index into the usbPrinterClientLanguages[] array of USB_PRINTER_INTERFACE structures defined in usb_config.c.
    USB_PRINTER_FUNCTION_SUPPORT    support;        // Support flags that are set by this printer.
} USB_PRINTER_SPECIFIC_INTERFACE;


// *****************************************************************************
// *****************************************************************************
// Section: Host Stack Interface Functions
// *****************************************************************************
// *****************************************************************************

/***************************************************************************
  Function:
    BOOL USBHostPrinterInitialize ( BYTE address, DWORD flags, BYTE clientDriverID )

  Summary:
    This function is called by the USB Embedded Host layer when a printer
    attaches.

  Description:
    This routine is a call out from the USB Embedded Host layer to the USB
    printer client driver.  It is called when a "printer" device has been
    connected to the host.  Its purpose is to initialize and activate the USB
    Printer client driver.

  Preconditions:
    The device has been configured.

  Parameters:
    BYTE address    - Device's address on the bus
    DWORD flags     - Initialization flags
    BYTE clientDriverID - Client driver identification for device requests

  Return Values:
    TRUE    - Initialization was successful
    FALSE   - Initialization failed

  Remarks:
    Multiple client drivers may be used in a single application.  The USB
    Embedded Host layer will call the initialize routine required for the
    attached device.
  ***************************************************************************/

BOOL USBHostPrinterInitialize ( BYTE address, DWORD flags, BYTE clientDriverID );


/****************************************************************************
  Function:
    BOOL USBHostPrinterEventHandler ( BYTE address, USB_EVENT event,
                            void *data, DWORD size )

  Summary:
    This routine is called by the Host layer to notify the printer client of
    events that occur.

  Description:
    This routine is called by the Host layer to notify the printer client of
    events that occur.  If the event is recognized, it is handled and the
    routine returns TRUE.  Otherwise, it is ignored and the routine returns
    FALSE.

    This routine can notify the application with the following events:
        * EVENT_PRINTER_ATTACH
        * EVENT_PRINTER_DETACH
        * EVENT_PRINTER_TX_DONE
        * EVENT_PRINTER_RX_DONE
        * EVENT_PRINTER_REQUEST_DONE
        * EVENT_PRINTER_UNSUPPORTED

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

BOOL USBHostPrinterEventHandler ( BYTE address, USB_EVENT event, void *data, DWORD size );


// *****************************************************************************
// *****************************************************************************
// Section: Application Interface Function Prototypes and Macro Functions
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    BYTE USBHostPrinterCommand( BYTE deviceAddress, USB_PRINTER_COMMAND command,
                    USB_DATA_POINTER data, DWORD size, BYTE flags )

  Summary:
    This is the primary user interface function for the printer client
    driver.  It is used to issue all printer commands.

  Description:
    This is the primary user interface function for the printer client
    driver.  It is used to issue all printer commands.  Each generic printer
    command is translated to the appropriate command for the supported
    language and is enqueued for transfer to the printer.  Before calling
    this routine, it is recommended to call the function
    USBHostPrinterCommandReady() to determine if there is space available
    in the printer's output queue.

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

  Returns:
    See the USB_PRINTER_ERRORS enumeration.  Also, refer to the printer
    language command handler function, such as
    USBHostPrinterLanguagePostScript().

  Example:
    <code>
    if (USBHostPrinterCommandReady( address ))
    {
        USBHostPrinterCommand( address, USB_PRINTER_JOB_START, USB_NULL, 0, 0 );
    }
    </code>

  Remarks:
    When developing new commands, keep in mind that the function
    USBHostPrinterCommandReady() will be used before calling this function to
    see if there is space available in the output transfer queue.
    USBHostPrinterCommandReady() will routine TRUE if a single space is
    available in the output queue.  Therefore, each command can generate only
    one output transfer.
  ***************************************************************************/

BYTE USBHostPrinterCommand( BYTE deviceAddress, USB_PRINTER_COMMAND command,
                    USB_DATA_POINTER data, DWORD size, BYTE flags );


/****************************************************************************
  Function:
    BOOL USBHostPrinterCommandReady( BYTE deviceAddress )

  Description:
    This interface is used to check if the client driver has space available
    to enqueue another transfer.

  Preconditions:
    None

  Parameters:
    deviceAddress     - USB Address of the device

  Return Values:
    TRUE    - The printer client driver has room for at least one more
                transfer request, or the device is not attached.  The latter
                allows this routine to be called without generating an
                infinite loop if the device detaches.
    FALSE   - The transfer queue is full.

  Example:
    <code>
    if (USBHostPrinterCommandReady( address ))
    {
        USBHostPrinterCommand( address, USB_PRINTER_JOB_START, USB_NULL, 0, 0 );
    }
    </code>

  Remarks:
    Use the definitions USB_DATA_POINTER_RAM() and USB_DATA_POINTER_ROM()
    to cast data pointers.  For example:
    <code>
        USBHostPrinterCommand( address, USB_PRINTER_TEXT, USB_DATA_POINTER_RAM(buffer), strlen(buffer), 0 );
    </code>

    This routine will return TRUE if a single transfer can be enqueued.  Since
    this routine is the check to see if USBHostPrinterCommand() can be called,
    every command can generate at most one transfer.
  ***************************************************************************/

BOOL USBHostPrinterCommandReady( BYTE deviceAddress );


/****************************************************************************
  Function:
    BYTE USBHostPrinterCommandWithReadyWait( BYTE &returnCode,
                BYTE deviceAddress, USB_PRINTER_COMMAND command,
                USB_DATA_POINTER data, DWORD size, BYTE flags )

  Description:
    This function is intended to be a short-cut to perform blocking calls to
    USBHostPrinterCommand().  While there is no space available in the
    printer queue (USBHostPrinterCommandReady() returns FALSE), USBTasks() is
    called.  When space becomes available, USBHostPrinterCommand() is called.
    The return value from USBHostPrinterCommand() is returned in the
    returnCode parameter.

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

  Returns:
    See the USB_PRINTER_ERRORS enumeration.  Also, refer to the printer
    language command handler function, such as
    USBHostPrinterLanguagePostScript().

  Remarks:
    Use the definitions USB_DATA_POINTER_RAM() and USB_DATA_POINTER_ROM()
    to cast data pointers.  For example:
    <code>
        USBHostPrinterCommandWithReadyWait( &rc, address, USB_PRINTER_TEXT, USB_DATA_POINTER_RAM(buffer), strlen(buffer), 0 );
    </code>

    In the event that the device detaches during this routine,
    USBHostPrinterCommandReady() will return TRUE, and this function will
    return USB_PRINTER_UNKNOWN_DEVICE.
  ***************************************************************************/
#ifdef DEBUG_MODE
    #define USBHostPrinterCommandWithReadyWait( returnCode, deviceAddress, command, data, size, flags ) \
        {                                                                                               \
            while (!USBHostPrinterCommandReady( deviceAddress )) USBTasks();                            \
            *(returnCode) = USBHostPrinterCommand( deviceAddress, command, data, size, flags );         \
            if (*(returnCode))                                                                          \
            {                                                                                           \
                UART2PrintString( "APP: Printer Command Error " );                                      \
                UART2PutHex( *(returnCode) );                                                           \
                UART2PrintString( "\r\n" );                                                             \
            }                                                                                           \
            else                                                                                        \
            {                                                                                           \
                UART2PrintString( "APP: Printer Command successful\r\n" );                              \
            }                                                                                           \
        }
#else
    #define USBHostPrinterCommandWithReadyWait( returnCode, deviceAddress, command, data, size, flags ) \
        {                                                                                               \
            while (!USBHostPrinterCommandReady( deviceAddress )) USBTasks();                            \
            *(returnCode) = USBHostPrinterCommand( deviceAddress, command, data, size, flags );         \
        }
#endif

/****************************************************************************
  Function:
    BOOL USBHostPrinterDeviceDetached( BYTE deviceAddress )

  Description:
    This interface is used to check if the device has been detached from the
    bus.

  Preconditions:
    None

  Parameters:
    BYTE deviceAddress  - USB Address of the device.

  Return Values:
    TRUE    - The device has been detached, or an invalid deviceAddress is given.
    FALSE   - The device is attached

  Example:
    <code>
    if (USBHostPrinterDeviceDetached( deviceAddress ))
    {
        // Handle detach
    }
    </code>

  Remarks:
    The event EVENT_PRINTER_DETACH can also be used to detect a detach.
  ***************************************************************************/

BOOL USBHostPrinterDeviceDetached( BYTE deviceAddress );


/****************************************************************************
  Function:
    DWORD USBHostPrinterGetRxLength( BYTE deviceAddress )

  Description:
    This function retrieves the number of bytes copied to user's buffer by
    the most recent call to the USBHostPrinterRead() function.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    BYTE deviceAddress  - USB Address of the device

  Returns:
    Returns the number of bytes most recently received from the Printer
    device with address deviceAddress.

  Remarks:
    None
  ***************************************************************************/

DWORD USBHostPrinterGetRxLength( BYTE deviceAddress );


/****************************************************************************
  Function:
    BYTE USBHostPrinterGetStatus( BYTE deviceAddress, BYTE *status )

  Summary:
    This function issues the Printer class-specific Device Request to
    obtain the printer status.

  Description:
    This function issues the Printer class-specific Device Request to
    obtain the printer status.  The returned status should have the following
    format, per the USB specification.  Any deviation will be due to the
    specific printer implementation.
    * Bit 5 - Paper Empty; 1 = paper empty, 0 = paper not empty
    * Bit 4 - Select; 1 = selected, 0 = not selected
    * Bit 3 - Not Error; 1 = no error, 0 = error
    * All other bits are reserved.

    The *status parameter is not updated until the EVENT_PRINTER_REQUEST_DONE
    event is thrown.  Until that point the value of *status is unknown.

    The *status parameter will only be updated if this function returns
    USB_SUCCESS.  If this function returns with any other error code then the
    EVENT_PRINTER_REQUEST_DONE event will not be thrown and the status field
    will not be updated.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    deviceAddress   - USB Address of the device
    *status         - pointer to the returned status byte

  Returns:
    See the return values for the USBHostIssueDeviceRequest() function.

  Remarks:
    None
  ***************************************************************************/

BYTE USBHostPrinterGetStatus( BYTE deviceAddress, BYTE *status );


/****************************************************************************
  Function:
    DWORD USBHostPrinterPosition( WORD X, WORD Y )

  Summary:
    This function is used to simplify the call to the printer command
    USB_PRINTER_SET_POSITION by generating the value needed for the specified
    (X,Y) coordinate.

  Description:
    This function is used to simplify the call to the printer command
    USB_PRINTER_SET_POSITION by generating the value needed for the specified
    (X,Y) coordinate.  The USB_PRINTER_SET_POSITION command requires that the
    (X,Y) coordinate be passed in the (DWORD) size parameter of the
    USBHostPrinterCommand() function.  This function takes the specified
    coordinate and packs it in the DWORD as required.

  Preconditions:
    None

  Parameters:
    X   - X coordinate (horizontal)
    Y   - Y coordinate (vertical)

  Returns:
    DWORD value that can be used in the USBHostPrinterCommand() function
    call with the command USB_PRINTER_SET_POSITION.

  Example:
    <code>
    USBHostPrinterCommand( printer, USB_PRINTER_SET_POSITION, USB_NULL,
                    USBHostPrinterPosition( 100, 100 ), 0 );
    </code>

  Remarks:
    None
  ***************************************************************************/

#define USBHostPrinterPosition( X, Y )  (((DWORD)(X) << 16) | ((DWORD)(Y) & 0xFFFF))


/****************************************************************************
  Function:
    DWORD USBHostPrinterPositionRelative( SHORT dX, SHORT dY )

  Summary:
    This function is used to simplify the call to some of the printer
    graphics commands by generating the value needed for the specified
    change in X and change in Y coordinates.

  Description:
    This function is used to simplify the call to some of the printer
    graphics commands by generating the value needed for the specified
    change in X and change in Y coordinates. The
    USB_PRINTER_GRAPHICS_MOVE_RELATIVE and the
    USB_PRINTER_GRAPHICS_LINE_TO_RELATIVE commands requires that the
    change in the (X,Y) coordinates be passed in the (DWORD) size parameter
    of the USBHostPrinterCommand() function.  This function takes the
    specified coordinate changes and packs them in the DWORD as required.

  Preconditions:
    None

  Parameters:
    dX   - Change in the X coordinate (horizontal)
    dY   - Change in the Y coordinate (vertical)

  Returns:
    DWORD value that can be used in the USBHostPrinterCommand() function
    call with the commands USB_PRINTER_GRAPHICS_MOVE_RELATIVE and
    USB_PRINTER_GRAPHICS_LINE_TO_RELATIVE.

  Example:
    <code>
    USBHostPrinterCommand( printer, USB_PRINTER_GRAPHICS_LINE_TO_RELATIVE, USB_NULL,
                    USBHostPrinterPositionRelative( 0, -100 ), 0 );
    </code>

  Remarks:
    None
  ***************************************************************************/

#define USBHostPrinterPositionRelative( dX, dY )  (((DWORD)(dX) << 16) | ((DWORD)(dY) & 0xFFFF))


/****************************************************************************
  Function:
    BYTE USBHostPrinterRead( BYTE deviceAddress, BYTE *buffer, DWORD length,
                BYTE transferFlags )

  Description:
    Use this routine to receive from the device and store it into memory.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    deviceAddress  - USB Address of the device.
    buffer         - Pointer to the data buffer
    length         - Number of bytes to be transferred
    transferFlags  - Flags for how to perform the operation

  Return Values:
    USB_SUCCESS         - The Read was started successfully
    (USB error code)    - The Read was not started.  See USBHostRead() for
                            a list of errors.

  Example:
    <code>
    if (!USBHostPrinterRxIsBusy( deviceAddress ))
    {
        USBHostPrinterRead( deviceAddress, &buffer, sizeof(buffer), 0 );
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BYTE USBHostPrinterRead( BYTE deviceAddress, void *buffer, DWORD length,
            BYTE transferFlags );


/****************************************************************************
  Function:
    BYTE USBHostPrinterReset( BYTE deviceAddress )

  Description:
    This function issues the Printer class-specific Device Request to
    perform a soft reset.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    BYTE deviceAddress  - USB Address of the device

  Returns:
    See the return values for the USBHostIssueDeviceRequest() function.

  Remarks:
    Not all printers support this command.
  ***************************************************************************/

BYTE USBHostPrinterReset( BYTE deviceAddress );

/****************************************************************************
  Function:
    BOOL USBHostPrinterRxIsBusy( BYTE deviceAddress )

  Summary:
    This interface is used to check if the client driver is currently busy
    receiving data from the device.

  Description:
    This interface is used to check if the client driver is currently busy
    receiving data from the device.

  Preconditions:
    None

  Parameters:
    deviceAddress     - USB Address of the device

  Return Values:
    TRUE    - The device is receiving data or an invalid deviceAddress is
                given.
    FALSE   - The device is not receiving data

  Example:
    <code>
    if (!USBHostPrinterRxIsBusy( deviceAddress ))
    {
        USBHostPrinterRead( deviceAddress, &buffer, sizeof( buffer ) );
    }
    </code>

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostPrinterRxIsBusy( BYTE deviceAddress );


/****************************************************************************
  Function:
    BYTE USBHostPrinterWrite( BYTE deviceAddress, void *buffer, DWORD length,
                BYTE transferFlags )

  Description:
    Use this routine to transmit data from memory to the device.  This
    routine will not usually be called by the application directly.  The
    application will use the USBHostPrinterCommand() function, which will
    call the appropriate printer language support function, which will
    utilize this routine.

  Preconditions:
    The device must be connected and enumerated.

  Parameters:
    BYTE deviceAddress  - USB Address of the device.
    void *buffer        - Pointer to the data buffer
    DWORD length        - Number of bytes to be transferred
    BYTE transferFlags  - Flags for how to perform the operation

  Return Values:
    USB_SUCCESS                 - The Write was started successfully.
    USB_PRINTER_UNKNOWN_DEVICE  - Device not found or has not been initialized.
    USB_PRINTER_BUSY            - The printer's output queue is full.
    (USB error code)            - The Write was not started.  See USBHostWrite() for
                                    a list of errors.

  Remarks:
    None
  ***************************************************************************/

BYTE USBHostPrinterWrite( BYTE deviceAddress, void *buffer, DWORD length,
            BYTE flags);


/****************************************************************************
  Function:
    BOOL USBHostPrinterWriteComplete( BYTE deviceAddress )

  Description:
    This interface is used to check if the client driver is currently
    transmitting data to the printer, or if it is between transfers but still
    has transfers queued.

  Preconditions:
    None

  Parameters:
    deviceAddress     - USB Address of the device

  Return Values:
    TRUE    - The device is done transmitting data or an invalid deviceAddress
                is given.
    FALSE   - The device is transmitting data or has a transfer in the queue.

  Remarks:
    None
  ***************************************************************************/

BOOL USBHostPrinterWriteComplete( BYTE deviceAddress );



/*************************************************************************
 * EOF usb_client_generic.h
 */

#endif
