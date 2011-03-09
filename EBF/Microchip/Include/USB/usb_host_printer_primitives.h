/*****************************************************************************

    Microchip USB Host Printer Client Driver, Graphics Library Interfaces Layer

  Summary:
    This file contains the routines needed to utilize the Microchip Graphics
    Library functions to create graphic images on a USB printer.

  Description:
    This file contains the routines needed to utilize the Microchip Graphics
    Library functions to create graphic images on a USB printer.

    The label USE_GRAPHICS_LIBRARY_PRINTER_INTERFACE must be defining in the
    USB configuration header file usb_config.h to utilize these functions.

  Remarks:
    None

******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

* FileName:        usb_host_printer_primitives.c
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

#ifndef _USB_HOST_PRINTER_PRIMITIVES_H
#define _USB_HOST_PRINTER_PRIMITIVES_H


#include "GenericTypedefs.h"
#include "USB/usb.h"
#include "USB/usb_host_printer.h"


// *****************************************************************************
// *****************************************************************************
// Section: USB Data Structures
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Print Screen Information

This structure is designed for use when the USB Embedded Host Printer support
is integrated with the graphics library.  The structure contains the information
needed to print a portion of the graphics screen as a bitmapped graphic image.
*/
typedef struct
{
    WORD    xL;             // X-axis position of the left side of the screen image.
    WORD    yT;             // Y-axis position of the top of the screen image.
    WORD    xR;             // X-axis position of the right side of the screen image.
    WORD    yB;             // Y-axis position of the bottom of the screen image.
    WORD    colorBlack;     // Screen color that should be printed as black.
    USB_PRINTER_FUNCTION_SUPPORT    printerType;    // The capabilities of the
                                                    // current printer, so we know
                                                    // what structure members are
                                                    // valid.
    USB_PRINTER_IMAGE_INFO  printerInfo;    // Store all the info needed to print
                                            // the image.  The width and height
                                            // parameters will be determined by
                                            // the screen coordinates specified
                                            // above.  The application must
                                            // provide the other values.
} USB_PRINT_SCREEN_INFO;


// *****************************************************************************
// *****************************************************************************
// Section: Subroutines
// *****************************************************************************
// *****************************************************************************

/****************************************************************************
  Function:
    SHORT PrintScreen( BYTE address, USB_PRINT_SCREEN_INFO *printScreenInfo )

  Summary:
    This routine will extract the image that is currently on the specified
    portion of the graphics display, and print it at the specified location.

  Description:
    This routine is intended for use in an application that is using the
    Graphics Library to control a graphics display.  This routine will
    extract the image that is currently on the specified portion of
    the graphics display, and print it at the specified location.
    Since the display may be in color and the printer can print only black
    and white, the pixel color to interpret as black must be specified in the
    USB_PRINT_SCREEN_INFO structure.

    The function can be compiled as either a blocking function or a
    non-blocking function.  When compiled as a blocking function, the routine
    will wait to enqueue all printer instructions.  If an error occurs, then
    this function will return the error.  If all printer instructions are
    enqueued successfully, the function will return -1.  When compiled as a
    non-blocking function, this function will return 0 if the operation is
    proceeding correctly but has not yet completed.  The application must
    continue to call this function, with the same parameters, until a
    non-zero value is returned.  A value of -1 indicates that all printer
    instructions have been enqueued successfully.  Any other value is an error
    code, and the state machine will be set back to the beginning state.

  Precondition:
    None

  Parameters:
    BYTE address                            - USB address of the printer.
    USB_PRINT_SCREEN_INFO *printScreenInfo  - Information about the screen
                            area to print, how to interpret the screen image,
                            and how and where to print the image.  Note that
                            the width and height members of the structure do
                            not need to be filled in by the application.

  Return Values:
    0       -   Non-blocking configuration only.  Image output is not yet
                complete, but is proceeding normally.
    (-1)    -   Image output was completed successfully.
    other   -   Printing was aborted due to an error.  See the return values
                for USBHostPrinterCommand().  Note that the return code
                USB_PRINTER_SUCCESS will not be returned.  Instead, (-1) will
                be returned upon successful completion.

  Remarks:
    None
  ***************************************************************************/

SHORT PrintScreen( BYTE address, USB_PRINT_SCREEN_INFO *printScreenInfo );




#endif

