/******************************************************************************

  USB POS Printer Definition File - Seiko DPU-V445

  Summary:
    This is a definition file for the Seiko DPU-V445 Point-of-sale USB
    Printer.

  Description:
    This is a definition file for the Seiko DPU-V445 Point-of-sale USB
    Printer.  This definition file may be used for other compatible printers,
    including the following printers.

    Manufacturer    Model       Tested
    ------------    -----       ------
    Seiko           DPU-V445     Yes
    Seiko           DPU-S445     No (based on spec review only)

    Various POS printers that support the ESC/POS printer language have
    deviations or limitations from the language specification.  This file
    allows the ESC/POS printer language file to configure itself
    properly for the requirements and/or limitations of the particular printer.
    It will also allow the printer language support file to return an error
    (USB_PRINTER_UNKNOWN_COMMAND) if an unsupported command is issued.

    Some deviations are minor, and may have no effect on the printed output.
    Others, however, can result in printing failures if the configuration is
    incorrect.  For best results:
        * Determine either a single target printer or a set of compatible
            target printers when designing the application.
        * Specify those printers explicitly in the TPL.
        * Specify explicit printer language support for those printers.
        * Test the application on each specified printer.

  Remarks:
    This file should be specified as the "POS Printer Header File" on the
    "Printer" tab of the USB Configuration Tool (USBConfig.exe or MPLAB VDI)
    when ESC/POS support is enabled.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/******************************************************************************

* FileName:        usb_printer_pos_seiko_dpu_v445.h
* Dependencies:    None
* Processor:       PIC24/dsPIC30/dsPIC33/PIC32MX
* Compiler:        C30 v3.10b/C32 v0.00.18
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

#ifndef USB_PRINTER_POS_SEIKO_DPU_V445
#define USB_PRINTER_POS_SEIKO_DPU_V445

//------------------------------------------------------------------------------
// Bar Code Support Configuration

// If the printer supports bar code printing, uncomment the following
// definition.  This will provide support for UPC-A, UPC-E, JAN/EAN13, JAN/EAN8,
// CODE39, ITF, and CODABAR bar codes using Format A of the "Print Bar Code"
// command.  If only only bar code format is specified, it is Format A.
#define USB_PRINTER_POS_BARCODE_SUPPORT

// If the printer supports both Format A and Format B of the "Print Bar Code"
// command, also uncomment this definition.  In addition to the bar codes listed
// above, CODE93 and CODE128 bar codes will be supported.
//#define USE_PRINTER_POS_EXTENDED_BARCODE_FORMAT


//------------------------------------------------------------------------------
// Bitmap Image Support Configuration

// If the printer supports 24-dot vertical density image printing, uncomment the
// following definition.
#define USB_PRINTER_POS_24_DOT_IMAGE_SUPPORT

// If the printer supports 36-dot vertical density image printing, uncomment the
// following definition.  This support is not common.  Note that 36-dot vertical
// density image printing itself is not supported.  However, printers with this
// capability use different parameter values for other image printing, and must
// be configured appropriately.  If this is not configured correctly, 24-bit
// vertical density images will not print correctly.
//#define USB_PRINTER_POS_36_DOT_IMAGE_SUPPORT

// Set this label to the line spacing required between bit image lines.  Often,
// the value 0 (zero) can be used.  If the printer prints all image lines on
// top of each other, set this value to the height of the printed image line.
#define USB_PRINTER_POS_IMAGE_LINE_SPACING              0


//------------------------------------------------------------------------------
// Text Printing Support Configuration

// If the printer supports reverse text (white letters on a black background)
// printing, uncomment the following definition.
#define USB_PRINTER_POS_REVERSE_TEXT_SUPPORT


//------------------------------------------------------------------------------
// Color Support Configuration

// If the printer supports two color printing, uncomment the following line.
// This is not common.
//#define USB_PRINTER_POS_TWO_COLOR_SUPPORT


//------------------------------------------------------------------------------
// Mechanism Support Configuration

// If the printer has an automatic cutter, uncomment the following line.
//#define USB_PRINTER_POS_CUTTER_SUPPORT

#endif

