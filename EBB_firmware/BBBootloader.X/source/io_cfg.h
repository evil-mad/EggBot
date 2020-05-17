/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.2
 *
 *********************************************************************
 * FileName:        io_cfg.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 3.11+
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * File Version  Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 1.0			 04/09/2008	Started from MCHPFSUSB v1.3 HID Mouse
 *							demo project.  Commented out items that
 *							are not particularly useful for the
 *							bootloader.
 ********************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H

/** I N C L U D E S *************************************************/
#include "usbcfg.h"

/** T R I S *********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0

//Uncomment below if using the YOUR_BOARD hardware platform
#if defined(EBB_V10)
/** U S B ***********************************************************/
#if defined(USE_USB_BUS_SENSE_IO)
#define usb_bus_sense       PORTDbits.RD3
#else
#define usb_bus_sense       1
#endif

#define self_power          0

///** L E D ***********************************************************/
// USB LED = RD0
// USR LED = RD1
#define mInitAllLEDs()      LATDbits.LATD0 = 0; LATDbits.LATD1 = 0; TRISDbits.TRISD0 = 0; TRISDbits.TRISD1 = 0;

#define mLED_1              LATDbits.LATD0
#define mLED_2              LATDbits.LATD1

#define mLED_1_On()         mLED_1 = 1;
#define mLED_2_On()         mLED_2 = 1;

#define mLED_1_Off()        mLED_1 = 0;
#define mLED_2_Off()        mLED_2 = 0;

#define mLED_1_Toggle()     mLED_1 = !mLED_1;
#define mLED_2_Toggle()     mLED_2 = !mLED_2;
//
///** S W I T C H *****************************************************/
// PRG switch = RD2
#define mInitAllSwitches()  TRISDbits.TRISD2=1;
#define mInitSwitch2()      TRISDbits.TRISD2=1;

#define sw2                 PORTDbits.RD2

#elif defined(EBB_V11)
/** U S B ***********************************************************/
//#define tris_usb_bus_sense  TRISBbits.TRISB5    // Input

#if defined(USE_USB_BUS_SENSE_IO)
#define usb_bus_sense       PORTCbits.RC7
#else
#define usb_bus_sense       1
#endif

#define self_power          0

///** L E D ***********************************************************/
// USB LED = RD3
// USR LED = RD2
#define mInitAllLEDs()      LATDbits.LATD3 = 0; LATDbits.LATD2 = 0; TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;

#define mLED_1              LATDbits.LATD3
#define mLED_2              LATDbits.LATD2

#define mLED_1_On()         mLED_1 = 1;
#define mLED_2_On()         mLED_2 = 1;

#define mLED_1_Off()        mLED_1 = 0;
#define mLED_2_Off()        mLED_2 = 0;

#define mLED_1_Toggle()     mLED_1 = !mLED_1;
#define mLED_2_Toggle()     mLED_2 = !mLED_2;
//
///** S W I T C H *****************************************************/
// PRG switch = RA7
#define mInitAllSwitches()  TRISAbits.TRISA7=1;
#define mInitSwitch2()      TRISAbits.TRISA7=1;

#define sw2                 PORTAbits.RA7

#elif defined(THREEBEEBEE_V10)
/** U S B ***********************************************************/
//#define tris_usb_bus_sense  TRISAbits.TRISA7    // Input

#if defined(USE_USB_BUS_SENSE_IO)
#define usb_bus_sense       PORTAbits.RA7
#else
#define usb_bus_sense       1
#endif

#define self_power          0

///** L E D ***********************************************************/
// USB (GREEN) LED = RD3
// USR (RED) LED = RD2
#define mInitAllLEDs()      LATDbits.LATD3 = 0; LATDbits.LATD2 = 0; TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;

#define mLED_1              LATDbits.LATD3
#define mLED_2              LATDbits.LATD2

#define mLED_1_On()         mLED_1 = 1;
#define mLED_2_On()         mLED_2 = 1;

#define mLED_1_Off()        mLED_1 = 0;
#define mLED_2_Off()        mLED_2 = 0;

#define mLED_1_Toggle()     mLED_1 = !mLED_1;
#define mLED_2_Toggle()     mLED_2 = !mLED_2;
//
///** S W I T C H *****************************************************/
// PRG switch = RA7
#define mInitAllSwitches()  TRISAbits.TRISA6=1;
#define mInitSwitch2()      TRISAbits.TRISA6=1;

#define sw2                 PORTAbits.RA6


/********************************************************************/
/********************************************************************/
/********************************************************************/

#else
    #error Not a supported board (yet), add I/O pin mapping in __FILE__, line __LINE__
#endif

#endif //IO_CFG_H
