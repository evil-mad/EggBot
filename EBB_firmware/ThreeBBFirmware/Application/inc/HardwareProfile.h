/*********************************************************************
 *
 *                BBFirmware
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Based on original files by Microchip Inc. in MAL USB example.
 *
 * Software License Agreement
 *
 * Copyright (c) 2014-2020, Brian Schmalz of Schmalz Haus LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials
 * provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of
 * its contributors may be used to endorse or promote products
 * derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H


  /** Board definition ***********************************************/
  // These definitions will tell the main() function which board is
  //  currently selected.  This will allow the application to add
  //  the correct configuration bits as wells use the correct
  //  initialization functions for the board.  These definitions are only
  //  required in the stack provided demos.  They are not required in
  //  final application design.
///  #define CLOCK_FREQ        48000000

  // How many stepper motors does this board support? (3BB has 3)
  #define NUMBER_OF_STEPPERS      3

  // Maximum size of motion command queue
  #define COMMAND_QUEUE_LENGTH     32

  // The ADC channel number for the SCALED_V+ input on 3BB
  // Allows us to see what V+ motor voltage has been apPlied to the board
///  #define SCALED_V_ADC_CHAN       11

  // ADC counts (out of 1024) that represent 6.5V on SCALED_V+ ADC input
  // since 5.5V is the minimum our drivers need in order to work and we
  // want a bit of headroom (so that CPU detects a V+ glitch before drivers do)
///  #define V_PLUS_VOLTAGE_POWERED   435

  /** L E D ***********************************************************/
  /* On EBB v13 and above, LED1 (USB) = RD3, LED2 (USR) = RD2, SW = RA7 */
///  #define mInitAllLEDs()      LATDbits.LATD3 = 0; LATDbits.LATD2 = 0; TRISDbits.TRISD3 = 0; TRISDbits.TRISD2 = 0;
///  #define mLED_1              LATDbits.LATD3
///  #define mLED_2              LATDbits.LATD2

  /** S W I T C H *****************************************************/
///  #define mInitSwitch()       TRISAbits.TRISA6 = INPUT_PIN;
///  #define swProgram           PORTAbits.RA6

  /** P E N   U P  D O W N *******************************************/
///  #define PenUpDownIO         LATBbits.LATB4
///  #define PEN_UP_DOWN_RPN     7
///  #define PenUpDownIO_TRIS    TRISBbits.TRISB4

  /** D R I V E R   E N A B L E **************************************/
///  #define EnableIO            LATEbits.LATE2
///  #define EnableIO_TRIS       TRISEbits.TRISE2

  /** S C A L E D   V+ ***********************************************/
///  #define ScalaedVPlusIO      PORTEbits.RE0
///  #define ScaledVPlusIO_TRIS  TRISEbits.TRISE0

  /** S T E P  A N D  D I R ******************************************/
///  #define Step1IO             LATDbits.LATD6
///  #define Step1IO_TRIS        TRISDbits.TRISD6
///  #define Dir1IO              LATDbits.LATD7
///  #define Dir1IO_TRIS         TRISDbits.TRISD7
///  #define Step2IO             LATAbits.LATA2
///  #define Step2IO_TRIS        TRISAbits.TRISA2
///  #define Dir2IO              LATAbits.LATA3
///  #define Dir2IO_TRIS         TRISAbits.TRISA3
///  #define Step3IO             LATDbits.LATD4
///  #define Step3IO_TRIS        TRISDbits.TRISD4
///  #define Dir3IO              LATDbits.LATD5
///  #define Dir3IO_TRIS         TRISDbits.TRISD5

  /** G E N E R I C ***************************************************/

///  #define mLED_USB_Toggle()   mLED_1 = !mLED_1;

///  #define mLED_1_On()         mLED_1 = 1;
///  #define mLED_2_On()         mLED_2 = 1;

///  #define mLED_1_Off()        mLED_1 = 0;
///  #define mLED_2_Off()        mLED_2 = 0;

///  #define mLED_1_Toggle()     mLED_1 = !mLED_1;
///  #define mLED_2_Toggle()     mLED_2 = !mLED_2;

///  #define mLED_Both_Off()     {mLED_1_Off(); mLED_2_Off();}
///  #define mLED_Both_On()      {mLED_1_On(); mLED_2_On();}
///  #define mLED_Only_1_On()    {mLED_1_On(); mLED_2_Off();}
///  #define mLED_Only_2_On()    {mLED_1_Off(); mLED_2_On();}

#endif  //HARDWARE_PROFILE_H
