/*********************************************************************
 *
 *                UBW Firmware
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
 * Copyright (c) 2014, Brian Schmalz of Schmalz Haus LLC
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

#define DEMO_BOARD

#if defined(UBW)
	#include "HardwareProfile_UBW.h"
#elif defined(BOARD_EBB_V10)
	#include "HardwareProfile_EBB_V10.h"
#elif defined(BOARD_EBB_V11)
	#include "HardwareProfile_EBB_V11.h"
#elif defined(BOARD_EBB_V12)
	#include "HardwareProfile_EBB_V12.h"
#elif defined(BOARD_EBB_V13_AND_ABOVE)
	#include "HardwareProfile_EBB_V13_and_above.h"
#endif

#if !defined(DEMO_BOARD)
    #if defined(__C32__)
        #if defined(__32MX460F512L__)
            #if defined(PIC32MX460F512L_PIM)
                #include "HardwareProfile - PIC32MX460F512L PIM.h"
            #elif defined(PIC32_USB_STARTER_KIT)
                #include "HardwareProfile - PIC32 USB Starter Kit.h"
            #endif
        #elif defined(__32MX795F512L__)
            #if defined(PIC32MX795F512L_PIM)
                #include "HardwareProfile - PIC32MX795F512L PIM.h"
            #elif defined(PIC32_USB_STARTER_KIT)
                //PIC32 USB Starter Kit II
                #include "HardwareProfile - PIC32 USB Starter Kit.h"
            #endif
        #endif
    #endif

    #if defined(__C30__)
        #if defined(__PIC24FJ256GB110__)
            #include "HardwareProfile - PIC24FJ256GB110 PIM.h"
        #elif defined(__PIC24FJ256GB106__)
            #include "HardwareProfile - PIC24F Starter Kit.h"
        #elif defined(__PIC24FJ64GB004__)
            #include "HardwareProfile - PIC24FJ64GB004 PIM.h"
        #elif defined(__PIC24FJ256DA210__)
            #include "HardwareProfile - PIC24FJ256DA210 Development Board.h"
        #endif
    #endif

    #if defined(__18CXX)
        #if defined(__18F4550)
            #include "HardwareProfile - PICDEM FSUSB.h"
        #elif defined(__18F87J50)
            #include "HardwareProfile - PIC18F87J50 PIM.h"
        #elif defined(__18F14K50)
            #include "HardwareProfile - Low Pin Count USB Development Kit.h"
        #elif defined(__18F46J50)
            #if defined(PIC18F_STARTER_KIT_1)
                #include "HardwareProfile - PIC18F Starter Kit 1.h"
            #else
                #include "HardwareProfile - PIC18F46J50 PIM.h"
            #endif
        #endif
    #endif
#endif

#if !defined(DEMO_BOARD)
    #error "Demo board not defined.  Either define DEMO_BOARD for a custom board or select the correct processor for the demo board."
#endif

#endif  //HARDWARE_PROFILE_H
