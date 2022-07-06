/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        commands.h
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Software License Agreement
 *
 * Copyright (c) 2020-2021, Brian Schmalz of Schmalz Haus LLC
 * All rights reserved.
 * Based on EiBotBoard (EBB) Firmware, written by Brian Schmalz of
 *   Schmalz Haus LLC
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMANDS_H__
#define __COMMANDS_H__


extern uint8_t gPulsesOn;
// For Pulse Mode, how long should each pulse be on for in ms?
extern uint16_t gPulseLen[4];
// For Pulse Mode, how many ms between rising edges of pulses?
extern uint16_t gPulseRate[4];
// For Pulse Mode, counters keeping track of where we are
extern uint16_t gPulseCounters[4];

void commands_SCCommand(void);      // SC for Stepper/Servo Configure
//void parseRSCommand(void);     // R for resetting UBW
//void parseCBCommand(void);     // C for configuring I/O and analog pins
//void parseODCommand(void);     // O for output digital to pins
//void parseIDCommand(void);     // I for input digital from pins
void commands_VRCommand(void);      // VR for printing version
void commands_VCommand(void);      // V for printing version
//void parsePICommand(void);     // PI for reading a single pin
//void parsePOCommand(void);     // PO for setting a single pin state
//void parsePDCommand(void);     // PD for setting a pin's direction
void commands_MRCommand(void);     // MR for Memory Read
void commands_MWCommand(void);     // MW for Memory Write
void commands_CUCommand(void);     // CU configures UBW (system wide parameters)
//void parsePGCommand(void);     // PG Pulse Go
//void parsePCCommand(void);     // PC Pulse Configure
void commands_BLCommand(void);     // BL Boot Load command
//void parseT1Command(void);     // T1 Test command for input parameters
//void parseT2Command(void);     // T1 Test command for input parameters
//void parseMRCommand(void);     // MR Motors Run command
//void parseRBCommand(void);     // RB ReBoot command
void commands_QCCommand(void);    // QC command for query voltages
void commands_QBCommand(void);    // QB command for Query Button


/// THESE GO AWAY ONCE WE DON'T NEED "LEGACY" MODE ANYMORE
void commands_QLCommand(void);     // QL For Query Layer
void commands_SLCommand(void);     // MW for Set Layer


#endif	/* COMMANDS_H */
