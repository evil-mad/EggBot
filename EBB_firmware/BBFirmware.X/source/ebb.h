/*********************************************************************
 *
 *                EiBotBoard Firmware
 *
 *********************************************************************
 * FileName:        ebb.h
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

#ifndef EBB_H
#define EBB_H


#include <GenericTypeDefs.h>

// These are used for Enable<X>IO to control the enable lines for the driver
#define ENABLE_MOTOR        0
#define DISABLE_MOTOR       1



/// TODO: Only for EBB, not 3BB
typedef enum
{
  PIC_CONTROLS_DRIVERS = 0,
  PIC_CONTROLS_EXTERNAL,
  EXTERNAL_CONTROLS_DRIVERS
} DriverConfigurationType;

extern DriverConfigurationType DriverConfiguration;

extern unsigned int DemoModeActive;

extern unsigned int comd_counter;
extern unsigned char QC_ms_timer;
extern BOOL gLimitChecks;
extern UINT StoredEngraverPower;
extern BOOL ButtonPushed;
extern BOOL UseAltPause;
extern unsigned long NodeCount;
extern char Layer;

// Default to on, comes out on pin RB4 for EBB v1.3 and above
extern BOOL gUseSolenoid;
void parse_SC_packet(void);
void parse_TP_packet(void);
void parse_QP_packet(void);
void parse_SN_packet(void);
void parse_QN_packet(void);
void parse_NI_packet(void);
void parse_ND_packet(void);
void parse_SL_packet(void);
void parse_QL_packet(void);
void parse_QB_packet(void);
void parse_QC_packet(void);
void parse_QG_packet(void);
void parse_SE_packet(void);
void parse_RM_packet(void);
#endif