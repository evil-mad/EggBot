/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        debug.h
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
#ifndef __INC_DEBUG_H__
#define __INC_DEBUG_H__

/* Define the I/O macros for debug pins (G0 to G12)
 * G0 to G11 are available on the J3 GPIO header. G12 is on a Test Point
 */
#define DEBUG_G0_SET()     G0_GPIO_Port->BSRR =  (uint32_t)G0_Pin
#define DEBUG_G0_RESET()   G0_GPIO_Port->BRR  =  (uint32_t)G0_Pin
#define DEBUG_G1_SET()     G1_GPIO_Port->BSRR =  (uint32_t)G1_Pin
#define DEBUG_G1_RESET()   G1_GPIO_Port->BRR  =  (uint32_t)G1_Pin
#define DEBUG_G2_SET()     G2_GPIO_Port->BSRR =  (uint32_t)G2_Pin
#define DEBUG_G2_RESET()   G2_GPIO_Port->BRR  =  (uint32_t)G2_Pin
#define DEBUG_G3_SET()     G3_GPIO_Port->BSRR =  (uint32_t)G3_Pin
#define DEBUG_G3_RESET()   G3_GPIO_Port->BRR  =  (uint32_t)G3_Pin
#define DEBUG_G4_SET()     G4_GPIO_Port->BSRR =  (uint32_t)G4_Pin
#define DEBUG_G4_RESET()   G4_GPIO_Port->BRR  =  (uint32_t)G4_Pin
#define DEBUG_G5_SET()     G5_GPIO_Port->BSRR =  (uint32_t)G5_Pin
#define DEBUG_G5_RESET()   G5_GPIO_Port->BRR  =  (uint32_t)G5_Pin
#define DEBUG_G6_SET()     G6_GPIO_Port->BSRR =  (uint32_t)G6_Pin
#define DEBUG_G6_RESET()   G6_GPIO_Port->BRR  =  (uint32_t)G6_Pin
#define DEBUG_G7_SET()     G7_GPIO_Port->BSRR =  (uint32_t)G7_Pin
#define DEBUG_G7_RESET()   G7_GPIO_Port->BRR  =  (uint32_t)G7_Pin
#define DEBUG_G8_SET()     G8_GPIO_Port->BSRR =  (uint32_t)G8_Pin
#define DEBUG_G8_RESET()   G8_GPIO_Port->BRR  =  (uint32_t)G8_Pin
#define DEBUG_G9_SET()     G9_GPIO_Port->BSRR =  (uint32_t)G9_Pin
#define DEBUG_G9_RESET()   G9_GPIO_Port->BRR  =  (uint32_t)G9_Pin
#define DEBUG_G10_SET()    G10_GPIO_Port->BSRR = (uint32_t)G10_Pin
#define DEBUG_G10_RESET()  G10_GPIO_Port->BRR  = (uint32_t)G10_Pin
#define DEBUG_G11_SET()    G11_GPIO_Port->BSRR = (uint32_t)G11_Pin
#define DEBUG_G11_RESET()  G11_GPIO_Port->BRR  = (uint32_t)G11_Pin
#define DEBUG_G12_SET()    G12_GPIO_Port->BSRR = (uint32_t)G12_Pin
#define DEBUG_G12_RESET()  G12_GPIO_Port->BRR  = (uint32_t)G12_Pin


void DebugInit(void);
void Debug_SWOInit(uint32_t portMask, uint32_t cpuCoreFreqHz, uint32_t baudrate);

#endif /* INC_DEBUG_H_ */
