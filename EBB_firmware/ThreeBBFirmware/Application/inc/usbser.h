/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        usbser.h
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
#ifndef __USBSER_H__
#define	__USBSER_H__

#define kTX_BUF_SIZE          64        // In bytes
#define kRX_BUF_SIZE          255       // In bytes
#define kRX_COMMAND_BUF_SIZE  64        // In bytes (must be 64)

#define kCR                     0x0D
#define kLF                     0x0A


extern unsigned char g_RX_buf[kRX_BUF_SIZE];
extern unsigned char g_TX_buf_out;
extern unsigned char g_RX_buf_out;
extern unsigned char g_RX_buf_in;
extern const char st_OK[];
extern const char st_LFCR[];

void usbser_Init(void);
void parsePacket(void);       // Take a full packet and dispatch it to the right function
signed char extract_digit(unsigned long * acc, unsigned char digits); // Pull a character out of the packet
void ProcessIO(void);
int _user_putc(char c);
void check_and_send_TX_data(void);

#endif	/* USBSER_H */
