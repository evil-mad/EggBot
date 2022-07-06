/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        TMC2209.h
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
#ifndef __TMC2209_H__
#define __TMC2209_H__

/* Register addresses */
#define GCONF         0x00  //   0
#define GSTAT         0x01  //   1
#define IFCNT         0x02  //   2
#define SLAVECONF     0x03  //   3
#define OTP_PROG      0x04  //   4
#define OTP_READ      0x05  //   5
#define IOIN          0x06  //   6
#define FACTORY_CONF  0x07  //   7

#define IHOLD_IRUN    0x10  //  16
#define TPOWERDOWN    0x11  //  17
#define TSTEP         0x12  //  18
#define TPWMTHRS      0x13  //  19
#define VACTUAL       0x22  //  34

#define TCOOLTHRS     0x14  //  20
#define SGTHRS        0x40  //  64
#define SG_RESULT     0x41  //  65
#define COOLCONF      0x42  //  66

#define MSCNT         0x6A  // 106
#define MSCURACT      0x6B  // 107

#define CHOPCONF      0x6C  // 108
#define DRV_STATUS    0x6F  // 111
#define PWMCONF       0x70  // 112
#define PWM_SCALE     0x71  // 113
#define PWM_AUTO      0x72  // 114

#endif	/* TMC2209_H */

