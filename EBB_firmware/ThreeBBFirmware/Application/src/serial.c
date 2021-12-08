/*********************************************************************
 *
 *                3BB Firmware
 *
 *********************************************************************
 * FileName:        serial.c
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

/*
 * This module implements serial communication with the three TMC2209 stepper
 * driver chips (for configuration and status).
 */

/************** INCLUDES ******************************************************/

#include <stdio.h>
#include <stdbool.h>
#include "serial.h"
#include "utility.h"
#include "usart.h"
#include "TMC2209.h"
#include "HardwareProfile.h"
#include "servo.h"
#include "parse.h"
#include "isr.h"
#include "debug.h"

/************** PRIVATE TYPEDEFS **********************************************/

/************** PRIVATE DEFINES ***********************************************/

// The number of 32-bit init values to send to the drivers over serial
#define MAX_DRIVER_INIT_VALUES      4

/* OTP (one Time Programmable) bit setup
 * We need the drivers to be disabled in OTP memory so that when they are reset
 * or boot up for the first time the motors are not consuming any current nor
 * are they able to step. Then, once the CPU detects that the drivers are fresh
 * from a reset, it can program them with the proper initialization values and
 * from then on its OK to start stepping.
 * 
 * The en_SpreadCycle bit of GCONF needs to be cleared from OTP to use 
 * StealthChop PWM. This is the factory default (OTP bits come cleared from the
 * factory) so we don't have to change it.
 * 
 * OTP byte 1 bits 3,2,1 and 0 are the OTP_CHOPCONF3...0 bits and need to be
 */

// Mask off all other bits in 32 bit word except the otp_internalSense bit from
// OTP_READ register
#define OTP_INTERNALSENSE_MASK      0x00000040UL

// The 32 bit value to write into OTP_PROG register in order to cause the
//   otp_internalSense bit to get set high
// 0b0000 0000 0000 0000 1011 1101 0000 0110
#define OTP_INTERNALSENSE_PROGRAM   0x0000BD06UL

// Mask off all other bits in 32 bit word except the low bit of OTP_IHOLD which 
// we want set.
#define OTP_IHOLD_LOW_BIT_MASK      0x00200000UL

// The 32 bit value to write into OTP_PROG register in order to cause the
//   OTP_IHOLD low bit bit (OTP byte 2, bit 5) to get set high
// 0b0000 0000 0000 0000 1011 1101 0000 0110
#define OTP_IHOLD_LOW_BIT_PROGRAM   0x0000BD25UL

// 32-bit mask representing the 'reset' bit in the GSTAT register
#define GSTAT_RESET_MASK            0x00000001UL

/************** MODULE GLOBAL VARIABLE DEFINITIONS ****************************/

// Table of each address to send DriverInitTableValues to (coordinate with values)
static const uint8_t DriverInitTableAddress[MAX_DRIVER_INIT_VALUES] =
{
  GCONF,
  IHOLD_IRUN,
  CHOPCONF,
  GSTAT
};

// Table of 32 values to send to drivers on init (coordinate with addresses)
// (Used with X and Y steppers - Motor1 and Motor2)
static const uint32_t DriverInitTableValuesM12[MAX_DRIVER_INIT_VALUES] =
{
  /* Set up default general config settings */

  /* GCONF Setup:
   * 0x000001C2
   * 0b0000 0000 0000 0000 0000 0001 1100 0010
   * 
   * b9   0     : test_mode : 0 for normal operation
   * b8   1     : multistep_filt : 1 means enable filter for TSTEP
   * b7   1     : mstep_reg_select : 1 means microstep resolution selected by MSTEP register
   * b6   1     : pdn_disable : 1 means PDN_UART input function disabled (set when using UART)
   * b5   0     : index_step : 0 means INDEX output as selected by index_otpw (we don't use index)
   * b4   0     : index_otpw : 0 means INDEX shows first microstep position of sequencer
   * b3   0     : shaft : 0 means normal motor direction
   * b2   0     : en_SpreadCycle : 0 means StealthChop PWM mode enabled
   * b1   0     : internal_Rsense : 0 means use external sense resistors
   * b0   0     : I_scale_analog : 0 means use internal reference derived from 5VOUT
   */
  0x000001C0,     // GCONF

  /* Set up idle and run current levels
   b20:31  0000 0000 000
   b16:19  0 0100 : IHOLDDELAY : How slowly to ramp current down after TPOWERDOWN 
   b13:15  000
   b8:12   1 0000 : IRUN : Motor run current
   b5:7    000
   b0:4    0 1000 : IHOLD : Motor hold current
    
   0000 0000 0000 0100 0000 0100 0000 0001
   0    0    0    4    0    4    0    1
   */
  0x00000A06,     // IHOLD_RUN

  /* On boot, TMC2209 has the following values in CHOPCONF:
   0x10010053
   b0001 0000 0000 0001 0000 0000 1001 0111
   
   b31    0      : diss2vs : low side short protection disable : protection enabled
   b30    0      : diss2g  : short to GND protection disable : protection enabled
   b29    0      : dedge   : enable double edge step pulses : disabled
   b28    1      : intpol  : interpoation to 256 microsteps : enabled
   b24:27 0000   : mres    : MRES micro step resolution : 256 microsteps
   b18:23 000000 : reserved
   b17    0      : vsense  : sense resistor voltage based current scaling : Low sensitivity, high sense resistor voltage
   b15:16 10     : tbl     : TBL blank time select : comprator blank time at 32 clocks
   b11:14 0000   : reserved
   b7:10  0001   : hend    : HEND hystersis low value OFFSET sine wave offset :
   b4:6   001    : hstrt   : HSTRT hystersis start value added to HEND
   b0:3   0111   : toff    : TOFF off time and driver enable;
   
   Our default is 16x microstepping, so we are going to take the above default
   and simply edit the microstep value:
    
   0x14010053
   */
  0x14010053,      // CHOPCONF

  /* Write a 1 to GSTAT's reset bit to clear it */
  0x00000001       // GSTAT
};

// Table of 32 values to send to drivers on init (coordinate with addresses)
// (Used for the pen up/down motor - Motor3)
static const uint32_t DriverInitTableValuesM3[MAX_DRIVER_INIT_VALUES] =
{
  /* Set up default general config settings */

  /* GCONF Setup:
   * 0x000001C2
   * 0b0000 0000 0000 0000 0000 0001 1100 0010
   * 
   * b9   0     : test_mode : 0 for normal operation
   * b8   1     : multistep_filt : 1 means enable filter for TSTEP
   * b7   1     : mstep_reg_select : 1 means microstep resolution selected by MSTEP register
   * b6   1     : pdn_disable : 1 means PDN_UART input function disabled (set when using UART)
   * b5   0     : index_step : 0 means INDEX output as selected by index_otpw (we don't use index)
   * b4   0     : index_otpw : 0 means INDEX shows first microstep position of sequencer
   * b3   0     : shaft : 0 means normal motor direction
   * b2   0     : en_SpreadCycle : 0 means StealthChop PWM mode enabled
   * b1   0     : internal_Rsense : 0 means use external sense resistors
   * b0   0     : I_scale_analog : 0 means use internal reference derived from 5VOUT
   */
  0x000001C0,     // GCONF

  /* Set up idle and run current levels
   b20:31  0000 0000 000
   b16:19  0 0100 : IHOLDDELAY : How slowly to ramp current down after TPOWERDOWN 
   b13:15  000
   b8:12   0 0100 : IRUN : Motor run current
   b5:7    000
   b0:4    0 0001 : IHOLD : Motor hold current
    
   0000 0000 0000 0100 0000 0100 0000 0001
   0    0    0    4    0    4    0    1
   */
  0x00000401,     // IHOLD_RUN

  /* On boot, TMC2209 has the following values in CHOPCONF:
   0x10010053
   b0001 0000 0000 0001 0000 0000 1001 0111
   
   b31    0      : diss2vs : low side short protection disable : protection enabled
   b30    0      : diss2g  : short to GND protection disable : protection enabled
   b29    0      : dedge   : enable double edge step pulses : disabled
   b28    1      : intpol  : interpoation to 256 microsteps : enabled
   b24:27 0000   : mres    : MRES micro step resolution : 256 microsteps
   b18:23 000000 : reserved
   b17    0      : vsense  : sense resistor voltage based current scaling : Low sensitivity, high sense resistor voltage
   b15:16 10     : tbl     : TBL blank time select : comprator blank time at 32 clocks
   b11:14 0000   : reserved
   b7:10  0001   : hend    : HEND hystersis low value OFFSET sine wave offset :
   b4:6   001    : hstrt   : HSTRT hystersis start value added to HEND
   b0:3   0111   : toff    : TOFF off time and driver enable;
   
   Our default is 16x microstepping, so we are going to take the above default
   and simply edit the microstep value:
    
   0x14010053
   */
  0x14010053,      // CHOPCONF

  /* Write a 1 to GSTAT's reset bit to clear it */
  0x00000001       // GSTAT
};

/************** PRIVATE FUNCTION PROTOTYPES ***********************************/


void WriteDatagram(uint8_t addr, uint8_t reg, uint32_t data);
uint32_t ReadDatagram(uint8_t addr, uint8_t reg);
void CalcCRC(uint8_t* datagram, uint8_t datagramLength);
void WriteOTP(uint8_t addr, uint32_t data, uint32_t mask);

/************** PRIVATE FUNCTIONS *********************************************/



void CalcCRC(uint8_t * datagram, uint8_t datagramLength)
{
  uint8_t i,j;
  uint8_t * crc = datagram + (datagramLength-1); // CRC located in last byte of message
  uint8_t currentByte;
  *crc = 0;
  // Execute for all bytes of a message
  for (i=0; i<(datagramLength-1); i++) 
  { 
    currentByte = datagram[i]; // Retrieve a byte to be sent from Array
    for (j=0; j<8; j++) 
    {
      if ((*crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
      {
        *crc = (*crc << 1) ^ 0x07;
      }
      else
      {
        *crc = (*crc << 1);
      }
      currentByte = currentByte >> 1;
    } // for CRC bit
  } //
}

void WriteOTP(uint8_t addr, uint32_t data, uint32_t mask)
{
  uint8_t i;
  
  // We will try writing it up to 10 times before giving up
  for(i=0; i < 10; i++)
  {
    WriteDatagram(addr, OTP_PROG, data);
    // Delay 10ms by waiting on ISR GlobalDelayMS
    GlobalDelayMS = 10;
    while (GlobalDelayMS);
    if (ReadDatagram(addr, OTP_READ) & mask)
    {
      // We have made it a 1, so we are done
      break;
    }
  }
}


/* Create a 63 bit long datagram to send out to stepper drivers */
/* Note: This is a blocking function and won't return until the datagram is sent */
/* TMC2209 Write (64 bit) Datagram Format:
 * Byte 0: Value: 0x05
 * Byte 1: Driver address (0 through 2 for 3BB)
 * Byte 2: Register to write (with high bit set, indicating a write)
 * Byte 3: Bits 31-24 of the register value to write
 * Byte 4: Bits 23-16 of the register value to write
 * Byte 5: Bits 15-8 of the register value to write
 * Byte 6: Bits 7-0 of the register value to write
 * Byte 7: CRC byte of previous 7 bytes
 */
void WriteDatagram(uint8_t addr, uint8_t reg, uint32_t data)
{
  uint8_t datagram[8];
  
  datagram[0] = 0x05;
  if (addr > 3)
  {
    addr = 3;
  }
  datagram[1] = addr;
  datagram[2] = reg | 0x80;
  datagram[3] = (data >> 24) & 0xFF;
  datagram[4] = (data >> 16) & 0xFF;
  datagram[5] = (data >> 8) & 0xFF;
  datagram[6] = data & 0xFF;
  CalcCRC(&datagram[0], 8);
  
  serial_TurnOnTX();
  // Send the full 64 bits out
  HAL_UART_Transmit(&huart1, datagram, 8, 1000);
}


/* Create a 32 bit long datagram to send out to stepper drivers requesting a
 * read of a particular register. Then read in the response from the driver.
 * A timeout value of 500uS will be used - if the driver does not respond
 * within that time, then the driver is not powered or something else is wrong
 * and a value of 0x0000000 will be returned.
 */
/* TMC2209 Read (32 bit) Datagram Format:
 * (Datagram from MCU to TMC2209)
 * Byte 0: Value: 0x05
 * Byte 1: Driver address (0 through 2 for 3BB)
 * Byte 2: Register to read (with high bit clear, indicating a read)
 * Byte 4: CRC byte of previous 3 bytes
 *
 * (TMC2209 then waits for 8 bit times)
 *
 * (Datagram from TMC2209 back to MCU)
 * Byte 0: Value: 0x05
 * Byte 1: Value: 0xFF (Master address)
 * Byte 2: Register that got read (with high bit clear, indicating a read)
 * Byte 3: Bits 31-24 of the register value read
 * Byte 4: Bits 23-16 of the register value read
 * Byte 5: Bits 15-8 of the register value read
 * Byte 6: Bits 7-0 of the register value read
 * Byte 7: CRC byte of previous 7 bytes
 *
 */
uint32_t ReadDatagram(uint8_t addr, uint8_t reg)
{
  uint8_t write_datagram[4];
  uint8_t read_datagram[8];
  uint8_t i;
  union {
    uint32_t  word;
    uint8_t   byte[4];
  } retval;
  retval.word = 0;
  
  write_datagram[0] = 0x05;
  if (addr > 3)
  {
    addr = 3;
  }
  write_datagram[1] = addr;
  write_datagram[2] = reg & 0x7F;
  CalcCRC(&write_datagram[0], 4);
  
  serial_TurnOnTX();

  // Send the 32 bits of the read request out
  /// TODO: Check return value?
  HAL_UART_Transmit(&huart1, write_datagram, 4, 10);

  // Zero out the datagram
  for (i=0; i < 8; i++)
  {
    read_datagram[i] = 0x00;
  }

  serial_TurnOffTX();
  __HAL_UART_FLUSH_DRREGISTER(&huart1);

  /// TODO: Check return value?
  HAL_UART_Receive(&huart1, read_datagram, 8, 10);

  serial_TurnOnTX();

  /// TODO: Check that CRC is correct here too
  if (read_datagram[0] == 0x05 && read_datagram[1] == 0xFF)
  {
    retval.byte[0] = read_datagram[6];
    retval.byte[1] = read_datagram[5];
    retval.byte[2] = read_datagram[4];
    retval.byte[3] = read_datagram[3];
  }
  return retval.word;
}

/************** PUBLIC FUNCTIONS **********************************************/

void serial_TurnOnTX(void)
{
  // Turn on the TX pin

  // This is copied from the HAL - not totally sure if it is necessary to clear both before setting one or both
  CLEAR_BIT(huart1.Instance->CR1, (USART_CR1_TE | USART_CR1_RE));

  /* Enable the USART's RX and TX */
  SET_BIT(huart1.Instance->CR1, USART_CR1_TE);

  /// TODO: Do we need a short delay here?
}

void serial_TurnOffTX(void)
{
  // Turn off the TX pin
  // This is copied from the HAL - not totally sure if it is necessary to clear both before setting one or both
  CLEAR_BIT(huart1.Instance->CR1, (USART_CR1_TE | USART_CR1_RE));

  /* Enable the USART's receive interface by setting the RE bit in the USART CR1 register */
  SET_BIT(huart1.Instance->CR1, USART_CR1_RE);

  /// TODO: Do we need a short delay here?
}

/*
 * Walk through each of the values in the table that we want to send to the
 * drivers, and send them out, with a little delay in between.
 */
void serial_InitDrivers(void)
{
  uint8_t i;
//  uint32_t reg;
  serial_TurnOnTX();
  for (i=0; i < MAX_DRIVER_INIT_VALUES; i++)
  {
    // There are two tables - one for motors 1/2 and a second for motor 3
    // This way we can make the pen lift servo (3) have different settings
    WriteDatagram(0, DriverInitTableAddress[i], DriverInitTableValuesM12[i]);
    HAL_Delay(1);
    WriteDatagram(1, DriverInitTableAddress[i], DriverInitTableValuesM12[i]);
    HAL_Delay(1);
    WriteDatagram(2, DriverInitTableAddress[i], DriverInitTableValuesM3[i]);
    HAL_Delay(1);
  }
#if 0   // Now that we have external sense resistors we don't need this right?
  // Read Byte0 of OPT memory and check bit 6. This is the otp_internalSense
  // bit and needs to be high. If it is not high, then use the writeOPT() 
  // function to write it to a 1.
  for (i = 0; i <= 2; i++)
  {
    reg = ReadDatagram(i, OTP_READ);
    if (!(reg & OTP_INTERNALSENSE_MASK))
    {
      WriteOTP(i, OTP_INTERNALSENSE_PROGRAM, OTP_INTERNALSENSE_MASK);
    }
    HAL_Delay(1);
  }
#endif
}

/* Initialize EUSART2 to talk to the three TMC2209 stepper drivers.
 * Then send the first set of UART commands which set up the drivers
 * with the options we need to have at boot time for things to run.
 * The UART RX pin is on RP11 (RC0), and the TX pin is RP12 (RC1). */
void serial_Init(void)
{
  // Start out by calling for a driver init
  DriversNeedInit = true;
}

/*
 * DR command (Driver Read)
 * 
 * "DR,<driver_number>,<register_address><CR>"
 * <driver_number> is 1, 2 or 3 and tells which driver to read from
 * <register_address> is 0 to 127 and tells which register to read from
 * 
 * This command replies with:
 * "DR,<register_value><CR>"
 * <register_value> is a 32 bit unsigned int expressed in hex with 0x on the front
 */
void serial_DRCommand(void)
{
  uint8_t driverNumber = 0;
  uint8_t registerAddress = 0;
  uint32_t registerValue = 0;

  // Extract each of the values.
  extract_number (kUINT8, &driverNumber, kOPTIONAL);
  extract_number (kUINT8, &registerAddress, kOPTIONAL);
  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (driverNumber == 0 || driverNumber > 3)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  if (registerAddress > 0x7F)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  registerValue = ReadDatagram(driverNumber-1, registerAddress);

  printf("DR,%08lX\n", registerValue);

  print_ack();
}

/*
 * DW command (Driver Write)
 * 
 * "DW,<driver_number>,<register_address>,<register_value><CR>"
 * <driver_number> is 1, 2 or 3 and tells which driver to write to
 * <register_address> is 0 to 127 and tells which register to write to
 * <register_value> is a 32 bit unsigned integer value and is what is written to
 *   the register
 * 
 * This command replies with:
 * "DW,<CR>"
 */
void serial_DWCommand(void)
{
  uint8_t driverNumber = 0;
  uint8_t registerAddress = 0;
  uint32_t registerValue = 0;

  // Extract each of the values.
  extract_number (kUINT8, &driverNumber, kREQUIRED);
  extract_number (kUINT8, &registerAddress, kREQUIRED);
  extract_number (kHEX32, &registerValue, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (driverNumber == 0 || driverNumber > 3)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  if (registerAddress > 0x7F)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  WriteDatagram(driverNumber - 1, registerAddress, registerValue);
  
  printf("DW\n");

  print_ack();
}
