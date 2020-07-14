
#include <p18cxxx.h>
#include "serial.h"
#include "utility.h"
#include <usart.h>
#include <delays.h>
#include "TMC2209.h"
#include "HardwareProfile.h"
#include "servo.h"
#include "analog.h"
#include "parse.h"
#include "isr.h"

// The number of 32-bit init values to send to the drivers over serial
#define MAX_DRIVER_INIT_VALUES      4

// Mask off all other bits in 32 bit word except the otp_internalSense bit from
// OTP_READ register
#define OTP_INTERNALSENSE_MASK      0x00000040UL

// The 32 bit value to write into OTP_PROG register in order to cause the
//   otp_internalSense bit to get set high
// 0b0000 0000 0000 0000 1011 1101 0000 0110
#define OTP_INTERNALSENSE_PROGRAM   0x0000BD06UL


// Table of each address to send DriverInitTableValues to (coordinate with values)
UINT8 DriverInitTableAddress[MAX_DRIVER_INIT_VALUES] = 
{
  GSTAT,
  GCONF,
  IHOLD_RUN,
  CHOPCONF
};

// Table of 32 values to send to drivers on init (coordinate with addresses)
UINT32 DriverInitTableValues[MAX_DRIVER_INIT_VALUES] = 
{
  /* Write a 1 to GSTAT's reset bit to clear it */
  0x00000001,
  
  /* Set up default general config settings */
  0x000000C2,     // GCONF

  /* Set up idle and run current levels */
  0x000021E1,     // IHOLD_RUN

  /* Set up default power-on stepper microstep resolutions */
  
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
  0x14010053      // CHOPCONF
};

void WriteDatagram(UINT8 addr, UINT8 reg, UINT32 data);
UINT32 ReadDatagram(UINT8 addr, UINT8 reg);
void CalcCRC(UINT8* datagram, UINT8 datagramLength);
void WriteOTP(UINT8 addr, UINT32 data, UINT32 mask);

void CalcCRC(UINT8* datagram, UINT8 datagramLength)
{
  int i,j;
  UINT8* crc = datagram + (datagramLength-1); // CRC located in last byte of message
  UINT8 currentByte;
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

void WriteOTP(UINT8 addr, UINT32 data, UINT32 mask)
{
  UINT8 i;
  
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
void WriteDatagram(UINT8 addr, UINT8 reg, UINT32 data)
{
  UINT8 datagram[8];
  UINT8 i;
  
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
  
  // Send the full 64 bits out
  for (i=0; i < 8; i++)
  {
    TXREG2 = datagram[i];
    while(!TXSTA2bits.TRMT);
  }
}

/* Create a 32 bit long datagram to send out to stepper drivers requesting a
 * read of a particular register. Then read in the response from the driver.
 * A timeout value of 500uS will be used - if the driver does not respond
 * within that time, then the driver is not powered or something else is wrong
 * and a value of 0x0000000 will be returned.
 */
UINT32 ReadDatagram(UINT8 addr, UINT8 reg)
{
  UINT8 datagram[8];
  UINT8 i, j;
  UINT32 retval = 0;
  volatile UINT8 dummy;

  datagram[0] = 0x05;
  if (addr > 3)
  {
    addr = 3;
  }
  datagram[1] = addr;
  datagram[2] = reg & 0x7F;
  CalcCRC(&datagram[0], 4);
  
  // Clear and set CREN before every transaction to get rid of overrun error bit
  // and clear out any bytes in the receive FIFO
  dummy = RCREG2;
  dummy = RCREG2;
  RCSTA2bits.CREN = 0;
  RCSTA2bits.CREN = 1;
  
  // Send the full 32 bits out
  for (i=0; i < 4; i++)
  {
    TXREG2 = datagram[i];
    // Always read in the byte we just sent out so overrun bit doesn't get set
    /// TODO: We could check this against what we just sent out and throw an error
    /// if it's wrong. But what kind of error?
DEBUG_A1_SET()
    while(!TXSTA2bits.TRMT);
DEBUG_A1_CLEAR()
    dummy = RCREG2;
  }
  
//DEBUG_A0_SET()
  // Zero out the datagram, as we'll reuse it for the reply
  for (i=0; i < 8; i++)
  {
    datagram[i] = 0x00;
  }

  // Transmission of the last byte has finished
  // Now immediately empty our UART's receive FIFO of any bytes in it
  // Since it's a 2 deep FIFO, there will be 2 bytes in there (from the loopback
  // during our send above)
  if (PIR3bits.RC2IF)
  {
DEBUG_A5_SET()
    dummy = RCREG2;
DEBUG_A5_CLEAR()
  }
  if (PIR3bits.RC2IF)
  {
DEBUG_A5_SET()
    dummy = RCREG2;
DEBUG_A5_CLEAR()
  }
    
  // This loop runs for about 500uS. During that time, any bytes that come
  // from the driver chip are pulled in and stored. If none come in (because
  // the driver isn't powered for example) then the loop finishes without seeing
  // any bytes. The only danger here is that the driver takes more than the 
  // total loop time to send all 8 bytes - in that case some of the last bytes
  // would come after the loop was finished and would get missed. This situation
  // is helped out because taking in bytes takes time, which extends the total
  // duration of the loop
  i=0;
  for (j=0; j < 35; j++)
  {
DEBUG_A1_SET()
    if (PIR3bits.RC2IF)
    {
DEBUG_A5_SET()
      // Read out our data byte
      datagram[i] = RCREG2;
      i++;

      // If we've got all of the datagram from the driver, then no point in
      // doing more timing loop looking for more bytes. So break out.
      if (i == 8)
      {
        // We have a full answer, so check to see if the CRC is correct, 
        // and if it is, then copy the data over to the result value and 
        // leave.
        
        /// TOOD: Is it necessary to check CRC every time here? What do we do
        /// if we get an error? Higher level retries? Ugh.
        
        
        /// TODO: There's got to be a faster way of doing this even on this PIC
        retval = datagram[3];
        retval = (retval << 8) | datagram[4];
        retval = (retval << 8) | datagram[5];
        retval = (retval << 8) | datagram[6];
        
DEBUG_A5_CLEAR()
DEBUG_A1_CLEAR()
        break;
      }
      // Check for framing error or overrun error bits
DEBUG_A5_CLEAR()
      // If we got a byte then extend our outer loop a bit
      j=0;
    }

DEBUG_A1_CLEAR()  
  
    Delay10TCYx(4);
  }

//DEBUG_A0_CLEAR()

  return retval;
}

/*
 * Walk through each of the values in the table that we want to send to the
 * drivers, and send them out, with a little delay in between.
 */
void SerialInitDrivers(void)
{
  UINT8 i;
  UINT32 reg;
  
  for (i=0; i < MAX_DRIVER_INIT_VALUES; i++)
  {
    // For now, we're going to send exactly the same thing to each of the three
    // stepper drivers. This will need to change if we don't want exactly the
    // same values going to all three.
    WriteDatagram(1, DriverInitTableAddress[i], DriverInitTableValues[i]);
    Delay100TCYx(20);
    WriteDatagram(2, DriverInitTableAddress[i], DriverInitTableValues[i]);
    Delay100TCYx(20);
    WriteDatagram(3, DriverInitTableAddress[i], DriverInitTableValues[i]);
    Delay100TCYx(20);
  }
  // Read Byte0 of OPT memory and check bit 6. This is the otp_internalSense
  // bit and needs to be high. If it is not high, then use the writeOPT() 
  // function to write it to a 1.
  reg = ReadDatagram(1, OTP_READ);
  if (!(reg & OTP_INTERNALSENSE_MASK))
  {
    WriteOTP(1, OTP_INTERNALSENSE_PROGRAM, OTP_INTERNALSENSE_MASK);
  }
}

/* Initialize EUSART2 to talk to the three TMC2209 stepper drivers.
 * Then send the first set of UART commands which set up the drivers
 * with the options we need to have at boot time for things to run.
 * The UART RX pin is on RP11 (RC0), and the TX pin is RP12 (RC1). */
void SerialInit(void)
{
  // Set up initial states
  LATCbits.LATC0 = 1;
  LATCbits.LATC1 = 1;
  // Set up I/O directions
  TRISCbits.TRISC0 = INPUT_PIN;
  TRISCbits.TRISC1 = OUTPUT_PIN;

  // Set up EUSART2 RX on pin RP11
  RPINR16 = 11; /// TODO: Are these constants defined in MCHP header files?
  // Set up EUSART2 TX on pin RP12
  RPOR12 = 5;
  
  TXSTA2bits.BRGH = 1;
  RCSTA2bits.CREN = 1;
  // Let's make the baud rate about 115200
  BAUDCON2bits.BRG16 = 1;
  SPBRGH2 = 0;
  SPBRG2 = 103;
  // And finally turn the UART on
  RCSTA2bits.SPEN = 1;
  TXSTA2bits.TXEN = 1;
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
void ParseDRCommand(void)
{
  UINT8 driverNumber = 0;
  UINT8 registerAddress = 0;
  UINT32 registerValue = 0;

  // Extract each of the values.
  extract_number (kUINT8, &driverNumber, kOPTIONAL);
  extract_number (kUINT8, &registerAddress, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (driverNumber > 3)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  if (registerAddress > 0x7F)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  registerValue = ReadDatagram(driverNumber, registerAddress);
  
  printf((far rom char *)"DR,%08lX\r", registerValue);

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
void ParseDWCommand(void)
{
  UINT8 driverNumber = 0;
  UINT8 registerAddress = 0;
  UINT32 registerValue = 0;

  // Extract each of the values.
  extract_number (kUINT8, &driverNumber, kREQUIRED);
  extract_number (kUINT8, &registerAddress, kREQUIRED);
  extract_number (kHEX32, &registerValue, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  if (driverNumber > 3)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  if (registerAddress > 0x7F)
  {
    bitset(error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT);
    return;
  }

  WriteDatagram(driverNumber, registerAddress, registerValue);
  
  printf((far rom char *)"DW\r");

  print_ack();
}
