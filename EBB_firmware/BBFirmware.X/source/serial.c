
#include <p18cxxx.h>
#include "serial.h"
#include "utility.h"
#include <usart.h>
#include <delays.h>
#include "TMC2209.h"


void calcCRC(UINT8* datagram, UINT8 datagramLength)
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

/* Create a 63 bit long datagram to send out to stepper drivers */
void writeDatagram(UINT8 addr, UINT8 reg, UINT32 data)
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
  calcCRC(&datagram[0], 8);
  
  // Send the full 64 bits out
  for (i=0; i < 8; i++)
  {
    TXREG2 = datagram[i];
    while(!TXSTA2bits.TRMT);
  }
}

/* Create a 32 bit long datagram to send out to stepper drivers requesting a
 * read of a particular register. Then read in the response from the driver. */
UINT32 readDatagram(UINT8 addr, UINT8 reg)
{
  UINT8 datagram[8];
  UINT8 i;
  
  datagram[0] = 0x05;
  if (addr > 3)
  {
    addr = 3;
  }
  datagram[1] = addr;
  datagram[2] = reg & 0x7F;
  calcCRC(&datagram[0], 4);
  
  // Send the full 32 bits out
  for (i=0; i < 4; i++)
  {
    TXREG2 = datagram[i];
    while(!TXSTA2bits.TRMT);
  }
  
  return 0;
}

/* Initialize EUSART2 to talk to the three TMC2209 stepper drivers.
 * Then send the first set of UART commands which set up the drivers
 * with the options we need to have at boot time for things to run.
 * The UART RX pin is on RP11 (RC0), and the TX pin is RP12 (RC1). */
void serial_Init(void)
{
  DEBUG_INIT()
  
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
  
  /* Set up default general config settings */
  writeDatagram(1, GCONF, 0x000000C2);
  Delay100TCYx(100);
  readDatagram(1, GCONF);
  Delay100TCYx(100);
  writeDatagram(2, GCONF, 0x000000C2);
  Delay100TCYx(100);
  readDatagram(2, GCONF);
  Delay100TCYx(100);  
  writeDatagram(3, GCONF, 0x000000C2);
  Delay100TCYx(100);
  readDatagram(3, GCONF);
  Delay100TCYx(100);
  
  /* Set up idle and run current levels */
  writeDatagram(1, IHOLD_RUN, 0x000021E1);
  Delay100TCYx(100);
  readDatagram(1, IHOLD_RUN);
  Delay100TCYx(100);
  writeDatagram(2, IHOLD_RUN, 0x000021E1);
  Delay100TCYx(100);
  readDatagram(2, IHOLD_RUN);
  Delay100TCYx(100);
  writeDatagram(3, IHOLD_RUN, 0x000021E1);
  Delay100TCYx(100);
  readDatagram(3, IHOLD_RUN);
  Delay100TCYx(100);
  
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
  writeDatagram(1, CHOPCONF, 0x14010053);
  Delay100TCYx(100);
  readDatagram(1, CHOPCONF);
  Delay100TCYx(100);
  writeDatagram(2, CHOPCONF, 0x14010053);
  Delay100TCYx(100);
  readDatagram(2, CHOPCONF);
  Delay100TCYx(100);
  writeDatagram(3, CHOPCONF, 0x14010053);
  Delay100TCYx(100);
  readDatagram(3, CHOPCONF);
  Delay100TCYx(100);
  

}


