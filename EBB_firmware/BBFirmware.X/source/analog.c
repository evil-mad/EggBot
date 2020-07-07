#include <p18cxxx.h>
#include "parse.h"
#include "analog.h"
#include <stdio.h>

unsigned char A_cur_channel;
unsigned char AnalogInitiate;
volatile unsigned int AnalogEnabledChannels;
volatile unsigned int ChannelBit;



// This function turns on or off an analog channel
// It is called from other pieces of code, not the user
void AnalogConfigure(unsigned char Channel, unsigned char Enable)
{
  if (Channel > 16)
  {
    Channel = 16;
  }

  if (Enable)
  {
    AnalogEnabledChannels |= ((unsigned int)0x0001 << Channel);
    // Make sure to turn this analog input on
    if (Channel < 8)
    {
      // Clear the right bit in ANCON0
      ANCON0 &= ~(1 << Channel);
    }
    else
    {
      if (Channel <= 12)
      {
        // Clear the right bit in ANCON1
        ANCON1 &= ~(1 << (Channel-8));
      }
    }
  }
  else
  {
    AnalogEnabledChannels &= ~((unsigned int)0x0001 << Channel);
    // Make sure to turn this analog input off
    if (Channel < 8)
    {
      // Set the right bit in ANCON0
      ANCON0 |= (1 << Channel);
    }
    else
    {
      if (Channel <= 12)
      {
        // Set the right bit in ANCON1
        ANCON1 |= (1 << (Channel-8));
      }
    }
  }
}

/*
 * Convert one channel of ADC and return the result
 */
UINT16 analogConvert(UINT8 channel)
{
  
}

/*
 * Perform a blocking calibration of the ADC
 */
void analogCalibrate(void)
{
  
}


// Analog Configure
// "AC,<channel>,<enable><CR>"
// <channel> is one of the 16 possible analog channels, from 0 through 15
// <enable> is 0 to disable, or 1 to enable
// To turn on a particular analog channel, use the AC command to enable it.
// To turn off a particular analog channel, use the AC command to disable it.
// Once enabled, that channel will be converted at the normal ADC conversion
// rate and will show up in A packets.
void parse_AC_packet(void)
{
  unsigned char Channel, Enable;

  // Extract each of the two values.
  extract_number (kUCHAR, &Channel, kREQUIRED);
  extract_number (kUCHAR, &Enable, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  AnalogConfigure(Channel, Enable);
    
  print_ack ();
}

// A is for read Analog inputs
// Just print out the analog values for each of the
// enabled channels.
// Returned packet will look like 
// "A,2:421,5:891,9:3921<CR>" if channels 2, 5 and 9
// are enabled.
void parse_A_packet(void)
{
  char channel = 0;
  unsigned int ChannelBit = 0x0001;

  // Put the beginning of the packet in place
  printf ((far rom char *)"A");

  // Sit and spin, waiting for one set of analog conversions to complete
  while (PIE1bits.ADIE);

  // Now print each analog value
  for (channel = 0; channel < 16; channel++)
  {
    if (ChannelBit & AnalogEnabledChannels)
    {
/// TODO: Perform a conversions
/// then print it out
//      printf(
//        (far rom char *)",%02u:%04u"
//        ,channel
//        ,ISR_A_FIFO[channel]
//      );
    }
    ChannelBit = ChannelBit << 1;
  }

  print_ack ();
}

void analog_Init(void)
{  
  // Turn off our own idea of how many analog channels to convert
  AnalogEnabledChannels = 0;

   // Set up the Analog to Digital converter
  // Clear out the FIFO data
  //for (i = 0; i < 16; i++)
  //{
  //  ISR_A_FIFO[i] = 0;
  //}
  // Turn on band-gap
  ANCON1bits.VBGEN = 1;

  // Set up ADCON1 options
  // A/D Result right justified
  // Normal A/D (no calibration)
  // Acq time = 20 Tad (?)
  // Tad = Fosc/64
  ADCON1 = 0b10111110;

  // And make sure to always use low priority for ADC
  IPR1bits.ADIP = 0;

  // Make sure it's on!
  ADCON0bits.ADON = 1;
}
