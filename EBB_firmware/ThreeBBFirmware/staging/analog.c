#include <p18cxxx.h>
#include "parse.h"
#include "analog.h"
#include <stdio.h>
#include "HardwareProfile.h"

// Bit field keeping track of which ADC channels are enabled
UINT16 AnalogEnabledChannels;

// This function turns on or off an analog channel
// It is called from other pieces of code, not the user
void analogConfigure(UINT8 Channel, UINT8 Enable)
{
  if (Channel > 16)
  {
    Channel = 16;
  }

  if (Enable)
  {
    AnalogEnabledChannels |= ((UINT16)0x0001 << Channel);
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
    AnalogEnabledChannels &= ~((UINT16)0x0001 << Channel);
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
  UINT16 result;
  
  // Set which ADC channel we want to convert
  ADCON0 = (channel << 2) + 1;
  // And start the next conversion
  ADCON0bits.GO_DONE = 1;
  // Wait until the conversion is complete
  while(ADCON0bits.GO_DONE);
  // Read out the freshly converted result
  result = (UINT16)ADRESL | ((UINT16)ADRESH << 8);

  return(result);
}

/*
 * Perform a blocking calibration of the ADC
 */
void analogCalibrate(void)
{
  // Set calibration to happen on next conversion
  ADCON1bits.ADCAL = 1;
  // And start the calibration
  ADCON0bits.GO_DONE = 1;
  // Wait until the calibration is complete
  while(ADCON0bits.GO_DONE);
  // Disable calibration
  ADCON1bits.ADCAL = 0;
}

// Analog Configure
// "AC,<channel>,<enable><CR>"
// <channel> is one of the 16 possible analog channels, from 0 through 15
// <enable> is 0 to disable, or 1 to enable
// To turn on a particular analog channel, use the AC command to enable it.
// To turn off a particular analog channel, use the AC command to disable it.
// Once enabled, that channel will be converted at the normal ADC conversion
// rate and will show up in A packets.
void parseACCommand(void)
{
  UINT8 Channel, Enable;

  // Extract each of the two values.
  extract_number (kUINT8, &Channel, kREQUIRED);
  extract_number (kUINT8, &Enable, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }

  analogConfigure(Channel, Enable);
    
  print_ack ();
}

// AR is for Read Analog inputs
// Just print out the analog values for each of the
// enabled channels.
// Returned packet will look like 
// "AR,2:421,5:891,9:3921<CR>" if channels 2, 5 and 9
// are enabled.
void parseARCommand(void)
{
  UINT8 channel = 0;
  UINT16 ChannelBit = 0x0001;

  // Put the beginning of the packet in place
  printf ((far rom char *)"AR");

  // Walk through each channel, and if it is enabled, convert it and print it
  for (channel = 0; channel < 16; channel++)
  {
    if (ChannelBit & AnalogEnabledChannels)
    {
      printf(
        (far rom char *)",%02u:%04u"
        ,channel
        ,analogConvert(channel)
      );
    }
    ChannelBit = ChannelBit << 1;
  }
  printf ((far rom char *)"\n");

  print_ack ();
}

void analogInit(void)
{  
  // Start out with no analog channels enabled
  AnalogEnabledChannels = 0;

  // Set up the Analog to Digital converter

  // Turn on band-gap
  ANCON1bits.VBGEN = 1;

  // Set up ADCON1 options
  // A/D Result right justified
  // Normal A/D (no calibration)
  // Acq time = 20 Tad (?)
  // Tad = Fosc/64
  ADCON1 = 0b10111110;

  // Make sure it's on!
  ADCON0bits.ADON = 1;
  
  analogCalibrate();

#if defined(BOARD_EBB)
  // Turn on AN0 (RA0) as analog input
  analogConfigure(0,1);
#endif
  // Turn on AN11 (V+) as analog input
  //analogConfigure(SCALED_V_ADC_CHAN,1);
}
