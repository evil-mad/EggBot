#include "parse.h"

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

