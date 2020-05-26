

/// TODO: Update so that version number is a define in a header file
#if defined(BOARD_EBB)
  const rom char st_version[] = {"EBB Firmware Version 3.0.0\r\n"};
#elif defined(BOARD_3BB)
  const rom char st_version[] = {"3BB Firmware Version 3.0.0\r\n"};
#endif


/******************************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs corresponding to
 *                  the USB device state.
 *
 * Note:            mLED macros can be found in io_cfg.h
 *                  usb_device_state is declared in usbmmap.c and is modified
 *                  in usbdrv.c, usbctrltrf.c, and usb9.c
 *****************************************************************************/
void BlinkUSBStatus(void)
{
  static WORD LEDCount = 0;
  static unsigned char LEDState = 0;

  if (
    USBDeviceState == DETACHED_STATE
    ||
    1 == USBSuspendControl
  )
  {
    LEDCount--;
    if (0 == LEDState)
    {
      if (0 == LEDCount)
      {
        mLED_1_On();
        LEDCount = 4000U;
        LEDState = 1;
      }
    }
    else
    {
      if (0 == LEDCount)
      {
        mLED_1_Off();
        LEDCount = 4000U;
        LEDState = 0;
      }
    }
  }
  else if (
    USBDeviceState == ATTACHED_STATE
    ||
    USBDeviceState == POWERED_STATE
    ||
    USBDeviceState == DEFAULT_STATE
    ||
    USBDeviceState == ADDRESS_STATE
  )
  {
    LEDCount--;
    if (0 == LEDState)
    {
      if (0 == LEDCount)
      {
        mLED_1_On();
        LEDCount = 20000U;
        LEDState = 1;
      }
    }
    else
    {
      if (0 == LEDCount)
      {
        mLED_1_Off();
        LEDCount = 20000U;
        LEDState = 0;
      }
    }
  }
  else if (USBDeviceState == CONFIGURED_STATE)
  {
    LEDCount--;
    if (0 == LEDState)
    {
      if (0 == LEDCount)
      {
        mLED_1_On();
        LEDCount = 10000U;
        LEDState = 1;
      }
    }
    else if (1 == LEDState)
    {
      if (0 == LEDCount)
      {
        mLED_1_Off();
        LEDCount = 10000U;
        LEDState = 2;
      }
    }
    else if (2 == LEDState)
    {
      if (0 == LEDCount)
      {
        mLED_1_On();
        LEDCount = 100000U;
        LEDState = 3;
      }
    }
    else
    {
      if (0 == LEDCount)
      {
        mLED_1_Off();
        LEDCount = 10000U;
        LEDState = 0;
      }
    }
  }
}

volatile near unsigned char * RPnTRISPort[25] = 
{
  &TRISA,      // RP0
  &TRISA,      // RP1
  &TRISA,      // RP2
  &TRISB,      // RP3
  &TRISB,      // RP4
  &TRISB,      // RP5
  &TRISB,      // RP6
  &TRISB,      // RP7
  &TRISB,      // RP8
  &TRISB,      // RP9
  &TRISB,      // RP10
  &TRISC,      // RP11
  &TRISC,      // RP12
  &TRISC,      // RP13
  &TRISC,      // RP14
  &TRISC,      // RP15
  &TRISC,      // RP16
  &TRISC,      // RP17
  &TRISC,      // RP18
  &TRISD,      // RP19
  &TRISD,      // RP20
  &TRISD,      // RP21
  &TRISD,      // RP22
  &TRISD,      // RP23
  &TRISD,      // RP24
};

volatile near unsigned char * RPnLATPort[25] = 
{
  &LATA,      // RP0
  &LATA,      // RP1
  &LATA,      // RP2
  &LATB,      // RP3
  &LATB,      // RP4
  &LATB,      // RP5
  &LATB,      // RP6
  &LATB,      // RP7
  &LATB,      // RP8
  &LATB,      // RP9
  &LATB,      // RP10
  &LATC,      // RP11
  &LATC,      // RP12
  &LATC,      // RP13
  &LATC,      // RP14
  &LATC,      // RP15
  &LATC,      // RP16
  &LATC,      // RP17
  &LATC,      // RP18
  &LATD,      // RP19
  &LATD,      // RP20
  &LATD,      // RP21
  &LATD,      // RP22
  &LATD,      // RP23
  &LATD,      // RP24
};

const char RPnBit[25] = 
{
  0,          // RP0
  1,          // RP1
  5,          // RP2
  0,          // RP3
  1,          // RP4
  2,          // RP5
  3,          // RP6
  4,          // RP7
  5,          // RP8
  6,          // RP9
  7,          // RP10
  0,          // RP11
  1,          // RP12
  2,          // RP13
  3,          // RP14
  4,          // RP15
  5,          // RP16
  6,          // RP17
  7,          // RP18
  2,          // RP19
  3,          // RP20
  4,          // RP21
  5,          // RP22
  6,          // RP23
  7,          // RP24
};

// From RPn (Pin) number, set LAT value for that pin
void SetPinLATFromRPn(char Pin, char State)
{
  if (Pin > 25)
  {
      return;
  }

  if (State)
  {
      bitset (*RPnLATPort[Pin], RPnBit[Pin]);
  }
  else
  {
      bitclr (*RPnLATPort[Pin], RPnBit[Pin]);
  }
}

// From RPn (Pin) number, set TRIS value for that pin
void SetPinTRISFromRPn(char Pin, char State)
{
  if (Pin > 25)
  {
      return;
  }

  if (OUTPUT_PIN == State)
  {
      bitclr (*RPnTRISPort[Pin], RPnBit[Pin]);
  }
  else
  {
      bitset (*RPnTRISPort[Pin], RPnBit[Pin]);
  }
}

void populateDeviceStringWithName(void)
{
  extern BYTE * USB_SD_Ptr[];

  unsigned char name[FLASH_NAME_LENGTH+1];    
  UINT8 i;

  // Clear out our name array
  for (i=0; i < FLASH_NAME_LENGTH+1; i++)
  {
    name[i] = 0x00;
  }

  // We always read 16, knowing that any unused bytes will be set to zero
  ReadFlash(FLASH_NAME_ADDRESS, FLASH_NAME_LENGTH, name);

  // The EEB's name is now in the 'name' local variable as a straight string
  // of bytes. We need to move it to the proper locations in the sd002
  // USB string descriptor (which is in RAM now). But it needs to be a 
  // unicode string, so we've got to skip every other byte.
  // Since the FLASH copy of 'name' is padded with zeros and is always 16
  // bytes long, we are safe to always copy 16 bytes over to the string
  // descriptor. 
  // Because sd002 is an anonymous structure without any names for its
  // members, we are totally going to just hack this bad boy and jump
  // into a known offset from the beginning of the structure.
  // As of 2.5.5, we now not only update the Product string, but also the
  // serial number string.
  for (i=0; i < FLASH_NAME_LENGTH; i++)
  {
    // Only copy over valid ASCII characters. On the first invalid
    // one, bail out.
    if (name[i] <= 128 && name[i] >= 32)
    {
      *(USB_SD_Ptr[2] + 24 + (i*2)) = name[i];
      *(USB_SD_Ptr[3] + 2 + (i*2)) = name[i];
    }
    else
    {
      break;
    }
  }
  // Now update the string descriptor lengths based on how many characters
  // we copied over from Flash
  *(USB_SD_Ptr[2]) = 24 + (i * 2);
  *(USB_SD_Ptr[3]) = 2 + (i * 2);
}


