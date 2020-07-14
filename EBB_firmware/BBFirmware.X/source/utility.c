
#include <GenericTypeDefs.h>
#include "usb_config.h"
#include "Usb\usb.h"
#include "Usb\usb_function_cdc.h"
#include "HardwareProfile.h"
#include "utility.h"
#include <flash.h>
#include "parse.h"
#include "isr.h"
#include "analog.h"
#include "serial.h"
#include "servo.h"
#include <delays.h>

#define FLASH_NAME_ADDRESS      0xF800          // Starting address in FLASH where we store our EBB's name
#define FLASH_NAME_LENGTH       16              // Size of store for EBB's name in FLASH

// Milliseconds between serial checks to see if drivers are online yet
#define DRIVER_INIT_CHECK_PERIOD_MS 10

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

/*
 * Disables the 1Khz interrupt, grabs a copy of the 32-bit millisecond tick 
 * counter, re-enables the interrupt, and returns the value. The reason we have
 * to jump through this hoop just to read the time value is because this is an
 * 8-bit processor, so copies of 32 bit values are not atomic. Thus, without
 * the interrupt protection here, we could start to copy, then get interrupted
 * which could change the tick counter, then finish the copy with a corrupted
 * value.
 */
UINT32 GetTick(void)
{
  UINT32 retval = 0;
  
  // Disable the 1KHz interrupt
  PIE3bits.TMR4IE = 0;
  
  // Grab a copy of the time
  retval = TickCounterMS;
  
  // Re-enable the 1KHz interrupt
  PIE3bits.TMR4IE = 1;

  return(retval);
}

// ST command : Set Tag
// "ST,<new name><CR>"
// <new name> is a 0 to 16 character ASCII string.
// This string gets saved in FLASH, and is returned by the "QT" command, as
// well as being appended to the USB name that shows up in the OS
void parseSTCommand()
{
  unsigned char name[FLASH_NAME_LENGTH+1];
  UINT8 bytes = 0;
  UINT8 i;

  // Clear out our name array
  for (i=0; i < FLASH_NAME_LENGTH+1; i++)
  {
    name[i] = 0x00;
  }

  bytes = extract_string(name, FLASH_NAME_LENGTH);

  // We have reserved FLASH addresses 0xF800 to 0xFBFF (1024 bytes) for
  // storing persistent variables like the EEB's name. Note that no wear-leveling
  // is done, so it's not a good idea to change these values more than 10K times. :-)

  EraseFlash(FLASH_NAME_ADDRESS, FLASH_NAME_ADDRESS + 0x3FF);

  WriteBytesFlash(FLASH_NAME_ADDRESS, FLASH_NAME_LENGTH, name);

  print_ack();
}

// QT command : Query Tag
// "QT<CR>"
// Prints out the 'tag' that was set with the "ST" command previously, if any
void parseQTCommand()
{
  unsigned char name[FLASH_NAME_LENGTH+1];    
  UINT8 i;

  // Clear out our name array
  for (i=0; i < FLASH_NAME_LENGTH+1; i++)
  {
    name[i] = 0x00;
  }

  // We always read 16, knowing that any unused bytes will be set to zero
  ReadFlash(FLASH_NAME_ADDRESS, FLASH_NAME_LENGTH, name);

  // Only print it out if the first character is printable ASCII
  if (name[0] >= 128 || name[0] < 32)
  {
    printf ((rom char far *)"\r\n");
  }
  else
  {
    printf ((rom char far *)"%s\r\n", name);
  }
  print_ack();
}

/*
 * Called from main loop every time through. Main task here is to check to see
 * if the stepper drivers just came on line (i.e. were just powered by 12V).
 * If they did, then we need to initialize them ASAP. We perform the check by
 * reading the voltage on the SCALED_V+ net each time, and seeing when it goes
 * above 6.5V. At that point we know we can init the drivers. Also watch for
 * it going below 6.5V - if so, disable drivers so they're not enabled on next
 * power on.
 */
void utilityRun(void)
{
  UINT16 currentVPlusVoltage;
  static UINT16 lastVPlusVoltage = 0;

  currentVPlusVoltage = analogConvert(SCALED_V_ADC_CHAN);

  if (
    (lastVPlusVoltage < V_PLUS_VOLTAGE_POWERED) 
    && 
    (currentVPlusVoltage >= V_PLUS_VOLTAGE_POWERED)
  )
  {
    // Because getting these bytes of config data into the drivers is really
    // important (otherwise they will consume a ton of current and make the 
    // motors super hot), we'll send it three times here.
    SerialInitDrivers();
    SerialInitDrivers();
    SerialInitDrivers();
    Delay10KTCYx(10);     // Wait about 10 ms before enableing drivers
    analogCalibrate();    // Because our voltage situation may have changed
    // Enable the drivers by setting their enable pin low
    EnableIO = 0;      
    servoPenHome();       // The drivers were limped, so home the pen
  }
  else if (
    (lastVPlusVoltage > V_PLUS_VOLTAGE_POWERED) 
    && 
    (currentVPlusVoltage <= V_PLUS_VOLTAGE_POWERED)
  )
  {
    // Disable the drivers so they don't consume tons of power the next time we get 9V
    EnableIO = 1;
  }
  lastVPlusVoltage = currentVPlusVoltage;
}
