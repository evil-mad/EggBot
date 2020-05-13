/*********************************************************************
 *
 *   Microchip USB HID Bootloader v1.01 for PIC18F46J50 Family Devices
 *
 *********************************************************************
 * FileName:        main.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 3.22+
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * File Version  Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 1.0  04/09/2008  Started from MCHPFSUSB v1.3 HID Mouse
 *                  demo project.  Main modifications include
 *                  changes to HID report descriptor, and
 *                  replacement of the "user_mouse.c" file
 *                  with the contents contained in the
 *                  Boot46J50Family.c file.
 * 1.01 01/23/2009  Minor modifications.  Added firmware only
 *                  entry point (goto 0x001C)
 * 1.02 02/27/2010  Modified to work with EBB_V10 (BPS)
 * 1.03 07/06/2009  Slight change to report descriptor (Microchip)
********************************************************************/

/*********************************************************************
IMPORTANT NOTE: This code is currently configured to work with the
PIC18F46J50 FS USB Demo Board.  It can be readily adapted to
work with other members of the PIC18F46J50 Family of USB microcontrollers 
as well (PIC18F24J50/25J50/26J50/44J50/45J50/46J50).

To do so, modify the linker script for the appropriate FILES includes,
and the new memory ranges (assuming a different memory size device), and
click "Configure --> Select Device" and select the proper
microcontroller.  Also double check to verify that the io_cfg.h and
usbcfg.h are properly configured to match your desired application
platform.

It is also recommended to configure the default I/O pin usage in this code.
See the InitializeSystem() function.

This code is meant to be compiled with the C18 compiler version 3.22+
with all optimizations turned on.  If some of the optimizations are not
enabled, the total code size may grow to exceed the 0x000-0xFFF memory
region this code is designed to occupy.  In this case, linker errors will
occur.  If this happens, the vector remapping in the _entry() function 
will have to be modified, as will the application firmware projects that 
may get programmed using this bootloader firmware (to have an entry
point higher than 0x1000).  Additionally, the linker script in this
project will have to be modified to make the BootPage section larger.
*********************************************************************/


//----------------------------------------------------
//Usage tips for this HID USB bootloader firwmare
//----------------------------------------------------

//To enter this bootloader firmware, hold the RB2 I/O pin low at power
//up or after a reset.  Alternatively, application firmware may enter
//the bootloader firmware by clearing the INTCON<GIE> bit and then
//executing an "_asm goto 0x001C _endasm" instruction.

//If a high priority interrupt occurs, the PC will jump to 0x1008
//If a low priority interrupt occurs, the PC will jump to 0x1018

//If RB2 is high at power up/after reset, this code will jump to
//the application firmware, instead of staying in this bootloader firmware.
//The start of the application firmware should be at 0x1000
//In other words, when developing the application firmware which will be
//programmed with this bootloader, place the following in the code, if
//it is a C18 based project:

//extern void _startup (void);    // See c018i.c in your C18 compiler dir
//#pragma code AppFirmwareStartLocation = 0x1000
//void _reset (void)
//{
//    _asm goto _startup _endasm
//}

//Build the application project with a linker script that marks
//the address range 0x000-0xFFF as "PROTECTED".  This is the program
//memory region that this bootloader is currently configured to occupy.

//Although the bootloader can re-program the program memory page that
//contains the configuration bits (the last page of implemented flash)
//it is not always preferrable to do so in case a user attempts to
//program a hex file with configuration bit settings that are not compatible
//with USB operation.  This would prevent further entry into the bootloader.
//If the bootloader will not be used to program the configuration
//words page, the application firmware's linker script should mark
//the entire page as PROTECTED.



/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "typedefs.h"
#include "usb.h"
#include "io_cfg.h"
#if defined(EBB_V11) || defined(THREEBEEBEE_V10)
  #include "Boot46J50Family.h"
#elif defined(EBB_V10)
  #include "Boot87J50Family.h"
#endif

/** C O N F I G U R A T I O N ************************************************/
// Note: For a complete list of the available config pragmas and their values, 
// see the compiler documentation, and/or click "Help --> Topics..." and then 
// select "PIC18 Config Settings" in the Language Tools section.

#if defined(PIC18F46J50_PIM)
    #pragma config WDTEN = OFF          // WDT disabled (enabled by SWDTEN bit)
    #pragma config PLLDIV = 3           // Divide by 3 (12 MHz oscillator input)
    #pragma config STVREN = ON          // stack overflow/underflow reset enabled
    #pragma config XINST = OFF          // Extended instruction set disabled
    #pragma config CPUDIV = OSC1        // No CPU system clock divide
    #pragma config CP0 = OFF            // Program memory is not code-protected
    #pragma config OSC = HSPLL          // HS oscillator, PLL enabled, HSPLL used by USB
    #pragma config T1DIG = OFF          // Sec Osc clock source may not be selected, unless T1OSCEN = 1
    #pragma config LPT1OSC = OFF        // high power Timer1 mode
    #pragma config FCMEN = OFF          // Fail-Safe Clock Monitor disabled
    #pragma config IESO = OFF           // Two-Speed Start-up disabled
    #pragma config WDTPS = 32768        // 1:32768
    #pragma config DSWDTOSC = INTOSCREF // DSWDT uses INTOSC/INTRC as clock
    #pragma config RTCOSC = T1OSCREF    // RTCC uses T1OSC/T1CKI as clock
    #pragma config DSBOREN = OFF        // Zero-Power BOR disabled in Deep Sleep
    #pragma config DSWDTEN = OFF        // Disabled
    #pragma config DSWDTPS = 8192       // 1:8,192 (8.5 seconds)
    #pragma config IOL1WAY = OFF        // IOLOCK bit can be set and cleared
    #pragma config MSSP7B_EN = MSK7     // 7 Bit address masking
    #pragma config WPFP = PAGE_1        // Write Protect Program Flash Page 0
    #pragma config WPEND = PAGE_0       // Start protection at page 0
    #pragma config WPCFG = OFF          // Write/Erase last page protect Disabled
    #pragma config WPDIS = OFF          // WPFP[5:0], WPEND, and WPCFG bits ignored 
//If using the YOUR_BOARD hardware platform (see usbcfg.h), uncomment below and add pragmas
#elif defined(EBB_V11) || defined(THREEBEEBEE_V10)
    #pragma config WDTEN = OFF          // WDT disabled (enabled by SWDTEN bit)
    #pragma config PLLDIV = 2           // Divide by 2 (8 MHz internal oscillator)
    #pragma config STVREN = ON          // stack overflow/underflow reset enabled
    #pragma config XINST = OFF          // Extended instruction set disabled
    #pragma config CPUDIV = OSC1        // No CPU system clock divide
    #pragma config CP0 = OFF            // Program memory is not code-protected
    #pragma config OSC = INTOSCPLL      // Internal oscillator, PLL enabled, PLL used by USB, RA6 and RA7 for I/O use
    #pragma config T1DIG = ON           // Sec Osc clock source may be selected
    #pragma config LPT1OSC = ON         // high power Timer1 mode
    #pragma config FCMEN = OFF          // Fail-Safe Clock Monitor disabled
    #pragma config IESO = OFF           // Two-Speed Start-up disabled
    #pragma config WDTPS = 32768        // 1:32768
    #pragma config DSWDTOSC = INTOSCREF // DSWDT uses INTOSC/INTRC as clock
    #pragma config RTCOSC = T1OSCREF    // RTCC uses T1OSC/T1CKI as clock
    #pragma config DSBOREN = OFF        // Zero-Power BOR disabled in Deep Sleep
    #pragma config DSWDTEN = OFF        // Disabled
    #pragma config DSWDTPS = 8192       // 1:8,192 (8.5 seconds)
    #pragma config IOL1WAY = OFF        // IOLOCK bit can be set and cleared
    #pragma config MSSP7B_EN = MSK7     // 7 Bit address masking
    #pragma config WPFP = PAGE_1        // Write Protect Program Flash Page 0
    #pragma config WPEND = PAGE_0       // Start protection at page 0
    #pragma config WPCFG = ON           // Write/Erase last page protect enabled
    #pragma config WPDIS = ON           // WPFP[5:0], WPEND, and WPCFG bits not ignored 
#elif defined(EBB_V10)			// Configuration bits for EggBotBoard V1.0
    #pragma config DEBUG	= OFF         // Disable debugger
    #pragma config XINST    = OFF       // Extended instruction set
    #pragma config STVREN   = ON        // Stack overflow reset
    #pragma config PLLDIV   = 6         // (24 MHz crystal used on this board)
    #pragma config WDTEN    = OFF       // Watch Dog Timer (WDT)
    #pragma config CP0      = OFF       // Code protect
    #pragma config CPUDIV   = OSC1      // OSC1 = divide by 1 mode
    #pragma config IESO     = OFF       // Internal External (clock) Switchover
    #pragma config FCMEN    = OFF       // Fail Safe Clock Monitor
    #pragma config FOSC     = HSPLL     // Firmware must also set OSCTUNE<PLLEN> to start PLL!
    #pragma config WDTPS    = 32768
    #pragma config MSSPMSK  = MSK5
    #pragma config CCP2MX   = DEFAULT
  #if defined(__18F87J50)||defined(__18F86J55)|| \
      defined(__18F86J50)||defined(__18F85J50)
      #pragma config WAIT     = OFF     // 
      #pragma config BW       = 16      // Only available on the
      #pragma config MODE     = MM      // 80 pin devices in the 
      #pragma config EASHFT   = OFF     // family.
      #pragma config PMPMX    = DEFAULT //
      #pragma config ECCPMX   = DEFAULT //
  #endif
#else	
  #error Not a supported board (yet), make sure the proper board is selected in usbcfg.h, and if so, set configuration bits in __FILE__, line __LINE__
#endif

/** V A R I A B L E S ********************************************************/
#pragma udata access fast_vars
near unsigned int pll_startup_counter;  // Used for software delay while pll is starting up

#pragma udata
rom unsigned char * ROMptr;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void USBTasks(void);
void BlinkUSBStatus(void);
void Init(void);
void Main(void);
void _entry (void);
//externs
extern void LongDelay(void);

#pragma code _entry_scn = 0x000000		// Reset vector is at 0x00.  Device begins executing code from 0x00 after a reset or POR event
void _entry (void)
{
  _asm goto Init _endasm
}

#pragma code high_isr_scn = 0x000008
void _high_isr (void)
{
  _asm goto 0x1008 _endasm
}

#pragma code low_isr_scn = 0x000018
void _low_isr (void)
{
  _asm goto 0x1018 _endasm
}

// If executing the main application firmware, and user wishes to enter the bootloader
// simply execute an "_asm goto 0x001E _endasm" instruction.  This will go to this BootAppStart section,
// which in turn will enter the bootloader firmware.
#pragma code boot_entry = 0x00001E
void _boot_entry (void)
{
  _asm
    lfsr 1, _stack
    lfsr 2, _stack
    clrf TBLPTRU, 0
  _endasm
  _asm goto Main _endasm
}

/** D E C L A R A T I O N S **************************************************/
#pragma code
/******************************************************************************
 * Function:        void Init(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main bootloader firmware entry point.
 *
 * Note:            None
 *****************************************************************************/

void Init(void)
{
  // NOTE: The c018.o file is not included in the linker script for this project.
  // The C initialization code in the c018.c (comes with C18 compiler in the src directory)
  // file is instead modified and included here manually.  This is done so as to provide
  // a more convenient entry method into the bootloader firmware.  Ordinarily the _entry_scn
  // program code section starts at 0x00 and is created by the code of c018.o.  However,
  // the linker will not work if there is more than one section of code trying to occupy 0x00.
  // Therefore, must not use the c018.o code, must instead manually include the useful code
  // here instead.

  // Initialize the C stack pointer, and other compiler managed items as normally done in the c018.c file.
  _asm
    lfsr 1, _stack
    lfsr 2, _stack
    clrf TBLPTRU, 0
  _endasm
  // End of the important parts of the C initializer.  This bootloader firmware does not use
  // any C initialized user variables (idata memory sections).  Therefore, the above is all
  // the initialization that is required.

  // Check to see if memory at 0x1000 is 'erased' (i.e. 0xFF)
  // If so, then we have a blank device (other than the bootloader) so stay in the bootloader
  // code even if user is not pressing the button
  if (
    (*(rom unsigned char *)0x1000 != 0xFF)
    ||
    (*(rom unsigned char *)0x1001 != 0xFF)
  )
  {
    // Check to see if the PRG button is pressed. If so, wait until it is released and then
    // stay in the bootloader code
    if (sw2)
    {
      // We need to jump into the main app (firmware).
      _asm goto 0x1000 _endasm
    }
    // Wait until the let up on the button
    while(!sw2);
  }

  Main();
}

/******************************************************************************
 * Function:        void Main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main bootloader firmware entry point.
 *
 * Note:            None
 *****************************************************************************/
void Main(void)
{
  InitializeSystem();   // Some USB, I/O pins, and other initialization

  while(1)
  {
    ClrWdt();
    USBTasks();         // Need to call USBTasks() periodically
                        // it handles SETUP packets needed for enumeration

    BlinkUSBStatus();   // Blink the LEDs based on current USB state

    if((usb_device_state == CONFIGURED_STATE) && (UCONbits.SUSPND != 1))
    {
      ProcessIO();      // This is where all the actual bootloader related data transfer/self programming takes place
    }                   // see ProcessIO() function in the Boot87J50Family.c file.
  }//end while	
}

/******************************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization routine.
 *                  All required USB initialization routines are called from
 *                  here.
 *
 *                  User application initialization routine should also be
 *                  called from here.
 *
 * Note:            None
 *****************************************************************************/
static void InitializeSystem(void)
{
  OSCCON = 0x60;  // Clock switch to primary clock source.  May not have been running
                  // from this if the bootloader is called from the application firmware.

  // On the PIC18F46J50 Family of USB microcontrollers, the PLL will not power up and be enabled
  // by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
  // This allows the device to power up at a lower initial operating frequency, which can be
  // advantageous when powered from a source which is not guaranteed to be adequate for 48MHz
  // operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
  // power up the PLL.

  #if defined(__18F87J50)||defined(__18F86J55)|| \
      defined(__18F86J50)||defined(__18F85J50)|| \
      defined(__18F67J50)||defined(__18F66J55)|| \
      defined(__18F66J50)||defined(__18F65J50)|| \
      defined(__18F24J50)||defined(__18F25J50)|| \
      defined(__18F26J50)||defined(__18F44J50)|| \
      defined(__18F45J50)||defined(__18F46J50) 

    OSCTUNEbits.PLLEN = 1;  // Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
    pll_startup_counter = 600;
    while(pll_startup_counter--);
    // Device switches over automatically to PLL output after PLL is locked and ready.

  #else
    #error Double Click this message.  Please make sure the InitializeSystem() function correctly configures your hardware platform.  
    // Also make sure the correct board is selected in usbcfg.h.  If 
    // everything is correct, comment out the above "#error ..." line
    // to suppress the error message.
  #endif


  // USB module may have already been on if the application firmware calls the bootloader
  // without first disabling the USB module.  If this happens, need
  // to temporarily soft-detach from the host, wait a delay (allows cable capacitance
  // to discharge, and to allow host software to recognize detach), then
  // re-enable the USB module, so the host knows to re-enumerate the
  // USB device.
  if(UCONbits.USBEN == 1)
  {
    UCONbits.SUSPND = 0;
    UCON = 0;
    LongDelay();
  }


//  The USB specifications require that USB peripheral devices must never source
//  current onto the Vbus pin.  Additionally, USB peripherals should not source
//  current on D+ or D- when the host/hub is not actively powering the Vbus line.
//  When designing a self powered (as opposed to bus powered) USB peripheral
//  device, the firmware should make sure not to turn on the USB module and D+
//  or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//  firmware needs some means to detect when Vbus is being powered by the host.
//  A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
//  can be used to detect when Vbus is high (host actively powering), or low
//  (host is shut down or otherwise not supplying power).  The USB firmware
//  can then periodically poll this I/O pin to know when it is okay to turn on
//  the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//  peripheral device, it is not possible to source current on D+ or D- when the
//  host is not actively providing power on Vbus. Therefore, implementing this
//  bus sense feature is optional.  This firmware can be made to use this bus
//  sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//  usbcfg.h file.
  #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See io_cfg.h
  #endif

//  If the host PC sends a GetStatus (device) request, the firmware must respond
//  and let the host know if the USB peripheral device is currently bus powered
//  or self powered.  See chapter 9 in the official USB specifications for details
//  regarding this request.  If the peripheral device is capable of being both
//  self and bus powered, it should not return a hard coded value for this request.
//  Instead, firmware should check if it is currently self or bus powered, and
//  respond accordingly.  If the hardware has been configured like demonstrated
//  on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//  currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//  is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//  has been defined in usbcfg.h, and that an appropriate I/O pin has been mapped
//  to it in io_cfg.h.
  #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;
  #endif

  mInitializeUSBDriver();         // See usbdrv.h

  UserInit();                     // See Boot46J50Family.c.  Initializes the bootloader firmware state machine variables.

  mInitAllLEDs();                 // Init them off.

#if defined(EBB_V10)
  // Turn off digital input buffers on analog pins to minimize power consumption
  // if the I/O pins happen to be floating in the target application.
  WDTCONbits.ADSHR = 1;           // ANCON registers in shared address space region
  ANCON0 = 0x00;                  // All analog, to disable the digital input buffers
  ANCON1 = 0x00;                  // All analog, digital input buffers off
  WDTCONbits.ADSHR = 0;
  // Also to minimize sleep current consumption (sleep used in this bootloader
  // firmware during USB Suspend conditions), use REGSLP feature
  WDTCONbits.REGSLP = 1;
#else
  // Initialize I/O pins for "lowest" power.  When in USB suspend mode, total +5V VBUS current consumption 
  // should reduce to <2.5mA in order to meet USB compliance specifications.

  // Ordinarily, to initialize I/O pins for lowest power, any unused I/O pins would be configured
  // as outputs and driven either high or low.  However, if this code is left unmodified, but is used in a real
  // application, I/O pins as outputs could cause contention with externally connected signals.  Therefore
  // this code does not actually drive unused I/Os as outputs, but uses "softer" methods, like making
  // analog capable pins as analog (to disable the digital input buffer, which wastes power when left floating)

  // This code should be replaced with code more specific to the intended target application I/O pin usage.
  // The below code by itself will not achieve the lowest possible power consumption.

  ANCON0 = 0x00;                  // All analog, to disable the digital input buffers
  ANCON1 = 0x00;                  // All analog, digital input buffers off, bandgap off
#endif

}// end InitializeSystem

/******************************************************************************
 * Function:        void USBTasks(void)
 *
 * PreCondition:    InitializeSystem has been called.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Service loop for USB tasks.
 *
 * Note:            None
 *****************************************************************************/
void USBTasks(void)
{
  /*
   * Servicing Hardware
   */
  USBCheckBusStatus();                    // Must use polling method
  USBDriverService();                     // Interrupt or polling method

}// end USBTasks


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
  static word led_count = 0;

  #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
  #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
  #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
  #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

  if (usb_device_state < CONFIGURED_STATE)
  {
    mLED_Only_1_On();
  }
  if (usb_device_state == CONFIGURED_STATE)
  {
    if(led_count >= 20000U)
    {
      mLED_Only_1_On();
    }
    if(led_count >= 40000U)
    {
      mLED_Only_2_On();
      led_count = 0U;
    }
    led_count++;
  }
}//end BlinkUSBStatus





// Placeholder code at address 0x1000 (the start of the non-bootloader firmware space)
// This gets overwritten when a real hex file gets programmed by the bootloader.
// If however no hex file has been programmed, might as well stay in the bootloader
// firmware, even if the RB2 pushbutton was not pressed after coming out of reset.
// #pragma code user_app_vector=0x1000
// void userApp(void)
// {
//  _asm goto 0x001C _endasm 	//Goes to the "BootAppStart:" section which will enter the bootloader firmware
// }
/** EOF main.c ***************************************************************/
