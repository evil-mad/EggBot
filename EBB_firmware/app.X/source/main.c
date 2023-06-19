/*********************************************************************
 *
 *                UBW Firmware
 *
 *********************************************************************
 * FileName:        main.c
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Based on original files by Microchip Inc. in MAL USB example.
 *
 * Software License Agreement
 *
 * Copyright (c) 2014-2023, Brian Schmalz of Schmalz Haus LLC
 * All rights reserved.
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

/** INCLUDES *******************************************************/
#include "Usb\usb.h"
#include "Usb\usb_function_cdc.h"

#include "HardwareProfile.h"

#if !defined (PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
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
  #pragma config WPCFG = OFF          // Write/Erase last page protect Disabled
  #pragma config WPDIS = OFF          // WPFP[5:0], WPEND, and WPCFG bits ignored 
#endif

/** I N C L U D E S **********************************************************/

#include "usb_config.h"
#include "UBW.h"

/** V A R I A B L E S ********************************************************/
#pragma udata

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void USBDeviceTasks(void);

/** VECTOR REMAPPING ***********************************************/
// On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
// the reset, high priority interrupt, and low priority interrupt
// vectors.  However, the current Microchip USB bootloader 
// examples are intended to occupy addresses 0x00-0x7FF or
// 0x00-0xFFF depending on which bootloader is used.  Therefore,
// the bootloader code remaps these vectors to new locations
// as indicated below.  This remapping is only necessary if you
// wish to program the hex file generated from this project with
// the USB bootloader.  If no bootloader is used, edit the
// usb_config.h file and comment out the following defines:
//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
  #define REMAPPED_RESET_VECTOR_ADDRESS           0x1000
  #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x1008
  #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS   0x1018
#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
  #define REMAPPED_RESET_VECTOR_ADDRESS           0x800
  #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x808
  #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS   0x818
#else
  #define REMAPPED_RESET_VECTOR_ADDRESS           0x00
  #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
  #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
#endif

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER) || defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
extern void _startup (void);        // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset (void)
{
  _asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR (void)
{
   _asm goto high_ISR _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR (void)
{
   _asm goto low_ISR _endasm
}

/** D E C L A R A T I O N S **************************************************/
#pragma code
/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/
void main(void)
{
  InitializeSystem();

  while(1)
  {
#if defined(USB_INTERRUPT)
    if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE))
    {
      USBDeviceAttach();
    }
#endif
#if defined(USB_POLLING)
    // Check bus status and service USB interrupts.
    USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
                      // this function periodically.  This function will take care
                      // of processing and responding to SETUP transactions 
                      // (such as during the enumeration process when you first
                      // plug in).  USB hosts require that USB devices should accept
                      // and process SETUP packets in a timely fashion.  Therefore,
                      // when using polling, this function should be called 
                      // frequently (such as once about every 100 microseconds) at any
                      // time that a SETUP packet might reasonably be expected to
                      // be sent by the host to your device.  In most cases, the
                      // USBDeviceTasks() function does not take very long to
                      // execute (~50 instruction cycles) before it returns.
#endif

    // Application-specific tasks.
    // Application related code may be added here, or in the ProcessIO() function.
    ProcessIO();
  }
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
#if defined(BOARD_EBB_V10)
  unsigned int pll_startup_counter = 600;
  OSCTUNEbits.PLLEN = 1;    // Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
  while(pll_startup_counter--);

  // Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
  // use the ANCONx registers to control this, which is different from other devices which
  // use the ADCON1 register for this purpose.
  WDTCONbits.ADSHR = 1;     // Select alternate SFR location to access ANCONx registers
  ANCON0 = 0xFF;            // Default all pins to digital
  ANCON1 = 0xFF;            // Default all pins to digital
  WDTCONbits.ADSHR = 0;     // Select normal SFR locations
#elif defined(BOARD_EBB_V11) || defined(BOARD_EBB_V12) || defined(BOARD_EBB_V13_AND_ABOVE)
  unsigned int pll_startup_counter; //Used for software delay while PLL is starting up

  // Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
  // use the ANCONx registers to control this, which is different from other devices which
  // use the ADCON1 register for this purpose.
  ANCON0 = 0xFF;                  // Default all pins to digital
  ANCON1 = 0xFF;                  // Default all pins to digital

  OSCCON = 0x60;                  // Clock switch to primary clock source.  May not have been running
                                  // from this if the bootloader is called from the application firmware.

  // On the PIC18F46J50 Family of USB microcontrollers, the PLL will not power up and be enabled
  // by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
  // This allows the device to power up at a lower initial operating frequency, which can be
  // advantageous when powered from a source which is not guaranteed to be adequate for 48MHz
  // operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
  // power up the PLL.
  #if defined(__18F24J50)||defined(__18F25J50) || \
      defined(__18F26J50)||defined(__18F44J50) || \
      defined(__18F45J50)||defined(__18F46J50) 

  OSCTUNEbits.PLLEN = 1;  // Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
  pll_startup_counter = 600;
  while(pll_startup_counter--)
    ;
  // Device switches over automatically to PLL output after PLL is locked and ready.
  #else
    #error Double Click this message.  Please make sure the InitializeSystem() function correctly configures your hardware platform.  
    // Also make sure the correct board is selected in usbcfg.h.  If 
    // everything is correct, comment out the above "#error ..." line
    // to suppress the error message.
  #endif
#endif

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
  //  HardwareProfile.h file.    
#if defined(USE_USB_BUS_SENSE_IO)
  tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
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
  //  is used for this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
  //  has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
  //  to it in HardwareProfile.h.
#if defined(USE_SELF_POWER_SENSE_IO)
  tris_self_power = INPUT_PIN;  // See HardwareProfile.h
#endif
  UserInit();

  USBDeviceInit();  // usb_device.c.  Initializes USB module SFRs and firmware
                    // variables to known states.
}

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
  // Example power saving code.  Insert appropriate code here for the desired
  // application behavior.  If the microcontroller will be put to sleep, a
  // process similar to that shown below may be used:

  // ConfigureIOPinsForLowPower();
  // SaveStateOfAllInterruptEnableBits();
  // DisableAllInterruptEnableBits();
  // EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro(); // should enable at least USBActivityIF as a wake source
  // Sleep();
  // RestoreStateOfAllPreviouslySavedInterruptEnableBits();  // Preferably, this should be done in the USBCBWakeFromSuspend() function instead.
  // RestoreIOPinsToNormal();                                // Preferably, this should be done in the USBCBWakeFromSuspend() function instead.

  // IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
  // cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
  // things to not work as intended.

#if defined(__C30__)
  USBSleepOnSuspend();
#endif
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *                  suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *                  mode, the host may wake the device back up by sending non-
 *                  idle state signaling.
 *
 *                  This call back is invoked when a wakeup from USB suspend 
 *                  is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
  // If clock switching or other power savings measures were taken when
  // executing the USBCBSuspend() function, now would be a good time to
  // switch back to normal full power run mode conditions.  The host allows
  // a few milliseconds of wakeup time, after which the device must be 
  // fully back to normal, and capable of receiving and processing USB
  // packets.  In order to do this, the USB module must receive proper
  // clocking (IE: 48MHz clock must be available to SIE for full speed USB
  // operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
  // No need to clear UIRbits.SOFIF to 0 here.
  // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
  // No need to clear UEIR to 0 here.
  // Callback caller is already doing that.

  // Typically, user firmware does not need to do anything special
  // if a USB error occurs.  For example, if the host sends an OUT
  // packet to your device, but the packet gets corrupted (ex:
  // because of a bad connection, or the user unplugs the
  // USB cable during the transmission) this will typically set
  // one or more USB error interrupt flags.  Nothing specific
  // needs to be done however, since the SIE will automatically
  // send a "NAK" packet to the host.  In response to this, the
  // host will normally retry to send the packet again, and no
  // data loss occurs.  The system will typically recover
  // automatically, without the need for application firmware
  // intervention.

  // Nevertheless, this callback function is provided, such as
  // for debugging purposes.
}

/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 *                  firmware must process the request and respond
 *                  appropriately to fulfill the request.  Some of
 *                  the SETUP packets will be for standard
 *                  USB "chapter 9" (as in, fulfilling chapter 9 of
 *                  the official USB specifications) requests, while
 *                  others may be specific to the USB device class
 *                  that is being implemented.  For example, a HID
 *                  class device needs to be able to respond to
 *                  "GET REPORT" type of requests.  This
 *                  is not a standard USB chapter 9 request, and 
 *                  therefore not handled by usb_device.c.  Instead
 *                  this request should be handled by class specific 
 *                  firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
  USBCheckCDCRequest();
}

/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *                  called when a SETUP, bRequest: SET_DESCRIPTOR request
 *                  arrives.  Typically SET_DESCRIPTOR requests are
 *                  not used in most applications, and it is
 *                  optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
  // Must claim session ownership if supporting this request
}

/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 *                  SET_CONFIGURATION (wValue not = 0) request.  This 
 *                  callback function should initialize the endpoints 
 *                  for the device's usage according to the current 
 *                  configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
  CDCInitEP();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 *                  peripheral devices to wake up a host PC (such
 *                  as if it is in a low power suspend to RAM state).
 *                  This can be a very useful feature in some
 *                  USB applications, such as an Infrared remote
 *                  control receiver.  If a user presses the "power"
 *                  button on a remote control, it is nice that the
 *                  IR receiver can detect this signaling, and then
 *                  send a USB "command" to the PC to wake up.
 *
 *                  The USBCBSendResume() "callback" function is used
 *                  to send this special USB signaling which wakes 
 *                  up the PC.  This function may be called by
 *                  application firmware to wake up the PC.  This
 *                  function will only be able to wake up the host if
 *                  all of the below are true:
 *
 *                  1.  The USB driver used on the host PC supports
 *                      the remote wakeup capability.
 *                  2.  The USB configuration descriptor indicates
 *                      the device is remote wakeup capable in the
 *                      bmAttributes field.
 *                  3.  The USB host PC is currently sleeping,
 *                      and has previously sent your device a SET 
 *                      FEATURE setup packet which "armed" the
 *                      remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signaling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *                  This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
  static WORD delay_count;

  // First verify that the host has armed us to perform remote wakeup.
  // It does this by sending a SET_FEATURE request to enable remote wakeup,
  // usually just before the host goes to standby mode (note: it will only
  // send this SET_FEATURE request if the configuration descriptor declares
  // the device as remote wakeup capable, AND, if the feature is enabled
  // on the host (ex: on Windows based hosts, in the device manager 
  // properties page for the USB device, power management tab, the 
  // "Allow this device to bring the computer out of standby." check box 
  // should be checked).
  if (USBGetRemoteWakeupStatus() == TRUE) 
  {
    // Verify that the USB bus is in fact suspended, before we send
    // remote wakeup signaling.
    if (USBIsBusSuspended() == TRUE)
    {
      USBMaskInterrupts();

      // Clock switch to settings consistent with normal USB operation.
      USBCBWakeFromSuspend();
      USBSuspendControl = 0; 
      USBBusIsSuspended = FALSE;  // So we don't execute this code again, 
                                  // until a new suspend condition is detected.

      // Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
      // device must continuously see 5ms+ of idle on the bus, before it sends
      // remote wakeup signaling.  One way to be certain that this parameter
      // gets met, is to add a 2ms+ blocking delay here (2ms plus at 
      // least 3ms from bus idle to USBIsBusSuspended() == TRUE, yields
      // 5ms+ total delay since start of idle).
      delay_count = 3600U;
      do
      {
        delay_count--;
      }while(delay_count);

      // Now drive the resume K-state signaling onto the USB bus.
      USBResumeControl = 1;       // Start RESUME signaling
      delay_count = 1800U;        // Set RESUME line for 1-13 ms
      do
      {
        delay_count--;
      }while(delay_count);
      USBResumeControl = 0;       // Finished driving resume signaling

      USBUnmaskInterrupts();
    }
  }
}

/*******************************************************************
 * Function:        void USBCBEP0DataReceived(void)
 *
 * PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
 *                  defined already (in usb_config.h)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called whenever a EP0 data
 *                  packet is received.  This gives the user (and
 *                  thus the various class examples a way to get
 *                  data that is received via the control endpoint.
 *                  This function needs to be used in conjunction
 *                  with the USBCBCheckOtherReq() function since 
 *                  the USBCBCheckOtherReq() function is the application's
 *                  method for getting the initial control transfer
 *                  before the data arrives.
 *
 * Note:            None
 *******************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occurred.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
  switch(event)
  {
    case EVENT_TRANSFER:
      // Add application specific callback task or callback function here if desired.
      break;
    case EVENT_SOF:
      USBCB_SOF_Handler();
      break;
    case EVENT_SUSPEND:
      USBCBSuspend();
      break;
    case EVENT_RESUME:
      USBCBWakeFromSuspend();
      break;
    case EVENT_CONFIGURED: 
      USBCBInitEP();
      break;
    case EVENT_SET_DESCRIPTOR:
      USBCBStdSetDscHandler();
      break;
    case EVENT_EP0_REQUEST:
      USBCBCheckOtherReq();
      break;
    case EVENT_BUS_ERROR:
      USBCBErrorHandler();
      break;
    case EVENT_TRANSFER_TERMINATED:
      // Add application specific callback task or callback function here if desired.
      // The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
      // FEATURE (endpoint halt) request on an application endpoint which was 
      // previously armed (UOWN was = 1).  Here would be a good place to:
      // 1.  Determine which endpoint the transaction that just got terminated was 
      //     on, by checking the handle value in the *pdata.
      // 2.  Re-arm the endpoint if desired (typically would be the case for OUT 
      //     endpoints).
      break;
    default:
      break;
  }
  return TRUE; 
}

/** EOF main.c ***************************************************************/
