/*********************************************************************
 *
 *                EiBotBoard Firmware
 *
 *********************************************************************
 * FileName:        ebb.c
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
// Versions:
// 1.8 - 
// 1.8.1 5/19/10 - Only change is to recompile with Microchip USB Stack v2.7
// 1.8.2 5/31/10 - Only change is to change name in USB enumeration string to Ei
//                  Bot Board - using new PID for SchmalzHaus
// 1.9   6/11/10 - Added two commands:
//                  SQ - Solenoid Query - returns 0 or 1 for down and up
//                  ST - Solenoid Toggle - toggles state of the servo/solenoid
// 1.9.2 6/15/10 - Added commands:
//                  SC,11 sets pen up speed
//                  SC,12 sets pen down speed
//                  SL - sets the current layer
//                  QL - queries the current layer
//                  SN - sets move (node) count
//                  QN - Query node count
//                  QB - Query Button command
// 1.9.3 6/16/10 - Replaced SN with CL (Clear Node) command
// 1.9.4 6/22/10 - Node Count now incremented on pauses (SM with zero step size)
//                  as well
// 1.9.5 7/2/10 - Node count no longer incremented at all except for NI command
//                  NI - Node count Increment
//                  ND - Node count Decrement
//                  SN - Set Node count (with 8 byte variable)
//                  BL - With latest bootloader, will jump to Boot Load mode
// 1.9.6 7/3/10 - Removed extra vectors below 0x1000 for easier merging of HEX
//                      files
//                - use c018i_HID_BL.o now
// 2.0.0 9/9/10 - Add in
//                  QC - Query Current - reads voltage of current adjustment pot
//                  NOTE: This is NOT done the 'right way'. Instead, we set
//                      up the pin for analog input at boot, then when the QC
//                      comes in, we activate the ADC and take one reading and
//                      then shut it down. Eventually, we should re-write the
//                      'UBW' ADC routines to work with the much more flexible
//                      ADC in the 46J50 part and then just use that generic
//                      code for reading the value of the pot.
//                  SC,13,{0,1} - enables/disables RB0 as another PRG button for
//                      pause detection
// 2.0.1 9/13/10 - Bug fix - on v1.1 EBB hardware, need to disable RB0 alt pause
//                      button. Switched it to RB2 on v1.1 hardware
// 2.0.2 10/3/10 - Bug fix - QC command not returning proper results - added
//                      cast and now works OK
// 2.1.0 10/21/10- Added in
//                  SE - Set Engraver - turns engraver (on RB3) on or off, or
//                      set to PWM power level
//                    Added code in init to pre-charge RC7 (USB_SENSE_IO) high
//                      before running rest of code
//                      to get around wrong resistor value on hardware.
// 2.1.1 11/21/10- Removed Microchip USB stack v2.7, replaced it with v2.8 from 
//                  MAL 2010_10_19.
//                  Also using generic Microchip folder now rather than re-named
//                      one (simpler to update).
//                  Updated code in main.c (and others) to match updates from 
//                      latest MAL CDC example.
// 2.1.1cTest1 01/17/11 - Added third parameter to SP command to use any PortB 
//                      pin for servo output.
//                  For this version only - used PortB2 as standard servo output
// 2.1.1d 02/11/11 - Reverted back to RB1 for servo output
//                 - Updated check_and_send_TX_data() to allow unlimited data to
//                      go out without overrunning the output buffer, same as
//                      UBW 1.4.7.
// 2.1.2 11/04/11 - Fixed PI command to return just a 0 or a 1
//                - Updated to USB stack 2.9a
//                - Created MPLAB X project for this firmware
//                - Added SC,14,<state> to enable/disable solenoid output on RB4
//                - Fixed bug with S2 command and solenoid command interaction -
//                      we now turn off solenoid output on RB4 if user uses S2
//                      command to use RB4 for RC servo output.
//                - Fixed bug with S2 command where a duration of 0 would not
//                      shut off the PWM channel
//                - Fixed bug in S2 command where <rate> variable was not being
//                      used correctly
//                - Switched default number of S2 channels to 8 (from 7 before)
// 2.1.3 12/12/11 - RB3 now defaults to digital I/O on boot, can still use SE
//                      command to do PWM later if you want
//                - Compiled with latest UBW stack - 2.9b from MAL 2011-10-18
// 2.1.4 12/14/11 - RB3 now defaults to OFF, rather than ON, at boot.
// 2.1.5 12/15/11 - Fixed problem with pen servo (RB1) being inverted on boot
// 2.2.0 11/07/12 - Fixed problem with SP command not working properly with 
//                      ports other than RB1 because we don't properly use S2
//                      commands for SP up/down within ISR. Tested on all PortB.
// 2.2.1 09/19/13 - Expanded internal delay counter to 32 bits so we can have
//                      delays longer than 2.1s. Now up to 64K ms.
//                - Fixed bug with all <duration> parameters, SP, TP
//                      commands. Now in ms, defaults to 500ms, and actually
//                      works up to 64Kms.
//                - Fixed uninitialized data bug with command FIFO. We were seeing
//                      very long random delays first time SM,<delay>,0,0 was
//                      used.
//                - SP command was executing servo move at end of <duration>. It
//                      now starts servo move and <duration> delay at same time.
//                - Updated USB stack to Microchip MAL USB v2.9j
// 2.2.2 10/15/13 - Fixed bug with SE command that was preventing anything other
//                  than 50% duty cycle from working.
//                - Updated SC,2,{0,1,2} to control PIC and drivers connection
//                  0 = PIC controls built in drivers
//                  1 = PIC controls external step/dir/en drives
//                  2 = external step/dir/en controls built-in drivers
//                - Also updated the pins that are used for driving external
//                  step/dir/en drives. (See documentation)
//                - Updated SC,1,{0,1,2} documentation to match code and removed
//                  SC,14 which is not needed.
//                - Updated logic for EM command to use state of SC,2,{0,1,2}
//                  properly.
//                - Added SC,14,{0,1} to switch between default of 1/25Khz units
//                  for SP and TP command <duration> parameters, and 1ms units
// 2.2.3 01/11/14 - Rewrote analog system so we don't have problems with QC
//                  commands anymore. New command AC.
// 2.2.4 04/26/14 - Fixed bug where 0 for duration in SM would cause problems
//                - Found bug where the <servo_min> and <servo_max> values were
//                  reversed in the SP command. This has now been fixed so that
//                  SP commands will operate the same as 2.0.1 version.
//                - Set initial  'position' of main servo to be 1mS to mimic
//                  behavior of v2.0.1 firmware.
//                - Updated license to BSD 3-clause
//                - Changed SP and TP commands so that if no <duration> parameter
//                  is used, it does not default to 500mS delay, but rather 0mS.
//                  This should now work exactly as 2.0.1 when no parameter is
//                  used.
//                - Tested 2.2.4 against 2.0.1 with Saleae Logic analyzer. Looked
//                  at several Inkscape plots. Confirmed that timing of steppers
//                  and servo are the same. Confirmed that all RB0 through RB7
//                  outputs are the same between the two versions.
// 2.2.5 04/29/14 - Added 'long' arguments to SM for <move_duration> and <axis1>
//                  and <axis2>. All can be 3 bytes now. Also added checks in
//                  SM command to make sure that arguments don't result in a
//                  step speed that's too low (<0.76Hz).
// 2.2.6 01/01/15 - Added 'QM' command - Query Motor, tells PC what is moving.
// 2.2.7 08/13/15 - Added 'ES' command, which will immediately abort any SM command
//                  but leave the motors energized. (E-stop) It returns "1" if
//                  it aborted a move in progress, "0" otherwise. It will also 
//                  delete any pending SM command in the FIFO.
// 2.2.8 08/14/15 - Corrected error checking in SM command to accurately reflect
//                  too fast and too slow requests. (>25K or <1.31 steps per
//                  second). Also added type cast that now allows for full range
//                  of step values to work properly.
// 2.2.9 08/18/15 - Added extra values to output of ES command to indicate how 
//                  many steps were aborted.
// 2.3.0 08/28/15 - Added new XM command as per issue #29 for driving mixed-axis
//                  geometry machines.
// 2.4.0 03/14/16 - Added new AM command for using accelerated
//                  stepper motion. Includes going to 32 bit accumulators in ISR
//                  to achieve necessary resolution, which includes changes to
//                  SM command as well. Also added "CU,2,0" to turn of SM command
//                  parameter checking for speed. 
//                  NOTE: AM command may not be quite 'right' yet. Although it 
//                  has passed simple tests, it could not be made to reliably
//                  work from the Inkscape plugin, so it is not currently being
//                  used. It may have to change in future versions of this
//                  firmware.
// 2.4.1 08/08/16 - Added new form of SE command, with optional parameter that
//                  puts SE in motion queue. (issue #51)
//                  Fixed issue #52 (bug in parameter check in parse_SM_packet())
// 2.4.2 08/10/16 - Fixed bug in SE command that would set engraver to 50% if
//                  SE,1,0 was used. Also added engraver power to FIFO structure
//                  for when third SE parameter is 1.
// 2.4.3 11/07/16 - Added QS (Query Step position) and CS (Clear Step position)
//                  commands.
// 2.4.4 11/16/16 - Added extra value to QM command to show status of FIFO
// 2.4.5 01/07/17 - Fixed math error in SM/XM commands (see issue #71)
// 2.4.6 01/08/17 - Added special case code for moves less than 30ms long (this
//                  special case just un-does the change for issue #71 for these
//                  short moves)
// 2.5.0 01/09/17 - Added LM (low level move) command to allow PC to do all math
// 2.5.1 01/18/17 - Fixed LM command to take negative and positive Accel
//                  Fixed 25KHz ISR to be much closer to 25KHz
// 2.5.2 07/07/17 - Fixed issue #78 : detected and reject 0,0 for LM command
//                  Fixed some uninitialized variables
//                  LM Accel parameter went to 32 bits signed from 16 bit signed
// 2.5.3 07/09/17 - Fixed bug in LM command that would corrupt currently running
//                    moves with new data.
// 2.5.4 01/06/18 - Added ST (Set Tag), QT (Query Tag) commands.
//                  Added RB (ReBoot) command
//                  Added any name set with ST command to end of USB Device Name
//                  Fixed problem with reading bit 7 of any I/O port (issue #82)
// 2.5.5 07/06/18 - Enhanced ST command by writing to both Device name and 
//                    serial number USB fields.
// 2.5.6 07/11/18 - Added RC Servo power control functionality on pin RA3
//                    Includes new QR and SR commands. See issue #103 for more
// 2.6.0 09/08/18 - Added direct servo power toggle command SR
//                  Set servo default timeout to 15 minutes, pen up at boot,
//                    servo power off at boot
//                  Changed default pen up/down positions
// 2.6.1 01/07/19 - Added "QG" general query command
// 2.6.2 01/11/19 - Added "HM" Home Motor command
// 2.6.3 05/24/19 - Changed default RC servo power down time from 15min to 60s
// 2.6.4 11/05/19 - Fixed bug #124 (Math error in SM command for edge case 
//                    input parameters)
// 2.6.5 11/29/19 - Changed SR command behavior so it only enables servo power
//                    after SP command, not also after stepper movement
// 2.6.6 05/26/20 - Fixed a bug with the PC/PG command which caused some changes
//                    not to take effect for up to 64K more milliseconds.
//       11/10/20 - Fixed bug where S2 command wouldn't turn power on to RB1
//                    servo output if it had been turned off.
// 2.7.0 11/19/20 - No longer allow Rate to go negative (like from LM command) 
//                    inside ISR
//                  Removed AM command
//                  Added optional parameter <Clear> to all stepper motion 
//                    commands to allow explicit zeroing of accumulators
//                  Renamed internal parameters for some commands (LT/LM) for
//                    clarity. Also queue structure members got renamed.
//                  Added optional absolute position values to HM command
//                  Added LT command, based on LM command
//                  Updated math in LM command based on Kinematics analysis
//                  Updated math in ISR (for LM/LT) based on Kinematics analysis
//                  Fixed bug where negative accelerations could cause delays before 
//                    last step.
//                  EM command now always clears accumulators
//                  Reduced effective pulse width for step pulses down to
//                    between 1.6 and 2.3 uS.
// 2.8.0 04/22/21 - Issue 151: Added QE command to return state of motor enables
//                    and their microstep resolution
//                  Issue 152: Move EM command to motion queue
//                  Issue 153: Add optional parameter to ES command to disable
//                    motors.
//                  Fix bug in ES command that didn't send return packet
// 2.8.1 07/26/22 - Issue 180: Add CU,3,1 to turn on RED LED reporting of FIFO empty
// 2.8.2 05/05/23 - Add CU,50 to control automatic motor enable
//
// 2.9.0 11/2/22  - Issue 185 : Get LT and LM commands to allow accelerations
//                    through zero (i.e. reverse direction mid-move)
//                  Massive refactor of motion ISR to improve performance
//                  Proper formatting/indenting of entire EBB codebase for readability
//                  Turned all warnings/messages on in compiler, fixed all

#include <p18cxxx.h>
#include <usart.h>
#include <ctype.h>
#include <delays.h>
#include <math.h>
#include "Usb\usb.h"
#include "Usb\usb_function_cdc.h"
#include "usb_config.h"
#include "HardwareProfile.h"
#include "ubw.h"
#include "ebb.h"
#include "delays.h"
#include "ebb_demo.h"
#include "RCServo2.h"
#include "ebb_print.h"

// Define this to enable the use of the C step (high) ISR. Undefine for
// the assembly language version
#define USE_C_ISR

// This is the value that gets multiplied by Steps/Duration to compute
// the StepAdd values.
#define OVERFLOW_MUL            (0x8000 / HIGH_ISR_TICKS_PER_MS)

#define MAX_RC_DURATION         11890

// The number of individual addressable unsigned bytes that are available
// in the SL and QL commands.
#define SL_STORAGE_SIZE         32u

// This is the FIFO that stores the motion commands. It spans multiple RAM
// banks from 0x600 through 0xDFF (length 0x800 or 2048d), and must only be 
// accessed via pointer.
// Note that no matter what COMMAND_FIFO_MAX_LENGTH is set to, we reserve the
// entire 0x800 bytes of space in the FIFO_scn here. That prevents the linker
// from putting variables after this section.
#pragma udata FIFO_scn
UINT8 CommandFIFO[COMMAND_FIFO_SIZE_BYTES];

// These global variables are deliberately put into "Bank" 0 of RAM (the
// access/near bank).
// They are the small global variables that the 25KHz ISR has to access.
// With them all being in bank 0, no bank switch instructions are needed
// to access them.
#pragma udata access ISR_access

//static UINT8 TookStep;       // LSb set if a step was taken
static near UINT8 AllDone;        // LSb set if this command is complete
static near UINT8 isr_i;
static near UINT8 AxisActive[NUMBER_OF_STEPPERS];     // LSb set if an axis is not done stepping

near DriverConfigurationType DriverConfiguration;
// LSb set to enable RC Servo output for pen up/down
near UINT8 gUseRCPenServo;
// LSb set to enable red LED lit when FIFO is empty
volatile near UINT8 gRedLEDEmptyFIFO;
// LSb set when user presses the PRG or alternate PRG button
static volatile near UINT8 ButtonPushed;
// LSb set to enable use of alternate PRG (pause) button
static volatile near UINT8 UseAltPause;
// Bitfield used to keep track of what test modes are enabled/disabled
volatile near UINT8 TestMode;

// Used as temporary variable inside ISR for servo channel index
static near UINT8 ISR_Channel;

// Temporary holder for PortB value in ISR so we don't have to sample more than once
static near UINT8 PortBTemp;

// Global variables used for limit switch feature
volatile near UINT8 gLimitSwitchMask;      // 8-bit PortB mask
volatile near UINT8 gLimitSwitchTarget;    // 8-bit PortB target values
volatile near UINT8 gLimitSwitchTriggered; // Non-zero if limit switch trigger has fired

// Global variables for managing FIFO depth/indexes
volatile near UINT8 gFIFOLength;
volatile near UINT8 gFIFOIn;
volatile near UINT8 gFIFOOut;

// Holds a local copy of the Command from CommandFIFO[gFIFOOut].Command 
static near UINT8 gFIFOCommand;

// Current length of FIFO
volatile near UINT8 gCurrentFIFOLength;

// Temporarily store FSR0 during command copy assembly (note these can be in any
// bank)
//UINT8 near isr_FSR0L_temp;
//UINT8 near isr_FSR0H_temp;

// Pointer in RAM to the first byte of the next unplayed FIFO command
UINT8 near FIFO_out_ptr_high;
UINT8 near FIFO_out_ptr_low;

// These values hold the global step position of each axis
volatile static near INT32 globalStepCounter1;
volatile static near INT32 globalStepCounter2;

// Pointer to a MoveCommandType element of the FIFO array
near MoveCommandType * FIFOPtr = (near MoveCommandType *)&CommandFIFO[0];

// Accumulator for each axis
near u32b4_t acc_union[2];

// ISR globals used in test modes for keeping track of each move
static near UINT32 gISRTickCountForThisCommand;
static near UINT32 gISRStepCount0ForThisCommand;
static near INT32  gISRPosition0ForThisCommand;
static near UINT32 gISRStepCount1ForThisCommand;
static near INT32  gISRPosition1ForThisCommand;

// Bank 1 is the "ISR Bank". By placing all variables that the ISR needs
// access to either in the access bank (above) or in Bank 1, then the bank 
// select register can be kept pointing to Bank 1 and no bank switch instructions
// are needed in the iSR.
#pragma udata ISR_globals = 0x180

// Variables used during debug printing within ISR
static u32b4_t xx;
static UINT8 nib;

// The move command containing the currently executing move command in the ISR
MoveCommandType CurrentCommand;

unsigned int DemoModeActive;
SolenoidStateType SolenoidState;
static unsigned int SolenoidDelay;

// track the latest state of the pen
PenStateType PenState;

static unsigned long NodeCount;
unsigned char QC_ms_timer;
static UINT StoredEngraverPower;
// Set TRUE to enable solenoid output for pen up/down
BOOL gUseSolenoid;
// When FALSE, we skip parameter checks for motor move commands so they can run faster
BOOL gLimitChecks;

// LSb set to enable all strings to use '\n' line endings, and use standardized
// command reply format
UINT8 gStandardizedCommandFormat;

// Temp storage in ISR for late ISR compensation value
UINT8 CurTime;

// These globals replace the local (stack) variables used during processing
// and parsing of stepper motor commands. They also take the place of parameters
// for the two low-level (process_low_level and process_simple) move functions.
// Since we know that we can never be working on more than one of these commands
// at the same time, and since we have enough global RAM space, it makes some
// sense to make these static globals. This reduces the change of a stack
// overflow.

static UINT32  gUInt32_1;
static UINT32  gUInt32_2;
static UINT32  gUInt32_3;
static INT32   gInt32_4;
static INT32   gInt32_5;
static INT32   gInt32_6;
static INT32   gInt32_7;
static INT32   gInt32_8;
static INT32   gInt32_9;
static INT32   gInt32_10;
static INT32   gInt32_11;

// These next set of defines create local, meaningful names. They reference the
// above globals, but in ways that make sense for the routine they are used in.

// For HM
#define gHM_StepRate    gUInt32_2
#define gHM_Pos1        gInt32_6
#define gHM_Pos2        gInt32_10

// For XM
#define gXM_ASteps      gInt32_6
#define gXM_BSteps      gInt32_10

// For simple_motor_move_fp
#define gDurationMS     gUInt32_1
#define gIntervals      gUInt32_2
#define gClearAccs      gUInt32_3
#define gRate1          gInt32_4
#define gSteps1         gInt32_5
#define gAccel1         gInt32_6
#define gJerk1          gInt32_7
#define gRate2          gInt32_8
#define gSteps2         gInt32_9
#define gAccel2         gInt32_10
#define gJerk2          gInt32_11

// For simple_rate_move_fp()
#define gSRM_Rate       gUInt32_2

/* List of functions which use (in some way) the above globals/defines

process_simple_motor_move_fp()
  Inputs:
    gDurationMS   UINT32  gUInt32_1 not modified
    gSteps1       INT32   gInt32_5  modified
    gSteps2       INT32   gInt32_9  modified
    gClearAccs    UINT32  gUInt32_3 not modified
  Working values:
    gIntervals    UINT32  gUInt32_2 modified

process_simple_rate_move_fp()
  Inputs:
    gSRM_Rate     UINT32  gUInt32_2 not modified
    gSteps1       INT32   gInt32_5  modified
    gSteps2       INT32   gInt32_9  modified
    gClearAccs    UINT32  gUInt32_3 not modified
  Working values:
    (none)

parse_XM_packet()
  Working values:
    gDurationMS   UINT32  gUInt32_1 modified
    gSteps1       INT32   gInt32_5  modified
    gSteps2       INT32   gInt32_9  modified
    gClearAccs    UINT32  gUInt32_3 modified
  Working values:
    gXM_ASteps    INT32   gInt32_6  modified
    gXM_BSteps    INT32   gInt32_10 modified
  
*/

// These globals are now set to be put anywhere the linker can find space for them
#pragma udata

static MoveCommandType gMoveTemp;     // Commands fill this then copy to FIFO

// Storage for the 32 bytes of "SL" command values
static UINT8   gSL_Storage[32];

// Local function definitions
static void clear_StepCounters(void);
static void process_low_level_move(BOOL TimedMove,  ExtractReturnType ClearRet);
static void process_simple_motor_move_fp(void);
static void clear_parmaeter_globals(void);
static void process_simple_rate_move_fp(void);


extern void FIFO_COPY(void);

// High ISR
#pragma interrupt high_ISR
void high_ISR(void)
{
  // 25KHz ISR fire. Note: For speed, we don't check that INTCONbits.TMR0IF is set
  // here. We assume it is, as it's the only interrupt that should be triggering
  // calls to high_ISR()
  
  // Clear the interrupt 
  INTCONbits.TMR0IF = 0;
  
  // Load the current value of the timer (it will be above zero) so we know
  // how long it took from when the interrupt fired until we actually 
  // serviced the interrupted. 
  CurTime = TMR0L;

  // Because the timer is a 'count up' timer, and because it generates an 
  // interrupt when it rolls over at 0x00, our TIMER0_RELOAD value is effectively
  // a negative value. By adding CurTime to that reload value, we decrease the
  // amount of time we want to wait until the next ISR fires by the exact 
  // amount that we were delayed by in getting to this ISR tick. Thus any
  // 'lateness' in this ISR tick is compensated for in the next tick, keeping
  // an overall steady 40uS between ticks even if one or two ISRs take too long
  // and delay the start of the next tick by some amount.
  TMR0L = TIMER0_RELOAD + CurTime;  // Reload for 25KHz ISR fire

  if (bittst(TestMode, TEST_MODE_GPIO_NUM))
  {
    LATDbits.LATD1 = 1;
  }
  
  bitsetzero(AllDone);      // Start every ISR assuming we are done with the current command - set bit 0 of AllDone

  // Process a motor move command of any type
  // This is the main chunk of code for EBB : the step generation code in the 25KHz ISR.
  // The first section determines if we need to take any steps this time through the ISR.
  // It is broken into sections, one for each type of stepper motion command because 
  // they each have different amounts of processing needed to determine if a step is 
  // necessary or not.
  // Then the second section is common to all stepper motion commands and handles
  // the actual step pulse generation as well as direction bit control.

  // The Active bits will be set if there is still motion left to 
  // generate on an axis. They are set when the command is first loaded
  // into CurrentCommand and then cleared once all steps have been taken
  // (either in time or in step count). They are checked inside each
  // command's step processing (below) so that we only spend time working
  // on axis that have activity left

  // Important assumptions:
  // There is only one bit set in the CurrentCommand.Command byte
  
  // A very complete mathematical analysis of the way the various stepper 
  // commands are expected to work can be found here;
  // https://evilmadscience.s3.amazonaws.com/dl/ad/public/AxiDrawKinematics.pdf

  // Here we handle the 'simple' (non accelerating) stepper commands.
  // These three command (SM, XM and HM) do not use
  // acceleration and so that code is left out to save time. This is also
  // the most common command used by the various PC softwares so we check 
  // for it first.
  if (CurrentCommand.Command == COMMAND_SM_XM_HM_MOVE)
  {
    // Direction bits need to be output before step bits. Since we know step bis
    // are clear at this point, it's safe to always output direction bits here
    // before we start messing with the step bits
    if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
    {
      // The Step and Direction bits are all output on the top four bits
      // of PortD. So we can be very efficient here and simply output those
      // four bits directly to port D.
      LATD = (PORTD & ~(DIR1_BIT | DIR2_BIT)) | CurrentCommand.m.sm.DirBits;
    }

    // If the test mode is active, count this tick as part of the move if there
    // are steps to take in either axis. 
    if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
    {
      if (bittstzero(AxisActive[0]) || bittstzero(AxisActive[1]))
      {
        gISRTickCountForThisCommand++;
      }
    }
    
    //// MOTOR 1     SM XM HM ////
    
    // Only do this if there are steps left to take
    if (bittstzero(AxisActive[0]))
    {
      // Add the rate to the accumulator and see if the MSb got set. If so
      // then take a step and record that the step was taken
      acc_union[0].value += CurrentCommand.m.sm.Rate[0].value;
      if (acc_union[0].bytes.b4 & 0x80)
      {
        acc_union[0].bytes.b4 &= 0x7F;
        CurrentCommand.m.sm.DirBits |= STEP1_BIT;
        CurrentCommand.m.sm.Steps[0]--;

        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRStepCount0ForThisCommand++;
        }

        // For these stepper motion commands zero steps left means
        // the axis is no longer active
        if (CurrentCommand.m.sm.Steps[0] == 0u)
        {
          bitclrzero(AxisActive[0]);
        }
      }
    }

    //// MOTOR 2     SM XM HM ////

    if (bittstzero(AxisActive[1]))
    {
      acc_union[1].value += CurrentCommand.m.sm.Rate[1].value;
      if (acc_union[1].bytes.b4 & 0x80)
      {
        acc_union[1].bytes.b4 &= 0x7F;
        CurrentCommand.m.sm.DirBits |= STEP2_BIT;
        CurrentCommand.m.sm.Steps[1]--;

        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRStepCount1ForThisCommand++;
        }

        if (CurrentCommand.m.sm.Steps[1] == 0u)
        {
          bitclrzero(AxisActive[1]);
        }
      }
    }

    // We want to allow for a one-ISR tick move, which requires us to check
    // to see if the move has been completed here (to load the next command
    // this tick rather than waiting for the next tick). This primarily gives
    // us simpler math when figuring out how long moves will take.
    if (bittstzero(AxisActive[0]) || bittstzero(AxisActive[1]))
    {
      bitclrzero(AllDone);
    }
    goto OutputBits;
  }

  // Low level Move (LM) : This one uses acceleration, so take that
  // into account.
  if (CurrentCommand.Command == COMMAND_LM_MOVE)
  {
    // If the test mode is active, count this tick as part of the move if there
    // are steps to take in either axis. 
    if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
    {
      if (bittstzero(AxisActive[0]) || bittstzero(AxisActive[1]))
      {
        gISRTickCountForThisCommand++;
      }
    }

    //// MOTOR 1   LM ////

    // Only do this if there are steps left to take
    if (bittstzero(AxisActive[0]))
    {
      CurrentCommand.m.sm.Accel[0] += CurrentCommand.m.sm.Jerk[0];
      CurrentCommand.m.sm.Rate[0].value += CurrentCommand.m.sm.Accel[0];
      acc_union[0].value += CurrentCommand.m.sm.Rate[0].value;

      if (acc_union[0].bytes.b4 & 0x80)
      {
        acc_union[0].bytes.b4 = acc_union[0].bytes.b4 & 0x7F;
        CurrentCommand.m.sm.DirBits |= STEP1_BIT;
        CurrentCommand.m.sm.Steps[0]--;
        
        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRStepCount0ForThisCommand++;
        }
        // Set the direction bit based on the sign of rate
        if (CurrentCommand.m.sm.Rate[0].bytes.b4 & 0x80)
        {
          CurrentCommand.m.sm.DirBits |= DIR1_BIT;
          Dir1IO = 1;
        }
        else
        {
          CurrentCommand.m.sm.DirBits &= ~DIR1_BIT;
          Dir1IO = 0;
        }

        if (CurrentCommand.m.sm.Steps[0] == 0u)
        {
          bitclrzero(AxisActive[0]);
        }
      }
    }

    //// MOTOR 2    LM ////

    if (bittstzero(AxisActive[1]))
    {
      CurrentCommand.m.sm.Accel[1] += CurrentCommand.m.sm.Jerk[1];
      CurrentCommand.m.sm.Rate[1].value += CurrentCommand.m.sm.Accel[1];
      acc_union[1].value += CurrentCommand.m.sm.Rate[1].value;

      if (acc_union[1].bytes.b4 & 0x80)
      {
        acc_union[1].bytes.b4 = acc_union[1].bytes.b4 & 0x7F;
        CurrentCommand.m.sm.DirBits |= STEP2_BIT;
        CurrentCommand.m.sm.Steps[1]--;

        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRStepCount1ForThisCommand++;
        }

        // Set the direction bit based on the sign of rate
        if (CurrentCommand.m.sm.Rate[1].bytes.b4 & 0x80)
        {
          CurrentCommand.m.sm.DirBits |= DIR2_BIT;
          Dir2IO = 1;
        }
        else
        {
          CurrentCommand.m.sm.DirBits &= ~DIR2_BIT;
          Dir2IO = 0;
        }

        if (CurrentCommand.m.sm.Steps[1] == 0u)
        {
          bitclrzero(AxisActive[1]);
        }
      }
    }

    // We want to allow for a one-ISR tick move, which requires us to check
    // to see if the move has been completed here (to load the next command
    // this tick rather than waiting for the next tick). This primarily gives
    // us simpler math when figuring out how long moves will take.
    if (bittstzero(AxisActive[0]) || bittstzero(AxisActive[1]))
    {
      bitclrzero(AllDone);
    }
    goto OutputBits;
  }

  // The Low level Timed (LT) command is pretty special. Instead of running
  // until all step counts are zero, it runs for a certain amount of 25Khz
  // ISR ticks (stored in .Steps[0]). Both axis continue to produce steps
  // until the time is used up. This affects how AxisActive is computed.
  // Only AxisActive[0] is used for this command, not AxisActive[1].
  if (CurrentCommand.Command == COMMAND_LT_MOVE)
  {
    // Has time run out for this command yet?
    if (bittstzero(AxisActive[0]))
    {
      // If the test mode is active, count this tick as part of the move if there
      // are steps to take. 
      if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
      {
        gISRTickCountForThisCommand++;
      }
      
      // A simple optimization: we only have one 'count' for LT, to know
      // when we're done, so we can directly clear AllDone here if we are
      // not yet done with this move.
      bitclrzero(AllDone);

      // Nope not done. So count this ISR tick, and then see if we need to take 
      // a step
      CurrentCommand.m.sm.Steps[0]--;
      if (CurrentCommand.m.sm.Steps[0] == 0u)
      {
        bitclrzero(AxisActive[0]);
      }

      //// MOTOR 1   LT ////
      
      CurrentCommand.m.sm.Accel[0] += CurrentCommand.m.sm.Jerk[0];
      CurrentCommand.m.sm.Rate[0].value += CurrentCommand.m.sm.Accel[0];
      acc_union[0].value += CurrentCommand.m.sm.Rate[0].value;

      if (acc_union[0].bytes.b4 & 0x80)
      {
        acc_union[0].bytes.b4 = acc_union[0].bytes.b4 & 0x7F;
        CurrentCommand.m.sm.DirBits |= STEP1_BIT;
        
        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRStepCount0ForThisCommand++;
        }
        
        // Set the direction bit based on the sign of rate
        if (CurrentCommand.m.sm.Rate[0].bytes.b4 & 0x80)
        {
          CurrentCommand.m.sm.DirBits |= DIR1_BIT;
          Dir1IO = 1;
        }
        else
        {
          CurrentCommand.m.sm.DirBits &= ~DIR1_BIT;
          Dir1IO = 0;
        }
      }
      
      //// MOTOR 2    LT  ////

      CurrentCommand.m.sm.Accel[1] += CurrentCommand.m.sm.Jerk[1];
      CurrentCommand.m.sm.Rate[1].value += CurrentCommand.m.sm.Accel[1];
      acc_union[1].value += CurrentCommand.m.sm.Rate[1].value;

      if (acc_union[1].bytes.b4 & 0x80)
      {
        acc_union[1].bytes.b4 = acc_union[1].bytes.b4 & 0x7F;
        CurrentCommand.m.sm.DirBits |= STEP2_BIT;

        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRStepCount1ForThisCommand++;
        }

        // Set the direction bit based on the sign of rate
        if (CurrentCommand.m.sm.Rate[1].bytes.b4 & 0x80)
        {
          CurrentCommand.m.sm.DirBits |= DIR2_BIT;
          Dir2IO = 1;
        }
        else
        {
          CurrentCommand.m.sm.DirBits &= ~DIR2_BIT;
          Dir2IO = 0;
        }
      }
    }    
  }

  // This next block of code (OutputBits:) is common to all of the above
  // stepper motion commands. Each of them either jumps here when they 
  // are done or falls through in the case of the last command above.
  // This block sets/clears step and direction GPIO bits if the command that just
  // ran needs to output something new. Also takes care of recording this
  // step properly.
  //
  // NOTE: Even though it seems like a bad idea to have non-stepper
  // commands execute this block before they are caught below, doing it this
  // way saves a few instruction cycles for the commands where speed matters
  // most - the stepper commands.

OutputBits:
  // Now check to see if either stepper needs to actually output a step pulse
  if ((CurrentCommand.m.sm.DirBits & (STEP1_BIT | STEP2_BIT)) != 0u)
  {
    if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
    {
      // The Step and Direction bits are all output on the top four bits
      // of PortD. So we can be very efficient here and simply output those
      // four bits directly to port D.
      LATD = (PORTD & ~(STEP1_BIT | STEP2_BIT | DIR1_BIT | DIR2_BIT)) | CurrentCommand.m.sm.DirBits;
    }
    else
    {
      // Set the DIR Bits
      if (bittst(CurrentCommand.m.sm.DirBits, DIR1_BIT_NUM))
      {
        Dir1AltIO = 1;
      }
      else
      {
        Dir1AltIO = 0;
      }
      if (bittst(CurrentCommand.m.sm.DirBits, DIR2_BIT_NUM))
      {
        Dir2AltIO = 1;
      }
      else
      {
        Dir2AltIO = 0;
      }
      // Set the STEP bits
      if (bittst(CurrentCommand.m.sm.DirBits, STEP1_BIT_NUM))
      {
        Step1AltIO = 1;
      }
      if (bittst(CurrentCommand.m.sm.DirBits, STEP2_BIT_NUM))
      {
        Step2AltIO = 1;
      }
    }

    // This next section not only counts the step(s) we are taking, but
    // also acts as a delay to keep the step bit set for a little while.
    if (bittst(CurrentCommand.m.sm.DirBits, STEP1_BIT_NUM))
    {
      if (bittst(CurrentCommand.m.sm.DirBits, DIR1_BIT_NUM))
      {
        globalStepCounter1--;
        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRPosition0ForThisCommand--;
        }
      }
      else
      {
        globalStepCounter1++;
        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRPosition0ForThisCommand++;
        }
      }
    }
    if (bittst(CurrentCommand.m.sm.DirBits, STEP2_BIT_NUM))
    {
      if (bittst(CurrentCommand.m.sm.DirBits, DIR2_BIT_NUM))
      {
        globalStepCounter2--;
        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRPosition1ForThisCommand--;
        }
      }
      else
      {
        globalStepCounter2++;
        if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
        {
          gISRPosition1ForThisCommand++;
        }
      }
    }
   
    // Clear the two step bits so they're empty for the next pass through ISR
    CurrentCommand.m.sm.DirBits &= ~(STEP1_BIT | STEP2_BIT);
    // We need to skip over all the rest of command checks, since we already
    // handled the command that is currently executing.
    goto CheckForNextCommand;
  }

NonStepperCommands:
  // Now check for all the other (non-stepper based) motion FIFO commands
  if (CurrentCommand.Command == COMMAND_SERVO_MOVE)
  {
    // Check to see if we should change the state of the pen
    if (bittstzero(gUseRCPenServo))
    {
      // Precompute the channel, since we use it all over the place
      ISR_Channel = CurrentCommand.m.sm.ServoChannel - 1;

      // This code below is the meat of the RCServo2_Move() function
      // We have to manually write it in here rather than calling
      // the function because a real function inside the ISR
      // causes the compiler to generate enormous amounts of setup/tear down
      // code and things run way too slowly.

      // If the user is trying to turn off this channel's RC servo output
      if (0u == CurrentCommand.m.sm.ServoPosition)
      {
        // Turn off the PPS routing to the pin
        *(gRC2RPORPtr + gRC2RPn[ISR_Channel]) = 0;
        // Clear everything else out for this channel
        gRC2Rate[ISR_Channel] = 0;
        gRC2Target[ISR_Channel] = 0;
        gRC2RPn[ISR_Channel] = 0;
        gRC2Value[ISR_Channel] = 0;
      }
      else
      {
        // Otherwise, set all of the values that start this RC servo moving
        gRC2Rate[ISR_Channel] = CurrentCommand.m.sm.ServoRate;
        gRC2Target[ISR_Channel] = CurrentCommand.m.sm.ServoPosition;
        gRC2RPn[ISR_Channel] = CurrentCommand.m.sm.ServoRPn;
        if (gRC2Value[ISR_Channel] == 0u)
        {
          gRC2Value[ISR_Channel] = CurrentCommand.m.sm.ServoPosition;
        }
      }
    }

    // If this servo is the pen servo (on g_servo2_RPn)
    if (CurrentCommand.m.sm.ServoRPn == g_servo2_RPn)
    {
      // Then set its new state based on the new position
      if (CurrentCommand.m.sm.ServoPosition == g_servo2_min)
      {
        PenState = PEN_UP;
        SolenoidState = SOLENOID_OFF;
        if (gUseSolenoid)
        {
          PenUpDownIO = 0;
        }
      }
      else
      {
        PenState = PEN_DOWN;
        SolenoidState = SOLENOID_ON;
        if (gUseSolenoid)
        {
          PenUpDownIO = 1;
        }
      }
    }
  }

  if (
    (CurrentCommand.Command == COMMAND_SERVO_MOVE)
    |
    (CurrentCommand.Command == COMMAND_DELAY)
  )
  {
    // NOTE: Intentional fall-through to COMMAND_DELAY here
    // Because the only two commands that ever use DelayCounter are
    // COMMAND_DELAY and COMMAND_SERVO_MOVE. So we handle the servo stuff
    // above, and then let it fall through to the delay code below.

    // Handle a delay
    if (CurrentCommand.m.sm.DelayCounter)
    {
      CurrentCommand.m.sm.DelayCounter--;

      if (CurrentCommand.m.sm.DelayCounter)
      {
        bitclrzero(AllDone);
      }
    }
    goto CheckForNextCommand;
  }

  if (CurrentCommand.Command == COMMAND_SE)
  {
    // Check to see if we should start or stop the engraver
    // Now act on the State of the SE command
    if (CurrentCommand.m.sm.SEState)
    {
      // Set RB3 to StoredEngraverPower
      CCPR1L = CurrentCommand.m.sm.SEPower >> 2;
      CCP1CON = (CCP1CON & 0b11001111) | ((StoredEngraverPower << 4) & 0b00110000);
    }
    else
    {
      // Set RB3 to low by setting PWM duty cycle to zero
      CCPR1L = 0;
      CCP1CON = (CCP1CON & 0b11001111);
    }
    // This is probably unnecessary, but it's critical to be sure to indicate that the current command is finished
    bitsetzero(AllDone);
    goto CheckForNextCommand;
  }

  if (CurrentCommand.Command == COMMAND_EM)
  {
    // As of version 2.8.0, we now have the EM command transplanted here into
    // the motion queue. This is so that changes to motor enable or microstep
    // resolution happen at predictable times rather than randomly.
    // We use CurrentCommand.m.sm.DirBits as "EA1" (the first parameter) and
    // CurrentCommand.m.sm.ServoRPn as "EA2" (the second parameter) since they're
    // both UINT8s.
    if (CurrentCommand.m.sm.DirBits > 0u)
    {
      if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
      {
        Enable1IO = ENABLE_MOTOR;
      }
      else
      {
        Enable1AltIO = ENABLE_MOTOR;
      }
      if (CurrentCommand.m.sm.DirBits == 1u)
      {
        MS1_IO = 1;
        MS2_IO = 1;
        MS3_IO = 1;
      }
      if (CurrentCommand.m.sm.DirBits == 2u)
      {
        MS1_IO = 1;
        MS2_IO = 1;
        MS3_IO = 0;
      }
      if (CurrentCommand.m.sm.DirBits == 3u)
      {
        MS1_IO = 0;
        MS2_IO = 1;
        MS3_IO = 0;
      }
      if (CurrentCommand.m.sm.DirBits == 4u)
      {
        MS1_IO = 1;
        MS2_IO = 0;
        MS3_IO = 0;
      }
      if (CurrentCommand.m.sm.DirBits == 5u)
      {
        MS1_IO = 0;
        MS2_IO = 0;
        MS3_IO = 0;
      }
    }
    else
    {
      if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
      {
        Enable1IO = DISABLE_MOTOR;
      }
      else
      {
        Enable1AltIO = DISABLE_MOTOR;
      }
    }

    if (CurrentCommand.m.sm.ServoRPn > 0u)
    {
      if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
      {
        Enable2IO = ENABLE_MOTOR;
      }
      else
      {
        Enable2AltIO = ENABLE_MOTOR;
      }
    }
    else
    {
      if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
      {
        Enable2IO = DISABLE_MOTOR;
      }
      else
      {
        Enable2AltIO = DISABLE_MOTOR;
      }
    }

    // Always clear the step counts if motors are enabled/disabled or 
    // resolution is changed.
    // NOTE: This section is the same code as clear_StepCounters(), but since
    // we can't have any function calls in an ISR (or the ISR blows up in 
    // space and speed) we have to copy the guts of that function here.

    // Clear out the global step counters
    globalStepCounter1 = 0;
    globalStepCounter2 = 0;

    // Clear both step accumulators as well
    acc_union[0].value = 0;
    acc_union[1].value = 0;

    // All EM command clear the limit switch trigger
    bitclrzero(gLimitSwitchTriggered);

    // This is probably unnecessary, but it's critical to be sure to indicate that the current command is finished
    bitsetzero(AllDone);
  }        
  // If no bits in CurrentCommand.Command are set then AllDone will be true, 
  // so we'll go on to the next command (if there is one)
CheckForNextCommand:
  // Deal with printing out internal ISR values
  // Note that TEST_MODE_USART_ISR_BIT is set for both normal (only end of move)
  // and full (every ISR tick) printing.
  if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
  {
    if (bittstzero(AllDone) || bittst(TestMode, TEST_MODE_USART_ISR_FULL_NUM))
    {
      // If this is the end of an LM move, then print out all the important values
      // of everything (now that global step positions have been taken into
      // account) for checking that our math is working right
      if (
        (CurrentCommand.Command == COMMAND_LM_MOVE)
        | 
        (CurrentCommand.Command == COMMAND_LT_MOVE)
        | 
        (CurrentCommand.Command == COMMAND_SM_XM_HM_MOVE)
      )
      {
        // Setting this bit when we have something to print out will cause the next
        // ISR to be rescheduled so we have as much time as we need to print
        bitset(TestMode, TEST_MODE_PRINT_TRIGGER_NUM);

        // Move is complete, output "T:" (ISR ticks), "Steps:" (total steps from 
        // this move) "C:" (accumulator value), "R:" (rate value), "Pos:" 
        // (step position)

        /// TODO: This could use some MASSIVE cleanup - turn this into a buffered
        /// ISR driven serial output for best efficiency. Or at the very least
        /// make signed and unsigned printing macros.
//#define ISR_DEBUG_PRINT_ASCII 1
#if ISR_DEBUG_PRINT_ASCII
        // Write out the total ISR ticks for this move (unsigned)
        PrintChar('T')
        PrintChar(',')
        HexPrint(gISRTickCountForThisCommand) // Macro for printing HEX value in ASCII
        PrintChar(',')

        // Write out the total steps made during this move (unsigned)
        PrintChar('S')
        PrintChar(',')
        HexPrint(gISRStepCount0ForThisCommand)
        PrintChar(',')

        // Write out the accumulator1 value after all math is complete (unsigned)
        PrintChar('C')
        PrintChar(',')
        HexPrint(acc_union[0].value)
        PrintChar(',')

        // Write out the rate1 value (signed)
        PrintChar('R')
        PrintChar(',')
        HexPrint(CurrentCommand.m.sm.Rate[0].value)
        PrintChar(',')

        // Write out the current position for this command (signed)
        PrintChar('P')
        PrintChar(',')
        HexPrint(gISRPosition0ForThisCommand);

        // Write out the total steps made during this move (unsigned)
        PrintChar('S')
        PrintChar(',')
        HexPrint(gISRStepCount1ForThisCommand)
        PrintChar(',')

        // Write out the accumulator2 value after all math is complete (unsigned)
        PrintChar('C')
        PrintChar(',')
        HexPrint(acc_union[1].value)
        PrintChar(',')

        // Write out the rate2 value (signed)
        PrintChar('R')
        PrintChar(',')
        HexPrint(CurrentCommand.m.sm.Rate[1].value)
        PrintChar(',')

        // Write out the current position2 for this command (signed)
        PrintChar('P')
        PrintChar(',')
        HexPrint(gISRPosition1ForThisCommand);
#else
        // Print all values using raw binary
        // Write out the total ISR ticks for this move (unsigned)
        BinPrint(gISRTickCountForThisCommand) // Macro for printing raw binary value

        // Write out the total steps1 made during this move (unsigned)
        BinPrint(gISRStepCount0ForThisCommand)

        // Write out the accumulator1 value after all math is complete (unsigned)
        BinPrint(acc_union[0].value)

        // Write out the rate1 value (signed)
        BinPrint(CurrentCommand.m.sm.Rate[0].value)

        // Write out the current position1 for this command (signed)
        BinPrint(gISRPosition0ForThisCommand);        

        // Write out the total steps2 made during this move (unsigned)
        BinPrint(gISRStepCount1ForThisCommand)

        // Write out the accumulator2 value after all math is complete (unsigned)
        BinPrint(acc_union[1].value)

        // Write out the rate2 value (signed)
        BinPrint(CurrentCommand.m.sm.Rate[1].value)

        // Write out the current position2 for this command (signed)
        BinPrint(gISRPosition1ForThisCommand);        
#endif
        PrintChar('\n')
      }
    }
  }

  // It limit switch checking is enabled, check to see if a limit switch value
  // has been seen (CU,51 and CU,52)
  if (gLimitSwitchMask)
  {
    PortBTemp = PORTB;

    if (((~(PortBTemp ^ gLimitSwitchTarget)) & gLimitSwitchMask) && !bittstzero(gLimitSwitchTriggered))
    {
      // At least one of the bits in PortB that we are looking at is in a state
      // now where it has 'triggered' and so we need to shut down the current
      // move, and remove all remaining commands from the FIFO
      gLimitSwitchPortB = PortBTemp;
      bitsetzero(gLimitSwitchTriggered);

      AxisActive[0] = 0;
      AxisActive[1] = 0;
      bitsetzero(AllDone);
      gFIFOLength = 0;
      gFIFOIn = 0;
      gFIFOOut = 0;

      // Flush the whole FIFO by clearing each element's Command field
      for (isr_i=0; isr_i < COMMAND_FIFO_MAX_LENGTH; isr_i++)
      {
        FIFOPtr[isr_i].Command = COMMAND_NONE;
      }
    }
  }

  // If we're done with our current command, load in the next one
  if (bittstzero(AllDone))
  {
    CurrentCommand.Command = COMMAND_NONE;
    if (gFIFOLength != 0u)
    {
      if (bittst(TestMode, TEST_MODE_GPIO_NUM))
      {
        LATDbits.LATD0 = 1;
      }
      if (bittstzero(gRedLEDEmptyFIFO))
      {
        mLED_2_Off()
      }

      // If enabled, move stepper disable state to primed
      if (g_StepperDisableState != kSTEPPER_TIMEOUT_DISABLED)
      {
        g_StepperDisableState = kSTEPPER_TIMEOUT_PRIMED;
      }
      
//      FIFO_COPY();

      // Check to see if the FIFO_out_ptr needs wrapping
      
#if defined(USE_C_ISR)
      // Instead of copying over the entire MoveCommandType every time, to save
      // time we will check which command is next in the FIFO, and then only 
      // copy over those fields that the new command actually uses.
      // The order that we check these is the same as the main ISR check order
      // above, where we want to have the most common commands checked first
      // since they will then happen faster.
      gFIFOCommand = FIFOPtr[gFIFOOut].Command;
      
      if (gFIFOCommand == COMMAND_SM_XM_HM_MOVE)
      {
        CurrentCommand.Command        = FIFOPtr[gFIFOOut].Command;
        CurrentCommand.m.sm.Rate[0]        = FIFOPtr[gFIFOOut].m.sm.Rate[0];
        CurrentCommand.m.sm.Rate[1]        = FIFOPtr[gFIFOOut].m.sm.Rate[1];
        CurrentCommand.m.sm.Steps[0]       = FIFOPtr[gFIFOOut].m.sm.Steps[0];
        CurrentCommand.m.sm.Steps[1]       = FIFOPtr[gFIFOOut].m.sm.Steps[1];
        CurrentCommand.m.sm.DirBits        = FIFOPtr[gFIFOOut].m.sm.DirBits;
        CurrentCommand.m.sm.DelayCounter   = FIFOPtr[gFIFOOut].m.sm.DelayCounter;
        CurrentCommand.m.sm.SEState        = FIFOPtr[gFIFOOut].m.sm.SEState;
      }
      else if (gFIFOCommand == COMMAND_LM_MOVE)
      {
        CurrentCommand.Command        = FIFOPtr[gFIFOOut].Command;
        CurrentCommand.m.sm.Rate[0]        = FIFOPtr[gFIFOOut].m.sm.Rate[0];
        CurrentCommand.m.sm.Rate[1]        = FIFOPtr[gFIFOOut].m.sm.Rate[1];
        CurrentCommand.m.sm.Accel[0]       = FIFOPtr[gFIFOOut].m.sm.Accel[0];
        CurrentCommand.m.sm.Accel[1]       = FIFOPtr[gFIFOOut].m.sm.Accel[1];
        CurrentCommand.m.sm.Jerk[0]        = FIFOPtr[gFIFOOut].m.sm.Jerk[0];
        CurrentCommand.m.sm.Jerk[1]        = FIFOPtr[gFIFOOut].m.sm.Jerk[1];
        CurrentCommand.m.sm.Steps[0]       = FIFOPtr[gFIFOOut].m.sm.Steps[0];
        CurrentCommand.m.sm.Steps[1]       = FIFOPtr[gFIFOOut].m.sm.Steps[1];
        CurrentCommand.m.sm.DirBits        = FIFOPtr[gFIFOOut].m.sm.DirBits;
        CurrentCommand.m.sm.DelayCounter   = FIFOPtr[gFIFOOut].m.sm.DelayCounter;
        CurrentCommand.m.sm.SEState        = FIFOPtr[gFIFOOut].m.sm.SEState;
      }
      else if (gFIFOCommand == COMMAND_LT_MOVE)
      {
        CurrentCommand.Command        = FIFOPtr[gFIFOOut].Command;
        CurrentCommand.m.sm.Rate[0]        = FIFOPtr[gFIFOOut].m.sm.Rate[0];
        CurrentCommand.m.sm.Rate[1]        = FIFOPtr[gFIFOOut].m.sm.Rate[1];
        CurrentCommand.m.sm.Accel[0]       = FIFOPtr[gFIFOOut].m.sm.Accel[0];
        CurrentCommand.m.sm.Accel[1]       = FIFOPtr[gFIFOOut].m.sm.Accel[1];
        CurrentCommand.m.sm.Jerk[0]        = FIFOPtr[gFIFOOut].m.sm.Jerk[0];
        CurrentCommand.m.sm.Jerk[1]        = FIFOPtr[gFIFOOut].m.sm.Jerk[1];
        CurrentCommand.m.sm.Steps[0]       = FIFOPtr[gFIFOOut].m.sm.Steps[0];
        CurrentCommand.m.sm.DirBits        = FIFOPtr[gFIFOOut].m.sm.DirBits;
        CurrentCommand.m.sm.DelayCounter   = FIFOPtr[gFIFOOut].m.sm.DelayCounter;
        CurrentCommand.m.sm.SEState        = FIFOPtr[gFIFOOut].m.sm.SEState;
      }
      else if (gFIFOCommand == COMMAND_SERVO_MOVE)
      {
        CurrentCommand.Command        = FIFOPtr[gFIFOOut].Command;
        CurrentCommand.m.sm.DelayCounter   = FIFOPtr[gFIFOOut].m.sm.DelayCounter;
        CurrentCommand.m.sm.ServoPosition  = FIFOPtr[gFIFOOut].m.sm.ServoPosition;
        CurrentCommand.m.sm.ServoRPn       = FIFOPtr[gFIFOOut].m.sm.ServoRPn;
        CurrentCommand.m.sm.ServoChannel   = FIFOPtr[gFIFOOut].m.sm.ServoChannel;
        CurrentCommand.m.sm.ServoRate      = FIFOPtr[gFIFOOut].m.sm.ServoRate;
      }
      else if (gFIFOCommand == COMMAND_DELAY)
      {
        CurrentCommand.Command        = FIFOPtr[gFIFOOut].Command;
        CurrentCommand.m.sm.DelayCounter   = FIFOPtr[gFIFOOut].m.sm.DelayCounter;
      }
      else if (gFIFOCommand == COMMAND_SE)
      {
        CurrentCommand.Command        = FIFOPtr[gFIFOOut].Command;
        CurrentCommand.m.sm.DelayCounter   = FIFOPtr[gFIFOOut].m.sm.DelayCounter;
        CurrentCommand.m.sm.SEState        = FIFOPtr[gFIFOOut].m.sm.SEState;
        CurrentCommand.m.sm.SEPower        = FIFOPtr[gFIFOOut].m.sm.SEPower;
      }
      else if (gFIFOCommand == COMMAND_EM)
      {
        CurrentCommand.Command       = FIFOPtr[gFIFOOut].Command;
        CurrentCommand.m.sm.DirBits       = FIFOPtr[gFIFOOut].m.sm.DirBits;
        CurrentCommand.m.sm.ServoRPn      = FIFOPtr[gFIFOOut].m.sm.ServoRPn;
      }
      else
      {
        // gFIFOCommand had a value that is not allowed
      }
#endif

      // Take care of clearing the step accumulators for the next move if
      // it's a motor move (of any type) - if the command requests it
      if (
        (CurrentCommand.Command == COMMAND_SM_XM_HM_MOVE)
        | 
        (CurrentCommand.Command == COMMAND_LM_MOVE)
        | 
        (CurrentCommand.Command == COMMAND_LT_MOVE)
      )
      {
        // These are used in TEST_MODE_USART_ISR_NUM to allow an arbitrary 
        // value to be loaded into one or both accumulators for testing
        if (CurrentCommand.m.sm.SEState & SESTATE_ARBITRARY_ACC1_BIT)
        {
          acc_union[0].value = CurrentCommand.m.sm.DelayCounter;
        }
        if (CurrentCommand.m.sm.SEState & SESTATE_ARBITRARY_ACC2_BIT)
        {
          acc_union[1].value = CurrentCommand.m.sm.DelayCounter;
        }
        // Use the SEState to determine which accumulators to clear.
        if (CurrentCommand.m.sm.SEState & SESTATE_CLEAR_ACC1_BIT)
        {
          acc_union[0].value = 0;
        }
        if (CurrentCommand.m.sm.SEState & SESTATE_CLEAR_ACC2_BIT)
        {
          acc_union[1].value = 0;
        }
        if (CurrentCommand.m.sm.SEState & SESTATE_NEGATE_ACC1_BIT)
        {
          acc_union[0].value = 0x7FFFFFFFUL;
        }
        if (CurrentCommand.m.sm.SEState & SESTATE_NEGATE_ACC2_BIT)
        {
          acc_union[1].value = 0x7FFFFFFFUL;
        }
        
        // Set the "Active" flags for this move based on steps for each axis
        if (CurrentCommand.m.sm.Steps[0])
        {
          bitsetzero(AxisActive[0]);
        }
        else
        {
          bitclrzero(AxisActive[0]);
        }
        if (CurrentCommand.m.sm.Steps[1])
        {
          bitsetzero(AxisActive[1]);
        }
        else
        {
          bitclrzero(AxisActive[1]);
        }
      }
      if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
      {
        gISRTickCountForThisCommand = 0;
        gISRStepCount0ForThisCommand = 0;
        gISRPosition0ForThisCommand = 0;
        gISRStepCount1ForThisCommand = 0;
        gISRPosition1ForThisCommand = 0;
      }

      // Zero out command in FIFO we just copied, but leave the rest of the fields alone
      FIFOPtr[gFIFOOut].Command = COMMAND_NONE;
      
      // Increment gFIFO_Out
      gFIFOOut++;
      if (gFIFOOut >= gCurrentFIFOLength)
      {
        gFIFOOut = 0;
      }
      if (gFIFOLength)
      {
        gFIFOLength--;
      }
    }
    else 
    {
      // Current command is done, and the FIFO is completely empty.
      
      // Start up stepper motor disable timeout (if enabled)
      if (g_StepperDisableState == kSTEPPER_TIMEOUT_PRIMED)
      {
        g_StepperDisableSecondCounter = 1000u;
        g_StepperDisableCountdownS = g_StepperDisableTimeoutS;
        g_StepperDisableState = kSTEPPER_TIMEOUT_TIMING;
      }

      CurrentCommand.m.sm.DelayCounter = 0;

      if (bittstzero(gRedLEDEmptyFIFO))
      {
        mLED_2_On()
      }
      if (bittst(TestMode, TEST_MODE_GPIO_NUM))
      {
        LATAbits.LATA1 = 1;
      }
    }
  }

  // Check for button being pushed
  if (!swProgram)
  {
    bitsetzero(ButtonPushed);
  }
  if (bittstzero(UseAltPause))
  {
    if (!PORTBbits.RB0)
    {
      bitsetzero(ButtonPushed);
    }
  }

  // To give as much time for the step pulses to be high, we clear them as the
  // very last thing we do before leaving the ISR. It never hurts us to clear 
  // these bits even if no step happened this ISR tick
  if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
  {
    Step1IO = 0;
    Step2IO = 0;
  }
  else
  {
    Step1AltIO = 0;
    Step2AltIO = 0;
  }

  if (bittst(TestMode, TEST_MODE_PRINT_TRIGGER_NUM))
  {
    // Clear the interrupt as the last thing we do. This allows us to have
    // arbitrarily long interrupts to print debug information out
    INTCONbits.TMR0IF = 0;
    TMR0L = TIMER0_RELOAD;  // Reload for 25KHz ISR fire
    bitclr(TestMode, TEST_MODE_PRINT_TRIGGER_NUM);
  }
  if (bittst(TestMode, TEST_MODE_GPIO_NUM))
  {
    LATAbits.LATA1 = 0;
    LATDbits.LATD0 = 0;
    LATDbits.LATD1 = 0;
  }
}

// Zero out all global variables used for parameter passing between motion
// parsing commands
static void clear_parmaeter_globals(void)
{
  gUInt32_1 = 0;
  gUInt32_2 = 0;
  gUInt32_3 = 0;
  gInt32_4 = 0;
  gInt32_5 = 0;
  gInt32_6 = 0;
  gInt32_7 = 0;
  gInt32_8 = 0;
  gInt32_9 = 0;
  gInt32_10 = 0;
  gInt32_11 = 0;
}

// Init code
void EBB_Init(void)
{
  UINT8 i;

  // Initialize all Current Command values
  for (i = 0; i < NUMBER_OF_STEPPERS; i++)
  {
    CurrentCommand.m.sm.Rate[i].value = 1;
    CurrentCommand.m.sm.Steps[i] = 0;
    CurrentCommand.m.sm.Accel[i] = 0;
  }
  CurrentCommand.Command = COMMAND_NONE;
  CurrentCommand.m.sm.DirBits = 0;
  CurrentCommand.m.sm.DelayCounter = 0;
  CurrentCommand.m.sm.ServoPosition = 0;
  CurrentCommand.m.sm.ServoRPn = 0;
  CurrentCommand.m.sm.ServoChannel = 0;
  CurrentCommand.m.sm.ServoRate = 0;

  bitclrzero(gRedLEDEmptyFIFO);

  TestMode = 0;
  
//  PORTA = 0;
  RefRA0_IO_TRIS = INPUT_PIN;
//  PORTB = 0;
//  INTCON2bits.RBPU = 0;   // Turn on weak-pull ups for port B
//  PORTC = 0;              // Start out low
//  TRISC = 0x80;           // Make portC output except for PortC bit 7, USB bus sense
//  PORTD = 0;
//  TRISD = 0;
//  PORTE = 0;
//  TRISE = 0;

  // And make sure to always use low priority for ADC
  IPR1bits.ADIP = 0;

  // Turn on AN0 (RA0) as analog input
  AnalogConfigure(RA0_CUR_ADJ_ADC_CHANNEL,1);
  // Turn on AN11 (V+) as analog input
  AnalogConfigure(RA11_VPLUS_POWER_ADC_CHANNEL,1);

  MS1_IO = 1;
  MS1_IO_TRIS = OUTPUT_PIN;
  MS2_IO = 1;
  MS2_IO_TRIS = OUTPUT_PIN;
  MS3_IO = 1;
  MS3_IO_TRIS = OUTPUT_PIN;

  Enable1IO = 1;
  Enable1IO_TRIS = OUTPUT_PIN;
  Enable2IO = 1;
  Enable2IO_TRIS = OUTPUT_PIN;

  Step1IO = 0;
  Step1IO_TRIS = OUTPUT_PIN;
  Dir1IO = 0;
  Dir1IO_TRIS = OUTPUT_PIN;
  Step2IO = 0;
  Step2IO_TRIS = OUTPUT_PIN;
  Dir2IO = 0;
  Dir2IO_TRIS = OUTPUT_PIN;

  // For bug in VUSB divider resistor, set RC7 as output and set high
  // Wait a little while to charge up
  // Then set back as an input
  // The idea here is to get the Schmidt trigger input RC7 high before
  // we make it an input, thus getting it above the 2.65V ST threshold
  // And allowing VUSB to keep the logic level on the pin high at 2.5V
#if defined(USE_USB_BUS_SENSE_IO)
  tris_usb_bus_sense = OUTPUT_PIN; // See HardwareProfile.h
  USB_BUS_SENSE = 1;
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  Delay1TCY();
  tris_usb_bus_sense = INPUT_PIN;
  USB_BUS_SENSE = 0;
#endif
  gUseSolenoid = TRUE;
  bitsetzero(gUseRCPenServo);

  // Set up pen up/down direction as output
  PenUpDownIO = 0;
  PenUpDownIO_TRIS = OUTPUT_PIN;
    
  // Set up RC Servo power control to be off
  RCServoPowerIO = RCSERVO_POWER_OFF;
  RCServoPowerIO_TRIS = OUTPUT_PIN;

  SolenoidState = SOLENOID_ON;
  DriverConfiguration = PIC_CONTROLS_DRIVERS;
  PenState = PEN_UP;
  for (i=0; i < SL_STORAGE_SIZE; i++)
  {
    gSL_Storage[i] = 0;
  }
  NodeCount = 0;
  ButtonPushed = 0;
  gStandardizedCommandFormat = 0;
  gLimitChecks = TRUE;
  gFIFOLength = 0;
  gFIFOIn = 0;
  gFIFOOut = 0;
  gCurrentFIFOLength = 1; // Default the FIFO length to 1 on boot
  //isr_FSR0L_temp = 0;
  //isr_FSR0H_temp = 0;
  // Start out FIFO out pointer on first element in FIFO array, which starts at
  // address 0x500
  FIFO_out_ptr_high = 0x05;
  FIFO_out_ptr_low  = 0x00;
  
  // Default RB0 to be an input, with the pull-up enabled, for use as alternate
  // PAUSE button (just like PRG)
  // Except for v1.1 hardware, use RB2
  TRISBbits.TRISB0 = 1;
  INTCON2bits.RBPU = 0;       // Turn on all of PortB pull-ups
  bitsetzero(UseAltPause);

  TRISBbits.TRISB3 = 0;       // Make RB3 an output (for engraver)
  PORTBbits.RB3 = 0;          // And make sure it starts out off

  // These are ISR variables, can't set them to zero in their definitions or
  // the linker puts them in a different bank and ISR expands with more MOVLB
  // instructions.
  PortBTemp = 0;
  gLimitSwitchMask = 0;
  gLimitSwitchTarget = 0;
  bitclrzero(gLimitSwitchTriggered);

  // Clear out global stepper positions
  clear_StepCounters();

// Set up TMR0 for our 25KHz High ISR for stepping
  T0CONbits.T08BIT = 1;     // Set 8 bit mode
  T0CONbits.T0CS = 0;       // System clocked from Fosc/4
  T0CONbits.PSA = 0;        // Use prescaler
  T0CONbits.T0PS2 = 0;      // Use 4:1 Prescale value
  T0CONbits.T0PS1 = 0;
  T0CONbits.T0PS0 = 1;
  TMR0L = 0x00;             // Give the timer some time before it fires the first time

  T0CONbits.TMR0ON = 1;     // Turn the timer on

  INTCON2bits.TMR0IP = 1;   // Use high priority interrupt for Timer 0
  INTCONbits.TMR0IF = 0;    // Clear the interrupt
  INTCONbits.TMR0IE = 1;    // Turn on the interrupt

  
}

// Stepper (mode) Configure command
// SC,1,0<CR> will use just solenoid output for pen up/down
// SC,1,1<CR> will use servo on RB1 for pen up/down
// SC,1,2<CR> will use servo on RB1 for pen up/down, but with ECCP2 (PWM) in hardware (default)
// SC,2,0<CR> will make PIC control drivers (default)
// SC,2,1<CR> will make PIC control external drivers using these pins
//    ENABLE1 = RD1
//    ENABLE2 = RA1
//    STEP1 = RC6
//    DIR1 = RC2
//    STEP2 = RA5
//    DIR2 = RA2
// SC,2,2<CR> will disconnect PIC from drivers and allow external step/dir source
// SC,4,<servo2_min><CR> will set <servo2_min> as the minimum value for the servo (1 to 65535)
// SC,5,<servo2_max><CR> will set <servo2_max> as the maximum value for the servo (1 to 65535)
// SC,6,<servo_min><CR> will set <servo_min> as the minimum value for the servo (1 to 11890)
// SC,7,<servo_max><CR> will set <servo_max> as the maximum value for the servo (1 to 11890)
// SC,8,<servo2_slots><CR> sets the number of slots for the servo2 system (1 to 24)
// SC,9,<servo2_slotMS><CR> sets the number of ms in duration for each slot (1 to 6)
// SC,10,<servo2_rate><CR> sets the rate of change for the servo (both up and down)
// SC,11,<servo2_rate><CR> sets the pen up speed
// SC,12,<servo2_rate><CR> sets the pen down speed
// SC,13,1<CR> enables RB3 as parallel input to PRG button for pause detection
// SC,13,0<CR> disables RB3 as parallel input to PRG button for pause detection
void parse_SC_packet (void)
{
  unsigned char Para1 = 0;
  unsigned int Para2 = 0;

  print_command(FALSE, FALSE);
  
  // Extract each of the values.
  extract_number(kUCHAR, &Para1, kREQUIRED);
  extract_number(kUINT, &Para2, kREQUIRED);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  // Check for command to select which (solenoid/servo) gets used for pen
  if (Para1 == 1u)
  {
    // Use just solenoid
    if (Para2 == 0u)
    {
      gUseSolenoid = TRUE;
      bitclrzero(gUseRCPenServo);
      // Turn off RC signal on Pen Servo output
      RCServo2_Move(0, g_servo2_RPn, 0, 0, FALSE);
    }
    // Use just RC servo
    else if (Para2 == 1u)
    {
      gUseSolenoid = FALSE;
      bitsetzero(gUseRCPenServo);
    }
    // Use solenoid AND servo (default)
    else
    {
      gUseSolenoid = TRUE;
      bitsetzero(gUseRCPenServo);
    }
    // Send a new command to set the state of the servo/solenoid
    process_SP(PenState, 0);
  }
  // Check for command to switch between built-in drivers and external drivers
  else if (Para1 == 2u)
  {
    if (Para2 == 0u)
    {
      DriverConfiguration = PIC_CONTROLS_DRIVERS;
      // Connections to drivers become outputs
      Enable1IO_TRIS = OUTPUT_PIN;
      Enable2IO_TRIS = OUTPUT_PIN;
      Step1IO_TRIS = OUTPUT_PIN;
      Dir1IO_TRIS = OUTPUT_PIN;
      Step2IO_TRIS = OUTPUT_PIN;
      Dir2IO_TRIS = OUTPUT_PIN;
      // Alternate I/O pins become inputs
      Dir1AltIO_TRIS = INPUT_PIN;
      Dir2AltIO_TRIS = INPUT_PIN;
      Step1AltIO_TRIS = INPUT_PIN;
      Step2AltIO_TRIS = INPUT_PIN;
      Enable1AltIO_TRIS = INPUT_PIN;
      Enable2AltIO_TRIS = INPUT_PIN;
    }
    else if (Para2 == 1u)
    {
      DriverConfiguration = PIC_CONTROLS_EXTERNAL;
      // Connections to drivers become inputs
      Enable1IO_TRIS = INPUT_PIN;
      Enable2IO_TRIS = INPUT_PIN;
      Step1IO_TRIS = INPUT_PIN;
      Dir1IO_TRIS = INPUT_PIN;
      Step2IO_TRIS = INPUT_PIN;
      Dir2IO_TRIS = INPUT_PIN;
      // Alternate I/O pins become outputs
      Dir1AltIO_TRIS = OUTPUT_PIN;
      Dir2AltIO_TRIS = OUTPUT_PIN;
      Step1AltIO_TRIS = OUTPUT_PIN;
      Step2AltIO_TRIS = OUTPUT_PIN;
      Enable1AltIO_TRIS = OUTPUT_PIN;
      Enable2AltIO_TRIS = OUTPUT_PIN;
    }
    else if (Para2 == 2u)
    {
      DriverConfiguration = EXTERNAL_CONTROLS_DRIVERS;
      // Connections to drivers become inputs
      Enable1IO_TRIS = INPUT_PIN;
      Enable2IO_TRIS = INPUT_PIN;
      Step1IO_TRIS = INPUT_PIN;
      Dir1IO_TRIS = INPUT_PIN;
      Step2IO_TRIS = INPUT_PIN;
      Dir2IO_TRIS = INPUT_PIN;
      // Alternate I/O pins become inputs
      Dir1AltIO_TRIS = INPUT_PIN;
      Dir2AltIO_TRIS = INPUT_PIN;
      Step1AltIO_TRIS = INPUT_PIN;
      Step2AltIO_TRIS = INPUT_PIN;
      Enable1AltIO_TRIS = INPUT_PIN;
      Enable2AltIO_TRIS = INPUT_PIN;
     }
  }
  // Set <min_servo> for Servo2 method
  else if (Para1 == 4u)
  {
    g_servo2_min = Para2;
  }
  // Set <max_servo> for Servo2
  else if (Para1 == 5u)
  {
    g_servo2_max = Para2;
  }
  // Set <gRC2Slots>
  else if (Para1 == 8u)
  {
    if (Para2 > MAX_RC2_SERVOS)
    {
      Para2 = MAX_RC2_SERVOS;
    }
    gRC2Slots = Para2;
  }
  else if (Para1 == 9u)
  {
    if (Para2 > 6u)
    {
      Para2 = 6;
    }
    gRC2SlotMS = Para2;
  }
  else if (Para1 == 10u)
  {
    g_servo2_rate_up = Para2;
    g_servo2_rate_down = Para2;
  }
  else if (Para1 == 11u)
  {
    g_servo2_rate_up = Para2;
  }
  else if (Para1 == 12u)
  {
    g_servo2_rate_down = Para2;
  }
    else if (Para1 == 13u)
  {
    if (Para2)
    {
      bitsetzero(UseAltPause);
    }
    else
    {
      bitclrzero(UseAltPause);
    }
  }
  print_line_ending(kLE_OK_NORM);
}

// Low Level Move command
// Usage: LM,<Rate1>,<Steps1>,<Accel1>,<Rate2>,<Steps2>,<Accel2>,<ClearAccs><CR>
//
// Is for doing low level moves with optional acceleration. 
//
// <Rate1> and <Rate2> are signed 32-bit integers
// Negative values indicate movement in the opposite direction.
// They are the values added to the accumulator every 25KHz.
// <Steps1> and <Steps2> are signed 32-bit integers. Each axis will take 
// <steps> steps and then stop. 
// Once both axis are done moving, the command is complete.
// Note that as a legacy mode for v2.8.0 and below
// software versions you can use the sign of the steps values to control the 
// initial direction of each axis.
// <Accel1> and <Accel2> are 32 bit signed integers. Their values are added to 
// <Rate1> and <Rate2> respectively every ISR tick.
//
// <ClearAccs> is optional. A value of 0 will do nothing. A value of 1 will 
// clear Motor 1's accumulator before starting the move. A value of 2 will 
// clear Motor 2's accumulator. And a value of 3 will clear both.
void parse_LM_packet(void)
{
  ExtractReturnType ClearRet;

  clear_parmaeter_globals();

  print_command(FALSE, FALSE);
  
  // Extract each of the values.
  extract_number(kLONG,  &gRate1,     kREQUIRED);
  extract_number(kLONG,  &gSteps1,    kREQUIRED);
  extract_number(kLONG,  &gAccel1,    kREQUIRED);
  extract_number(kLONG,  &gRate2,     kREQUIRED);
  extract_number(kLONG,  &gSteps2,    kREQUIRED);
  extract_number(kLONG,  &gAccel2,    kREQUIRED);
  ClearRet = extract_number(kULONG, &gClearAccs, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  if (gLimitChecks)
  {
    // Quickly eliminate obvious invalid parameter combinations,
    // like LM,1,0,2,0,0,0. Or LM,0,1,0,0,0,0,0 GH issue #78
    if (
      (
        ((gRate1 == 0) && (gAccel1 == 0))
        ||
        (gSteps1 == 0)
      )
      &&
      (
        ((gRate2 == 0) && (gAccel2 == 0))
        ||
        (gSteps2 == 0)
      )
    )
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
  }
  
  // We don't need Intervals or Jerks, so set them to zero before calling low_level
  gIntervals = 0;
  gJerk1 = 0;
  gJerk2 = 0;
  
  process_low_level_move(FALSE, ClearRet);
    
  print_line_ending(kLE_OK_NORM);
}

// Low Level third derivative Move command
// Usage: L3,<Rate1>,<Steps1>,<Accel1>,<Jerk1>,<Rate2>,<Steps2>,<Accel2>,<Jerk2>,<ClearAccs><CR>
//
// Is for doing low level moves with optional acceleration and jerk,
//
// <Rate1> and <Rate2> are signed 32-bit integers
// Negative values indicate movement in the opposite direction.
// They are the values added to the accumulator every 25KHz.
// <Steps1> and <Steps2> are signed 32-bit integers. Each axis will take 
// <steps> steps and then stop. 
// Once both axis are done moving, the command is complete.
// <Accel1> and <Accel2> are 32 bit signed integers. Their values are added to 
// <Rate1> and <Rate2> respectively every ISR tick.
// <Jerk1> and <Jerk2> are applied to <Rate1> and <Rate2> every ISR tick.
// See full EBB docs for more complete documentation.
// Note that as a legacy mode for compatibility with v2.8.0 and below
// software versions. You can use the sign of the steps values to control the 
// initial direction of each axis, but then <Rate> must not be negative.
//
// <ClearAccs> is optional. A value of 0 will do nothing. A value of 1 will 
// clear Motor 1's accumulator before starting the move. A value of 2 will 
// clear Motor 2's accumulator. And a value of 3 will clear both.
void parse_L3_packet(void)
{
  ExtractReturnType ClearRet;

  clear_parmaeter_globals();
  
  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number(kLONG,  &gRate1,     kREQUIRED);
  extract_number(kLONG,  &gSteps1,    kREQUIRED);
  extract_number(kLONG,  &gAccel1,    kREQUIRED);
  extract_number(kLONG,  &gJerk1,     kREQUIRED);
  extract_number(kLONG,  &gRate2,     kREQUIRED);
  extract_number(kLONG,  &gSteps2,    kREQUIRED);
  extract_number(kLONG,  &gAccel2,    kREQUIRED);
  extract_number(kLONG,  &gJerk2,     kREQUIRED);
  ClearRet = extract_number(kULONG, &gClearAccs, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  if (gLimitChecks)
  {
    // Quickly eliminate obvious invalid parameter combinations,
    // like L3,1,0,2,3,0,0,0,0. Or L3,0,1,0,0,0,0,0,0,0 GH issue #78
    if (
      (
        ((gRate1 == 0) && (gAccel1 == 0) && (gJerk1 == 0))
        ||
        (gSteps1 == 0)
      )
      &&
      (
        ((gRate2 == 0) && (gAccel2 == 0) && (gJerk2 == 0))
        ||
        (gSteps2 == 0)
      )
    )
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
  }
  
  // We don't use Intervals, so set it to zero here
  gIntervals = 0;
  
  process_low_level_move(FALSE, ClearRet);
    
  print_line_ending(kLE_OK_NORM);
}

// Low Level Timed third derivative Move command for S-curves
// Usage: TD,<Intervals>,<Rate1A>,<Rate1B>,<Accel1>,<Jerk1>,<Rate2A>,<Rate2B>,<Accel2>,<Jerk2>,<ClearAccs><CR>
//
// This command is for implementing S-curves. It takes its parameters, and
// creates to T3 commands for adding to the FIFO like this:
//
// T3,Intervals,Rate1A,0,Jerk1,Rate2A,0,Jerk2[,Clear]
// T3,Intervals,Rate1B,Accel1,-Jerk1,Rate2B,Accel2,-Jerk2
//
// The parameter values are all explained in the comment header for T3
void parse_TD_packet(void)
{
  ExtractReturnType ClearRet;
  INT32 Rate1B, Rate2B, Accel1, Accel2;

  clear_parmaeter_globals();
  
  print_command(FALSE, FALSE);
  
  // Extract each of the values.
  extract_number(kULONG, &gIntervals, kREQUIRED);
  extract_number(kLONG,  &gRate1,     kREQUIRED);
  extract_number(kLONG,  &Rate1B,     kREQUIRED);
  extract_number(kLONG,  &Accel1,     kREQUIRED);
  extract_number(kLONG,  &gJerk1,     kREQUIRED);
  extract_number(kLONG,  &gRate2,     kREQUIRED);
  extract_number(kLONG,  &Rate2B,     kREQUIRED);
  extract_number(kLONG,  &Accel2,     kREQUIRED);
  extract_number(kLONG,  &gJerk2,     kREQUIRED);
  ClearRet = extract_number(kULONG, &gClearAccs, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  if (gLimitChecks)
  {
    // Eliminate obvious invalid parameter combinations,
    // like LT,0,X,X,X,X,X
    if (gIntervals == 0u)
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
  }

  // We don't use Steps so clear them here before calling low_level
  gSteps1 = 0;
  gSteps2 = 0;
  
  // For our first T3 command, we need both accel terms to be zero
  gAccel1 = 0;
  gAccel2 = 0;
  
  process_low_level_move(TRUE, ClearRet);

  // Now for the second T3 command, use the B rates
  gRate1 = Rate1B;
  gRate2 = Rate2B;
  
  // And the accels
  gAccel1 = Accel1;
  gAccel2 = Accel2;
  
  // And the jerks need to switch sign
  gJerk1 = -gJerk1;
  gJerk2 = -gJerk2;
  
  // Always force no-change to the accumulators for the second move
  gClearAccs = 0;
  
  // We also don't want to do anything to clear the accumulators for the second T3
  process_low_level_move(TRUE, ClearRet);
    
  print_line_ending(kLE_OK_NORM);
}


// Low Level Timed third derivative Move command
// Usage: T3,<Intervals>,<Rate1>,<Accel1>,<Jerk1>,<Rate2>,<Accel2>,<Jerk2>,<ClearAccs><CR>
//
// This command is a modified version of the LM command. Instead of stepping for a certain number of steps
// on each axis at a given rate (with an acceleration term for each as well), this command will step 
// for a certain duration, no matter the step count. The rate and acceleration of each axis are still
// specified separately.
//
// Note that <Intervals> is a 32-bit unsigned int and is in units of ISR ticks.
// <Accel1> and <Accel2> are 32 bit signed ints
// <Rate1> and <Rate2> are 32 bit signed ints
// <Jerk1> and <Jerk2> are 32 bit signed ints
// The sign of <Rate1> and <Rate2> determine the direction that the axis will move.
// After the signs are taken into account for direction purposes, the Rate values
// are converted to unsigned 31 bit numbers.
//
// <ClearAccs> is optional. A value of 0 will do nothing. A value of 1 will clear Motor 1's accumulator before
// starting the move. A value of 2 will clear Motor 2's accumulator. And a value of 3 will clear both.
void parse_T3_packet(void)
{
  ExtractReturnType ClearRet;

  clear_parmaeter_globals();
  
  print_command(FALSE, FALSE);
  
  // Extract each of the values.
  extract_number(kULONG, &gIntervals, kREQUIRED);
  extract_number(kLONG,  &gRate1,     kREQUIRED);
  extract_number(kLONG,  &gAccel1,    kREQUIRED);
  extract_number(kLONG,  &gJerk1,     kREQUIRED);
  extract_number(kLONG,  &gRate2,     kREQUIRED);
  extract_number(kLONG,  &gAccel2,    kREQUIRED);
  extract_number(kLONG,  &gJerk2,     kREQUIRED);
  ClearRet = extract_number(kULONG, &gClearAccs, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  if (gLimitChecks)
  {
    // Eliminate obvious invalid parameter combinations,
    // like LT,0,X,X,X,X,X
    if (gIntervals == 0u)
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
  }

  // We don't use Steps so clear them here before calling low_level
  gSteps1 = 0;
  gSteps2 = 0;
  
  process_low_level_move(TRUE, ClearRet);
    
  print_line_ending(kLE_OK_NORM);
}

// Low Level Timed Move command
// Usage: LT,<Intervals>,<Rate1>,<Accel1>,<Rate2>,<Accel2>,<ClearAccs><CR>
//
// This command is a modified version of the LM command. Instead of stepping for a certain number of steps
// on each axis at a given rate (with an acceleration term for each as well), this command will step 
// for a certain duration, no matter the step count. The rate and acceleration of each axis are still
// specified separately.
//
// Note that <Intervals> is a 32-bit unsigned int and is in units of ISR ticks.
// <Accel1> and <Accel2> are 32 bit signed ints, and <Rate1> and <Rate2> are 32 bit signed ints. 
// The sign of <Rate1> and <Rate2> determine the direction that the axis will move.
// After the signs are taken into account for direction purposes, the Rate values
// are converted to unsigned 31 bit numbers.
//
// <ClearAccs> is optional. A value of 0 will do nothing. A value of 1 will clear Motor 1's accumulator before
// starting the move. A value of 2 will clear Motor 2's accumulator. And a value of 3 will clear both.
void parse_LT_packet(void)
{
  ExtractReturnType ClearRet;
  
  clear_parmaeter_globals();

  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number(kULONG, &gIntervals, kREQUIRED);
  extract_number(kLONG,  &gRate1,     kREQUIRED);
  extract_number(kLONG,  &gAccel1,    kREQUIRED);
  extract_number(kLONG,  &gRate2,     kREQUIRED);
  extract_number(kLONG,  &gAccel2,    kREQUIRED);
  ClearRet = extract_number(kULONG, &gClearAccs, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  if (gLimitChecks)
  {
    // Eliminate obvious invalid parameter combinations,
    // like LT,0,X,X,X,X,X
    if (gIntervals == 0u)
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
  }
  
  // We don't use Steps or Jerk, so set them to zero here
  gSteps1 = 0;
  gSteps2 = 0;
  gJerk1 = 0;
  gJerk2 = 0;
  
  process_low_level_move(TRUE, ClearRet);
    
  print_line_ending(kLE_OK_NORM);
}

// Because the code for LM, L3, LT and T3 is really the same we call a common 
// processing function from all of them once their parameters are parsed into 
// variables.
// The caller must understand that this function changes some/all of the
// global values used to pass in parameters. If it needs to preserve any
// of those values it should make copies.
//
// This function takes input parameters in:
//   gIntervals
//   gRate1
//   gRate2
//   gSteps1
//   gSteps2
//   gAccel1
//   gAccel2
//   gJerk1
//   gJerk2
//   gClearAccs

void process_low_level_move(BOOL TimedMove, ExtractReturnType ClearRet)
{
  INT32 RateTemp = 0;
  BOOL NeedNegativeAccumulator;
  
  // If we have a triggered limit switch, then ignore this move command
  if (bittstzero(gLimitSwitchTriggered))
  {
    return;
  }

  if (!bittst(TestMode, TEST_MODE_USART_ISR_NUM))
  {
    if (gClearAccs > 3u)
    {
      gClearAccs = 3;
    }
  }
  
  // If not a timed move, check for legacy opposite direction support
  if (!TimedMove)
  {
    // Since we will only use the sign of Rate as the signal for initial 
    // stepper direction, if the user has sent us negative step counts, we
    // apply a 'legacy' mode rule here : if steps are negative, make sure
    // rate is positive, then make steps positive and rate negative and invert
    // acceleration too to compensate. After this step, the sign of rate completely
    // controls the direction.

    if (gSteps1 < 0)
    {
      if (gRate1 < 0)
      {
        ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
        return;
      }
      else
      {
        gSteps1 = -gSteps1;
        gRate1 = -gRate1;
        gAccel1 = -gAccel1;
        gJerk1 = -gJerk1;
      }
    }
    if (gSteps2 < 0)
    {
      if (gRate2 < 0)
      {
        ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
        return;
      }
      else
      {
        gSteps2 = -gSteps2;
        gRate2 = -gRate2;
        gAccel2 = -gAccel2;
        gJerk2 = -gJerk2;
      }
    }
  }
  
  gMoveTemp.m.sm.DirBits = 0;       // Start by assuming motors start CW
  gMoveTemp.m.sm.DelayCounter = 0;  // No delay for motor moves

  // Subtract off half of the Accel term and add 1/6th of the Jerk before we add the
  // move to the queue. Why? Because it makes the math cleaner (see LM command
  // documentation)
  gRate1 = gRate1 - (gAccel1/2) + (gJerk1/6);
  gRate2 = gRate2 - (gAccel2/2) + (gJerk2/6);
  
  // We will use the SEState move parameter to hold a bitfield. This bitfield
  // will tell the ISR if the accumulators need any special treatment when
  // the command is loaded. We can zero it, or set it to 2^31-1, leave
  // it alone, or set it to an arbitrary value (when testing).
  // If clearing the accumulator, there are certain conditions which require 
  // us to start the accumulator at 2^31-1, so we check for those 
  // conditions and we use two bits in SEState to indicate this.

  gMoveTemp.m.sm.SEState = 0;           // Start with all bits clear  
  
  // Determine what to do with the accumulators in UART_ISR_NUM test mode.
  //
  // In this test mode, we are outputting all of the relevant variables after
  // each move, over the debug UART pin, for collection by the HIL test 
  // system for comparison to the correct values (which are pre-computed on
  // the PC).
  //
  // This test mode gives the user the ability to 'preload' the accumulators
  // before the move begins, thus allowing a test where you can simulate 
  // the move beginning with any arbitrary value of the accumulator. This is
  // critical as certain tests require this ability to correctly check the 
  // math in those situations.
  //
  // This ability is implemented by overloading the last parameter sent in
  // for this command, gClearAccs. This is an optional parameter. If the
  // parameter is not present, then in this test mode that fact is interpreted
  // as a command to clear both accumulators to zero before the move begins.
  //
  // However, if any value is present for this parameter, then ClearRet will
  // be set to kEXTRACT_OK, and gClearAccs will hold the desired starting value
  // of the accumulators. When this is true, both accumulators are set to 
  // the value of gClearAccs before the move begins.
  //
  // Because this extra logic is only triggered when the test mode is active, 
  // it does not impact normal operation at all.
  if (bittst(TestMode, TEST_MODE_USART_ISR_NUM))
  {
    // If the ISR_BIT_NUM test mode is on, then interpret the Clear parameter
    // as the initial value for Axis1 accumulator. There is no way currently
    // to perform this same trick for Axis2.
    
    // We got a Clear parameter
    if (ClearRet == kEXTRACT_OK)
    {
      gMoveTemp.m.sm.SEState |= SESTATE_ARBITRARY_ACC1_BIT;
      gMoveTemp.m.sm.SEState |= SESTATE_ARBITRARY_ACC2_BIT;
      gMoveTemp.m.sm.DelayCounter = gClearAccs;
      
      // We want to fall through to the 'normal' code that deals with
      // accumulator clearing (looking ahead and negating, etc.). Since we know
      // that we are setting the accumulators to a pre-calculated value and
      // have told the ISR this with the two SEState bits above, we can set
      // gClearAccs to zero, which will mean the 'normal' code below ignores
      // the accumulators and doesn't zero or negate them.
      gClearAccs = 0;
    }
    else
    {
      // But if we didn't get a Clear parameter AND we are in UART_ISR test 
      // mode, then treat it as if the user wants  both accumulators cleared.
      // We're going to fall through to the 'normal' code below that handles
      // this situation, by setting gClearAccs to 0x03, which tells the code
      // to clear (or negate) both accumulators.
      gClearAccs = 3;
    }
  }
  
  // 'Normal' accumulator clear/negate logic:
  //
  // Test mode not active, so just check Clear parameter bits 0 and 1
  // to see if user wants to start move with cleared accumulator. If so,
  // check for need to invert accumulator when move loaded into FIFO.
  if (gClearAccs & 0x01)
  {
    NeedNegativeAccumulator = FALSE;

    RateTemp = gRate1 + gAccel1;
    if (RateTemp < 0)
    {
      NeedNegativeAccumulator = TRUE;
    }
    else
    {
      if (RateTemp == 0)
      {
        RateTemp = gAccel1 + gJerk1;
        if (RateTemp < 0)
        {
          NeedNegativeAccumulator = TRUE;
        }
        else
        {
          if (RateTemp == 0)
          {
            if (gJerk1 < 0)
            {
              NeedNegativeAccumulator = TRUE;
            }
          }
        }
      }
    }

    if (NeedNegativeAccumulator)
    {
      gMoveTemp.m.sm.SEState |= SESTATE_NEGATE_ACC1_BIT;
    }
    else
    {
      gMoveTemp.m.sm.SEState |= SESTATE_CLEAR_ACC1_BIT;
    }
  }
  if (gClearAccs & 0x02)
  {
    NeedNegativeAccumulator = FALSE;

    RateTemp = gRate2 + gAccel2;
    if (RateTemp < 0)
    {
      NeedNegativeAccumulator = TRUE;
    }
    else
    {
      if (RateTemp == 0)
      {
        RateTemp = gAccel2 + gJerk2;
        if (RateTemp < 0)
        {
          NeedNegativeAccumulator = TRUE;
        }
        else
        {
          if (RateTemp == 0)
          {
            if (gJerk2 < 0)
            {
              NeedNegativeAccumulator = TRUE;
            }
          }
        }
      }
    }

    if (NeedNegativeAccumulator)
    {
      gMoveTemp.m.sm.SEState |= SESTATE_NEGATE_ACC2_BIT;
    }
    else
    {
      gMoveTemp.m.sm.SEState |= SESTATE_CLEAR_ACC2_BIT;
    }
  }
 
  gAccel1 = gAccel1 - gJerk1;
  gAccel2 = gAccel2 - gJerk2;

  if (gAutomaticMotorEnable == TRUE)
  {
    // Enable both motors when we want to move them
    // (To be more logical this should happen in the ISR when this move
    // is loaded rather than here.)
    Enable1IO = ENABLE_MOTOR;
    Enable2IO = ENABLE_MOTOR;
  }

  // Load up the move structure with all needed values
  gMoveTemp.m.sm.Rate[0].value = gRate1;
  if (TimedMove)
  {
    gMoveTemp.m.sm.Steps[0] = gIntervals;  
  }
  else
  {
    gMoveTemp.m.sm.Steps[0] = gSteps1;    
  }
  gMoveTemp.m.sm.Accel[0] = gAccel1;
  gMoveTemp.m.sm.Jerk[0] = gJerk1;
  gMoveTemp.m.sm.Rate[1].value = gRate2;
  if (TimedMove)
  {
    gMoveTemp.m.sm.Steps[1] = 0;
  }
  else
  {
    gMoveTemp.m.sm.Steps[1] = gSteps2;
  }
  gMoveTemp.m.sm.Accel[1] = gAccel2;
  gMoveTemp.m.sm.Jerk[1] = gJerk2;
  if (TimedMove)
  {
    gMoveTemp.Command = COMMAND_LT_MOVE;
  }
  else
  {
    gMoveTemp.Command = COMMAND_LM_MOVE;
  }
  
  if(bittst(TestMode, TEST_MODE_DEBUG_COMMAND_NUM))
  {
    // Print the final values used by the ISR for this move
    ebb_print((far rom char *)"R1=");
    ebb_print_int(gMoveTemp.m.sm.Rate[0].value);
    ebb_print((far rom char *)" S1=");
    ebb_print_uint(gMoveTemp.m.sm.Steps[0]);
    ebb_print((far rom char *)" A1=");
    ebb_print_int(gMoveTemp.m.sm.Accel[0]);
    ebb_print((far rom char *)" J1=");
    ebb_print_int(gMoveTemp.m.sm.Jerk[0]);
    ebb_print((far rom char *)" R2=");
    ebb_print_int(gMoveTemp.m.sm.Rate[1].value);
    ebb_print((far rom char *)" S2=");
    ebb_print_uint(gMoveTemp.m.sm.Steps[1]);
    ebb_print((far rom char *)" A2=");
    ebb_print_int(gMoveTemp.m.sm.Accel[1]);
    ebb_print((far rom char *)" J2=");
    ebb_print_int(gMoveTemp.m.sm.Jerk[1]);
    ebb_print((far rom char *)" SE=");
    ebb_print_int(gMoveTemp.m.sm.SEState);
    print_line_ending(kLE_REV);
  }

  if (!bittst(TestMode, TEST_MODE_DEBUG_BLOCK_FIFO_NUM))
  {
    // Spin here until there's space in the FIFO
    while(gFIFOLength >= gCurrentFIFOLength)
      ;

    // If the limit switch feature has triggered, then ignore this move command
    // Maybe the limit switch has become true between the top of this function 
    // and here? Better check for it.
    if (!bittstzero(gLimitSwitchTriggered))
    {
      // Now, quick copy over the computed command data to the command FIFO
      FIFOPtr[gFIFOIn] = gMoveTemp;
      gFIFOIn++;
      if (gFIFOIn >= gCurrentFIFOLength)
      {
        gFIFOIn = 0;
      }
      gFIFOLength++;
    }
  }
}

// The Stepper Motor command
// Usage: SM,<move_duration>,<axis1_steps>,<axis2_steps>,<CleaAccs><CR>
// <move_duration> is 32 bit unsigned, indicating the number of milliseconds this move should take
// <axisX_steps> is a signed 32 bit number indicating how many steps (and what direction) the axis should take
// NOTE1: <axis2_steps> is optional and can be left off
// <ClearAccs> is an optional value of 0, 1, 2 or 3. 
//    0 will leave accumulators alone at the beginning of the move
//    1 will clear Motor1 accumulator and leave Motor2 accumulator
//    2 will clear Motor2 accumulator and leave Motor1 accumulator
//    3 will clear both Motor1 and Motor2 accumulators
// If the EBB can not make the move in the specified time, it will take as long as it needs to at max speed
// i.e. SM,1,1000 will not produce 1000 steps in 1ms. Instead, it will take 40ms (25KHz max step rate)
// NOTE2: If you specify zero steps for the axis, then you effectively create a delay. Use for small
// pauses before raising or lowering the pen, for example.
void parse_SM_packet(void)
{
  clear_parmaeter_globals();

  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number(kULONG, &gDurationMS, kREQUIRED);
  extract_number(kLONG,  &gSteps1,  kREQUIRED);
  extract_number(kLONG,  &gSteps2,  kOPTIONAL);
  extract_number(kULONG, &gClearAccs, kOPTIONAL);

  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  // Input parameters: gDurationMS, gSteps1, gSteps2, gClearAccs
  process_simple_motor_move_fp();

  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  print_line_ending(kLE_OK_NORM);
}

// The Circle Move command
// Usage: CM,<frequency>,<dest_x>,<dest_y>,<center_x>,<center_y>,<direction><CR>
//
// <dest_x>,<dest_y>,<center_x>,<center_y> are required signed 16 bit numbers
// <frequency> is a required unsigned integer from 2 to 25000
// <direction> is a required 0 for CW or 1 for CCW
// <ClearAccs> is an optional 0, 1, 2 or 3
//
// <dest_x>,<dest_y> are the relative coordinates where the arc should end up
// <center_x>,<center_y> are the relative coordinates where the center of the arc is
// <rate> is the step frequency in steps/second
// <direction> specifies which direction the arc should take
// <ClearAccs> is the same as for SM
//
void parse_CM_packet(void)
{
  /// TODO: This burden on the stack is huge - use globals for some of this
  UINT16 frequency = 0;
  INT32 dest_x = 0;
  INT32 dest_y = 0;
  INT32 center_x = 0;
  INT32 center_y = 0;
  UINT8 direction = 0;
  UINT32 rate = 0;
  UINT32 temp;
  INT16 sTemp16_1;
  INT16 sTemp16_2;
  UINT32 radius;
  UINT16 typ_seg_long;
  
  clear_parmaeter_globals();

  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number(kUINT,  &frequency,  kREQUIRED);
  extract_number(kLONG,  &dest_x, kREQUIRED);
  extract_number(kLONG,  &dest_y,  kREQUIRED);
  extract_number(kLONG,  &center_x,  kREQUIRED);
  extract_number(kLONG,  &center_y,  kREQUIRED);
  extract_number(kUCHAR, &direction,  kREQUIRED);

  if (gLimitChecks)
  {
    // direction can only be 0 or 1
    if (direction > 1u) 
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
    // Rate has to be from 1 to 25000
    if ((frequency < 1u) || (frequency > 25000u))
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
    // Check four positions for out of bounds
    if ((dest_x > 32768) || (dest_x < -32768))
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
    if ((dest_y > 32768) || (dest_y < -32768))
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
    if ((center_x > 32768) || (center_x < -32768))
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
    if ((center_y > 32768) || (center_y < -32768))
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
    
    // Bail if a there is a parameter limit error
    if (error_byte != kERROR_NO_ERROR)
    {
      return;
    }
  }

  // If we have a triggered limit switch, then ignore this move command
  if (bittstzero(gLimitSwitchTriggered))
  {
    return;
  }
  
  // Print out input parameters for debugging
  if(bittst(TestMode, TEST_MODE_DEBUG_COMMAND_NUM))
  {
    ebb_print((far rom char *)"Freq=");
    ebb_print_int(frequency);
    ebb_print((far rom char *)" Dest_X=");
    ebb_print_int(dest_x);
    ebb_print((far rom char *)" Dest_Y=");
    ebb_print_int(dest_y);
    ebb_print((far rom char *)" Center_X=");
    ebb_print_int(center_x);
    ebb_print((far rom char *)" Center_Y=");
    ebb_print_int(center_y);
    ebb_print((far rom char *)" Direction=");
    ebb_print_int(direction);
    print_line_ending(kLE_REV);
  }

#if 1
  ebb_print((far rom char *)"CM command disabled in this build");
  print_line_ending(kLE_REV);
#else  
  
  // Current "turtle" position, relative to arc center: x_t, y_t:
  gMoveTemp.m.cm.x_t = -center_x;   // int32
  gMoveTemp.m.cm.y_t = -center_y;   // int32
  
  // Final ("target") position, relative to arc center: x_f, y_f
  gMoveTemp.m.cm.x_f = dest_x - center_x; // int16
  gMoveTemp.m.cm.y_f = dest_y - center_y; // int16
  
  // Here's that square root.
  // The argument, (center_x * center_x + center_y * center_y), is up to 31 bits.
  temp = (center_x * center_x) + (center_y * center_y);  
  radius = Sqrt(temp);
  
  // Next calculate the "m_alpha" factor. This finds the smallest integer value m_alpha,
  //       such that 2^(2 * m_alpha + 3) >= radius
  //   Possible values of m_alpha: 0 - 6
  //       The radius can be as small as 5, such that 2^(2*0 + 3) = 8, already > radius 5.
  //           -> m_alpha = 0 if radius = 5.
  //       Since the radius can be as large as 32767, (2^15), 2*alpha + 3 = 15
  //           -> m_alpha = 6 if radius = 32767.
  
  // Once m_alpha is computed, each straight subsegment around the circle spans an arc of
  //   alpha = 1/(2^m_alpha) radians between its vertices. 
  // Since m_alpha is in the range 0-6, each arc angle is between 0.9� and 57.3� degrees:
  //   At smallest radius, (1/2^0) = 1; angle between vertices is 1 radian (57.3�).
  //   At largest radius, (1/2^6) = 1/64; angle between vertices is 1/64 =0.15625 radian, (0.9�).
  
  // Rather than computing alpha (usually fractional), we keep track of m_alpha, and use it
  //   with bit shifts.

  gMoveTemp.m.cm.m_alpha = 0; // uint8
  temp = 1 << ((2 * 0) + 3);  // left this way for clarity
  while (temp < radius)
  {
    gMoveTemp.m.cm.m_alpha += 1;
    temp = (1 << ((2 * gMoveTemp.m.cm.m_alpha) + 3)); // temp is uint16 (use bit shift instead of power)
  }

  // Next, compute what 1.5 * the typical subsegment length is.
  //
  // If the final vertex is within this distance of the start, just use a single
  //   move between them, rather than an arc with multiple subsegments.
  // 
  // The typical length of a subsegment around the circumference is:
  // (radius * angle_between_vertices_in_radians) = radius * alpha 
  //                                   = radius * (1/2^m_alpha) = radius >> m_alpha
  // 0.5 times the length is radius >> (m_alpha + 1)
  // Thus 1.5 times the length is (radius >> m_alpha) + (radius >> (m_alpha + 1)):

  gMoveTemp.m.cm.typ_seg = (radius >> gMoveTemp.m.cm.m_alpha);   // uint8
  typ_seg_long = gMoveTemp.m.cm.typ_seg + (radius >> (gMoveTemp.m.cm.m_alpha + 1));

  // Get absolute value of move length in X and Y
  sTemp16_1 = (gMoveTemp.m.cm.x_t - gMoveTemp.m.cm.x_f);  // signed 32 = signed 16 - signed 16
  if (sTemp16_1 < 0)
  {
    sTemp16_1 = -sTemp16_1;
  }

  sTemp16_2 = (gMoveTemp.m.cm.y_t - gMoveTemp.m.cm.y_f);  // signed 32 = signed 16 - signed 16
  if (sTemp16_2 < 0)
  {
    sTemp16_2 = -sTemp16_2;
  }
  
  // And if both x and y move length are too small, then just do a simple
  // straight move by calling process_simple_rate_move_fp()
  if ((sTemp16_1 < (INT16)typ_seg_long) && (sTemp16_2 < (INT16)typ_seg_long))
  {
    // This is a very short move; cannot make an arc here.
    // Instead, process it like an SM or HM move with:
    //      step_dist_x = x_f - x_t
    //      step_dist_y = y_f - y_t
    //      And step frequency given by StepFreq.
    gSteps1 = gMoveTemp.m.cm.x_f - gMoveTemp.m.cm.x_t;
    gSteps2 = gMoveTemp.m.cm.y_f - gMoveTemp.m.cm.y_t;
    gHM_StepRate = frequency;
    gClearAccs = 0;
    
    process_simple_rate_move_fp();
  
    // After making our short straight line, we're done.
    print_line_ending(kLE_OK_NORM);
    return;
  }

#if 0
   # Next: Precompute VScaleK (Velocity Scaling Factor K):
    # 
    # This is a scaling factor that we will later use, in the ISR, to calculate the 
    #   correct velocity in each axis from the number of steps in each axis.
    #
    # At a given vertex around the circle, the distance to the next vertex is (Dx, Dy).
    #   The velocity at that vertex is (K * Dx, K * Dy), where the velocity scale factor K
    #   is K = (Forward_velocity/typical_length_of_subsegment)
    #        = (StepFreq * 2^31/ 25000) / (radius * alpha)
    #        = (StepFreq * 2^28/ 3125) / (radius >> m_alpha)
    #        = (StepFreq * 2^28 << m_alpha ) / ( 3125 * radius)
    #   (Here, we have noted that 25000 = 8 * 3125 = 3125 * 2^3)
    #
    # To keep the multiplication within the ISR as 16 x 16 bit, we need a final VScaleK
    #   that can be represented by uint16 (without losing too many digits of precision).
    #   Most of the special cases are to manage the great variation in K that is possible
    #   because radius can range from 5-32767, and StepFreq can range from 1-25000.


    bits_left = 28                  # Counter for bits remaining to shift (int8)
    bits_left += m_alpha

    if (StepFreq >= 256):   # StepFreq is in range 256 - 25000
                            # Possibly implement by checking that high byte != 0.

        StepFreq = StepFreq << 17
        bits_left -= 17
        # StepFreq now in range 3.35E7 - 3.27E9 (Almost 2^25 - Almost 2^32)

    else:                   # StepFreq is in range 1 - 255
        StepFreq = StepFreq << 24
        bits_left -= 24
        # StepFreq now in range 1.67E7 - 4.27E9 (2^24 - Almost 2^32)


    denom = 3125 * radius # uint32
        # denom is in range 1.67E7 - 4.27E9 (2^24 - Almost 2^32)

    if (radius >= 256):     # Radius is in range 256 - 32767
                            # Possibly implement by checking that high byte != 0.

        denom = denom >> 11
        bits_left -= 11
        # Denom now in range 3906 - 50000 (Almost 2^12 - Almost 2^16)

    else:                   # Radius is in range 1 - 255
        denom = denom >> 1
        bits_left -= 1
        # Denom now in range 1562 - 39843 (Almost 2^11 - Almost 2^16)


    # Now the "big" division operation -- just one, thankfully.

    VScaleK = int(StepFreq / denom) # (uint24?) 
    # Largest possible VScaleK: 4.27E9 / 1562 = 2.733E6 (almost 2^22)
    # Smallest possible VScaleK: 1.67E7 / 50000 = 334 (almost 2^9)

    if VScaleK > 65536:             # possibly check that high bytes are nonzero.
        VScaleK = VScaleK >> 4
        bits_left += 4
    # VScaleK value range is now 334 - 42,703, all < 2^16.
    # ->  VScaleK can now be re-cast as a uint16.


    # Finally, set up variables to manage "local" position in ISR.


    x_pos_last = x_t # int16 Variables to store last (16-bit) position value
    y_pos_last = y_t # int16


    # Scale X, Y values up by 65536. The step positions are 16-bit signed, but
    #   we need additional resolution to avoid adding rounding errors since we
    #   do compute each new position from the last position.

    x_t = x_t << 16
    y_t = y_t << 16

    -----------------------------------------
    # This concludes the "precomputation" stage. We now -- still in the "parsing"
    #   stage -- compute the velocities and step counts for the first subsegment only.
    
    # The nominal step distances to move are:
    #   d_x = -1 * ( x_t >> (2 * m_alpha + 1)) - (y_t >> m_alpha)
    #   d_y = -1 * ( y_t >> (2 * m_alpha + 1)) + (x_t >> m_alpha)
    #
    # HOWEVER, right-shift division of negative integers isn't "clean" (e.g., -1 >> 1 == -1),
    #   so we instead use several cases with positive integers and right shifts.




    alpha_shift = (m_alpha << 1) + 1
    if x_t < 0:
        x_1 = ( - x_t >> alpha_shift) 
        x_2 = -( - x_t >> m_alpha) 
    else:
        x_1 = -( x_t >> alpha_shift) 
        x_2 = ( x_t >> m_alpha) 

    if y_t < 0:
        y_1 = ( - y_t >> alpha_shift) 
        y_2 = -( - y_t >> m_alpha) 
    else:
        y_1 = -( y_t >> alpha_shift) 
        y_2 = ( y_t >> m_alpha) 

    if direction:
        d_x = x_1 + y_2
        d_y = y_1 - x_2
    else:
        d_x = x_1 - y_2
        d_y = y_1 + x_2





    # Update 32-bit position values:
    x_t += d_x
    y_t += d_y

    # Find new 16-bit position value. 
    # In firmware, take two high bytes of position; do not actually shift/divide.

    x_t_16 = x_t >> 16 # Psuedocode only;(DO NOT ACTUALLY SHIFT)
    y_t_16 = y_t >> 16 # Psuedocode only;(DO NOT ACTUALLY SHIFT)

    # Find Step count and directions.
    x_steps = x_t_16 - x_pos_last
    y_steps = y_t_16 - y_pos_last


    if x_steps < 0:
        stepdir_x = 0 # Set direction bits
        x_stepcount = -x_steps
    else:
        stepdir_x = 1 # Set direction bits
        x_stepcount = x_steps
    if y_steps < 0:
        stepdir_y = 0 # Set direction bits
        y_stepcount = -y_steps
    else:
        stepdir_y = 1 # Set direction bits
        y_stepcount = y_steps

    # Rate factors: Rate_x, Rate_y : 32-bit unsigned
    if bits_left >= 0:
        Rate_x = (x_stepcount * VScaleK) << bits_left
        Rate_y = (y_stepcount * VScaleK) << bits_left
    else:
        Rate_x = (x_stepcount * VScaleK) >> -bits_left
        Rate_y = (y_stepcount * VScaleK) >> -bits_left

    x_pos_last = x_t_16
    y_pos_last = y_t_16

    Now, begin an SM/XM/HM type linear move with:
        * stepdir_x, stepdir_y
        * x_stepcount, y_stepcount
        * Rate_x, Rate_y


    In addition to the step/direction/rate values (normally in the FIFO), 
        I think we'll need the following values attached to this command:
        
            - VScaleK       # uint16
            - m_alpha       # uint8
            - bits_left     # int8
            - x_f           # int16
            - y_f           # int16
            - x_t           # int32
            - y_t           # int32
            - direction     # uint8
            - y_pos_last    # int16
            - x_pos_last    # int16
            - typ_seg       # uint8


#endif

  if (gAutomaticMotorEnable == TRUE)
  {
    // Enable both motors when we want to move them
    Enable1IO = ENABLE_MOTOR;
    Enable2IO = ENABLE_MOTOR;
  }

  // First, set the direction bits
  /// For CM, how do we know which direction we need to move first?
///  if (gTmpSteps1 < 0)
///  {
///    gMoveTemp.m.sm.DirBits = gMoveTemp.m.sm.DirBits | DIR1_BIT;
///    gTmpSteps1 = -gTmpSteps1;
///  }
///  if (gTmpSteps2 < 0)
///  {
///    gMoveTemp.m.sm.DirBits = gMoveTemp.m.sm.DirBits | DIR2_BIT;
///    gTmpSteps2 = -gTmpSteps2;
///  }

///  gMoveTemp.m.sm.Rate[0].value = gTmpIntervals;
///  gMoveTemp.m.sm.Steps[0] = gTmpSteps1;
///  gMoveTemp.m.sm.Accel[0] = 0;


///  gMoveTemp.m.sm.Rate[1].value = gTmpIntervals;
///  gMoveTemp.m.sm.Steps[1] = gTmpSteps2;
///  gMoveTemp.m.sm.Accel[1] = 0;
  gMoveTemp.Command = COMMAND_CM_OUTER_MOVE;

  if(bittst(TestMode, TEST_MODE_DEBUG_COMMAND_BIT_NUM))
  {
    ebb_print((far rom char *)"R1=");
    ebb_print_uint(gMoveTemp.m.sm.Rate[0].value);
    ebb_print((far rom char *)" S1=");
    ebb_print_uint(gMoveTemp.m.sm.Steps[0]);
    ebb_print((far rom char *)" R2=");
    ebb_print_uint(gMoveTemp.m.sm.Rate[1].value);
    ebb_print((far rom char *)" S2=");
    ebb_print_uint(gMoveTemp.m.sm.Steps[1]);
    print_line_ending(kLE_REV);
  }
  
  // Spin here until there's space in the FIFO
  while(gFIFOLength >= gCurrentFIFOLength)
  ;
  
  // If the limit switch feature has triggered, then ignore this move command
  // Maybe the limit switch has become true between the top of this function 
  // and here? Better check for it.
  if (!bittstzero(gLimitSwitchTriggered))
  {
    // Now, quick copy over the computed command data to the command FIFO
    FIFOPtr[gFIFOIn] = gMoveTemp;
    gFIFOIn++;
    if (gFIFOIn >= gCurrentFIFOLength)
    {
      gFIFOIn = 0;
    }
    gFIFOLength++;
  }
#endif
  
  print_line_ending(kLE_OK_NORM);
}

// Home the motors
// "HM,<StepFrequency>,<Pos1>,<Pos2><CR>"
// <StepFrequency> is the desired rate of the primary (larger) axis in steps/s.
//   It is allowed to be from 1 to 25000. 
// <Pos1> and <Pos2> are both optional. If <Pos1> is present, <Pos2> must
// also be present. If not present they are both assumed to be 0 (i.e.
// a true 'home' move). If present, they will instruct the EBB to perform
// a move to the absolute position <Pos1>,<Pos2>.
// <Pos1> and <Pos2> are both signed 32 bit integers. Values from 
// -2147483648 to 2147483647 are allowed for each.
//
// This command uses the current global step counts compute the number of
// steps necessary to reach the target position. (either 0,0 or Pos1,Pos2)
//
// There is a limitation to <Pos1> and <Pos2>. When they are each added
// to the negative of the current global position to compute the number of
// steps necessary to complete this HM move, the sum must not overflow a
// signed 32 bit number. An overflow like this will generate an error.
// Note that this situation is very unlikely to occur, as 0x7FFFFFFF steps
// (the maximum number of steps a signed 32 bit int can represent) will take
// 23 hours to execute at the highest step rate (25Ks/s).
//
// The primary axis will be whichever one has to move more steps.
// The secondary axis will be the non-primary axis.
//
// For this function, if <StepFrequency> is too high or too low, we don't error 
// out, we just use a legal value instead (25,000 or 1 s/s).
//
// When parsing this command, we always wait until both the FIFO is empty and 
// the motion commands are finished. By waiting until both the FIFO (queue) and 
// motion commands are empty, we get an accurate picture of where the global 
// step position is.
//
// Once the move has been figured out, we call process_simple_rate_move_fp().
//
// Some global variables are used in this function instead of local ones:
//  gHM_StepRate is used for StepRate
//  gHM_Pos1 is used for Pos1
//  gHM_Pos2 is used for Pos2
//  gSteps1 is used as parameter for process_simple_rate_move
//  gSteps2 is used as parameter for process_simple_rate_move
void parse_HM_packet(void)
{
  BOOL   CommandExecuting = TRUE;

  clear_parmaeter_globals();

  print_command(FALSE, FALSE);

  // Extract the three parameters
  extract_number(kULONG, &gHM_StepRate, kREQUIRED);
  extract_number(kLONG,  &gHM_Pos1,     kOPTIONAL);
  extract_number(kLONG,  &gHM_Pos2,     kOPTIONAL);

  // StepRate must be from 1 to 25000
  if (gHM_StepRate < 1u)
  {
    gHM_StepRate = 1;
  }
  else if (gHM_StepRate > 25000u)
  {
    gHM_StepRate = 25000;
  }
  
  // Wait until FIFO is empty
  while(gFIFOLength >= gCurrentFIFOLength)
    ;

  // Then wait for motion command to finish (if one's running)
  while (CommandExecuting == TRUE)
  {
    // Need to turn off high priority interrupts briefly here to read out value that ISR uses
    INTCONbits.GIEH = 0;    // Turn high priority interrupts off

    // See if there is either a delay or a command executing
    if ((CurrentCommand.m.sm.DelayCounter == 0u) && (CurrentCommand.Command == COMMAND_NONE))
    {
      CommandExecuting = FALSE;
    }

    // Re-enable interrupts
    INTCONbits.GIEH = 1;    // Turn high priority interrupts on
  }

  // Make a local copy of the things we care about. This is how far we need to move.
  gSteps1 = -globalStepCounter1 + gHM_Pos1;
  gSteps2 = -globalStepCounter2 + gHM_Pos2;
  
  // Always clear accumulators for this type of move
  gClearAccs = 0;
  
  // Set up these values as input parameters
  //  gSRM_Rate
  //  gSteps1
  //  gSteps2
  //  gClearAccs
  gSRM_Rate = gHM_StepRate;
  process_simple_rate_move_fp();

  print_line_ending(kLE_OK_NORM);
}

// Test Rate
// Simple function to allow testing of the process_simple_rate_move_fp() command.
// Simply take the parameters from the user and pass them on to the move.
void parse_TR_packet(void)
{
  clear_parmaeter_globals();

  print_command(FALSE, FALSE);

  // Extract the three parameters
  extract_number(kULONG, &gHM_StepRate, kREQUIRED);
  extract_number(kLONG,  &gSteps1,      kREQUIRED);
  extract_number(kLONG,  &gSteps2,      kREQUIRED);
  extract_number(kULONG, &gClearAccs,   kOPTIONAL);

  // Set up these values as input parameters
  //  gHM_StepRate
  //  gSteps1
  //  gSteps2
  //  gClearAccs
  process_simple_rate_move_fp();

  print_line_ending(kLE_OK_NORM);
}

// This is the generic function for processing simple moves which use a 
// 'rate' input parameter rather than a duration. The 'rate' parameter
// is applied to the axis that has more steps (primary axis). The other axis
// (secondary axis) gets a rate which is appropriate to making a straight line.
// gHM_StepRate can be between 1 and 25000 inclusive
// gSteps1 and gSteps2 are signed 32 bit numbers
//
// This function uses these as input parameters:
//  gSRM_Rate   UINT32  not modified
//  gSteps1     INT32   modified
//  gSteps2     INT32   modified
//  gClearAccs  UINT32  not modified
//
void process_simple_rate_move_fp(void)
{
  float tempF;
  
  if(bittst(TestMode, TEST_MODE_DEBUG_COMMAND_NUM))
  {
    ebb_print((far rom char *)"Rate=");
    ebb_print_uint(gHM_StepRate);
    ebb_print((far rom char *)" S1=");
    ebb_print_int(gSteps1);
    ebb_print((far rom char *)" S2=");
    ebb_print_int(gSteps2);
    print_line_ending(kLE_REV);
  }
  // If we have a triggered limit switch, then ignore this move command
  if (bittstzero(gLimitSwitchTriggered))
  {
    return;
  }
  // And check for a bad rate value
  if ((gHM_StepRate > 25000u) || (gHM_StepRate == 0u))
  {
    ErrorSet(kERROR_STEP_RATE_INVALID);
    return;
  }
  // If, for some reason, we're called with zero steps for both axis, then bail
  if ((gSteps1 == 0) && (gSteps2 == 0))
  {
    return;
  }

  gMoveTemp.m.sm.SEState = (UINT8)gClearAccs;

  gMoveTemp.m.sm.DelayCounter = 0;    // A rate move is never a delay, so clear
  gMoveTemp.m.sm.DirBits = 0;         // Start with positive direction for both

  if (gAutomaticMotorEnable == TRUE)
  {
    // Enable both motors when we want to move them
    Enable1IO = ENABLE_MOTOR;
    Enable2IO = ENABLE_MOTOR;
  }

  // First, set the direction bits
  if (gSteps1 < 0)
  {
    gMoveTemp.m.sm.DirBits = gMoveTemp.m.sm.DirBits | DIR1_BIT;
    gSteps1 = -gSteps1;
  }
  if (gSteps2 < 0)
  {
    gMoveTemp.m.sm.DirBits = gMoveTemp.m.sm.DirBits | DIR2_BIT;
    gSteps2 = -gSteps2;
  }
  
  // Both Steps values are now positive because our directions are set. From 
  // now on we're dealing only with step counts and rates, no directions.
  
  // Figure out which axis is major
  if (gSteps1 >= gSteps2)
  {
    // Axis1 is major, Axis2 is minor
    // Computing the gMoveTemp.m.sm.Rate value for the primary axis is simple.
    // gMoveTemp.m.sm.Rate is simply gHM_StepRate * 85899.34592.
    gMoveTemp.m.sm.Rate[0].value = (INT32)((float)gHM_StepRate * 85899.34592f);

    if (gSteps2 != 0)
    {
      // We know secondary axis is non-zero move length, so compute it's rate
      // such that the ratio of primary steps/rate is the same as secondary
      // steps/rate.
      //
      // The ratio of steps/rate will need to be the same for both axis
      // to make a straight line. So if S1 is axis1 steps, S2 is axis2 steps,
      // R1 is axis1 rate and R2 is axis2 rate, then
      // S1/R1 = S2/R2.
      // We know all but R2, so solving for R2 we get
      // R2 = (S2*R1)/S1
      // We break one of the EBB firmware rules here and use floating point
      // operations, as it is the perfect solution to maintaining as much
      // resolution as possible when computing the secondary rate. And it's not
      // even that slow - both the above primary rate multiply and the following
      // two multiplies and a divide, this whole function take less than 350 uS.
      
      tempF = (float)gSteps2 * (float)gHM_StepRate;
      tempF /= (float)gSteps1;
      tempF *= 85899.34592f;
      
      // Limit check out results
      if (tempF < 1.0f)
      {
        gMoveTemp.m.sm.Rate[1].value = 1;
      }
      else if (tempF > 2147483647.0f)
      {
        gMoveTemp.m.sm.Rate[1].value = 2147483647u;
      }
      else
      {
        gMoveTemp.m.sm.Rate[1].value = (INT32)tempF;
      }
    }
    else
    {
      // Because we know Axis2 doesn't need to move, zero everything out
      gMoveTemp.m.sm.Rate[1].value = 0;
    }
  }
  else
  {
    // Axis2 is major, Axis1 is minor, other than that same as above
    gMoveTemp.m.sm.Rate[1].value = (INT32)((float)gHM_StepRate * 85899.34592f);

    if (gSteps1 != 0)
    {
      tempF = (float)gSteps1 * (float)gHM_StepRate;
      tempF /= (float)gSteps2;
      tempF *= 85899.34592f;
      
      if (tempF < 1.0f)
      {
        gMoveTemp.m.sm.Rate[0].value = 1;
      }
      else if (tempF > 2147483647.0f)
      {
        gMoveTemp.m.sm.Rate[0].value = 2147483647u;
      }
      else
      {
        gMoveTemp.m.sm.Rate[0].value = (INT32)tempF;
      }
    }
    else
    {
      gMoveTemp.m.sm.Rate[0].value = 0;
    }
  }

  gMoveTemp.m.sm.Steps[0] = gSteps1;
  gMoveTemp.m.sm.Accel[0] = 0;
  gMoveTemp.m.sm.Steps[1] = gSteps2;
  gMoveTemp.m.sm.Accel[1] = 0;
  gMoveTemp.Command = COMMAND_SM_XM_HM_MOVE;

  if(bittst(TestMode, TEST_MODE_DEBUG_COMMAND_NUM))
  {
    ebb_print((far rom char *)"R1=");
    ebb_print_uint(gMoveTemp.m.sm.Rate[0].value);
    ebb_print((far rom char *)" S1=");
    ebb_print_uint(gMoveTemp.m.sm.Steps[0]);
    ebb_print((far rom char *)" R2=");
    ebb_print_uint(gMoveTemp.m.sm.Rate[1].value);
    ebb_print((far rom char *)" S2=");
    ebb_print_uint(gMoveTemp.m.sm.Steps[1]);
    print_line_ending(kLE_REV);
  }

  // Spin here until there's space in the FIFO
  while (gFIFOLength >= gCurrentFIFOLength)
  ;

  // If the limit switch feature has triggered, then ignore this move command
  // Maybe the limit switch has become true between the top of this function 
  // and here? Better check for it.
  if (!bittstzero(gLimitSwitchTriggered))
  {
    // Now, quick copy over the computed command data to the command FIFO
    FIFOPtr[gFIFOIn] = gMoveTemp;
    gFIFOIn++;
    if (gFIFOIn >= gCurrentFIFOLength)
    {
      gFIFOIn = 0;
    }
    gFIFOLength++;
  }
}

// The X Stepper Motor command
// Usage: XM,<move_duration>,<axisA_steps>,<axisB_steps><CR>
// <move_duration> an unsigned 32 bit value indicating the number of milliseconds this move should take
// <axisA_steps> and <axisB_stetsp> are signed 32 bit numbers
// This command differs from the normal "SM" command in that it is designed to drive 'mixed-axis' geometry
// machines like H-Bot and CoreXY. Using XM will effectively call SM with Axis1 = <axisA_steps> + <axisB_steps> and
// Axis2 = <axisA_steps> - <axisB_steps>.
// Because of this, the caller must make sure that <axisA_steps> + <axisB_steps> and <axisA_steps> - <axisB_steps>
// do not overflow or underflow a 32 bit signed value.
//
// Temporary globals used in this function
//   gXM_ASteps will be used for ASteps
//   gXM_BSteps will be used for BSteps

void parse_XM_packet(void)
{
  clear_parmaeter_globals();

  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number(kULONG, &gDurationMS, kREQUIRED);
  extract_number(kLONG, &gXM_ASteps, kREQUIRED);
  extract_number(kLONG, &gXM_BSteps, kREQUIRED);
  extract_number(kULONG, &gClearAccs, kOPTIONAL);
  
  // Do the math to convert to Axis1 and Axis2
  gSteps1 = gXM_ASteps + gXM_BSteps;
  gSteps2 = gXM_ASteps - gXM_BSteps;
    
  // Input parameters: gDurationMS, gSteps1, gSteps2, gClearAccs
  process_simple_motor_move_fp();

  print_line_ending(kLE_OK_NORM);
}

// Main stepper move function. This is the reason EBB exists.
// <Duration> is a 32 bit unsigned int, the number of mS that the move should take
// <A1Stp> and <A2Stp> are the Axis 1 and Axis 2 number of steps to take in
//  <Duration> mS, as 32 bit signed values, where the sign determines the motor
//  direction.
// <ClearAccs> clears the accumulators (both if 3, none if 0)
// This function waits until there is room in the FIFO before placing
// the data in the FIFO.
//
// <Duration>, <A1Stp> and <A2Stp> can accept any values. However, if a step 
// rate of more than 25kHz is asked for, it will be capped at 25kHz. And if
// a step rate of less than 0.00001164 Hz it will be set to 0.00001164 Hz.
//
// Note that if used as a delay, the duration value is capped at 100000ms.
//
// Because we are now using global values to pass in parameters to this
// function, and because this function will modify the values 'passed in' to it
// the caller should make copies of any parameter values that it requires
// after the call.
//
// This function uses these as input parameters:
//  gTmpDurationMS    UINT32 not modified
//  gTmpSteps1        INT32  modified
//  gTmpSteps2        INT32  modified
//  gTmpClearAccs     UINT32 modified
//
// Working values:
//  gTmpIntervals     UINT32 modified
//
static void process_simple_motor_move_fp(void)
{
  float tempF;
  
  // If we have a triggered limit switch, then ignore this move command
  if (bittstzero(gLimitSwitchTriggered))
  {
    return;
  }
  if(bittst(TestMode, TEST_MODE_DEBUG_COMMAND_NUM))
  {
    ebb_print((far rom char *)"Duration=");
    ebb_print_uint(gDurationMS);
    ebb_print((far rom char *)" SA1=");
    ebb_print_int(gSteps1);
    ebb_print((far rom char *)" SA2=");
    ebb_print_int(gSteps2);
    print_line_ending(kLE_REV);
  }
  
  if (gLimitChecks)
  {
    // Check for invalid duration
    if (gDurationMS == 0u) 
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
    if (gClearAccs > 3u)
    {
      ErrorSet(kERROR_PARAMETER_OUTSIDE_LIMIT);
      return;
    }
  }
  
  gMoveTemp.m.sm.SEState = (UINT8)gClearAccs;

  // Check for delay
  if (gSteps1 == 0 && gSteps2 == 0)
  {
    gMoveTemp.Command = COMMAND_DELAY;
    
    // Delays over 100000ms long are capped at 100000ms.
    if (gDurationMS >= 100000u)
    {
      gDurationMS = 100000u;
    }
    gMoveTemp.m.sm.DelayCounter = HIGH_ISR_TICKS_PER_MS * gDurationMS;
  }
  else
  {
    gMoveTemp.m.sm.DelayCounter = 0; // No delay for motor moves
    gMoveTemp.m.sm.DirBits = 0;

    if (gAutomaticMotorEnable == TRUE)
    {
      // Enable both motors when we want to move them
      Enable1IO = ENABLE_MOTOR;
      Enable2IO = ENABLE_MOTOR;
    }
    
    // First, set the direction bits
    if (gSteps1 < 0)
    {
      gMoveTemp.m.sm.DirBits = gMoveTemp.m.sm.DirBits | DIR1_BIT;
      gSteps1 = -gSteps1;
    }
    if (gSteps2 < 0)
    {
      gMoveTemp.m.sm.DirBits = gMoveTemp.m.sm.DirBits | DIR2_BIT;
      gSteps2 = -gSteps2;
    }

    // As an optimization, pre-compute the common term used for both axes
    tempF = 85899345.92f / (float)gDurationMS;
    
    if (gSteps1 != 0)
    {
      // We need the Rate values for the ISR. We have the duration and the
      // number of steps. Use the formula:
      // rate = (steps/duration)*85899.34592
      // Rearranging a bit, we can also use:
      // rate = steps * ((85899.34592 * 1000)/duration)
      gIntervals = (UINT32)((float)(gSteps1) * tempF);
      
      if (gIntervals > 0x80000000u)
      {
        if (gLimitChecks)
        {
          if (bittstzero(gStandardizedCommandFormat))
          {
            ebb_print_char(',');
          }
          ebb_print((far rom char *)"!0 Err: <axis1> step rate too high");
          print_line_ending(kLE_REV);
          ErrorSet(kERROR_PRINTED_ERROR);
          return;
        }
        gIntervals = 0x80000000;
      }
      if (gIntervals == 0u)
      {
        if (gLimitChecks)
        {
          if (bittstzero(gStandardizedCommandFormat))
          {
            ebb_print_char(',');
          }
          ebb_print((far rom char *)"!0 Err: <axis1> step rate too slow");
          print_line_ending(kLE_REV);
          ErrorSet(kERROR_PRINTED_ERROR);
          return;
        }
        gIntervals = 1;
      }
      gMoveTemp.m.sm.Rate[0].value = gIntervals;
    }
    else
    {
      gMoveTemp.m.sm.Rate[0].value = 0;
    }
    gMoveTemp.m.sm.Steps[0] = (UINT32)gSteps1;
    gMoveTemp.m.sm.Accel[0] = 0;

    if (gSteps2 != 0)
    {
      gIntervals = (UINT32)((float)(gSteps2) * tempF);

      if (gIntervals > 0x80000000u)
      {
        if (gLimitChecks)
        {
          if (bittstzero(gStandardizedCommandFormat))
          {
            ebb_print_char(',');
          }
          ebb_print((far rom char *)"!0 Err: <axis2> step rate too high");
          print_line_ending(kLE_REV);
          ErrorSet(kERROR_PRINTED_ERROR);
          return;
        }
        gIntervals = 0x80000000;
      }
      if (gIntervals == 0u)
      {
        if (gLimitChecks)
        {
          if (bittstzero(gStandardizedCommandFormat))
          {
            ebb_print_char(',');
          }
          ebb_print((far rom char *)"!0 Err: <axis2> step rate too slow");
          print_line_ending(kLE_REV);
          ErrorSet(kERROR_PRINTED_ERROR);
          return;
        }
        gIntervals = 1;
      }
      gMoveTemp.m.sm.Rate[1].value = gIntervals;
    }
    else
    {
      gMoveTemp.m.sm.Rate[1].value = 0;
    }
    gMoveTemp.m.sm.Steps[1] = (UINT32)gSteps2;
    gMoveTemp.m.sm.Accel[1] = 0;
    gMoveTemp.Command = COMMAND_SM_XM_HM_MOVE;

    if(bittst(TestMode, TEST_MODE_DEBUG_COMMAND_NUM))
    {
      ebb_print((far rom char *)"R1=");
      ebb_print_uint(gMoveTemp.m.sm.Rate[0].value);
      ebb_print((far rom char *)" S1=");
      ebb_print_uint(gMoveTemp.m.sm.Steps[0]);
      ebb_print((far rom char *)" R2=");
      ebb_print_uint(gMoveTemp.m.sm.Rate[1].value);
      ebb_print((far rom char *)" S2=");
      ebb_print_uint(gMoveTemp.m.sm.Steps[1]);
      print_line_ending(kLE_REV);
    }
  }

  if (!bittst(TestMode, TEST_MODE_DEBUG_BLOCK_FIFO_NUM))
  {
    // Spin here until there's space in the FIFO
    while(gFIFOLength >= gCurrentFIFOLength)
    ;

    // If the limit switch feature has triggered, then ignore this move command
    // Maybe the limit switch has become true between the top of this function 
    // and here? Better check for it.
    if (!bittstzero(gLimitSwitchTriggered))
    {
      // Now, quick copy over the computed command data to the command FIFO
      FIFOPtr[gFIFOIn] = gMoveTemp;
      gFIFOIn++;
      if (gFIFOIn >= gCurrentFIFOLength)
      {
        gFIFOIn = 0;
      }
      gFIFOLength++;
    }
  }
}

// E-Stop
// Usage: ES,<disable_motors><CR>
// Returns: <command_interrupted><CR>OK<CR>
// This command will abort any in-progress motor move command.
// It will also clear out any pending command(s) in the FIFO.
// <disable_motors> This parameter is optional. If present, and if it is a 1,
//                  then both stepper drivers will be disabled as part of the command.
//                  If this parameter is not present or is not equal to 1, then
//                  the motors will not be disabled. (added in v2.8.0)
// <command_interrupted> = 0 if no FIFO or in-progress move commands were interrupted,
//                         1 if a motor move command was in progress or in the FIFO
void parse_ES_packet(void)
{
  UINT8 disable_motors = 0;
  UINT8 command_interrupted = 0;
  UINT8 i;

  print_command(FALSE, TRUE);

  // Extract each of the value.
  extract_number(kUCHAR, &disable_motors, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  // Need to turn off high priority interrupts briefly here to mess with ISR command parameters
  INTCONbits.GIEH = 0;  // Turn high priority interrupts off

  if 
  (
    CurrentCommand.Command == COMMAND_SM_XM_HM_MOVE
    ||
    CurrentCommand.Command == COMMAND_LM_MOVE
    ||
    CurrentCommand.Command == COMMAND_LT_MOVE
  )
  {
    command_interrupted = 1;
  }
  
  if (gFIFOLength != 0u)
  {
    command_interrupted = 1;
  }

  // Clear the currently executing motion command
  CurrentCommand.Command = COMMAND_NONE;

  // Clear out the entire FIFO
  for (i = 0; i < COMMAND_FIFO_MAX_LENGTH; i++)
  {
    FIFOPtr[i].Command = COMMAND_NONE;
  }
  gFIFOIn = 0;
  gFIFOOut = 0;
  gFIFOLength = 0;

  if (disable_motors == 1u)
  {
    if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
    {
      Enable1IO = DISABLE_MOTOR;
      Enable2IO = DISABLE_MOTOR;
    }
    else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
    {
      Enable1AltIO = DISABLE_MOTOR;
      Enable2AltIO = DISABLE_MOTOR;
    }
  }
  
  // Re-enable interrupts
  INTCONbits.GIEH = 1;    // Turn high priority interrupts on

  ebb_print_uint(command_interrupted);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_REV);
  }

  print_line_ending(kLE_OK_NORM);
}

// Query Pen
// Usage: QP<CR>
// Returns: 0 for down, 1 for up, then OK<CR>
void parse_QP_packet(void)
{
  print_command(FALSE, TRUE);

  ebb_print_uint(PenState);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_REV);
  }

  print_line_ending(kLE_OK_NORM);
}

// Query motor Enables and resolution
// Usage: QE<CR>
// Returns: <motor1_state>,<motor2_state><CR>OK<CR>
// Where <motor1_state> and <motor2_state> are one of the following values:
// 0 = Motor is disabled
// 1 = Motor is enabled and step resolution is full steps
// 2 = Motor is enabled and step resolution is 1/2 steps
// 4 = Motor is enabled and step resolution is 1/4 steps
// 8 = Motor is enabled and step resolution is 1/8 steps
// 16 = Motor is enabled and step resolution is 1/16 steps
void parse_QE_packet(void)
{
  UINT8 motor1_state = 0;
  UINT8 motor2_state = 0;
  UINT8 temp;
  
  print_command(FALSE, TRUE);

  if (MS1_IO_PORT == 0u && MS2_IO_PORT == 0u && MS3_IO_PORT == 0u)
  {
    temp = 1;
  }
  else if (MS1_IO_PORT == 1u && MS2_IO_PORT == 0u && MS3_IO_PORT == 0u)
  {
    temp = 2;
  }
  else if (MS1_IO_PORT == 0u && MS2_IO_PORT == 1u && MS3_IO_PORT == 0u)
  {
    temp = 4;
  }
  else if (MS1_IO_PORT == 1u && MS2_IO_PORT == 1u && MS3_IO_PORT == 0u)
  {
    temp = 8;
  }
  else
  {
    temp = 16;
  }
  
  if (DriverConfiguration == PIC_CONTROLS_DRIVERS)
  {
    if (Enable1IO_PORT == ENABLE_MOTOR)
    {
      motor1_state = temp;
    }
    if (Enable2IO_PORT == ENABLE_MOTOR)
    {
      motor2_state = temp;
    }
  }
  else if (DriverConfiguration == PIC_CONTROLS_EXTERNAL)
  {
    if (Enable1AltIO_PORT == ENABLE_MOTOR)
    {
      motor1_state = temp;
    }
    if (Enable2AltIO_PORT == ENABLE_MOTOR)
    {
      motor2_state = temp;
    }
  }

  ebb_print_uint(motor1_state);
  ebb_print_char(',');
  ebb_print_uint(motor2_state);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_REV);
  }
  print_line_ending(kLE_OK_NORM);
}

// Toggle Pen
// Usage: TP,<duration><CR>
// Returns: OK<CR>
// <duration> is optional, and defaults to 0mS
// Just toggles state of pen arm, then delays for the optional <duration>
// Duration is in units of 1ms
void parse_TP_packet(void)
{
  UINT16 CommandDuration = 0;

  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number (kUINT, &CommandDuration, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  if (PenState == PEN_UP)
  {
    process_SP(PEN_DOWN, CommandDuration);
  }
  else
  {
    process_SP(PEN_UP, CommandDuration);
  }

  print_line_ending(kLE_OK_NORM);
}

// Set Pen
// Usage: SP,<State>,<Duration>,<PortB_Pin><CR>
// <State> is 
//   0 : goto servo_max using a command put into FIFO (servo pen down)
//   1 : goto servo_min using a command put into FIFO (servo pen up)
//   2 : goto servo_min but skip the FIFO and just force a move immediately
//   3 : goto servo_min, skip the FIFO, and also set servo_max to servo_min
//       (this prevents any remaining servo moves in the FIFO from having any effect)
// <Duration> is how long to wait before the next command in the motion control 
//      FIFO should start. (defaults to 0mS)
//      Note that the units of this parameter is in units of milliseconds
// <PortB_Pin> Is a value from 0 to 7 and allows you to re-assign the Pen
//      RC Servo output to different PortB pins.
// This is a command that the user can send from the PC to set the pen state.
// Note that there is only one pen RC servo output - if you use the <PortB_Pin>
// parameter, then that new pin becomes the pen RC servo output. This command
// does not allow for multiple servo signals at the same time from port B pins.
// Use the S2 command for that.
//
// This function will use the values for <serv_min>, <servo_max>,
// <servo_rate_up> and <servo_rate_down> (SC,4 SC,5, SC,11, SC,10 commands)
// when it schedules the servo command.
// 
// Internally, the parse_SP_packet() function makes a call to
// process_SP() function to actually make the change in the servo output.
//
void parse_SP_packet(void)
{
  UINT8 State = 0;
  UINT16 CommandDuration = 0;
  UINT8 Pin = DEFAULT_EBB_SERVO_PORTB_PIN;
  ExtractReturnType Ret;

  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number(kUCHAR, &State, kREQUIRED);
  extract_number(kUINT, &CommandDuration, kOPTIONAL);
  Ret = extract_number(kUCHAR, &Pin, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  // Error check
  if (Pin > 7u)
  {
    Pin = DEFAULT_EBB_SERVO_PORTB_PIN;
  }

  if (State > 3u)
  {
    State = 3;
  }

  // Set the PRn of the Pen Servo output
  // Add 3 to get from PORTB pin number to RPn number
  if (g_servo2_RPn != (Pin + 3))
  {
    // if we are changing which pin the pen servo is on, we need to cancel
    // the servo output on the old channel first
    RCServo2_Move(0, g_servo2_RPn, 0, 0, FALSE);
    // Now record the new RPn
    g_servo2_RPn = Pin + 3;
  }

  // Execute the servo state change
  process_SP(State, CommandDuration);

  print_line_ending(kLE_OK_NORM);
}

// Internal use function -
// Perform a state change on the pen RC servo output. Move it up or move it down
// <NewState> is   
//   PEN_DOWN : Add move to FIFO to start servo towards servo_max
//   PEN_UP : Add move to FIFO to start servo towards servo_min
//   PEN_UP_IMMEDIATE : Start servo moving towards servo_min, not on FIFO
//   PEN_UP_IMMEDIATE_CLEAR : Start servo moving towards servo_min, not on FIFO, and set servo_max = servo_min
// <CommandDuration> is the number of milliseconds to wait before executing the
//      next command in the motion control FIFO. Only applies if <NewState> is 0 or 1.
//
// This function uses the g_servo2_min (servo_min), g_servo2_max (servo_max), rate_up, rate_down variables
// to schedule an RC Servo change with the RCServo2_Move() function.
//
void process_SP(PenStateType NewState, UINT16 CommandDuration)
{
  UINT16 Position;
  UINT16 Rate;
  BOOL UseFIFO;
  UINT8 i;

  if (NewState == PEN_UP)
  {
    Position = g_servo2_min;
    Rate = g_servo2_rate_up;
    UseFIFO = TRUE;
  }
  else if (NewState == PEN_UP_IMMEDIATE)
  {
    Position = g_servo2_min;
    Rate = g_servo2_rate_up;
    UseFIFO = FALSE;
  }
  else if (NewState == PEN_UP_IMMEDIATE_CLEAR)
  {
    Position = g_servo2_min;
    Rate = g_servo2_rate_up;
    UseFIFO = FALSE;
    g_servo2_max = g_servo2_min;
    
    // Need to turn off high priority interrupts briefly here. We need to stop
    // any changes to the FIFO, then walk through every command currently
    // in the FIFO. If that command is a servo move, we have to re-write it's
    // position target to be g_servo2_min (all the way up). This will then
    // prevent any remaining servo moves in the FIFO from moving the pen down.
    INTCONbits.GIEH = 0;  // Turn high priority interrupts off

    for (i=0; i < COMMAND_FIFO_MAX_LENGTH; i++)
    {
      if (FIFOPtr[i].Command == COMMAND_SERVO_MOVE)
      {
        // Check to make sure we only modify moves of the pen servo output
        if (FIFOPtr[i].m.sm.ServoRPn == g_servo2_RPn)
        {
          FIFOPtr[i].m.sm.ServoPosition = g_servo2_min;
        }
      }
    }
    
    // Re-enable interrupts
    INTCONbits.GIEH = 1;  // Turn high priority interrupts on
  }
  else
  {
    Position = g_servo2_max;
    Rate = g_servo2_rate_down;
    UseFIFO = TRUE;
  }

  RCServoPowerIO = RCSERVO_POWER_ON;
  gRCServoPoweroffCounterMS = gRCServoPoweroffCounterReloadMS;

  // Now schedule the movement with the RCServo2 function
  RCServo2_Move(Position, g_servo2_RPn, Rate, CommandDuration, UseFIFO);
}

// Enable Motor
// Usage: EM,<EnableAxis1>,<EnableAxis2><CR>
// Everything after EnableAxis1 is optional
// Each parameter can have a value of
//    0 to disable that motor driver
// (only first parameter applies, and it applies to both drivers)
//    1 to enable the driver in 1/16th step mode
//    2 to enable the driver in 1/8 step mode
//    3 to enable the driver in 1/4 step mode
//    4 to enable the driver in 1/2 step mode
//    5 to enable the driver in full step mode
// If you disable a motor, it goes 'limp' (we clear the ENABLE pin on that motor's
// driver chip)
// Note that when using 0 or 1 for a parameter, you can use both axis even
// on a 'new' driver chip board. (i.e. EM,0,1 will disable motor 1 and enable 2)
// Note that the MSx lines do not come to any headers, so even when an external
// source is controlling the drivers, the PIC still needs to control the
// MSx lines.
// As of 2.7.0 : We always clear out the step accumulators when the EM command
// As of 2.8.0 : This command is now handled as part of the motion queue
//   is executed.
void parse_EM_packet(void)
{
  unsigned char EA1, EA2;
  ExtractReturnType RetVal1, RetVal2;

  print_command(FALSE, FALSE);

  // Extract each of the values.
  RetVal1 = extract_number (kUCHAR, &EA1, kREQUIRED);
  // Bail if we got a conversion error
  if ((error_byte != kERROR_NO_ERROR) || (kEXTRACT_OK != RetVal1))
  {
    return;
  }
  RetVal2 = extract_number (kUCHAR, &EA2, kOPTIONAL);
  // Bail if we got a conversion error
  if ((error_byte != kERROR_NO_ERROR) || (kEXTRACT_OK != RetVal2))
  {
    return;
  }

  if (!bittst(TestMode, TEST_MODE_DEBUG_BLOCK_FIFO_NUM))
  {
    // Trial: Spin here until there's space in the fifo
    while(gFIFOLength >= gCurrentFIFOLength)
      ;

    // Set up the motion queue command
    FIFOPtr[gFIFOIn].m.sm.DirBits = EA1;
    FIFOPtr[gFIFOIn].m.sm.ServoRPn = EA2;
    FIFOPtr[gFIFOIn].Command = COMMAND_EM;

    gFIFOIn++;
    if (gFIFOIn >= gCurrentFIFOLength)
    {
      gFIFOIn = 0;
    }
    gFIFOLength++;
  }
  
  print_line_ending(kLE_OK_NORM);
}

// Node counter increment
// Usage: NI<CR>
void parse_NI_packet(void)
{
  print_command(FALSE, FALSE);

  if (NodeCount < 0xFFFFFFFEUL)
  {
    NodeCount++;
  }
  print_line_ending(kLE_OK_NORM);
}

// Node counter Decrement
// Usage: ND<CR>
void parse_ND_packet(void)
{
  print_command(FALSE, FALSE);

  if (NodeCount)
  {
    NodeCount--;
  }
  print_line_ending(kLE_OK_NORM);
}

// Set Node counter
// Usage: SN,<value><CR>
// <value> is a 4 byte unsigned value
void parse_SN_packet(void)
{
  unsigned long Temp;
  ExtractReturnType RetVal;

  print_command(FALSE, FALSE);

  RetVal = extract_number(kULONG, &Temp, kREQUIRED);
  if (kEXTRACT_OK == RetVal)
  {
    NodeCount = Temp;
  }
  print_line_ending(kLE_OK_NORM);
}

// Query Node counter
// Usage: QN<CR>
// Returns: <NodeCount><CR>
// OK<CR>
void parse_QN_packet(void)
{
  print_command(FALSE, TRUE);

  ebb_print_uint(NodeCount);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_NORM);
  }

  print_line_ending(kLE_OK_NORM);
}

// Set Layer
// Usage: SL,<Value>[,<Index>]<CR>
// Where <Value> is an unsigned 8-bit decimal number and
// <Index> is an optional parameter from 0 to 31. If not present then 
// <index> is assumed to be 0.
// Store <Value> in <Index> (variable space) in RAM
// Retrieve the values using the QL command
void parse_SL_packet(void)
{
  UINT8 Value = 0;
  UINT8 Index = 0;
  
  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number(kUCHAR, &Value, kREQUIRED);
  extract_number(kUCHAR, &Index, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  if (Index > (SL_STORAGE_SIZE - 1))
  {
    Index = (SL_STORAGE_SIZE - 1);
  }
  gSL_Storage[Index] = Value;
  
  print_line_ending(kLE_OK_NORM);
}

// Query Layer
// Usage: QL[,<Index>]<CR>
// Returns: QL,<ValueAtIndex><CR>
// Where <Index> is an optional parameter from 0 to 31
// If not present, <Index> is set to 0
// The <ValueAtIndex> is an unsigned byte which was previously stored at <Index>
void parse_QL_packet(void)
{
  UINT8 Value = 0;
  UINT8 Index = 0;
  
  print_command(FALSE, TRUE);

  extract_number(kUCHAR, &Index, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  if (Index > (SL_STORAGE_SIZE - 1))
  {
    Index = (SL_STORAGE_SIZE - 1);
  }
  
  ebb_print_uint(gSL_Storage[Index]);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_NORM);
  }
  print_line_ending(kLE_OK_NORM);
}

// Query Button
// Usage: QB<CR>
// Returns: <HasButtonBeenPushedSinceLastQB><CR> (0 or 1)
// OK<CR>
void  parse_QB_packet(void)
{
  print_command(FALSE, TRUE);

  ebb_print_uint(ButtonPushed);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_NORM);
  }
  if (bittstzero(ButtonPushed))
  {
    bitclrzero(ButtonPushed);
  }
  print_line_ending(kLE_OK_NORM);
}

// Query Current
// Usage: QC<CR>
// Returns: <voltage_on_REF_RA0_net>,<voltage_on_v+_net><CR>
// Both values have a range of 0 to 1023 (10-bit ADC)
// The ADC is set up with 0V = 0 and 3.3V = 1023
// For the REF_RA0 (current adjustment pot) a value of 0V-ADC (0 counts) = 46mA
// and a value of 2.58V-ADC (800 counts) = 1.35A
// For the V+ net a value of 0V-ADC (0 counts) = 0V on V+
// and a value of 2.79V-ADC (870 counts) = 31.4V on V+
// REF_RA0 comes in on AN0 (RA0)
// V+ comes in on AN11 (RC2)
void parse_QC_packet(void)
{
  // Since access to ISR_A_FIFO[] is not protected in any way from ISR and
  // mainline code accessing at the same time, we will just wait for
  // the cycle of ADC readings to finish before we spit out our value.
  while (PIE1bits.ADIE);

  // Print out our results
  print_command(FALSE, TRUE);
  ebb_print_int(ISR_A_FIFO[0]);
  ebb_print_char(',');
  ebb_print_int(ISR_A_FIFO[11]);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_NORM);
  }
  print_line_ending(kLE_OK_NORM);
}

// Query General
// Usage: QG<CR>
// Returns: <status><NL><CR>
// <status> is a 16 bit value, printed as a hexadecimal number "00" to "FF".
// Each bit in the byte represents the status of a single bit of information in the EBB.
// Bit 0 : Motion FIFO status (0 = FIFO empty, 1 = FIFO not empty)
// Bit 1 : Motor2 status (0 = not moving, 1 = moving)
// Bit 2 : Motor1 status (0 = not moving, 1 = moving)
// Bit 3 : CommandExecuting (0 = no command currently executing, 1 = a command is currently executing)
// Bit 4 : Pen status (0 = up, 1 = down)
// Bit 5 : PRG button status (0 = not pressed since last query, 1 = pressed since last query)
// Bit 6 : Power Lost (1 = V+ went below g_PowerMonitorThresholdADC, 0 = it did not)
// Bit 7 : gLimitSwitchTriggered
// Just like the QB command, the PRG button status is cleared (after being printed) if pressed since last QB/QG command
void parse_QG_packet(void)
{
  UINT8 result = process_QM();

  print_command(FALSE, TRUE);

  // process_QM() gives us the low 4 bits of our output result.
  result = result & 0x0F;

  if (PenState)
  {
    result = result | (1 << 4);
  }
  if (bittstzero(ButtonPushed))
  {
    result = result | (1 << 5);
  }
  if (g_PowerDropDetected)
  {
    result = result | (1 << 6);
    g_PowerDropDetected = FALSE;
  }
  if (bittstzero(gLimitSwitchTriggered))
  {
    result = result | (1 << 7);
  }

  ebb_print_hex(result, 2);
  print_line_ending(kLE_NORM);
  
  // Reset the button pushed flag
  if (bittstzero(ButtonPushed))
  {
    bitclrzero(ButtonPushed);
  }
}

// Set Engraver
// Usage: SE,<state>,<power>,<use_motion_queue><CR>
// <state> is 0 for off and 1 for on (required)
// <power> is 10 bit PWM power level (optional). 0 = 0%, 1023 = 100%
// <use_motion_queue> if 1 then put this command in motion queue (optional))
// We boot up with <power> at 0
// The engraver motor is always assumed to be on RB3
// So our init routine will map ECCP1
//
// Timer0 is stepper
// Timer1 is unused
// Timer2 and ECCP1 is engraver PWM
// Timer3 and ECCP2 is RC servo2 output
// Timer4 is 1ms ISR
void parse_SE_packet(void)
{
  UINT8 State = 0;
  UINT16 Power = 0;
  UINT8 SEUseMotionQueue = FALSE;
  ExtractReturnType PowerExtract;

  print_command(FALSE, FALSE);

  // Extract each of the values.
  extract_number(kUCHAR, &State, kREQUIRED);
  PowerExtract = extract_number(kUINT, &Power, kOPTIONAL);
  extract_number(kUCHAR, &SEUseMotionQueue, kOPTIONAL);

  // Bail if we got a conversion error
  if (error_byte != kERROR_NO_ERROR)
  {
    return;
  }

  // Limit check
  if (Power > 1023u)
  {
    Power = 1023;
  }
  if (State > 1u)
  {
    State = 1;
  }
  if (SEUseMotionQueue > 1u)
  {
    SEUseMotionQueue = 1;
  }
    
  // Set to %50 if no Power parameter specified, otherwise use parameter
  if (State == 1u && PowerExtract == kEXTRACT_MISSING_PARAMETER)
  {
    StoredEngraverPower = 512;
  }
  else
  {
    StoredEngraverPower = Power;
  }

  // If we're not on, then turn us on
  if (T2CONbits.TMR2ON != 1u)
  {
    // Set up PWM for Engraver control
    // We will use ECCP1 and Timer2 for the engraver PWM output on RB3
    // Our PWM will operate at about 40Khz.

    // Set our reload value
    PR2 = 0xFF;

    // Initialize Timer2

    // The prescaler will be at 1
    T2CONbits.T2CKPS = 0b00;

    // Do not generate an interrupt
    PIE1bits.TMR2IE = 0;

    TCLKCONbits.T3CCP1 = 1;       // ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4
    TCLKCONbits.T3CCP2 = 0;       // ECCP1 uses Timer1/2 and ECCP2 uses Timer3/4

    CCP1CONbits.CCP1M = 0b1100;   // Set EECP1 as PWM mode
    CCP1CONbits.P1M = 0b00;       // Enhanced PWM mode: single output

    // Set up output routing to go to RB3 (RP6)
    RPOR6 = 14;                   // 14 is CCP1/P1A - ECCP1 PWM Output Channel A

    T2CONbits.TMR2ON = 1;         // Turn it on
  }

  // Acting on the state is only done if the SE command is not put on the motion queue
  if (!SEUseMotionQueue)
  {
    // Now act on the State
    if (State)
    {
      // Set RB3 to StoredEngraverPower
      CCPR1L = StoredEngraverPower >> 2;
      CCP1CON = (CCP1CON & 0b11001111) | ((StoredEngraverPower << 4) & 0b00110000);
    }
    else
    {
      // Set RB3 to low by setting PWM duty cycle to zero
      CCPR1L = 0;
      CCP1CON = (CCP1CON & 0b11001111);
    }
  }
  else
  {
    if (!bittst(TestMode, TEST_MODE_DEBUG_BLOCK_FIFO_NUM))
    {
      // Trial: Spin here until there's space in the FIFO
      while(gFIFOLength >= gCurrentFIFOLength)
        ;

      // Set up the motion queue command
      FIFOPtr[gFIFOIn].m.sm.SEPower = StoredEngraverPower;
      FIFOPtr[gFIFOIn].m.sm.DelayCounter = 0;
      FIFOPtr[gFIFOIn].m.sm.SEState = State;
      FIFOPtr[gFIFOIn].Command = COMMAND_SE;

      gFIFOIn++;
      if (gFIFOIn >= gCurrentFIFOLength)
      {
        gFIFOIn = 0;
      }
      gFIFOLength++;
    }
  }

  print_line_ending(kLE_OK_NORM);
}

// Do the work of the QM command so we can use this same code for QM and
// for QG commands.
UINT8 process_QM(void)
{
  UINT8 CommandExecuting = 0;
  UINT8 Motor1Running = 0;
  UINT8 Motor2Running = 0;
  UINT8 FIFOStatus = 0;

  // Need to turn off high priority interrupts briefly here to read out value that ISR uses
  INTCONbits.GIEH = 0;  // Turn high priority interrupts off

  // Create our output values to print back to the PC
  if (CurrentCommand.m.sm.DelayCounter != 0u) 
  {
    CommandExecuting = 1;
  }
  if (CurrentCommand.Command != COMMAND_NONE)
  {
    CommandExecuting = 1;
  }
  if (gFIFOLength) 
  {
    CommandExecuting = 1;
    FIFOStatus = 1;
  }
  if (CommandExecuting && CurrentCommand.m.sm.Steps[0] != 0u) 
  {
    Motor1Running = 1;
  }
  if (CommandExecuting && CurrentCommand.m.sm.Steps[1] != 0u) 
  {
    Motor2Running = 1;
  }

  // Re-enable interrupts
  INTCONbits.GIEH = 1;  // Turn high priority interrupts on
  
  return ((CommandExecuting << 3) | (Motor1Running << 2) | (Motor2Running << 1) | FIFOStatus);
}

// QM command
// For Query Motor - returns the current status of each motor
// QM takes no parameters, so usage is just QM<CR>
// QM returns:
// QM,<CommandExecutingStatus>,<Motor1Satus>,<Motor2Status><CR>
// where:
//   <CommandExecutingStatus>: 0 if no 'motion command' is executing, > 0 if some 'motion command' is executing
//   <Motor1Status>: 0 if motor 1 is idle, 1 if motor is moving
//   <Motor2Status>: 0 if motor 2 is idle, 1 if motor is moving
//
// As of version 2.4.4, there is now a fourth parameter at the end of the reply packet.
// QM,<CommandExecutingStatus>,<Motor1Satus>,<Motor2Status>,<FIFOStatus><CR>
// Where <FIFOStatus> is either 1 (if there are any commands in the FIFO) or 0 (if the FIFO is empty)
void parse_QM_packet(void)
{
  UINT8 CommandExecuting = 0;
  UINT8 Motor1Running = 0;
  UINT8 Motor2Running = 0;
  UINT8 FIFOStatus = 0;
  UINT8 result = process_QM();

  print_command(TRUE, TRUE);

  if (result & 0x01)
  {
    FIFOStatus = 1;
  }
  if (result & 0x02)
  {
    Motor2Running = 1;
  }
  if (result & 0x04)
  {
    Motor1Running = 1;
  }
  if (result & 0x08)
  {
    CommandExecuting = 1;
  }

  ebb_print_int(CommandExecuting);
  ebb_print_char(',');
  ebb_print_int(Motor1Running);
  ebb_print_char(',');
  ebb_print_int(Motor2Running);
  ebb_print_char(',');
  ebb_print_int(FIFOStatus);
  print_line_ending(kLE_REV);
}

// QS command
// For Query Step position - returns the current x and y global step positions
// QS takes no parameters, so usage is just CS<CR>
// QS returns:
// QS,<global_step1_position>,<global_step2_position><CR>
// where:
//   <global_step1_position>: signed 32 bit value, current global motor 1 step position
//   <global_step2_position>: signed 32 bit value, current global motor 2 step position
void parse_QS_packet(void)
{
  INT32 step1, step2;

  print_command(FALSE, TRUE);

  // Need to turn off high priority interrupts briefly here to read out value that ISR uses
  INTCONbits.GIEH = 0;  // Turn high priority interrupts off

  // Make a local copy of the things we care about
  step1 = globalStepCounter1;
  step2 = globalStepCounter2;
  
  // Re-enable interrupts
  INTCONbits.GIEH = 1;  // Turn high priority interrupts on

  ebb_print_int(step1);
  ebb_print_char(',');
  ebb_print_int(step2);
  if (!bittstzero(gStandardizedCommandFormat))
  {
    print_line_ending(kLE_REV);
  }
  print_line_ending(kLE_OK_NORM);
}

// Perform the actual clearing of the step counters (used from several places)
void clear_StepCounters(void)
{
  // Need to turn off high priority interrupts briefly here to read out value that ISR uses
  INTCONbits.GIEH = 0;  // Turn high priority interrupts off

  // Clear out the global step counters
  globalStepCounter1 = 0;
  globalStepCounter2 = 0;
  
  // Clear both step accumulators as well
  acc_union[0].value = 0;
  acc_union[1].value = 0;
  
  // Re-enable interrupts
  INTCONbits.GIEH = 1;  // Turn high priority interrupts on
}

// CS command
// For Clear Stepper position - zeros out both step1 and step2 global positions
// CS takes no parameters, so usage is just CS<CR>
// QS returns:
// OK<CR>
// Note, as of 2.7.0 this also clears out the step accumulators as well
void parse_CS_packet(void)
{
  print_command(FALSE, FALSE);

  clear_StepCounters();
  
  print_line_ending(kLE_OK_NORM);
}

void parse_Y_packet(void)
{
  UINT32 tmp;
  
  // We should now have 
  // AAAABBBBCCCCDDDDEEEEFFFFGGGG
  // in g_RX_buf[] starting at g_RX_buf_out
  //
  // T3 command
  // T3,Intervals,Rate1,Accel1,Jerk1,Rate2,Accel2,Jerk2[,Clear]
  // Intervals = unsigned 32 bit
  // Rate = signed 32
  // Accel = signed 32 bit
  // Jerk = signed 32 bit
  // Clear = 2 bits
  // 
  // A = Intervals
  // B = Rate1
  // C = Accel1
  // D = Jerk1
  // E = Rate2
  // F = Accel2
  // G = Jerk2
  // For the Clear parameter, Y command should assume NEVER clear accumulators.
  // YAAAABBBBCCCCDDDDEEEEFFFFGGGG
  // 1 bytes command
  // 28 bytes data
  // No carriage return

  // For now, just print out all of the parameters to the debug UART
  tmp = g_RX_buf[0];
  tmp = (tmp << 8) | g_RX_buf[1];
  tmp = (tmp << 8) | g_RX_buf[2];
  tmp = (tmp << 8) | g_RX_buf[3];
  HexPrint(tmp);

  tmp = g_RX_buf[4];
  tmp = (tmp << 8) | g_RX_buf[5];
  tmp = (tmp << 8) | g_RX_buf[6];
  tmp = (tmp << 8) | g_RX_buf[7];
  HexPrint(tmp);
  
  tmp = g_RX_buf[8];
  tmp = (tmp << 8) | g_RX_buf[9];
  tmp = (tmp << 8) | g_RX_buf[10];
  tmp = (tmp << 8) | g_RX_buf[11];
  HexPrint(tmp);
  
  tmp = g_RX_buf[12];
  tmp = (tmp << 8) | g_RX_buf[13];
  tmp = (tmp << 8) | g_RX_buf[14];
  tmp = (tmp << 8) | g_RX_buf[15];
  HexPrint(tmp);
  
  tmp = g_RX_buf[16];
  tmp = (tmp << 8) | g_RX_buf[17];
  tmp = (tmp << 8) | g_RX_buf[18];
  tmp = (tmp << 8) | g_RX_buf[19];
  HexPrint(tmp);
  
  tmp = g_RX_buf[20];
  tmp = (tmp << 8) | g_RX_buf[21];
  tmp = (tmp << 8) | g_RX_buf[22];
  tmp = (tmp << 8) | g_RX_buf[23];
  HexPrint(tmp);
  
  tmp = g_RX_buf[24];
  tmp = (tmp << 8) | g_RX_buf[25];
  tmp = (tmp << 8) | g_RX_buf[26];
  tmp = (tmp << 8) | g_RX_buf[27];
  HexPrint(tmp);
  
  
}
