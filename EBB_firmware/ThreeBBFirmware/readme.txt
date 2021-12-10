ThreeBeeBee (3BB) Readme.txt file

== TODO ==

  Completed Commands
  * VR (software version)             (done)
  * SM (Steper Move)                  (done) - for all 3 axies
  * SP (Set Pen)                      (done) - only applies to servo, not pen stepper
  * TP (Toggle Pen)                   (done) - only applies to servo, not pen stepper
  * QP (Query Pen state)              (done)
  * SC (Stepper/servo Configure)      (done) - only for 4, 5, 10, 11, 12 (pen servo parameters)
  * S2 (General RC output)            (done)
  * QR (Query Pen Power state)        (done)
  * SR (Set RC Servo power timeout)   (done)
  * DR (Driver register Read)         (done)
  * DW (Driver register Write)        (done)
  * SS (driver Serial Status)         (done)
  * MW (Memory Write)                 (done) take 32 bit number, write to one of 512 locations in RAM
  * MR (Memory Read)                  (done) read out 32 bit number from one of 512 locations in RAM
  * V (Version)                       (done) legacy, with \r\n at end, so PC software can identify what board it is connected to
  * XM (Stepper Move, mixed axis)     (done)
  * QL (Query Layer)                  (done - legacy only)
  * SL (Set Layer)                    (done - legacy only)
  
  Commands to get working, in priority order
  * EM (Enable Motors)
  * PI (Pin Input)
  * PD (Pin Direction)
  * PO (Pin Output)
  * QC (Query Current)

  * HM (Home Motors/absolute move)
  * LM (Low level Move)
  * LT (Low level Timed move)
  * QE (Query motor Enables/microstep)
  * QG (Query General)
  * QM (Query Motors)
  * QS (Query Step position)
  * CU (Configure User options)
  * CS (Clear Step position)
  * ES (E stop)
  * RB (ReBoot)
  * R (Reset)
  * BL (enter BootLoader)
  * ND (Node count Decrement)
  * NI (Node count Increment)
  * CN (Clear Node count)
  * QN (Query Node count)
  * CK (Check Input)
  * SN (Set Node count)
  * ST (Set 3BB nickname Tag)
  * QT (Query nickname Tag)
  * C (configure)
  * QB (Query Button)
  
  Commands to add
  * Add "FULL" vs "OK" reply to all motion queue commands based on high water mark (need new command for setting this)
  
  Are these commands needed for 3BB?
  * PC (Pulse Configure)
  * PG (Pulse Go)
  * SE (Set Engraver)
  * O (Output)
  * I (Input)
  * A (Analog value get)
  * AC (Analog Configure)
  * T (Timed digital/analog read)
  * MR (Memory Read) (old version, new version proposed above)
  * MW (Memory Write)(old version, new version proposed above)

  Parts of the board confirmed working
  * MCU
  * All six RC servo outputs
  * RC servo power for S1
  * Stepper Drivers
    * External sense resistors
    * UART communication (both directions)
  * USB communications
  * Both LEDs
  * RESET and PRG buttons
  * 3.3V power supply
  * 5V power supply
  * I/O header
  
  Parts of the board left to check out
  * SCALED_5V and SCALED__V+ voltage monitoring ADC inptus
  * Motor current sense ADC input on CUR_SNS
  * Detecting stalls with Trinamic drivers? (What commands to use?)

  Misc ToDo 

  * Add default 'EBB' mode, and a new command to switch to 3BB mode
  * Figure out why SWO locks up software when enabled
  * Document the different timers used in RC servo outputs, and that within a timer the rising edges will be sychnonzied, but that between timers they won't be
  * Add EEPROM emulation layer
    - In one or more flash pages between bootloader and application
    - Find existing library to use
    - Use 256 byte address space
    - Possibly find way to allow extend address space and/or pages used in the future without breaking things
      - Would we need to extract all existing data before update, then re-write it all after update?
    - Add commands for mass erasing entire EEPROM?
  * All strings will be terminated with just \n
  * Put into user docs: The fact that P1/2 and P3/4/5 will all have synchronized rising edges, but 0, 1/2, and 3/4/5 may not be synchronized.
  * Add support for solenoid output (?) tied to pen, but what about z-axis stepper?
  * serial.c : fix or remove the SS command. Is it even needed?
  * Move serial command parsing out of CDC_Receive_FS() and into main()? (done)
  * Performance test USB command rate - currently only getting about 1 command/ms. Can we go faster?
  * Why does UART traffic stop SysTick? That's not right. UART send/receive should not be killing interrupts. 
    - Problem was because all USB commands were happening in ISR context during USB receive. Fixed by moving command processing to main() loop, looking at flag set in USB ISR.
  * Add feature to re-init driver (over UART) if detected 12V power appears
  * Do we want to keep the same error reporting format as EBB? ("!" followed by an error code and then string)
  * Update all stepper move commands limit checking to use higher max step rate of 3BB (100Khz compared with 25Khz)

  * Convert ISR over to using table for I/O, and loop for 3 steppers like XMount does (done)

  -- COMMANDS --
  * Add command "QE" (for Query Enable Motor) which would return the current state of each motor driver's enable (either the actual enable pin or the enable bit in the register), as well as the
    current resolution of each driver.
  
  -- BOARD --
  * Switch USB connector to USB C
    * Connect up USB C PD pins to Micro (where?) Use : 670-DX07S016JA1R1500CT-ND
  * Check why 3.3V rail is only at 3.17V. Need to adjust resistors on LDO?  


= Questions =
  * Should z-axis stepper perfectly emulate servo?
    - Answer (1/24/21 Windell) No. SP/TP should operate only on RC servo pen, and SM/XM should be used for all z-axis moves. Keep them separate. Maybe in the future we'll want to link them and have SP/TP control the z-axis stepper as well. 
    - Also, the PM will issue both sets of commands (RC servo and stepper) whenever it wants to create pen motion
    
  * Should all printed lines end in just <CR>? (NO! Use just Line Feed/new line - i.e. 0x0A, \n instead of Carrige Return 0x0D \r. Note that legacy "V" command will use \r\n at end, but that's the only one.)
  
  
= Code Cleanup =
  * Add proper top header comments to all .c/.h files (that are not CubeMX)
  * All function headers (comments right before functions) should use same comment type /* */
  * All parameters to function calls should start with lower case
  * All module globals should start with module name, underscore, then capitol letter
  * All modules should have proper section breaks ( /**** LOCAL FUNCTIONS ****/, etc.)
  * All non-public functions should be marked static
  * All module varaibles should be marked static
  * Change all 'FIFO' to 'queue'

