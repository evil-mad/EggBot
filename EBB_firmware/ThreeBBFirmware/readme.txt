ThreeBeeBee (3BB) Readme.txt file

== TODO ==

  * Document the different timers used in RC servo outputs, and that within a timer the rising edges will be sychnonzied, but that between timers they won't be
  * Add EEPROM emulation layer
    - In one or more flash pages between bootloader and application
    - Find existing library to use
    - Use 256 byte address space
    - Possibly find way to allow extend address space and/or pages used in the future without breaking things
      - Would we need to extract all existing data before update, then re-write it all after update?
    - Add commands for mass erasing entire EEPROM?
  *  


= Questions =
  * Should z-axis stepper perfectly emulate servo?
    - Answer (1/24/21 Windell) No. SP/TP should operate only on RC servo pen, and SM/XM should be used for all z-axis moves. Keep them separate. Maybe in the future we'll want to link them and have SP/TP control the z-axis stepper as well. 
    - Also, the PM will issue both sets of commands (RC servo and stepper) whenever it wants to create pen motion
    
  * Should all printed lines end in just <CR>?
  
  
= Code Cleanup =
  * Add proper top header comments to all .c/.h files (that are not CubeMX)
  * All function headers (comments right before functions) should use same comment type /* */
  * All parameters to function calls should start with lower case
  * All module globals should start with module name, underscore, then capitol letter
  * All modules should have proper section breaks ( /**** LOCAL FUNCTIONS ****/, etc.)
  * All non-public functions should be marked static
  * All module varaibles should be marked static

