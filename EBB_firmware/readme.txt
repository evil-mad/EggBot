The files in this directory (EBB_firmware) are the firmwre files for the EiBotBoard,
(http://www.schmalzhaus.com/EBB/) which is used primarily in the Egg-Bot (http://www.egg-bot.com/)
made by EMSL (http://evilmadscience.com/productsmenu/tinykitlist/171-egg-bot).

These files are only useful if you want to modify the firmware and/or redbuild it yourself.

The app.X directory contains MPLAB X and MPLAB 8 projects for the application.

The bootloader.X directory contains MPLAB X and MPLAB 8 projects for the bootloader.

The EBB_inf directory contains the two driver files needed for Windows when plugging in the EBB for the first time.

The Microchip directory is one you need to get yourself. It contains all of the MLA (Microchip Library for Application)
files, of which the EBB firmware needs the USB stack. Microchip does not allow anyone to re-districbute these
files, so we can not include those files here. In order to get this directory, you need to download the latest
MLA from Microchip (http://www.microchip.com/MLA) and then copy the Microchip folder from within the MLA download
here.

The Releases directory contains the released HEX files for bootloader, application, and combined HEX files.

Both bootloader and application can be modified and/or re-built using Microchip's free MPLAB 8 IDE 
(http://www.microchip.com/MPLAB8) or free MPLAB X IDE (http://www.microchip.com/MPLABX) and their free C18 compiler.
(http://www.microchip.com/C18)

*Brian Schmalz
www.schmalzhaus.com