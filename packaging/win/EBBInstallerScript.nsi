; installscript.nsi
;
; An NSIS document for creating a windows installer.
; Place this document in a directory with all of the 
; items that should be installed into the end user's
; extensions directory, including the 'serial' folder.
;
;--------------------------------

; The name of the installer
Name "UBWDriverInstaller"

; The file to write
OutFile "UBWDriverInstaller_v20.exe"

; The default installation directory
InstallDir $PROGRAMFILES\Inkscape\share\extensions\UBWDriverInstaller

; Request application privileges for Windows Vista
RequestExecutionLevel admin

; Directory dialog text header::
DirText "The Eggbot software needs to be installed within \
Inkscape, in the extensions folder.  Normally, \
all that you need to do is click 'Install' below. \
But, if your copy of Inkscape is not in the normal \
('Program files') \
location, please select the correct extensions directory. "

;--------------------------------

; Pages

Page directory
Page instfiles

;--------------------------------

; The stuff to install
Section "" ;No components page, name is not important

  ; Set output path to the installation directory.
  SetOutPath $INSTDIR
  
  ; Put file there
  File "USBDriverInstaller.exe"
  File "DIFxAPI_x64.dll"
  File "DIFxAPI_x86.dll"
  File "mchpcdc.cat"
  File "mchpcdc.inf"
  File "ReadMe.txt"
  ExecWait '"$INSTDIR\USBDriverInstaller.exe" -auto'
  
SectionEnd ; end the section
