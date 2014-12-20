; installscript.nsi
;
; An NSIS document for creating a windows installer.
; Place this document in a directory with all of the 
; items that should be installed into the end user's
; extensions directory, including the 'serial' folder.
;
;--------------------------------

  SetCompressor lzma

; Include Modern UI & Windows version checker

  !include "MUI2.nsh"
  !include "WinVer.nsh"
  
  
; --------------------------------

; The name of the installer
Name "EggBot v2.4.0"

; The file to write
OutFile "EggBot_240A.exe"

; The default installation directory
InstallDir $PROGRAMFILES\Inkscape

; Request application privileges for Windows Vista
RequestExecutionLevel admin

BrandingText "The Original EggBot, by Evil Mad Scientist Laboratories"
  
; Directory dialog text header::
DirText "The Eggbot software needs to be installed within \
Inkscape, in the extensions folder.  \
$\r$\r \
If you have installed Inkscape normally (in 'Program files'), simply \
click 'Install' below. \
If your copy of Inkscape is elsewhere, please select the Inkscape directory below. \
$\r$\r \
If you have not yet installed Inkscape, please download it from http://inkscape.org before \
proceeding."


!define MUI_ICON "icon\eggbotlogo_2014_256px.ico"
!define MUI_HEADERIMAGE
!define MUI_HEADERIMAGE_BITMAP "icon\EggbotLogo-2014.bmp"
!define MUI_HEADERIMAGE_RIGHT

;--------------------------------

; Pages

;Page directory
;Page instfiles
  
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES

;!insertmacro MUI_UNPAGE_CONFIRM
;!insertmacro MUI_UNPAGE_INSTFILES
  
   
; --------------------------------
; Languages

  !insertmacro MUI_LANGUAGE "English"
  
;--------------------------------

; The stuff to install


Section "Inkscape Extensions" SecMain

  ; Set output path to the installation directory.
  SetOutPath "$INSTDIR\share\extensions"
  
  ; Put file there
  File /r extensions\serial*
  File extensions\*
  
  SetOutPath "$INSTDIR\share\templates"
  File templates\*
  
SectionEnd ; end the section


Section "USB Driver" SecDriver
  SetOutPath "$INSTDIR\Driver"

  File "EBB_inf\mchpcdc.cat"
  File "EBB_inf\mchpcdc.inf"
  File "EBB_inf\DIFxAPI_x64.dll"
  File "EBB_inf\DIFxAPI_x86.dll"
  File "EBB_inf\ReadMe.txt"
  File "EBB_inf\USBDriverInstaller.exe"

  ExecWait '"$INSTDIR\Driver\USBDriverInstaller.exe" -auto'
SectionEnd

Section "EggBot Examples" SecExamples

  ; Set output path to the installation directory.
  SetOutPath "$INSTDIR"
  
  ; Put directory there
  File /r "EggBot Examples"
  
    ; Desktop Shortcut!
  createShortCut "$DESKTOP\EggBot Examples.lnk" "$INSTDIR\EggBot Examples"
  
SectionEnd ; end the section


; --------------------------------
; Descriptions

  ; Language strings
  LangString DESC_SecMain ${LANG_ENGLISH} "Installs the EggBot control software within Inkscape."
  LangString DESC_SecDriver ${LANG_ENGLISH} "The USB driver for the EggBot."
  LangString DESC_SecExamples ${LANG_ENGLISH} "Over 100 example files for EggBot, with a desktop shortcut."

  ; Assign language strings to sections
  !insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN
    !insertmacro MUI_DESCRIPTION_TEXT ${SecMain} $(DESC_SecMain)
    !insertmacro MUI_DESCRIPTION_TEXT ${SecDriver} $(DESC_SecDriver)
	!insertmacro MUI_DESCRIPTION_TEXT ${SecExamples} $(DESC_SecExamples)
  !insertmacro MUI_FUNCTION_DESCRIPTION_END
