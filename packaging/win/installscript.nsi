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
Name "EggBot v2.5.0"

; The file to write
OutFile "EggBot_250A.exe"

; The default installation directory
InstallDir $PROGRAMFILES64\Inkscape
;InstallDir $PROGRAMFILES\Inkscape









; Request application privileges for Windows Vista
RequestExecutionLevel admin

BrandingText "The Original EggBot, by Evil Mad Scientist Laboratories"
  
; Directory dialog text header::
DirText "The Eggbot software needs to be installed within \
Inkscape 0.91 (or newer), in the extensions folder.  \
$\r$\r \
If you have installed Inkscape normally (in 'Program files'), simply \
click 'Install' below. \
If your copy of Inkscape is elsewhere, please select the Inkscape directory below. \
If you have not yet installed Inkscape, please download it from http://inkscape.org before \
proceeding. "


ComponentText "Check the components you want to install and uncheck \
the components that you don't want to install. Click Install to \
begin. This installer requires Inkscape 0.91 (or newer)."


!define MUI_ICON "icon\eggbotlogo_2014_256px.ico"
!define MUI_HEADERIMAGE
!define MUI_HEADERIMAGE_BITMAP "icon\EggbotLogo-2014.bmp"
!define MUI_HEADERIMAGE_RIGHT

;--------------------------------

; Pages

;Page directory
;Page instfiles
  
!insertmacro MUI_PAGE_COMPONENTS
;!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES

;!insertmacro MUI_UNPAGE_CONFIRM
;!insertmacro MUI_UNPAGE_INSTFILES
  
   
; --------------------------------
; Languages

  !insertmacro MUI_LANGUAGE "English"
  
;--------------------------------

; The stuff to install


Section "Inkscape Extensions" SecMain

 Var /GLOBAL InkscapeDir
 
${If} ${FileExists} `$PROGRAMFILES64\Inkscape\AUTHORS`

  ; Inkscape found in 64-bit program files directory
	;InstallDir $PROGRAMFILES64\Inkscape
	StrCpy $InkscapeDir "$PROGRAMFILES64\Inkscape"

${ElseIf} ${FileExists} `$PROGRAMFILES\Inkscape\AUTHORS`

  ; file is a file
;	InstallDir $PROGRAMFILES\Inkscape
	StrCpy $InkscapeDir "$PROGRAMFILES\Inkscape"
${Else}

  ; file is neither a file or a directory (i.e. it doesn't exist)
Abort "Inkscape not found. Please install Inkscape 0.91 (or newer) and try again."

${EndIf}


  ; Set output path to the installation directory.
  SetOutPath "$InkscapeDir\share\extensions"
  
  ; Put file there
  File /r extensions\serial*
  File extensions\*
  
  SetOutPath "$InkscapeDir\share\templates"
  File templates\*
  
SectionEnd ; end the section


Section "USB Driver" SecDriver
  SetOutPath "$InkscapeDir\Driver"

  File "EBB_inf\mchpcdc.cat"
  File "EBB_inf\mchpcdc.inf"
  File "EBB_inf\DIFxAPI_x64.dll"
  File "EBB_inf\DIFxAPI_x86.dll"
  File "EBB_inf\ReadMe.txt"
  File "EBB_inf\USBDriverInstaller.exe"

  ExecWait '"$InkscapeDir\Driver\USBDriverInstaller.exe" -auto'
SectionEnd

Section "EggBot Examples" SecExamples

  ; Set output path to the installation directory.
  SetOutPath "$InkscapeDir"
  
  ; Put directory there
  File /r "EggBot Examples"
  
    ; Desktop Shortcut!
  createShortCut "$DESKTOP\EggBot Examples.lnk" "$InkscapeDir\EggBot Examples"
  
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
