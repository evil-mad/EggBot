; installscript.nsi
;
; An NSIS document for creating a windows installer.
; Place this document in a directory with all of the 
; items that should be installed into the end user's
; extensions directory.
;
;--------------------------------

  SetCompressor lzma

; Include Modern UI & Windows version checker

  !include "MUI2.nsh"
  !include "WinVer.nsh"
  
  
; --------------------------------

; The name of the installer
Name "EggBot v2.8.1"

; The file to write
OutFile "EggBot_281.exe"

; The default installation directory
InstallDir $PROGRAMFILES64\Inkscape
;InstallDir $PROGRAMFILES\Inkscape



; Request application privileges for Windows
RequestExecutionLevel admin

BrandingText "EggBot, by Evil Mad Scientist"
  
; Directory dialog text header::
DirText "The EggBot software needs to be installed within Inkscape 0.92.  \
$\r$\r \
If you have installed Inkscape normally (in 'Program files'), simply \
click 'Install' below. \
If your copy of Inkscape is elsewhere, please select the Inkscape directory below. \
If you have not yet installed Inkscape, please download it from http://inkscape.org\
and install before proceeding."


ComponentText "Check the components you want to install and uncheck \
the components that you don't want to install. Click Install to \
begin. This installer requires Inkscape 0.92."


CompletedText "Software installation completed successfully."

 Var /GLOBAL InkscapeDir


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

 
${If} ${FileExists} `$PROGRAMFILES64\Inkscape\lib\python2.7\site-packages\serial\__init__.py`

  ; Inkscape found in 64-bit program files directory
	;InstallDir $PROGRAMFILES64\Inkscape
	StrCpy $InkscapeDir "$PROGRAMFILES64\Inkscape"

${ElseIf} ${FileExists} `$PROGRAMFILES\Inkscape\lib\python2.7\site-packages\serial\__init__.py`

  ; Inkscape found in legacy program files directory
;	InstallDir $PROGRAMFILES\Inkscape
	StrCpy $InkscapeDir "$PROGRAMFILES\Inkscape"
${Else}

  ; file is neither a file or a directory (i.e. it doesn't exist)
Abort "Inkscape not found in Program Files. Please install Inkscape 0.92 and try again."

${EndIf}


  ; Set output path to the installation directory.
  SetOutPath "$InkscapeDir\share\extensions"
  
  ; Put file there
  File /r extensions\svg_fonts*
  File extensions\*

  ; If upgrading a prior installation, which would have included 
  ; the serial directory in the extensions directory, we must
  ; remove or otherwise disable that serial library. Rather than
  ; deleting the file, we will rename __init__.py, so that the
  ; library is disabled. Since we are not deleting a directory,
  ; there is no risk of deleting the wrong directory.  

Var /GLOBAL serialpath
StrCpy $serialpath "$InkscapeDir\share\extensions\serial"

${If} ${FileExists} `$serialpath\__init__.py`
	Rename $serialpath\__init__.py $serialpath\disabled__init__.py
${EndIf}

SectionEnd ; end the section


Section "USB Driver" SecDriver

 
${If} ${FileExists} `$PROGRAMFILES64\Inkscape\lib\python2.7\site-packages\serial\__init__.py`

  ; Inkscape found in 64-bit program files directory
	;InstallDir $PROGRAMFILES64\Inkscape
	StrCpy $InkscapeDir "$PROGRAMFILES64\Inkscape"

${ElseIf} ${FileExists} `$PROGRAMFILES\Inkscape\lib\python2.7\site-packages\serial\__init__.py`

  ; Inkscape found in legacy program files directory
;	InstallDir $PROGRAMFILES\Inkscape
	StrCpy $InkscapeDir "$PROGRAMFILES\Inkscape"
${Else}

  ; file is neither a file or a directory (i.e. it doesn't exist)
Abort "Inkscape not found in Program Files. Please install Inkscape 0.92 and try again."

${EndIf}


  SetOutPath "$InkscapeDir\Driver"

  File "EBB_inf\mchpcdc.cat"
  File "EBB_inf\mchpcdc.inf"
  File "EBB_inf\DIFxAPI_x64.dll"
  File "EBB_inf\DIFxAPI_x86.dll"
  File "EBB_inf\ReadMe.txt"
  File "EBB_inf\USBDriverInstaller.exe"

  ExecWait '"$InkscapeDir\Driver\USBDriverInstaller.exe" -auto'
SectionEnd


; --------------------------------
; Descriptions

  ; Language strings
  LangString DESC_SecMain ${LANG_ENGLISH} "Installs the EggBot software within Inkscape."
  LangString DESC_SecDriver ${LANG_ENGLISH} "The USB driver for the EggBot."

  ; Assign language strings to sections
  !insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN
    !insertmacro MUI_DESCRIPTION_TEXT ${SecMain} $(DESC_SecMain)
    !insertmacro MUI_DESCRIPTION_TEXT ${SecDriver} $(DESC_SecDriver)
  !insertmacro MUI_FUNCTION_DESCRIPTION_END
