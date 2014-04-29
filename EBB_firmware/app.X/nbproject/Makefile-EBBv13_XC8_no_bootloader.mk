#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-EBBv13_XC8_no_bootloader.mk)" "nbproject/Makefile-local-EBBv13_XC8_no_bootloader.mk"
include nbproject/Makefile-local-EBBv13_XC8_no_bootloader.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=EBBv13_XC8_no_bootloader
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../Microchip/USB/usb_device.c "../Microchip/USB/CDC Device Driver/usb_function_cdc.c" source/ebb.c source/main.c source/RCServo2.c source/UBW.c source/usb_descriptors.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/343710134/usb_device.p1 ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1 ${OBJECTDIR}/source/ebb.p1 ${OBJECTDIR}/source/main.p1 ${OBJECTDIR}/source/RCServo2.p1 ${OBJECTDIR}/source/UBW.p1 ${OBJECTDIR}/source/usb_descriptors.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/343710134/usb_device.p1.d ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1.d ${OBJECTDIR}/source/ebb.p1.d ${OBJECTDIR}/source/main.p1.d ${OBJECTDIR}/source/RCServo2.p1.d ${OBJECTDIR}/source/UBW.p1.d ${OBJECTDIR}/source/usb_descriptors.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/343710134/usb_device.p1 ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1 ${OBJECTDIR}/source/ebb.p1 ${OBJECTDIR}/source/main.p1 ${OBJECTDIR}/source/RCServo2.p1 ${OBJECTDIR}/source/UBW.p1 ${OBJECTDIR}/source/usb_descriptors.p1

# Source Files
SOURCEFILES=../Microchip/USB/usb_device.c ../Microchip/USB/CDC Device Driver/usb_function_cdc.c source/ebb.c source/main.c source/RCServo2.c source/UBW.c source/usb_descriptors.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-EBBv13_XC8_no_bootloader.mk dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F46J50
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/343710134/usb_device.p1: ../Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/343710134 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.p1.d 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/_ext/343710134/usb_device.p1  ../Microchip/USB/usb_device.c 
	@-${MV} ${OBJECTDIR}/_ext/343710134/usb_device.d ${OBJECTDIR}/_ext/343710134/usb_device.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/343710134/usb_device.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1: ../Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/131024517 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1.d 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1  "../Microchip/USB/CDC Device Driver/usb_function_cdc.c" 
	@-${MV} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.d ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/ebb.p1: source/ebb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/ebb.p1.d 
	@${RM} ${OBJECTDIR}/source/ebb.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/ebb.p1  source/ebb.c 
	@-${MV} ${OBJECTDIR}/source/ebb.d ${OBJECTDIR}/source/ebb.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/ebb.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/main.p1: source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/main.p1.d 
	@${RM} ${OBJECTDIR}/source/main.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/main.p1  source/main.c 
	@-${MV} ${OBJECTDIR}/source/main.d ${OBJECTDIR}/source/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/RCServo2.p1: source/RCServo2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/RCServo2.p1.d 
	@${RM} ${OBJECTDIR}/source/RCServo2.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/RCServo2.p1  source/RCServo2.c 
	@-${MV} ${OBJECTDIR}/source/RCServo2.d ${OBJECTDIR}/source/RCServo2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/RCServo2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/UBW.p1: source/UBW.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/UBW.p1.d 
	@${RM} ${OBJECTDIR}/source/UBW.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/UBW.p1  source/UBW.c 
	@-${MV} ${OBJECTDIR}/source/UBW.d ${OBJECTDIR}/source/UBW.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/UBW.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/usb_descriptors.p1: source/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/usb_descriptors.p1.d 
	@${RM} ${OBJECTDIR}/source/usb_descriptors.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/usb_descriptors.p1  source/usb_descriptors.c 
	@-${MV} ${OBJECTDIR}/source/usb_descriptors.d ${OBJECTDIR}/source/usb_descriptors.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/usb_descriptors.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/_ext/343710134/usb_device.p1: ../Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/343710134 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.p1.d 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/_ext/343710134/usb_device.p1  ../Microchip/USB/usb_device.c 
	@-${MV} ${OBJECTDIR}/_ext/343710134/usb_device.d ${OBJECTDIR}/_ext/343710134/usb_device.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/343710134/usb_device.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1: ../Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/131024517 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1.d 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1  "../Microchip/USB/CDC Device Driver/usb_function_cdc.c" 
	@-${MV} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.d ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/ebb.p1: source/ebb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/ebb.p1.d 
	@${RM} ${OBJECTDIR}/source/ebb.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/ebb.p1  source/ebb.c 
	@-${MV} ${OBJECTDIR}/source/ebb.d ${OBJECTDIR}/source/ebb.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/ebb.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/main.p1: source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/main.p1.d 
	@${RM} ${OBJECTDIR}/source/main.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/main.p1  source/main.c 
	@-${MV} ${OBJECTDIR}/source/main.d ${OBJECTDIR}/source/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/RCServo2.p1: source/RCServo2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/RCServo2.p1.d 
	@${RM} ${OBJECTDIR}/source/RCServo2.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/RCServo2.p1  source/RCServo2.c 
	@-${MV} ${OBJECTDIR}/source/RCServo2.d ${OBJECTDIR}/source/RCServo2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/RCServo2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/UBW.p1: source/UBW.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/UBW.p1.d 
	@${RM} ${OBJECTDIR}/source/UBW.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/UBW.p1  source/UBW.c 
	@-${MV} ${OBJECTDIR}/source/UBW.d ${OBJECTDIR}/source/UBW.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/UBW.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/source/usb_descriptors.p1: source/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/usb_descriptors.p1.d 
	@${RM} ${OBJECTDIR}/source/usb_descriptors.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: (%%n) %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"    -o${OBJECTDIR}/source/usb_descriptors.p1  source/usb_descriptors.c 
	@-${MV} ${OBJECTDIR}/source/usb_descriptors.d ${OBJECTDIR}/source/usb_descriptors.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/source/usb_descriptors.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    source/rm18f46j50_g.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.map  -D__DEBUG=1 --debugger=icd3  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"        -odist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.hex 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   source/rm18f46j50_g.lkr ../bootloader.X/dist/46J50/production/bootloader.X.production.hex
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.map  --double=24 --float=24 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,-debug --addrqual=ignore --mode=pro -DBOARD_EBB_V13_AND_ABOVE -P -N255 -I"../Microchip/Include/USB" -I"../Microchip/Include" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%%f:%%l: error: %%s" "--warnformat=%%f:%%l: warning: (%%n) %%s" "--msgformat=%%f:%%l: advisory: (%%n) %%s"     -odist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
	@echo "Creating unified hex file"
	@"C:/Program Files (x86)/Microchip/MPLABX_2_20/mplab_ide/mplab_ide/modules/../../bin/hexmate" --edf="C:/Program Files (x86)/Microchip/MPLABX_2_20/mplab_ide/mplab_ide/modules/../../dat/en_msgs.txt" -break=FFF8  dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.hex ../bootloader.X/dist/46J50/production/bootloader.X.production.hex -odist/${CND_CONF}/production/app.X.production.unified.hex

endif


# Subprojects
.build-subprojects:
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
	cd /D ../bootloader.X && ${MAKE} MAKE_OPTIONS=" -j 8" -f Makefile CONF=46J50 TYPE_IMAGE=DEBUG_RUN
else
	cd /D ../bootloader.X && ${MAKE} MAKE_OPTIONS=" -j 8" -f Makefile CONF=46J50
endif


# Subprojects
.clean-subprojects:
	cd /D ../bootloader.X && rm -rf "build/46J50" "dist/46J50"

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/EBBv13_XC8_no_bootloader
	${RM} -r dist/EBBv13_XC8_no_bootloader

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
