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
ifeq "$(wildcard nbproject/Makefile-local-EBBv13_with_bootloader.mk)" "nbproject/Makefile-local-EBBv13_with_bootloader.mk"
include nbproject/Makefile-local-EBBv13_with_bootloader.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=EBBv13_with_bootloader
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../Microchip/USB/usb_device.c "../Microchip/USB/CDC Device Driver/usb_function_cdc.c" source/ebb.c source/main.c source/RCServo2.c source/UBW.c source/usb_descriptors.c source/ebb_demo.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/343710134/usb_device.o ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o ${OBJECTDIR}/source/ebb.o ${OBJECTDIR}/source/main.o ${OBJECTDIR}/source/RCServo2.o ${OBJECTDIR}/source/UBW.o ${OBJECTDIR}/source/usb_descriptors.o ${OBJECTDIR}/source/ebb_demo.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/343710134/usb_device.o.d ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o.d ${OBJECTDIR}/source/ebb.o.d ${OBJECTDIR}/source/main.o.d ${OBJECTDIR}/source/RCServo2.o.d ${OBJECTDIR}/source/UBW.o.d ${OBJECTDIR}/source/usb_descriptors.o.d ${OBJECTDIR}/source/ebb_demo.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/343710134/usb_device.o ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o ${OBJECTDIR}/source/ebb.o ${OBJECTDIR}/source/main.o ${OBJECTDIR}/source/RCServo2.o ${OBJECTDIR}/source/UBW.o ${OBJECTDIR}/source/usb_descriptors.o ${OBJECTDIR}/source/ebb_demo.o

# Source Files
SOURCEFILES=../Microchip/USB/usb_device.c ../Microchip/USB/CDC Device Driver/usb_function_cdc.c source/ebb.c source/main.c source/RCServo2.c source/UBW.c source/usb_descriptors.c source/ebb_demo.c



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
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-EBBv13_with_bootloader.mk dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F46J50
MP_PROCESSOR_OPTION_LD=18f46j50
MP_LINKER_DEBUG_OPTION=  -u_DEBUGSTACK
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/343710134/usb_device.o: ../Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/343710134" 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/343710134/usb_device.o   ../Microchip/USB/usb_device.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/343710134/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/343710134/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/131024517/usb_function_cdc.o: ../Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/131024517" 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o   "../Microchip/USB/CDC Device Driver/usb_function_cdc.c" 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/131024517/usb_function_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/ebb.o: source/ebb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/ebb.o.d 
	@${RM} ${OBJECTDIR}/source/ebb.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/ebb.o   source/ebb.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/ebb.o 
	@${FIXDEPS} "${OBJECTDIR}/source/ebb.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/main.o: source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/main.o.d 
	@${RM} ${OBJECTDIR}/source/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/main.o   source/main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/main.o 
	@${FIXDEPS} "${OBJECTDIR}/source/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/RCServo2.o: source/RCServo2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/RCServo2.o.d 
	@${RM} ${OBJECTDIR}/source/RCServo2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/RCServo2.o   source/RCServo2.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/RCServo2.o 
	@${FIXDEPS} "${OBJECTDIR}/source/RCServo2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/UBW.o: source/UBW.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/UBW.o.d 
	@${RM} ${OBJECTDIR}/source/UBW.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/UBW.o   source/UBW.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/UBW.o 
	@${FIXDEPS} "${OBJECTDIR}/source/UBW.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/usb_descriptors.o: source/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/source/usb_descriptors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/usb_descriptors.o   source/usb_descriptors.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/usb_descriptors.o 
	@${FIXDEPS} "${OBJECTDIR}/source/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/ebb_demo.o: source/ebb_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/ebb_demo.o.d 
	@${RM} ${OBJECTDIR}/source/ebb_demo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/ebb_demo.o   source/ebb_demo.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/ebb_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/source/ebb_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
else
${OBJECTDIR}/_ext/343710134/usb_device.o: ../Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/343710134" 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/343710134/usb_device.o   ../Microchip/USB/usb_device.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/343710134/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/343710134/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/131024517/usb_function_cdc.o: ../Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/131024517" 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o   "../Microchip/USB/CDC Device Driver/usb_function_cdc.c" 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/131024517/usb_function_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/ebb.o: source/ebb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/ebb.o.d 
	@${RM} ${OBJECTDIR}/source/ebb.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/ebb.o   source/ebb.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/ebb.o 
	@${FIXDEPS} "${OBJECTDIR}/source/ebb.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/main.o: source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/main.o.d 
	@${RM} ${OBJECTDIR}/source/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/main.o   source/main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/main.o 
	@${FIXDEPS} "${OBJECTDIR}/source/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/RCServo2.o: source/RCServo2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/RCServo2.o.d 
	@${RM} ${OBJECTDIR}/source/RCServo2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/RCServo2.o   source/RCServo2.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/RCServo2.o 
	@${FIXDEPS} "${OBJECTDIR}/source/RCServo2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/UBW.o: source/UBW.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/UBW.o.d 
	@${RM} ${OBJECTDIR}/source/UBW.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/UBW.o   source/UBW.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/UBW.o 
	@${FIXDEPS} "${OBJECTDIR}/source/UBW.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/usb_descriptors.o: source/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/source/usb_descriptors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/usb_descriptors.o   source/usb_descriptors.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/usb_descriptors.o 
	@${FIXDEPS} "${OBJECTDIR}/source/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/source/ebb_demo.o: source/ebb_demo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/source" 
	@${RM} ${OBJECTDIR}/source/ebb_demo.o.d 
	@${RM} ${OBJECTDIR}/source/ebb_demo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -DPROGRAMMABLE_WITH_USB_HID_BOOTLOADER -I"../Microchip/Include" -I"../Microchip/Include/USB" -I"." -I"./source" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/source/ebb_demo.o   source/ebb_demo.c 
	@${DEP_GEN} -d ${OBJECTDIR}/source/ebb_demo.o 
	@${FIXDEPS} "${OBJECTDIR}/source/ebb_demo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    source/BL_rm18f46j50_g.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "source\BL_rm18f46j50_g.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG -m"${DISTDIR}/EBF.X.${IMAGE_TYPE}.map" -l"./source"  -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_PK3=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   source/BL_rm18f46j50_g.lkr ../bootloader.X/dist/46J50/production/bootloader.X.production.hex
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "source\BL_rm18f46j50_g.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w  -m"${DISTDIR}/EBF.X.${IMAGE_TYPE}.map" -l"./source"  -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
	@echo "Creating unified hex file"
	@"C:/Program Files (x86)/Microchip/MPLABX/v5.30/mplab_platform/platform/../mplab_ide/modules/../../bin/hexmate" --edf="C:/Program Files (x86)/Microchip/MPLABX/v5.30/mplab_platform/platform/../mplab_ide/modules/../../dat/en_msgs.txt" -break=FFF8  dist/${CND_CONF}/${IMAGE_TYPE}/app.X.${IMAGE_TYPE}.hex ../bootloader.X/dist/46J50/production/bootloader.X.production.hex -odist/${CND_CONF}/production/app.X.production.unified.hex

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
	${RM} -r build/EBBv13_with_bootloader
	${RM} -r dist/EBBv13_with_bootloader

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
