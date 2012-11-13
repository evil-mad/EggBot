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
ifeq "$(wildcard nbproject/Makefile-local-EBBv13_no_bootloader.mk)" "nbproject/Makefile-local-EBBv13_no_bootloader.mk"
include nbproject/Makefile-local-EBBv13_no_bootloader.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=EBBv13_no_bootloader
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/EBF.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/EBF.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/Microchip/USB/usb_device.o "${OBJECTDIR}/Microchip/USB/CDC Device Driver/usb_function_cdc.o" ${OBJECTDIR}/RCServo2.o ${OBJECTDIR}/UBW.o ${OBJECTDIR}/ebb.o ${OBJECTDIR}/main.o ${OBJECTDIR}/usb_descriptors.o
POSSIBLE_DEPFILES=${OBJECTDIR}/Microchip/USB/usb_device.o.d "${OBJECTDIR}/Microchip/USB/CDC Device Driver/usb_function_cdc.o.d" ${OBJECTDIR}/RCServo2.o.d ${OBJECTDIR}/UBW.o.d ${OBJECTDIR}/ebb.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/usb_descriptors.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/Microchip/USB/usb_device.o ${OBJECTDIR}/Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.o ${OBJECTDIR}/RCServo2.o ${OBJECTDIR}/UBW.o ${OBJECTDIR}/ebb.o ${OBJECTDIR}/main.o ${OBJECTDIR}/usb_descriptors.o


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-EBBv13_no_bootloader.mk dist/${CND_CONF}/${IMAGE_TYPE}/EBF.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F46J50
MP_PROCESSOR_OPTION_LD=18f46j50
MP_LINKER_DEBUG_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/Microchip/USB/usb_device.o: Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/Microchip/USB 
	@${RM} ${OBJECTDIR}/Microchip/USB/usb_device.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/Microchip/USB/usb_device.o   Microchip/USB/usb_device.c 
	@${DEP_GEN} -d ${OBJECTDIR}/Microchip/USB/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/Microchip/USB/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.o: Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/Microchip/USB/CDC\ Device\ Driver 
	@${RM} ${OBJECTDIR}/Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo "${OBJECTDIR}/Microchip/USB/CDC Device Driver/usb_function_cdc.o"   "Microchip/USB/CDC Device Driver/usb_function_cdc.c" 
	@${DEP_GEN} -d "${OBJECTDIR}/Microchip/USB/CDC Device Driver/usb_function_cdc.o" 
	@${FIXDEPS} "${OBJECTDIR}/Microchip/USB/CDC Device Driver/usb_function_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/RCServo2.o: RCServo2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/RCServo2.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/RCServo2.o   RCServo2.c 
	@${DEP_GEN} -d ${OBJECTDIR}/RCServo2.o 
	@${FIXDEPS} "${OBJECTDIR}/RCServo2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/UBW.o: UBW.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/UBW.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/UBW.o   UBW.c 
	@${DEP_GEN} -d ${OBJECTDIR}/UBW.o 
	@${FIXDEPS} "${OBJECTDIR}/UBW.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/ebb.o: ebb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/ebb.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/ebb.o   ebb.c 
	@${DEP_GEN} -d ${OBJECTDIR}/ebb.o 
	@${FIXDEPS} "${OBJECTDIR}/ebb.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/main.o   main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usb_descriptors.o: usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usb_descriptors.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usb_descriptors.o   usb_descriptors.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usb_descriptors.o 
	@${FIXDEPS} "${OBJECTDIR}/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
else
${OBJECTDIR}/Microchip/USB/usb_device.o: Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/Microchip/USB 
	@${RM} ${OBJECTDIR}/Microchip/USB/usb_device.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/Microchip/USB/usb_device.o   Microchip/USB/usb_device.c 
	@${DEP_GEN} -d ${OBJECTDIR}/Microchip/USB/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/Microchip/USB/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.o: Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/Microchip/USB/CDC\ Device\ Driver 
	@${RM} ${OBJECTDIR}/Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo "${OBJECTDIR}/Microchip/USB/CDC Device Driver/usb_function_cdc.o"   "Microchip/USB/CDC Device Driver/usb_function_cdc.c" 
	@${DEP_GEN} -d "${OBJECTDIR}/Microchip/USB/CDC Device Driver/usb_function_cdc.o" 
	@${FIXDEPS} "${OBJECTDIR}/Microchip/USB/CDC Device Driver/usb_function_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/RCServo2.o: RCServo2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/RCServo2.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/RCServo2.o   RCServo2.c 
	@${DEP_GEN} -d ${OBJECTDIR}/RCServo2.o 
	@${FIXDEPS} "${OBJECTDIR}/RCServo2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/UBW.o: UBW.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/UBW.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/UBW.o   UBW.c 
	@${DEP_GEN} -d ${OBJECTDIR}/UBW.o 
	@${FIXDEPS} "${OBJECTDIR}/UBW.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/ebb.o: ebb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/ebb.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/ebb.o   ebb.c 
	@${DEP_GEN} -d ${OBJECTDIR}/ebb.o 
	@${FIXDEPS} "${OBJECTDIR}/ebb.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/main.o   main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usb_descriptors.o: usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usb_descriptors.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"Microchip/Include" -I"../EBF" -I"Microchip/Include/USB" -ms -oa- -o-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usb_descriptors.o   usb_descriptors.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usb_descriptors.o 
	@${FIXDEPS} "${OBJECTDIR}/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/EBF.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    rm18f46j50_g.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "rm18f46j50_g.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG -l"/C/MCC18/lib" -l"/C/Program Files/Microchip/mplabc18/v3.37.01/lib"  -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_ICD3=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/EBF.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/EBF.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   rm18f46j50_g.lkr EBB_BL.X/dist/46J50/production/EBB_BL.X.production.hex
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "rm18f46j50_g.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w  -l"/C/MCC18/lib" -l"/C/Program Files/Microchip/mplabc18/v3.37.01/lib"  -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/EBF.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
	@echo "Creating unified hex file"
	@"C:/Program Files/Microchip/MPLABX/mplab_ide/mplab_ide/modules/../../bin/hexmate" --edf="C:/Program Files/Microchip/MPLABX/mplab_ide/mplab_ide/modules/../../bin/en_msgs.txt" dist/${CND_CONF}/${IMAGE_TYPE}/EBF.X.${IMAGE_TYPE}.hex EBB_BL.X/dist/46J50/production/EBB_BL.X.production.hex -odist/${CND_CONF}/production/EBF.X.production.unified.hex

endif


# Subprojects
.build-subprojects:
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
	cd EBB_BL.X && ${MAKE} MAKE_OPTIONS=" -j 8" -f Makefile CONF=46J50 TYPE_IMAGE=DEBUG_RUN
else
	cd EBB_BL.X && ${MAKE} MAKE_OPTIONS=" -j 8" -f Makefile CONF=46J50
endif


# Subprojects
.clean-subprojects:
	cd EBB_BL.X && rm -rf "build/46J50" "dist/46J50"

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/EBBv13_no_bootloader
	${RM} -r dist/EBBv13_no_bootloader

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
