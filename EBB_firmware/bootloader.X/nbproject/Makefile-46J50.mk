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
ifeq "$(wildcard nbproject/Makefile-local-46J50.mk)" "nbproject/Makefile-local-46J50.mk"
include nbproject/Makefile-local-46J50.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=46J50
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/Boot46J50Family.o ${OBJECTDIR}/hid.o ${OBJECTDIR}/main.o ${OBJECTDIR}/usb9.o ${OBJECTDIR}/usbctrltrf.o ${OBJECTDIR}/usbdrv.o ${OBJECTDIR}/usbdsc.o ${OBJECTDIR}/usbmmap.o
POSSIBLE_DEPFILES=${OBJECTDIR}/Boot46J50Family.o.d ${OBJECTDIR}/hid.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/usb9.o.d ${OBJECTDIR}/usbctrltrf.o.d ${OBJECTDIR}/usbdrv.o.d ${OBJECTDIR}/usbdsc.o.d ${OBJECTDIR}/usbmmap.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/Boot46J50Family.o ${OBJECTDIR}/hid.o ${OBJECTDIR}/main.o ${OBJECTDIR}/usb9.o ${OBJECTDIR}/usbctrltrf.o ${OBJECTDIR}/usbdrv.o ${OBJECTDIR}/usbdsc.o ${OBJECTDIR}/usbmmap.o


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-46J50.mk dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/Boot46J50Family.o: Boot46J50Family.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/Boot46J50Family.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/Boot46J50Family.o   Boot46J50Family.c 
	@${DEP_GEN} -d ${OBJECTDIR}/Boot46J50Family.o 
	@${FIXDEPS} "${OBJECTDIR}/Boot46J50Family.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/hid.o: hid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/hid.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/hid.o   hid.c 
	@${DEP_GEN} -d ${OBJECTDIR}/hid.o 
	@${FIXDEPS} "${OBJECTDIR}/hid.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/main.o   main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usb9.o: usb9.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usb9.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usb9.o   usb9.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usb9.o 
	@${FIXDEPS} "${OBJECTDIR}/usb9.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usbctrltrf.o: usbctrltrf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbctrltrf.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbctrltrf.o   usbctrltrf.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbctrltrf.o 
	@${FIXDEPS} "${OBJECTDIR}/usbctrltrf.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usbdrv.o: usbdrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbdrv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbdrv.o   usbdrv.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbdrv.o 
	@${FIXDEPS} "${OBJECTDIR}/usbdrv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usbdsc.o: usbdsc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbdsc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbdsc.o   usbdsc.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbdsc.o 
	@${FIXDEPS} "${OBJECTDIR}/usbdsc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usbmmap.o: usbmmap.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbmmap.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbmmap.o   usbmmap.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbmmap.o 
	@${FIXDEPS} "${OBJECTDIR}/usbmmap.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
else
${OBJECTDIR}/Boot46J50Family.o: Boot46J50Family.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/Boot46J50Family.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/Boot46J50Family.o   Boot46J50Family.c 
	@${DEP_GEN} -d ${OBJECTDIR}/Boot46J50Family.o 
	@${FIXDEPS} "${OBJECTDIR}/Boot46J50Family.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/hid.o: hid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/hid.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/hid.o   hid.c 
	@${DEP_GEN} -d ${OBJECTDIR}/hid.o 
	@${FIXDEPS} "${OBJECTDIR}/hid.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/main.o   main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usb9.o: usb9.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usb9.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usb9.o   usb9.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usb9.o 
	@${FIXDEPS} "${OBJECTDIR}/usb9.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usbctrltrf.o: usbctrltrf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbctrltrf.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbctrltrf.o   usbctrltrf.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbctrltrf.o 
	@${FIXDEPS} "${OBJECTDIR}/usbctrltrf.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usbdrv.o: usbdrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbdrv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbdrv.o   usbdrv.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbdrv.o 
	@${FIXDEPS} "${OBJECTDIR}/usbdrv.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usbdsc.o: usbdsc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbdsc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbdsc.o   usbdsc.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbdsc.o 
	@${FIXDEPS} "${OBJECTDIR}/usbdsc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/usbmmap.o: usbmmap.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbmmap.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -ms -oa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbmmap.o   usbmmap.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbmmap.o 
	@${FIXDEPS} "${OBJECTDIR}/usbmmap.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    BootModified.rm18f46j50_g.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "BootModified.rm18f46j50_g.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG -m"EBBv11_BL_46J50.map"  -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_ICD3=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   BootModified.rm18f46j50_g.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "BootModified.rm18f46j50_g.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w  -m"EBBv11_BL_46J50.map"  -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/46J50
	${RM} -r dist/46J50

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
