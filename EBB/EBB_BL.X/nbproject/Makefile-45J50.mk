#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile

# Environment
SHELL=cmd.exe
# Adding MPLAB X bin directory to path
PATH:=C:/Program Files/Microchip/MPLABX/mplab_ide/mplab_ide/modules/../../bin/:$(PATH)
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=45J50
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.cof
else
IMAGE_TYPE=production
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.cof
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/Boot46J50Family.o ${OBJECTDIR}/hid.o ${OBJECTDIR}/main.o ${OBJECTDIR}/usb9.o ${OBJECTDIR}/usbctrltrf.o ${OBJECTDIR}/usbdrv.o ${OBJECTDIR}/usbdsc.o ${OBJECTDIR}/usbmmap.o

# Object Files
OBJECTFILES=${OBJECTDIR}/Boot46J50Family.o ${OBJECTDIR}/hid.o ${OBJECTDIR}/main.o ${OBJECTDIR}/usb9.o ${OBJECTDIR}/usbctrltrf.o ${OBJECTDIR}/usbdrv.o ${OBJECTDIR}/usbdsc.o ${OBJECTDIR}/usbmmap.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

# Path to java used to run MPLAB X when this makefile was created
MP_JAVA_PATH="C:\Program Files\Java\jre6/bin/"
OS_CURRENT="$(shell uname -s)"
############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
MP_CC="C:\Program Files\Microchip\mplabc18\v3.40\bin\mcc18.exe"
# MP_BC is not defined
MP_AS="C:\Program Files\Microchip\mplabc18\v3.40\bin\..\mpasm\MPASMWIN.exe"
MP_LD="C:\Program Files\Microchip\mplabc18\v3.40\bin\mplink.exe"
MP_AR="C:\Program Files\Microchip\mplabc18\v3.40\bin\mplib.exe"
DEP_GEN=${MP_JAVA_PATH}java -jar "C:/Program Files/Microchip/MPLABX/mplab_ide/mplab_ide/modules/../../bin/extractobjectdependencies.jar" 
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps
MP_CC_DIR="C:\Program Files\Microchip\mplabc18\v3.40\bin"
# MP_BC_DIR is not defined
MP_AS_DIR="C:\Program Files\Microchip\mplabc18\v3.40\bin\..\mpasm"
MP_LD_DIR="C:\Program Files\Microchip\mplabc18\v3.40\bin"
MP_AR_DIR="C:\Program Files\Microchip\mplabc18\v3.40\bin"
# MP_BC_DIR is not defined

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-45J50.mk dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.cof

MP_PROCESSOR_OPTION=18F45J50
MP_PROCESSOR_OPTION_LD=18f45j50
MP_LINKER_DEBUG_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/usbdsc.o: usbdsc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbdsc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbdsc.o   usbdsc.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbdsc.o 
	
${OBJECTDIR}/usb9.o: usb9.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usb9.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usb9.o   usb9.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usb9.o 
	
${OBJECTDIR}/Boot46J50Family.o: Boot46J50Family.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/Boot46J50Family.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/Boot46J50Family.o   Boot46J50Family.c 
	@${DEP_GEN} -d ${OBJECTDIR}/Boot46J50Family.o 
	
${OBJECTDIR}/usbdrv.o: usbdrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbdrv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbdrv.o   usbdrv.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbdrv.o 
	
${OBJECTDIR}/usbctrltrf.o: usbctrltrf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbctrltrf.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbctrltrf.o   usbctrltrf.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbctrltrf.o 
	
${OBJECTDIR}/hid.o: hid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/hid.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/hid.o   hid.c 
	@${DEP_GEN} -d ${OBJECTDIR}/hid.o 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/main.o   main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/main.o 
	
${OBJECTDIR}/usbmmap.o: usbmmap.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbmmap.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbmmap.o   usbmmap.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbmmap.o 
	
else
${OBJECTDIR}/usbdsc.o: usbdsc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbdsc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbdsc.o   usbdsc.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbdsc.o 
	
${OBJECTDIR}/usb9.o: usb9.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usb9.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usb9.o   usb9.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usb9.o 
	
${OBJECTDIR}/Boot46J50Family.o: Boot46J50Family.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/Boot46J50Family.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/Boot46J50Family.o   Boot46J50Family.c 
	@${DEP_GEN} -d ${OBJECTDIR}/Boot46J50Family.o 
	
${OBJECTDIR}/usbdrv.o: usbdrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbdrv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbdrv.o   usbdrv.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbdrv.o 
	
${OBJECTDIR}/usbctrltrf.o: usbctrltrf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbctrltrf.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbctrltrf.o   usbctrltrf.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbctrltrf.o 
	
${OBJECTDIR}/hid.o: hid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/hid.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/hid.o   hid.c 
	@${DEP_GEN} -d ${OBJECTDIR}/hid.o 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/main.o   main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/main.o 
	
${OBJECTDIR}/usbmmap.o: usbmmap.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/usbmmap.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DEBB_V11 -I"/C/MCC18/h" -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"." -I"."  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/usbmmap.o   usbmmap.c 
	@${DEP_GEN} -d ${OBJECTDIR}/usbmmap.o 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.cof: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) BootModified.rm18f46j50_g.lkr  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG -l"/C/MCC18/lib"  -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -odist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.cof  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.cof: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) BootModified.rm18f46j50_g.lkr  -p$(MP_PROCESSOR_OPTION_LD)  -w  -l"/C/MCC18/lib"  -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}\\..\\lib  -odist/${CND_CONF}/${IMAGE_TYPE}/EBB_BL.X.${IMAGE_TYPE}.cof  ${OBJECTFILES_QUOTED_IF_SPACED}   
endif


# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/45J50
	${RM} -r dist/45J50

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard $(addsuffix .d, ${OBJECTFILES}))
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
