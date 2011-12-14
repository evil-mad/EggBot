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
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/EBF_EBBv13_and_above.X.${IMAGE_TYPE}.cof
else
IMAGE_TYPE=production
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/EBF_EBBv13_and_above.X.${IMAGE_TYPE}.cof
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/131024517/usb_function_cdc.o ${OBJECTDIR}/_ext/1472/RCServo2.o ${OBJECTDIR}/_ext/1472/UBW.o ${OBJECTDIR}/_ext/1472/ebb.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/usb_descriptors.o ${OBJECTDIR}/_ext/343710134/usb_device.o

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/131024517/usb_function_cdc.o ${OBJECTDIR}/_ext/1472/RCServo2.o ${OBJECTDIR}/_ext/1472/UBW.o ${OBJECTDIR}/_ext/1472/ebb.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/usb_descriptors.o ${OBJECTDIR}/_ext/343710134/usb_device.o


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/EBF_EBBv13_and_above.X.${IMAGE_TYPE}.cof

MP_PROCESSOR_OPTION=18F46J50
MP_PROCESSOR_OPTION_LD=18f46j50
MP_LINKER_DEBUG_OPTION= -u_DEBUGCODESTART=0xfe68 -u_DEBUGCODELEN=0x190 -u_DEBUGDATASTART=0xfdb -u_DEBUGDATALEN=0x4
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/main.o   ../main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/main.o 
	
${OBJECTDIR}/_ext/1472/usb_descriptors.o: ../usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_descriptors.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/usb_descriptors.o   ../usb_descriptors.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/usb_descriptors.o 
	
${OBJECTDIR}/_ext/343710134/usb_device.o: ../Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/343710134 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/343710134/usb_device.o   ../Microchip/USB/usb_device.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/343710134/usb_device.o 
	
${OBJECTDIR}/_ext/1472/UBW.o: ../UBW.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/UBW.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/UBW.o   ../UBW.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/UBW.o 
	
${OBJECTDIR}/_ext/1472/ebb.o: ../ebb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ebb.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/ebb.o   ../ebb.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/ebb.o 
	
${OBJECTDIR}/_ext/1472/RCServo2.o: ../RCServo2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/RCServo2.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/RCServo2.o   ../RCServo2.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/RCServo2.o 
	
${OBJECTDIR}/_ext/131024517/usb_function_cdc.o: ../Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/131024517 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PICKIT2=1 -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o   "../Microchip/USB/CDC Device Driver/usb_function_cdc.c" 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o 
	
else
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/main.o   ../main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/main.o 
	
${OBJECTDIR}/_ext/1472/usb_descriptors.o: ../usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_descriptors.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/usb_descriptors.o   ../usb_descriptors.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/usb_descriptors.o 
	
${OBJECTDIR}/_ext/343710134/usb_device.o: ../Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/343710134 
	@${RM} ${OBJECTDIR}/_ext/343710134/usb_device.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/343710134/usb_device.o   ../Microchip/USB/usb_device.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/343710134/usb_device.o 
	
${OBJECTDIR}/_ext/1472/UBW.o: ../UBW.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/UBW.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/UBW.o   ../UBW.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/UBW.o 
	
${OBJECTDIR}/_ext/1472/ebb.o: ../ebb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ebb.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/ebb.o   ../ebb.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/ebb.o 
	
${OBJECTDIR}/_ext/1472/RCServo2.o: ../RCServo2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/RCServo2.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/RCServo2.o   ../RCServo2.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1472/RCServo2.o 
	
${OBJECTDIR}/_ext/131024517/usb_function_cdc.o: ../Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/131024517 
	@${RM} ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DBOARD_EBB_V13_AND_ABOVE -I"/c/mcc18/h" -I"../Microchip/Include" -I".." -I"../Microchip/Include/USB"  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o   "../Microchip/USB/CDC Device Driver/usb_function_cdc.c" 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/131024517/usb_function_cdc.o 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/EBF_EBBv13_and_above.X.${IMAGE_TYPE}.cof: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) ../rm18f46j50_g.lkr  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG -l"/C/MCC18/lib" -l"/C/Program Files/Microchip/mplabc18/v3.37.01/lib"  -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_PICKIT2=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -odist/${CND_CONF}/${IMAGE_TYPE}/EBF_EBBv13_and_above.X.${IMAGE_TYPE}.cof  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/EBF_EBBv13_and_above.X.${IMAGE_TYPE}.cof: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) ../rm18f46j50_g.lkr  -p$(MP_PROCESSOR_OPTION_LD)  -w  -l"/C/MCC18/lib" -l"/C/Program Files/Microchip/mplabc18/v3.37.01/lib"  -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}\\..\\lib  -odist/${CND_CONF}/${IMAGE_TYPE}/EBF_EBBv13_and_above.X.${IMAGE_TYPE}.cof  ${OBJECTFILES_QUOTED_IF_SPACED}   
endif


# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard $(addsuffix .d, ${OBJECTFILES}))
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
