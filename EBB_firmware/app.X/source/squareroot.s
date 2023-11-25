#include <p18F46J50.inc>

; When assembly code is placed in a psect, it can be manipulated as a
; whole by the linker and placed in memory.  
;
; In this example, barfunc is the program section (psect) name, 'local' means
; that the section will not be combined with other sections even if they have
; the same name.  class=CODE means the barfunc must go in the CODE container.
; PIC18 should have a delta (addressible unit size) of 1 (default) since they
; are byte addressible.  PIC10/12/16 have a delta of 2 since they are word
; addressible.  PIC18 should have a reloc (alignment) flag of 2 for any
; psect which contains executable code.  PIC10/12/16 can use the default
; reloc value of 1.  Use one of the psects below for the device you use:

; psect   barfunc,local,class=CODE,delta=2 ; PIC10/12/16
;;;psect   barfunc,local,class=CODE,reloc=2 ; PIC18

;;;global _bar ; extern of bar function goes in the C source file
;;;_bar:
;;;    movf PORTA,w    ; here we use a symbol defined via xc.inc
;;;    return

    
    
    
;;;    RES0, RES1
;;;EXTERN Sqrt
; *******************************************************************
; *******************************************************************
;;;W equ 0 ; Standard constants
;;;F equ 1
;;;a equ 0
; *******************************************************************
; *******************************************************************
;;;R_Vctr CODE 0x0000
;;;goto Main
; *******************************************************************
;Software License Agreement
;The software supplied herewith by Microchip Technology Incorporated 
;(the ?Company?) for its PICmicro® Microcontroller is intended and 
;supplied to you, the Company?s customer, for use solely and 
;exclusively on Microchip PICmicro Microcontroller products. The 
;software is owned by the Company and/or its supplier, and is 
;protected under applicable copyright laws. All rights are reserved.
;Any use in violation of the foregoing restrictions may subject the 
;user to criminal sanctions under applicable laws, as well as to civil
;liability for the breach of the terms and conditions of this license.
;THIS SOFTWARE IS PROVIDED IN AN ?AS IS? CONDITION. NO WARRANTIES, 
;WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, 
;IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
;PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, IN ANY 
;CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL 
;DAMAGES, FOR ANY REASON WHATSOEVER.
; *******************************************************************
; Calling Routine
;;;SRoot CODE
;;;Main
;;;movlw 0xCF
;;;movwf ARGA1, a
;;;movlw 0x48
;;;movwf ARGA0, a
;;;call Sqrt ; Sqrt(0xCF48)
; RES0 should now contain 0xE6
;;;movlw 0xE0
;;;movwf ARGA3, a
;;;movlw 0x12
;;;movwf ARGA2, a
;;;movlw 0xA1
;;;movwf ARGA1, a
;;;movlw 0x40
;;;movwf ARGA0, a
;;;call Sqrt ; Sqrt(0xE012A140)
; RES1:RES0 should now contain 0xEF81
;;;bra Main
; *******************************************************************
;;;END

; *******************************************************************
    Title"16/32 bit Integer Square Root"
; *******************************************************************
; *******************************************************************
; *** ***
; *** Author: Ross Fosler ***
; *** Applications Engineer ***
; *** Microchip Technology Inc. ***
; *** ***
; *** Program:sqrt.asm ***
; *** This module contains code to perform fast integer ***
; *** square root functions on either 16 or 32 bit ***
; *** values. ***
; *** ***
; *** Last Rev:August 10, 2000 ***
; *** Ver 1.00 ***
; *** ***
; *******************************************************************
; *******************************************************************
;;;#include P18C252.INC
; *******************************************************************
; *******************************************************************
;;;MSB	equ 7		    ; general literal constants
;;;LSB	equ 0
;;;W	equ 0
;;;F	equ 1
;;;a	equ 0
; *******************************************************************
; *******************************************************************
sqrtVar	UDATA	0x500		    ; allocate RAM in grp5 section
ARGA0	res 1			    ; various argument registers
ARGA1	res 1
ARGA2	res 1
ARGA3	res 1
    GLOBAL ARGA0, ARGA1, ARGA2, ARGA3
ARG1H	res 1
ARG1L	res 1
ARG2H	res 1
ARG2L	res 1
    GLOBAL ARG1H, ARG1L, ARG2H, ARG2L
SARG1	res 1			    ; signed arguments
SARG2	res 1
    GLOBAL SARG1, SARG2
RES1	res 1			    ; result registers
RES0	res 1
    GLOBAL RES0, RES1
SQRES0	res 1
SQRES1	res 1
SQRES2	res 1
SQRES3	res 1
    GLOBAL SQRES0, SQRES1, SQRES2, SQRES3
BITLOC0 res 1			    ; temporary registers
BITLOC1 res 1
TEMP0	res 1
TEMP1	res 1
; *******************************************************************
; *******************************************************************
; The function of this square root routine is to determine the root
; to the nearest integer. At the same time the root is found at the
; best possible speed; therefore, the root is found a little differently
; for the two basic sizes of numbers, 16-bit and 32-bit. The following
; differentiates the two and jumps to the appropriate function.
; Sqrt(ARGA3:ARGA2:ARGA1:ARGA0) = RES1:RES0
	CODE
Sqrt	
	movlb	5		    ; All our RAM is in bank 5
	tstfsz	ARGA3, BANKED	    ; determine if the number is 16-bit
	bra	Sqrt32		    ; or 32-bit and call the best function
	tstfsz	ARGA2, BANKED
	bra	Sqrt32
	clrf	RES1, BANKED
	bra	Sqrt16
	
	GLOBAL	Sqrt

; *******************************************************************
; ******************** Square Root **********************************
; Sqrt16(ARGA1:ARGA0) = RES0
Sqrt16	
	movlb	5		    ; All our RAM is in bank 5
	clrf	TEMP0, BANKED	    ; clear the temp solution
	movlw	0x80		    ; setup the first bit
	movwf	BITLOC0, BANKED
	movwf	RES0, BANKED
Square8 movf	RES0, W, BANKED	    ; square the guess
	mulwf	RES0, BANKED
	movf	PRODL, W, ACCESS    ; ARGA - PROD test
	subwf	ARGA0, W, BANKED
	movf	PRODH, W, ACCESS
	subwfb	ARGA1, W, BANKED
	btfsc	STATUS, C, ACCESS
	bra	NextBit		    ; if positive then next bit
				    ; if negative then rotate right
	movff	TEMP0, RES0	    ; move last good value back into RES0
	rrncf	BITLOC0, F, BANKED  ; then rotote the bit and put it
	movf	BITLOC0, W, BANKED  ; back into RES0
	iorwf	RES0, F, BANKED
	btfsc	BITLOC0, 7, BANKED  ; if last value was tested then get
	bra	Done		    ; out
	bra	Square8		    ; elso go back for another test
NextBit movff	RES0, TEMP0	    ; copy the last good approximation
	rrncf	BITLOC0, F, BANKED  ; rotate the bit location register
	movf	BITLOC0, W, BANKED
	iorwf	RES0, F, BANKED
	btfsc	BITLOC0, 7, BANKED  ; if last value was tested then get
	bra	Done		    ; out
	bra	Square8
Done	movff	TEMP0,RES0	    ; put the final result in RES0
	return

	GLOBAL Sqrt16
; *******************************************************************
; ******************** Square Root **********************************
; Sqrt32(ARGA3:ARGA2:ARGA1:ARGA0) = RES1:RES0
Sqrt32	
	movlb	5		    ; All our RAM is in bank 5
	clrf	TEMP0, BANKED	    ; clear the temp solution
	clrf	TEMP1, BANKED
	clrf	BITLOC0, BANKED	    ; setup the first bit
	clrf	RES0, BANKED
	movlw	0x80
	movwf	BITLOC1, BANKED	    ; BitLoc = 0x8000
	movwf	RES1, BANKED	    ; RES = 0x8000
Squar16 movff	RES0, ARG1L	    ; square the guess
	movff	RES1, ARG1H
	call	Sq16
	movf	SQRES0, W, BANKED   ; ARGA - PROD test
	subwf	ARGA0, W, BANKED
	movf	SQRES1, W, BANKED
	subwfb	ARGA1, W, BANKED
	movf	SQRES2, W, BANKED
	subwfb	ARGA2, W, BANKED
	movf	SQRES3, W, BANKED
	subwfb	ARGA3, W, BANKED
	btfsc	STATUS, C, ACCESS
	bra	NxtBt16		    ; if positive then next bit
				    ; if negative then rotate right
	addlw	0x00		    ; clear carry
	movff	TEMP0, RES0	    ; move last good value back into RES0
	movff	TEMP1, RES1
	rrcf	BITLOC1, F, BANKED  ; then rotote the bit and put it
	rrcf	BITLOC0, F, BANKED
	movf	BITLOC1, W, BANKED  ; back into RES1:RES0
	iorwf	RES1, F, BANKED
	movf	BITLOC0, W, BANKED
	iorwf	RES0, F, BANKED
	btfsc	STATUS, C, ACCESS   ; if last value was tested then get
	bra	Done32		    ; out
	bra	Squar16		    ; elso go back for another test
NxtBt16 addlw	0x00		    ; clear carry
	movff	RES0, TEMP0	    ; copy the last good approximation
	movff	RES1, TEMP1
	rrcf	BITLOC1, F, BANKED  ; rotate the bit location register
	rrcf	BITLOC0, F, BANKED
	movf	BITLOC1, W, BANKED  ; and put back into RES1:RES0
	iorwf	RES1, F, BANKED
	movf	BITLOC0, W, BANKED
	iorwf	RES0, F, BANKED
	btfsc	STATUS, C, ACCESS   ; if last value was tested then get
	bra	Done32		    ; out
	bra	Squar16
Done32	movff	TEMP0,RES0	    ; put the final result in RES1:RES0
	movff	TEMP1,RES1
	return

	GLOBAL Sqrt32
 
; *******************************************************************
; *********** 16 X 16 Unsigned Square *****************************
; SQRES3:SQRES0 = ARG1H:ARG1L ^2
Sq16	movf	ARG1L, W, BANKED
	mulwf	ARG1L		    ; ARG1L * ARG2L ->
				    ; PRODH:PRODL
	movff	PRODH, SQRES1	    ;
	movff	PRODL, SQRES0	    ;
	movf	ARG1H, W, BANKED
	mulwf	ARG1H		    ; ARG1H * ARG2H ->
				    ; PRODH:PRODL
	movff	PRODH, SQRES3	    ;
	movff	PRODL, SQRES2	    ;
	movf	ARG1L, W, BANKED
	mulwf	ARG1H		    ; ARG1L * ARG2H ->
				    ; PRODH:PRODL
	movf	PRODL, W, ACCESS    ;
	addwf	SQRES1, F, BANKED   ; Add cross
	movf	PRODH, W, ACCESS    ; products
	addwfc	SQRES2, F, BANKED   ;
	clrf	WREG, ACCESS	    ;
	addwfc	SQRES3, F, BANKED   ;
	movf	ARG1H, W, BANKED    ;
	mulwf	ARG1L		    ; ARG1H * ARG2L ->
				    ; PRODH:PRODL
	movf	PRODL, W, ACCESS    ;
	addwf	SQRES1, F, BANKED   ; Add cross
	movf	PRODH, W, ACCESS    ; products
	addwfc	SQRES2, F, BANKED   ;
	clrf	WREG, ACCESS		    ;
	addwfc	SQRES3, F, BANKED   ;
	return
	
	GLOBAL Sq16
; *******************************************************************
	end