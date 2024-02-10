	    ; Assembly Theory: 
	    ; We want to copy out an entire Command structure as quickly as possible.
	    ; To do this we will temporarily take over the FSR0 register to point 
	    ; to the beginning of the source structure in RAM, and then hard code
	    ; enough MOVFF <location>, POSTINC0 instructions to copy out the whole thing.
	    ; The <location> addresses are the static addresses of the CurrentCommand
	    ; structure. We do need to save off FSR0 and restore it after to make sure
	    ; we don't mess anything up that the compiler expects.

	    INCLUDE "p18f46j50.inc"

	    ; Defines section
	    
	    ; Reload value for TIMER0
	    ; We need a 25KHz ISR to fire, so we take Fosc (48Mhz) and divide by 4
	    ; (normal CPU instruction rate of Fosc/4). We then set up Timer0 so that it
	    ; has a 1:4 clock prescaler. Thus Timer0 is being clocked by a 
	    ; 3MHz clock. We use a reload value of 0x8A to give us an ISR rate of 
	    ; 48MHz/4/4/120 = 25KHz. In order to get the timer to count 120 clocks,
	    ; we give it a reload value of 256-120=136 (since it's a count up timer)
	    ; but we also need to add 2 timer counts to bring it to 138 in order to account
	    ; for the additional time required to get into the ISR and read out the value
	    ; of the timer in order to compute the next reload value.

#define TIMER0_RELOAD 0x8A

	    ; Set to use RC0 as indicator that a command is being parsed
#define TEST_MODE_PARSING_COMMAND_NUM     0
#define TEST_MODE_PARSING_COMMAND_BIT     1
	    ; Set to make D0, D1, A1 outputs for Next Command, In ISR and FIFO Empty
#define TEST_MODE_GPIO_NUM                1
#define TEST_MODE_GPIO_BIT                (1 << TEST_MODE_GPIO_BIT_NUM)
	    ; Set for printing ISR debug info to UART at end of every move
#define TEST_MODE_USART_ISR_NUM           2
#define TEST_MODE_USART_ISR_BIT           (1 << TEST_MODE_USART_ISR_BIT_NUM)
	    ; Set this and TEST_MODE_USART_ISR_BIT_NUM for printing every ISR, not just end of move
#define TEST_MODE_USART_ISR_FULL_NUM      3
#define TEST_MODE_USART_ISR_FULL_BIT      (1 << TEST_MODE_USART_ISR_FULL_BIT_NUM)
	    ; Prints every received byte out to debug UART
#define TEST_MODE_USART_COMMAND_NUM       4
#define TEST_MODE_USART_COMMAND_BIT       (1 << TEST_MODE_USART_COMMAND_BIT_NUM)
	    ; Prints additional command debugging info to USB back to PC
#define TEST_MODE_DEBUG_COMMAND_NUM       5
#define TEST_MODE_DEBUG_COMMAND_BIT       (1 << TEST_MODE_DEBUG_COMMAND_BIT_NUM)
	    ; When 1, commands are parsed but not sent to FIFO. 0 (default) has commands go to FIFO
#define TEST_MODE_DEBUG_BLOCK_FIFO_NUM    6
#define TEST_MODE_DEBUG_BLOCK_FIFO_BIT    (1 << TEST_MODE_DEBUG_BLOCK_FIFO_NUM)
	    ; This last bit is used during the ISR and is not available as a general test mode bit
#define TEST_MODE_PRINT_TRIGGER_NUM       7
#define TEST_MODE_PRINT_TRIGGER_BIT       (1 << TEST_MDOE_PRINT_TRIGGER_BIT_NUM)

	        
; Defines for the CommandType BYTE in the MoveCommandType
; Note that three USB commands (SM, XM HM) are
; all the same and represented by one command value.
#define COMMAND_NONE                  0
#define COMMAND_DELAY                 1
#define COMMAND_SERVO_MOVE            2
#define COMMAND_SE                    3
#define COMMAND_EM                    4
#define COMMAND_SM_XM_HM_MOVE         5
#define COMMAND_LM_MOVE               6
#define COMMAND_LT_MOVE               7
#define COMMAND_CM_OUTER_MOVE         8
#define COMMAND_CM_INNER_MOVE         9
	   
; Define global things that depend on the board type
; STEP2 = RD4
#define STEP2_BIT_NUM   4
#define STEP2_BIT       (1 << STEP2_BIT_NUM)
; DIR2 = RD5
#define DIR2_BIT_NUM    5
#define DIR2_BIT        (1 << DIR2_BIT_NUM)
; STEP1 = RD6
#define STEP1_BIT_NUM   6
#define STEP1_BIT       (1 << STEP1_BIT_NUM)
; DIR1 = RD7
#define DIR1_BIT_NUM    7
#define DIR1_BIT        (1 << DIR1_BIT_NUM)

	    
	    
;	    EXTERN isr_FSR0L_temp
;	    EXTERN isr_FSR0H_temp
;	    EXTERN FIFO_out_ptr_high
;	    EXTERN FIFO_out_ptr_low
	    EXTERN DriverConfiguration
	    EXTERN CurrentCommand
	    EXTERN CurrentCommand.Command
	    EXTERN CurrentCommand.m.sm.DirBits
	    EXTERN TestMode
	    EXTERN AllDone

	    CODE

;FIFO_COPY:
	    ; First store off the current values in FSR0
;	    MOVFF FSR0L, isr_FSR0L_temp
;	    MOVFF FSR0H, isr_FSR0H_temp
         
	    ; Then load up FSR0 with the address of the beginning of the FIFO element
	    ; currently pointed to by the FIFO out pointer
;	    MOVFF FIFO_out_ptr_high, FSR0H
;	    MOVFF FIFO_out_ptr_low, FSR0L
              
	    ; Now walk through the whole length of the FIFO element, copying to 
	    ; the local CurrentCommand
;	    MOVFF POSTINC0, CurrentCommand	; 1
;	    MOVFF POSTINC0, CurrentCommand+.1	; 2
;	    MOVFF POSTINC0, CurrentCommand+.2	; 3
;	    MOVFF POSTINC0, CurrentCommand+.3	; 4
;	    MOVFF POSTINC0, CurrentCommand+.4	; 5
;	    MOVFF POSTINC0, CurrentCommand+.5	; 6
;	    MOVFF POSTINC0, CurrentCommand+.6	; 7
;	    MOVFF POSTINC0, CurrentCommand+.7	; 8
;	    MOVFF POSTINC0, CurrentCommand+.8	; 9
;	    MOVFF POSTINC0, CurrentCommand+.9	; 10
;	    MOVFF POSTINC0, CurrentCommand+.10	; 11
;	    MOVFF POSTINC0, CurrentCommand+.11	; 12
;	    MOVFF POSTINC0, CurrentCommand+.12	; 13
;	    MOVFF POSTINC0, CurrentCommand+.13	; 14
;	    MOVFF POSTINC0, CurrentCommand+.14	; 15
;	    MOVFF POSTINC0, CurrentCommand+.15	; 16
;	    MOVFF POSTINC0, CurrentCommand+.16	; 17
;	    MOVFF POSTINC0, CurrentCommand+.17	; 18
;	    MOVFF POSTINC0, CurrentCommand+.18	; 19
;	    MOVFF POSTINC0, CurrentCommand+.19	; 20
;	    MOVFF POSTINC0, CurrentCommand+.20	; 21
;	    MOVFF POSTINC0, CurrentCommand+.21	; 22
;	    MOVFF POSTINC0, CurrentCommand+.22	; 23
;	    MOVFF POSTINC0, CurrentCommand+.23	; 24
;	    MOVFF POSTINC0, CurrentCommand+.24	; 25
;	    MOVFF POSTINC0, CurrentCommand+.25	; 26
;	    MOVFF POSTINC0, CurrentCommand+.26	; 27
;	    MOVFF POSTINC0, CurrentCommand+.27	; 28
;	    MOVFF POSTINC0, CurrentCommand+.28	; 29
;	    MOVFF POSTINC0, CurrentCommand+.29	; 30
;	    MOVFF POSTINC0, CurrentCommand+.30	; 31
;	    MOVFF POSTINC0, CurrentCommand+.31	; 32
;	    MOVFF POSTINC0, CurrentCommand+.32	; 33
;	    MOVFF POSTINC0, CurrentCommand+.33	; 34
;	    MOVFF POSTINC0, CurrentCommand+.34	; 35
;	    MOVFF POSTINC0, CurrentCommand+.35	; 36
;	    MOVFF POSTINC0, CurrentCommand+.36	; 37
;	    MOVFF POSTINC0, CurrentCommand+.37	; 38
;	    MOVFF POSTINC0, CurrentCommand+.38	; 39
;	    MOVFF POSTINC0, CurrentCommand+.39	; 40
;	    MOVFF POSTINC0, CurrentCommand+.40	; 41
;	    MOVFF POSTINC0, CurrentCommand+.41	; 42
;	    MOVFF POSTINC0, CurrentCommand+.42	; 43
;	    MOVFF POSTINC0, CurrentCommand+.43	; 44
;	    MOVFF POSTINC0, CurrentCommand+.44	; 45
;	    MOVFF POSTINC0, CurrentCommand+.45	; 46
;	    MOVFF POSTINC0, CurrentCommand+.46	; 47
              
	    ; We've incremented FSR0 to the beginning of the next FIFO element now
	    ; so save it back to the out pointer for the next time
	    ; (We will check for wrap-around down in the C below)
;	    MOVFF FSR0H, FIFO_out_ptr_high
;	    MOVFF FSR0L, FIFO_out_ptr_low
              
	    ; Lastly retrieve the previous values in FSR0
;	    MOVFF isr_FSR0L_temp, FSR0L
;	    MOVFF isr_FSR0H_temp, FSR0H

;	    RETURN
            
;	    GLOBAL FIFO_COPY  ; export so linker can see it
	    
high_ISR:	    
	    MOVFF   FSR2H,PREINC1	; Push FSR2H, FSR0(L/H), PROD(L/H)
	    MOVFF   FSR0L,PREINC1                                                                                     
	    MOVFF   FSR0H,PREINC1                                                                                     
	    MOVFF   PRODL,PREINC1                                                                                     
	    MOVFF   PRODH,PREINC1                                                                                     
	    MOVF    POSTINC1,F,ACCESS	; Increment FSR1 (stack pointer) - not sure why yet                                                                  
	    
	    MOVLB   0x1			; Set up BANK1 as bank for all variables
	    
	    BCF	    INTCON,0x2,ACCESS	; INTCONbits.TMR0IF = 0 // Clear the interrupt
; Insert CurTime stuff here
	    MOVLW   TIMER0_RELOAD
	    MOVWF   TMR0L,ACCESS
	    
	    ; If TEST_MODE_GPIO_NUM is on, then set D1 to indicate 'In ISR'
	    ; if (bittst(TestMode, TEST_MODE_GPIO_NUM))
	    ; {
	    ;   LATDbits.LATD1 = 1;
	    ; }
	    BTFSC   TestMode,TEST_MODE_GPIO_NUM,ACCESS
	    
	    BSF	    LATD,0x1,ACCESS		; LATDbits.LATD1 = 1
	    
	    ; bitsetzero(AllDone)
	    BSF	    AllDone,0x0,ACCESS
	    	    
	    ; Process a motor move command of any type
            ; This is the main chunk of code for EBB : the step generation code in the 25KHz ISR.
            ; The first section determines if we need to take any steps this time through the ISR.
            ; It is broken into sections, one for each type of stepper motion command because
            ; they each have different amounts of processing needed to determine if a step is
            ; necessary or not.
            ; Then the second section is common to all stepper motion commands and handles
            ; the actual step pulse generation as well as direction bit control.
            
            ; The Active bits will be set if there is still motion left to
            ; generate on an axis. They are set when the command is first loaded
            ; into CurrentCommand and then cleared once all steps have been taken
            ; (either in time or in step count). They are checked inside each
            ; command's step processing (below) so that we only spend time working
            ; on axis that have activity left
            
            ; Important assumptions:
            ; There is only one bit set in the CurrentCommand.Command byte
            
            ; A very complete mathematical analysis of the way the various stepper
            ; commands are expected to work can be found here
            ; https://evilmadscience.s3.amazonaws.com/dl/ad/public/AxiDrawKinematics.pdf
            
            ; Here we handle the 'simple' (non accelerating) stepper commands.
            ; These three command (SM, XM and HM) do not use
            ; acceleration and so that code is left out to save time. This is also
            ; the most common command used by the various PC softwares so we check
            ; for it first.
	    ; Perform all processing of step math for SM, XM and HM moves

	    ; if (CurrentCommand.Commands == COMMAND_SM_XM_HM_MOVE)
	    ; {
	    MOVLW   COMMAND_SM_XM_HM_MOVE	    
	    SUBWF   CurrentCommand.Command,W,BANKED
	    BNZ	    CheckForLMCommand
	    
	    ; Direction bits need to be output before step bits. Since we know step bits
	    ; are clear at this point, it's safe to always output direction bits here  
	    ; before we start messing with the step bits                               
	    MOVF    DriverConfiguration,W,ACCESS
	    BNZ	    Label001

	    ; The Step and Direction bits are all output on the top four bits   
	    ; of PortD. So we can be very efficient here and simply output those
	    ; four bits directly to port D.
	    ; LATD = (PORTD & ~(DIR1_BIT | DIR2_BIT)) | CurrentCommand.m.sm.DirBits;
	    MOVLW   ~(DIR1_BIT | DIR2_BIT)
	    ANDWF   PORTD,W,ACCESS
	    IORWF   CurrentCommand.m.sm.DirBits,W,BANKED
	    MOVWF   LATD,ACCESS

Label001:
	    
	    ; }
CheckForLMCommand:
	    
	    
	    
TempEnd:
	    
	    
	    
	    ; If TEST_MODE_GPIO_NUM is on, then clear our three test mode GPIO pins
	    BTFSS   TestMode,TEST_MODE_GPIO_NUM,A
	    BRA	    ISREnd
	    
	    BCF	    LATA,0x1,A		; LATAbits.LATA1 = 0
	    BCF	    LATD,0x0,A		; LATDbits.LATD0 = 0
	    BCF	    LATD,0x1,A		; LATDbits.LATD1 = 0
	    
ISREnd:	    
	    MOVF    POSTDEC1,F,A	; Decrement FSR1 (stack pointer) - not sure why yet
	    MOVFF   POSTDEC1,PRODH	; Pop FSR2H, FSR0(L/H), PROD(L/H)
	    MOVFF   POSTDEC1,PRODL
	    MOVFF   POSTDEC1,FSR0H
	    MOVFF   POSTDEC1,FSR0L
	    MOVFF   POSTDEC1,FSR2H	    
	    RETFIE  0x1
    
	    GLOBAL  high_ISR
	    END
	    