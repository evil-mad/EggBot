	    ; Assembly Theory: 
	    ; We want to copy out an entire Command structure as quickly as possible.
	    ; To do this we will temporarily take over the FSR0 register to point 
	    ; to the beginning of the source structure in RAM, and then hard code
	    ; enough MOVFF <location>, POSTINC0 instructions to copy out the whole thing.
	    ; The <location> addresses are the static addresses of the CurrentCommand
	    ; structure. We do need to save off FSR0 and restore it after to make sure
	    ; we don't mess anything up that the compiler expects.

	    INCLUDE "p18f46j50.inc"

	    EXTERN isr_FSR0L_temp
	    EXTERN isr_FSR0H_temp
	    EXTERN FIFO_out_ptr_high
	    EXTERN FIFO_out_ptr_low
	    EXTERN CurrentCommand

	    CODE

FIFO_COPY:
	    ; First store off the current values in FSR0
	    MOVFF FSR0L, isr_FSR0L_temp
	    MOVFF FSR0H, isr_FSR0H_temp
              
	    ; Then load up FSR0 with the address of the beginning of the FIFO element
	    ; currently pointed to by the FIFO out pointer
	    MOVFF FIFO_out_ptr_high, FSR0H
	    MOVFF FIFO_out_ptr_low, FSR0L
              
	    ; Now walk through the whole length of the FIFO element, copying to 
	    ; the local CurrentCommand
	    MOVFF POSTINC0, CurrentCommand	; 1
	    MOVFF POSTINC0, CurrentCommand+.1	; 2
	    MOVFF POSTINC0, CurrentCommand+.2	; 3
	    MOVFF POSTINC0, CurrentCommand+.3	; 4
	    MOVFF POSTINC0, CurrentCommand+.4	; 5
	    MOVFF POSTINC0, CurrentCommand+.5	; 6
	    MOVFF POSTINC0, CurrentCommand+.6	; 7
	    MOVFF POSTINC0, CurrentCommand+.7	; 8
	    MOVFF POSTINC0, CurrentCommand+.8	; 9
	    MOVFF POSTINC0, CurrentCommand+.9	; 10
	    MOVFF POSTINC0, CurrentCommand+.10	; 11
	    MOVFF POSTINC0, CurrentCommand+.11	; 12
	    MOVFF POSTINC0, CurrentCommand+.12	; 13
	    MOVFF POSTINC0, CurrentCommand+.13	; 14
	    MOVFF POSTINC0, CurrentCommand+.14	; 15
	    MOVFF POSTINC0, CurrentCommand+.15	; 16
	    MOVFF POSTINC0, CurrentCommand+.16	; 17
	    MOVFF POSTINC0, CurrentCommand+.17	; 18
	    MOVFF POSTINC0, CurrentCommand+.18	; 19
	    MOVFF POSTINC0, CurrentCommand+.19	; 20
	    MOVFF POSTINC0, CurrentCommand+.20	; 21
	    MOVFF POSTINC0, CurrentCommand+.21	; 22
	    MOVFF POSTINC0, CurrentCommand+.22	; 23
	    MOVFF POSTINC0, CurrentCommand+.23	; 24
	    MOVFF POSTINC0, CurrentCommand+.24	; 25
	    MOVFF POSTINC0, CurrentCommand+.25	; 26
	    MOVFF POSTINC0, CurrentCommand+.26	; 27
	    MOVFF POSTINC0, CurrentCommand+.27	; 28
	    MOVFF POSTINC0, CurrentCommand+.28	; 29
	    MOVFF POSTINC0, CurrentCommand+.29	; 30
	    MOVFF POSTINC0, CurrentCommand+.30	; 31
	    MOVFF POSTINC0, CurrentCommand+.31	; 32
	    MOVFF POSTINC0, CurrentCommand+.32	; 33
	    MOVFF POSTINC0, CurrentCommand+.33	; 34
	    MOVFF POSTINC0, CurrentCommand+.34	; 35
	    MOVFF POSTINC0, CurrentCommand+.35	; 36
	    MOVFF POSTINC0, CurrentCommand+.36	; 37
	    MOVFF POSTINC0, CurrentCommand+.37	; 38
	    MOVFF POSTINC0, CurrentCommand+.38	; 39
	    MOVFF POSTINC0, CurrentCommand+.39	; 40
	    MOVFF POSTINC0, CurrentCommand+.40	; 41
	    MOVFF POSTINC0, CurrentCommand+.41	; 42
	    MOVFF POSTINC0, CurrentCommand+.42	; 43
	    MOVFF POSTINC0, CurrentCommand+.43	; 44
	    MOVFF POSTINC0, CurrentCommand+.44	; 45
	    MOVFF POSTINC0, CurrentCommand+.45	; 46
	    MOVFF POSTINC0, CurrentCommand+.46	; 47
              
	    ; We've incremented FSR0 to the beginning of the next FIFO element now
	    ; so save it back to the out pointer for the next time
	    ; (We will check for wrap-around down in the C below)
;	    MOVFF FSR0H, FIFO_out_ptr_high
;	    MOVFF FSR0L, FIFO_out_ptr_low
              
	    ; Lastly retrieve the previous values in FSR0
	    MOVFF isr_FSR0L_temp, FSR0L
	    MOVFF isr_FSR0H_temp, FSR0H

	    RETURN
            
	    GLOBAL FIFO_COPY  ; export so linker can see it
	    END
	    