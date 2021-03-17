/* 
 * File:   utility.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 7:52 PM
 */

#ifndef UTILITY_H
#define	UTILITY_H

#include <stdbool.h>
#include <stdint.h>

#define INPUT_PIN   1
#define OUTPUT_PIN  0

#define bitset(var,bitno) ((var) |= (1 << (bitno)))
#define bitclr(var,bitno) ((var) &= ~(1 << (bitno)))
#define bittst(var,bitno) (var & (1 << bitno))


/* DEBUG I/O definitions 
 * Note: These are only compiled in if the DEBUG symbol is present */
#if defined(BOARD_3BB)
  #if defined(DEBUG)
    #define DEBUG_INIT()                  \
        DEBUG_A0_CLEAR()                  \
        DEBUG_A1_CLEAR()                  \
        DEBUG_A5_CLEAR()                  \
        DEBUG_D0_CLEAR()                  \
        DEBUG_D1_CLEAR()                  \
        DEBUG_C6_CLEAR()                  \
        DEBUG_C7_CLEAR()                  \
        TRISAbits.TRISA0 = OUTPUT_PIN;    \
        TRISAbits.TRISA1 = OUTPUT_PIN;    \
        TRISAbits.TRISA5 = OUTPUT_PIN;    \
        TRISDbits.TRISD0 = OUTPUT_PIN;    \
        TRISDbits.TRISD1 = OUTPUT_PIN;    \
        TRISCbits.TRISC6 = OUTPUT_PIN;    \
        TRISCbits.TRISC7 = OUTPUT_PIN;    \
        DEBUG_A0_SET()                    \
        DEBUG_A1_SET()                    \
        DEBUG_A5_SET()                    \
        DEBUG_D0_SET()                    \
        DEBUG_D1_SET()                    \
        DEBUG_C6_SET()                    \
        DEBUG_C7_SET()                    \
        DEBUG_A0_CLEAR()                  \
        DEBUG_A1_CLEAR()                  \
        DEBUG_A5_CLEAR()                  \
        DEBUG_D0_CLEAR()                  \
        DEBUG_D1_CLEAR()                  \
        DEBUG_C6_CLEAR()                  \
        DEBUG_C7_CLEAR()

    #define DEBUG_A0_SET()    LATAbits.LATA0 = 1;
    #define DEBUG_A0_CLEAR()  LATAbits.LATA0 = 0;
    #define DEBUG_A1_SET()    LATAbits.LATA1 = 1;
    #define DEBUG_A1_CLEAR()  LATAbits.LATA1 = 0;
    #define DEBUG_A5_SET()    LATAbits.LATA5 = 1;
    #define DEBUG_A5_CLEAR()  LATAbits.LATA5 = 0;
    #define DEBUG_D0_SET()    LATDbits.LATD0 = 1;
    #define DEBUG_D0_CLEAR()  LATDbits.LATD0 = 0;
    #define DEBUG_D1_SET()    LATDbits.LATD1 = 1;
    #define DEBUG_D1_CLEAR()  LATDbits.LATD1 = 0;
    #define DEBUG_C6_SET()    LATCbits.LATC6 = 1;
    #define DEBUG_C6_CLEAR()  LATCbits.LATC6 = 0;
    #define DEBUG_C7_SET()    LATCbits.LATC7 = 1;
    #define DEBUG_C7_CLEAR()  LATCbits.LATC7 = 0;
  #else
    #define DEBUG_INIT()
    #define DEBUG_A0_SET()    
    #define DEBUG_A0_CLEAR()  
    #define DEBUG_A1_SET()    
    #define DEBUG_A1_CLEAR()  
    #define DEBUG_A5_SET()    
    #define DEBUG_A5_CLEAR()  
    #define DEBUG_D0_SET()    
    #define DEBUG_D0_CLEAR()  
    #define DEBUG_D1_SET()    
    #define DEBUG_D1_CLEAR()  
    #define DEBUG_C6_SET()    
    #define DEBUG_C6_CLEAR()  
    #define DEBUG_C7_SET()    
    #define DEBUG_C7_CLEAR()  
  #endif
#endif

extern const char st_version[];

extern volatile bool queue_NeedsInit;
// Flag set from ISR indicating that we need to initialize the 2209s
extern volatile bool DriversNeedInit;

void BlinkUSBStatus(void);     // Handles blinking the USB status LED
bool SwitchIsPressed(void);    // Check to see if the user (PRG) switch is pressed
void SetPinTRISFromRPn(char Pin, char State);
void SetPinLATFromRPn(char Pin, char State);
void populateDeviceStringWithName(void);
uint32_t GetTick(void);
void ParseSTCommand(void);    // ST Set Tag command
void ParseQTCommand(void);    // QT Query Tag command
void utility_SysTick(void);
#if defined(DEBUG)
void ParseSHCommand(void);    // SH print Stack Highwater
#endif
void utility_Run(void);
#if defined(DEBUG)
void UtilityFillStack(void);
void UtilityPrintStackHighWater(void);
#endif

#endif	/* UTILITY_H */

