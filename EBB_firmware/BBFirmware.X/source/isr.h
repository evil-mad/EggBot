/* 
 * File:   isr.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 2:42 PM
 */

#ifndef ISR_H
#define	ISR_H



// Reload value for TIMER1
// We need a 25KHz ISR to fire, so we take Fosc (48Mhz), divide by 4
// (normal CPU instruction rate of Fosc/4)
// Then we use a reload value of 480 (0x1E0) to give us
// a rate of 48MHz/4/480 = 25KHz.
// Note that because we can't reload the timer _exactly_ after it fires,
// we have to decrease our 480 value by a few to account for the instructions
// that happen after the timer fires but before we can reload the timer with new
// values.
// The values here are hand tuned for 25KHz ISR operation
#define TIMER1_L_RELOAD (61)
#define TIMER1_H_RELOAD (254)
#define HIGH_ISR_TICKS_PER_MS (25)  // Note: computed by hand, could be formula

#define kPR4_RELOAD             250               // For 1ms TMR4 tick


void high_ISR(void);
void low_ISR(void);

#endif	/* ISR_H */

