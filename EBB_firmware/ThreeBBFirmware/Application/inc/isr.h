/* 
 * File:   isr.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 2:42 PM
 */

#ifndef ISR_H
#define	ISR_H

/* TMR6 fires high_ISR() at 100 KHz rate, so this value is used in math to take this
 * rate into account.
 * TODO: Make this derive from CubeMX somehow?
 */
#define HIGH_ISR_TICKS_PER_MS (100)

// Tick counter. Increments every 1ms in interrupt
volatile extern uint32_t TickCounterMS;
volatile extern uint8_t GlobalDelayMS;
volatile extern uint8_t DriverInitDelayMS;

void high_ISR(void);
void low_ISR(void);

#endif	/* ISR_H */

