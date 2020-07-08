/* 
 * File:   analog.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 8:50 PM
 */

#ifndef ANALOG_H
#define	ANALOG_H

// ADC counts (out of 1024) that represent 5.5V on SCALED_V+ ADC input
// since 5.5V is the minimum our drivers need in order to work
#define V_PLUS_VOLTAGE_POWERED   378 

/** P U B L I C  P R O T O T Y P E S *****************************************/
void analogConfigure (UINT8 Channel, UINT8 Enable);
void parseARPacket(void);
void parseACPacket(void);
UINT16 analogConvert(UINT8 channel);
void analogCalibrate(void);
void analogInit(void);

#endif	/* ANALOG_H */

