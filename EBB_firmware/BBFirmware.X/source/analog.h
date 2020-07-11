/* 
 * File:   analog.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 8:50 PM
 */

#ifndef ANALOG_H
#define	ANALOG_H

/** P U B L I C  P R O T O T Y P E S *****************************************/
void analogConfigure (UINT8 Channel, UINT8 Enable);
void parseARPacket(void);
void parseACPacket(void);
UINT16 analogConvert(UINT8 channel);
void analogCalibrate(void);
void analogInit(void);

#endif	/* ANALOG_H */

