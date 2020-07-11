/* 
 * File:   serial.h
 * Author: bschmalz
 *
 * Created on May 29, 2020, 9:49 PM
 */

#ifndef SERIAL_H
#define	SERIAL_H

void serialInitDrivers(void);
void serialInit(void);
void parseDRPacket(void);
void parseDWPacket(void);

#endif	/* SERIAL_H */

