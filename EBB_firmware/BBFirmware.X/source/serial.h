/* 
 * File:   serial.h
 * Author: bschmalz
 *
 * Created on May 29, 2020, 9:49 PM
 */

#ifndef SERIAL_H
#define	SERIAL_H

void SerialInitDrivers(void);
void SerialInit(void);
void ParseDRCommand(void);
void ParseDWCommand(void);

#endif	/* SERIAL_H */

