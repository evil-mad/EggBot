/* 
 * File:   serial.h
 * Author: bschmalz
 *
 * Created on May 29, 2020, 9:49 PM
 */

#ifndef SERIAL_H
#define	SERIAL_H

#include <GenericTypeDefs.h>

void SerialInitDrivers(void);
void SerialInit(void);
void ParseDRCommand(void);
void ParseDWCommand(void);
void ParseSSCommand(void);
void SerialTurnOnTX(void);
void SerialTurnOffTX(void);
BOOL SerialGetGSTATreset(void);


#endif	/* SERIAL_H */

 