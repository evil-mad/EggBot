/* 
 * File:   commands.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 8:56 PM
 */

#ifndef COMMANDS_H
#define	COMMANDS_H


extern uint8_t gPulsesOn;
// For Pulse Mode, how long should each pulse be on for in ms?
extern uint16_t gPulseLen[4];
// For Pulse Mode, how many ms between rising edges of pulses?
extern uint16_t gPulseRate[4];
// For Pulse Mode, counters keeping track of where we are
extern uint16_t gPulseCounters[4];


//void parseRSCommand(void);     // R for resetting UBW
//void parseCBCommand(void);     // C for configuring I/O and analog pins
//void parseODCommand(void);     // O for output digital to pins
//void parseIDCommand(void);     // I for input digital from pins
void parseVRCommand(void);     // VR for printing version
//void parsePICommand(void);     // PI for reading a single pin
//void parsePOCommand(void);     // PO for setting a single pin state
//void parsePDCommand(void);     // PD for setting a pin's direction
//void parseMRCommand(void);     // MR for Memory Read
//void parseMWCommand(void);     // MW for Memory Write
//void parseCUCommand(void);     // CU configures UBW (system wide parameters)
//void parsePGCommand(void);     // PG Pulse Go
//void parsePCCommand(void);     // PC Pulse Configure
//void parseBLCommand(void);     // BL Boot Load command
//void parseT1Command(void);     // T1 Test command for input parameters
//void parseT2Command(void);     // T1 Test command for input parameters
//void parseMRCommand(void);     // MR Motors Run command
//void parseRBCommand(void);     // RB ReBoot command

#endif	/* COMMANDS_H */

