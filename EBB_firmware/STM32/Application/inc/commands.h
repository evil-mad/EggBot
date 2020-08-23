/* 
 * File:   commands.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 8:56 PM
 */

#ifndef COMMANDS_H
#define	COMMANDS_H


extern unsigned char gPulsesOn;
// For Pulse Mode, how long should each pulse be on for in ms?
extern unsigned int gPulseLen[4];
// For Pulse Mode, how many ms between rising edges of pulses?
extern unsigned int gPulseRate[4];
// For Pulse Mode, counters keeping track of where we are
extern unsigned int gPulseCounters[4];


void parseRSCommand(void);     // R for resetting UBW
void parseCBCommand(void);     // C for configuring I/O and analog pins
void parseODCommand(void);     // O for output digital to pins
void parseIDCommand(void);     // I for input digital from pins
void parseVRCommand(void);     // V for printing version
void parsePICommand(void);    // PI for reading a single pin
void parsePOCommand(void);    // PO for setting a single pin state
void parsePDCommand(void);    // PD for setting a pin's direction
void parseMRCommand(void);    // MR for Memory Read
void parseMWCommand(void);    // MW for Memory Write
void parseCUCommand(void);    // CU configures UBW (system wide parameters)
void parsePGCommand(void);    // PG Pulse Go
void parsePCCommand(void);    // PC Pulse Configure
void parseBLCommand(void);    // BL Boot Load command
void parseT1Command(void);     // T1 Test command for input parameters
void parseT2Command(void);     // T1 Test command for input parameters
void parseMRCommand(void);    // MR Motors Run command
void parseRBCommand(void);    // RB ReBoot command
#if defined(BOARD_EBB)
void parseQR_packet(void);    // QR Query RC Servo power state
void parseSR_packet(void);    // SR Set RC Servo power timeout
#endif

#endif	/* COMMANDS_H */

