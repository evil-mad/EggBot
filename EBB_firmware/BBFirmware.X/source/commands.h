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


void parse_R_packet(void);     // R for resetting UBW
void parse_C_packet(void);     // C for configuring I/O and analog pins
void parse_O_packet(void);     // O for output digital to pins
void parse_I_packet(void);     // I for input digital from pins
void parse_V_packet(void);     // V for printing version
void parse_PI_packet(void);    // PI for reading a single pin
void parse_PO_packet(void);    // PO for setting a single pin state
void parse_PD_packet(void);    // PD for setting a pin's direction
void parse_MR_packet(void);    // MR for Memory Read
void parse_MW_packet(void);    // MW for Memory Write
void parse_CU_packet(void);    // CU configures UBW (system wide parameters)
void parse_PG_packet(void);    // PG Pulse Go
void parse_PC_packet(void);    // PC Pulse Configure
void parse_BL_packet(void);    // BL Boot Load command
void parse_CK_packet(void);    // CK ChecK command
void parse_MR_packet(void);    // MR Motors Run command
void parse_RB_packet(void);    // RB ReBoot command
#if defined(BOARD_EBB)
void parse_QR_packet(void);    // QR Query RC Servo power state
void parse_SR_packet(void);    // SR Set RC Servo power timeout
#endif

#endif	/* COMMANDS_H */

