/* 
 * File:   stepper.h
 * Author: Brian Schmalz
 *
 * Created on May 27, 2020, 4:07 PM
 */

#ifndef STEPPER_H
#define	STEPPER_H

/* These values hold the global step position of each axis */
extern volatile INT32 globalStepCounter1;
extern volatile INT32 globalStepCounter2;
extern volatile INT32 globalStepCounter3;

void parseSMCommand(void);
void parseAMCommand(void);
void parseLMCommand(void);
void parseHMCommand(void);
void parseXMCommand(void);
void parseEMCommand(void);
void parseQMCommand(void);
void parseESCommand(void);
void parseQSCommand(void);
void parseCSCommand(void);
UINT8 process_QM(void);
void process_SM(
  UINT32 Duration,
  INT32 A1Stp,
  INT32 A2Stp,
  INT32 A3Stp
);

#endif	/* STEPPER_H */

