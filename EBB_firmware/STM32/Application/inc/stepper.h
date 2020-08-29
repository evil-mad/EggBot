/* 
 * File:   stepper.h
 * Author: Brian Schmalz
 *
 * Created on May 27, 2020, 4:07 PM
 */

#ifndef STEPPER_H
#define	STEPPER_H

#include <stdbool.h>
#include <stdint.h>

/* These values hold the global step position of each axis */
extern volatile int32_t globalStepCounter1;
extern volatile int32_t globalStepCounter2;
extern volatile int32_t globalStepCounter3;

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
uint8_t process_QM(void);
void process_SM(
  uint32_t Duration,
  int32_t A1Stp,
  int32_t A2Stp,
  int32_t A3Stp
);

#endif	/* STEPPER_H */

