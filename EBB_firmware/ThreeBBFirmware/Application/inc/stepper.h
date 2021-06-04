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

// How many stepper motors does this board support? (3BB has 3)
#define NUMBER_OF_STEPPERS      3

/* These values hold the global step position of each axis */
extern volatile int32_t globalStepCounter1;
extern volatile int32_t globalStepCounter2;
extern volatile int32_t globalStepCounter3;

void stepper_SMCommand(void);
void stepper_AMCommand(void);
void stepper_LMCommand(void);
void stepper_HMCommand(void);
void stepper_XMCommand(void);
void stepper_EMCommand(void);
void stepper_QMCommand(void);
void stepper_ESCommand(void);
void stepper_QSCommand(void);
void stepper_CSCommand(void);
uint8_t process_QM(void);
void process_SM(
  uint32_t Duration,
  int32_t A1Stp,
  int32_t A2Stp,
  int32_t A3Stp
);

#endif	/* STEPPER_H */
