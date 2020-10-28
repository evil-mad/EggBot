/* 
 * File:   solenoid.h
 * Author: Brian Schmalz
 *
 * Created on May 28, 2020, 10:34 AM
 */

#ifndef SOLENOID_H
#define	SOLENOID_H

#include <GenericTypeDefs.h>

typedef enum
{
  SOLENOID_OFF = 0,
  SOLENOID_ON,
  SOLENOID_PWM
} SolenoidStateType;

// Set TRUE to enable solenoid output for pen up/down
extern BOOL gUseSolenoid;
extern SolenoidStateType SolenoidState;

void solenoid_Init(void);

#endif	/* SOLENOID_H */

