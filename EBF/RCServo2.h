
#ifndef RCSERVO2_H
#define RCSERVO2_H
#include "GenericTypeDefs.h"
#include "Compiler.h"

#define MAX_RC2_SERVOS	24		// This is 24 because there are 24 RPn pins
#define DEFAULT_EBB_SERVO_PORTB_PIN	(1)	// Note, this indicates a PortB pin number, not RPn number

extern BOOL gUseRCServo2;
extern unsigned char gRC2msCounter;
extern unsigned int gRC2Value[MAX_RC2_SERVOS];
extern unsigned char gRC2Pin[MAX_RC2_SERVOS];
extern unsigned char gRC2Ptr;
extern unsigned int gRC2Target[MAX_RC2_SERVOS];
extern unsigned int gRC2Rate[MAX_RC2_SERVOS];
extern far ram unsigned char * gRC2RPORPtr;
extern unsigned int g_servo2_max;
extern unsigned int g_servo2_min;
extern unsigned char gRC2Slots;
extern unsigned char gRC2SlotMS;
extern unsigned int g_servo2_rate_up;
extern unsigned int g_servo2_rate_down;
extern unsigned char g_servo2_RPpin;

void RCServo2_Init(void);
void RCServo2_S2_command(void);
void Process_S2(unsigned char Channel, unsigned int Duration, unsigned char Pin, unsigned int Rate);

#endif
