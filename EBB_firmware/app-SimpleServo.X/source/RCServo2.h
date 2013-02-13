
#ifndef RCSERVO2_H
#define RCSERVO2_H
#include "GenericTypeDefs.h"
#include "Compiler.h"

#define MAX_RC2_SERVOS	24		// This is 24 because there are 24 RPn pins
#define INITAL_RC2_SLOTS 8      // Inital number of RC2 slots (determines repeat rate of pulses)
#define DEFAULT_EBB_SERVO_PORTB_PIN	(1)	// Note, this indicates a PortB pin number, not RPn number
#define DEFAULT_EBB_SERVO_RPN (DEFAULT_EBB_SERVO_PORTB_PIN + 3) // RPn number for default pen up/down servo

extern UINT8 gRC2msCounter;
extern UINT16 gRC2Value[MAX_RC2_SERVOS];
extern UINT8 gRC2RPn[MAX_RC2_SERVOS];
extern UINT8 gRC2Ptr;
extern UINT16 gRC2Target[MAX_RC2_SERVOS];
extern UINT16 gRC2Rate[MAX_RC2_SERVOS];
extern far ram UINT8 * gRC2RPORPtr;
extern UINT16 g_servo2_max;
extern UINT16 g_servo2_min;
extern UINT8 gRC2Slots;
extern UINT8 gRC2SlotMS;
extern UINT16 g_servo2_rate_up;
extern UINT16 g_servo2_rate_down;
extern UINT8 g_servo2_RPn;

void RCServo2_Init(void);
void RCServo2_S2_command(void);
UINT8 RCServo2_Move(UINT16 Position, UINT8 RPn, UINT16 Rate, UINT16 Delay);

#endif
