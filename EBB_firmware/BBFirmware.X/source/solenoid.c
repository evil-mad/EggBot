
#include "solenoid.h"
#include "ebb.h"
#include "servo.h"


static unsigned int SolenoidDelay;
// Set TRUE to enable solenoid output for pen up/down
BOOL gUseSolenoid;
SolenoidStateType SolenoidState;

void solenoid_Init(void)
{
  SolenoidState = SOLENOID_ON;
  DriverConfiguration = PIC_CONTROLS_DRIVERS;
  PenState = PEN_UP;
  Layer = 0;
  NodeCount = 0;
  ButtonPushed = FALSE;
}
