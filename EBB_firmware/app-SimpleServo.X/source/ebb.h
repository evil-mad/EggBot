
#ifndef EBB_H
#define EBB_H

// 	These are used for Enable<X>IO to control the enable lines for the driver
#define ENABLE_MOTOR        0
#define DISABLE_MOTOR       1

// How many stepper motors does this board support? (EBB is always 2)
#define NUMBER_OF_STEPPERS  2

typedef enum
{
	PEN_DOWN = 0,
	PEN_UP
} PenStateType;

typedef enum
{
	COMMAND_NONE = 0,
	COMMAND_MOTOR_MOVE,
	COMMAND_DELAY,
	COMMAND_SERVO_MOVE
} CommandType;

// This structure defines the elements of the move commands in the FIFO that
// are sent from the command parser to the ISR move engine.
typedef struct
{
    CommandType     Command;
    INT16           StepAdd[NUMBER_OF_STEPPERS];
    UINT16          StepsCounter[NUMBER_OF_STEPPERS];
    UINT8           DirBits;
    UINT16          DelayCounter;
    UINT16          ServoPosition;
    UINT8           ServoRPn;
    UINT8           ServoChannel;
    UINT16          ServoRate;
} MoveCommandType;

// Define global things that depend on the board type
#define STEP1_BIT	(0x01)
#define DIR1_BIT	(0x02)
#define STEP2_BIT	(0x04)
#define DIR2_BIT	(0x08)

#define NUMBER_OF_STEPPERS  2


extern MoveCommandType CommandFIFO[];
extern unsigned int DemoModeActive;
extern near BOOL FIFOEmpty;
extern unsigned int comd_counter;
extern unsigned char QC_ms_timer;
// Default to on, comes out on pin RB4 for EBB v1.3 and above
extern BOOL gUseSolenoid;
void parse_SM_packet(void);
void parse_SC_packet(void);
void parse_SP_packet(void);
void parse_TP_packet(void);
void parse_QP_packet(void);
void parse_SN_packet(void);
void parse_QN_packet(void);
void parse_NI_packet(void);
void parse_ND_packet(void);
void parse_SL_packet(void);
void parse_QL_packet(void);
void parse_QB_packet(void);
void parse_EM_packet(void);
void parse_QC_packet(void);
void parse_SE_packet(void);
void parse_RM_packet(void);
void parse_QM_packet(void);
void EBB_Init(void);
void process_SP(PenStateType NewState, UINT16 CommandDuration);
void process_SM(
	unsigned int Duration,
	signed int A1Stp,
	signed int A2Stp
);
#endif