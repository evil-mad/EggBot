
#ifndef EBB_H
#define EBB_H

// 	These are used for Enable<X>IO to control the enable lines for the driver
#define ENABLE_MOTOR	(0)
#define DISABLE_MOTOR	(1)

typedef enum
{
	PEN_DOWN = 0,
	PEN_UP
} PenStateType;

extern unsigned int DemoModeActive;
extern near unsigned char NextReady;
extern unsigned int comd_counter;
void parse_SM_packet(void);
void parse_SC_packet(void);
void parse_SP_packet(void);
void parse_TP_packet(void);
void parse_QP_packet(void);
void parse_SN_packet(void);
void parse_QN_packet(void);
void parse_SL_packet(void);
void parse_QL_packet(void);
void parse_QB_packet(void);
void EBB_Init(void);
void parse_EM_packet(void);
void DemoModeStateMachine(void);
void process_SP(PenStateType NewState, unsigned short CommandDuration);
#endif