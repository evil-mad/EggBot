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
#if defined(BOARD_3BB)
extern volatile INT32 globalStepCounter3;
#endif

void parse_SM_packet(void);
void parse_AM_packet(void);
void parse_LM_packet(void);
void parse_HM_packet(void);
void parse_XM_packet(void);
void parse_EM_packet(void);
void parse_QM_packet(void);
void parse_ES_packet(void);
void parse_QS_packet(void);
void parse_CS_packet(void);
UINT8 process_QM(void);

#endif	/* STEPPER_H */

