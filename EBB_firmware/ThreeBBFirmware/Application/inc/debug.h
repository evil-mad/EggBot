/*
 * debug.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Brian Schmalz
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

/* Define the I/O macros for debug pins (G0 to G12)
 * G0 to G11 are available on the J3 GPIO header. G12 is on a Test Point
 */
#define DEBUG_G0_SET()     G0_GPIO_Port->BSRR =  (uint32_t)G0_Pin
#define DEBUG_G0_RESET()   G0_GPIO_Port->BRR  =  (uint32_t)G0_Pin
#define DEBUG_G1_SET()     G1_GPIO_Port->BSRR =  (uint32_t)G1_Pin
#define DEBUG_G1_RESET()   G1_GPIO_Port->BRR  =  (uint32_t)G1_Pin
#define DEBUG_G2_SET()     G2_GPIO_Port->BSRR =  (uint32_t)G2_Pin
#define DEBUG_G2_RESET()   G2_GPIO_Port->BRR  =  (uint32_t)G2_Pin
#define DEBUG_G3_SET()     G3_GPIO_Port->BSRR =  (uint32_t)G3_Pin
#define DEBUG_G3_RESET()   G3_GPIO_Port->BRR  =  (uint32_t)G3_Pin
#define DEBUG_G4_SET()     G4_GPIO_Port->BSRR =  (uint32_t)G4_Pin
#define DEBUG_G4_RESET()   G4_GPIO_Port->BRR  =  (uint32_t)G4_Pin
#define DEBUG_G5_SET()     G5_GPIO_Port->BSRR =  (uint32_t)G5_Pin
#define DEBUG_G5_RESET()   G5_GPIO_Port->BRR  =  (uint32_t)G5_Pin
#define DEBUG_G6_SET()     G6_GPIO_Port->BSRR =  (uint32_t)G6_Pin
#define DEBUG_G6_RESET()   G6_GPIO_Port->BRR  =  (uint32_t)G6_Pin
#define DEBUG_G7_SET()     G7_GPIO_Port->BSRR =  (uint32_t)G7_Pin
#define DEBUG_G7_RESET()   G7_GPIO_Port->BRR  =  (uint32_t)G7_Pin
#define DEBUG_G8_SET()     G8_GPIO_Port->BSRR =  (uint32_t)G8_Pin
#define DEBUG_G8_RESET()   G8_GPIO_Port->BRR  =  (uint32_t)G8_Pin
#define DEBUG_G9_SET()     G9_GPIO_Port->BSRR =  (uint32_t)G9_Pin
#define DEBUG_G9_RESET()   G9_GPIO_Port->BRR  =  (uint32_t)G9_Pin
#define DEBUG_G10_SET()    G10_GPIO_Port->BSRR = (uint32_t)G10_Pin
#define DEBUG_G10_RESET()  G10_GPIO_Port->BRR  = (uint32_t)G10_Pin
#define DEBUG_G11_SET()    G11_GPIO_Port->BSRR = (uint32_t)G11_Pin
#define DEBUG_G11_RESET()  G11_GPIO_Port->BRR  = (uint32_t)G11_Pin
#define DEBUG_G12_SET()    G12_GPIO_Port->BSRR = (uint32_t)G12_Pin
#define DEBUG_G12_RESET()  G12_GPIO_Port->BRR  = (uint32_t)G12_Pin


void DebugInit(void);
void Debug_SWOInit(uint32_t portMask, uint32_t cpuCoreFreqHz, uint32_t baudrate);

#endif /* INC_DEBUG_H_ */
