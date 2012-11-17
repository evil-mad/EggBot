#ifndef BB_DEMO_H
#define EBB_DEMO_H

#include "GenericTypeDefs.h"
#include "Compiler.h"

typedef enum {
	COMD_END = 0,
	COMD_SM, 
	COMD_SP
} comd_type;

typedef struct {
	comd_type comd;
	unsigned int duration;
	signed int A1steps;
	signed int A2steps;
} packet_type;

extern packet_type far rom packet_list[];

#endif