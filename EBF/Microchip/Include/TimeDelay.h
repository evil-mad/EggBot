/******************************************************************************

File Name:       TimeDelay.c
Dependencies:    None
Processor:       PIC18/PIC24/dsPIC30/dsPIC33/PIC32
Compiler:        C30 v3.12
Company:         Microchip Technology, Inc.

Copyright ? 2009 Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute 
Software only when embedded on a Microchip microcontroller or digital signal 
controller that is integrated into your product or third party product 
(pursuant to the sublicense terms in the accompanying license agreement).  

You should refer to the license agreement accompanying this Software for 
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND, 
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF 
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. 
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER 
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR 
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES 
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR 
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF 
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES 
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Author          Date    Comments
--------------------------------------------------------------------------------
AKN	2009.10.14	FILE CREATED
AKN	2009.10.15	CHANGED C18 DELAY ROUTINE TO DECREMENT ENTIRE NUMBER OF CYCLES
*******************************************************************************/

#include "GenericTypedefs.h"

/****************************************************************************
  Function:
    void Delay10us( UINT32 tenMicroSecondCounter )

  Description:
    This routine performs a software delay in intervals of 10 microseconds.

  Precondition:
    None

  Parameters:
    UINT32 tenMicroSecondCounter - number of ten microsecond delays
    to perform at once.

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void Delay10us( UINT32 tenMicroSecondCounter );

/****************************************************************************
  Function:
    void DelayMs( UINT16 ms )

  Description:
    This routine performs a software delay in intervals of 1 millisecond.

  Precondition:
    None

  Parameters:
    UINT16 ms - number of one millisecond delays to perform at once.

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void DelayMs( UINT16 ms );
