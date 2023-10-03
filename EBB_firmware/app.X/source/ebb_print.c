/*
 * File:   ebb_print.c
 * Author: Brian Schmalz (brian@schmalzhaus.com)
 *
 * Created on October 2, 2023, 7:41 PM
 * 
 * The functions in this file are meant to replace (partially) the built-in
 * printf() functionality in MAPLB C18. Even simple printf() calls were using
 * up significant stack space. In an effort to maximize the number of possible
 * FIFO elements, we freed up as much RAM as possible, and put the stack in
 * GPR14, which is not an entire bank since there are some SFRs at the end of
 * that bank. So the stack only goes from 0xE00 to 0xEBF. With printf() taking
 * up lots of that space, and because the new higher-order move instructions 
 * need more stack space to operate (mainly to pass lots of paramters around)
 * we were seeing stack overflows in versions around v3.0.0-a18. This re-written
 * print functionality is an attempt at fixing that problem.
 * 
 * We do not need any floating point support.
 * All of our printing goes to the PC via the USB port.
 * Virtually all of our printing can be broken down into the following types:
 * - Simple strings with no variables
 * - A signed or unsigned byte
 * - A signed or unsigned 16-bit integer
 * - A signed or unsigned 32-bit integer
 * - A byte as hex
 * - A 16-bit integer as hex
 */

#include <p18cxxx.h>
#include "UBW.h"
#include "ebb_print.h"


/* Print a simple string */
void ebb_print(far rom char * print_str)
{
  while (*print_str != 0x00)
  {
    ebb_putc((char)*print_str);
    print_str++;
  }
}

void ebb_print_ram(char * print_str)
{
  while (*print_str != 0x00)
  {
    ebb_putc(*print_str);
    print_str++;
  }
}

// Print out <data> as a hex value, zero-padded to <length> digits
void ebb_print_hex(UINT32 data, UINT8 length)
{
  
}

// Print out <data> as an unsigned integer
void ebb_print_uint(UINT32 data)
{
  
}

// Print out <data> as a signed integer
void ebb_print_int(INT32 data)
{
  
}

