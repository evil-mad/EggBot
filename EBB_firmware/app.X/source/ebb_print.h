/*
 * File:   ebb_print.h
 * Author: Brian Schmalz (brian@schmalzhaus.com)
 *
 * Created on October 2, 2023, 7:41 PM
 */

#include <p18cxxx.h>

#define ebb_print_char(data) ebb_putc(data)

void ebb_print(far rom char * print_str);
void ebb_print_ram(char * print_str);
void ebb_print_hex(UINT32 data, UINT8 length);
void ebb_print_uint(UINT32 data);
void ebb_print_int(INT32 data);
void ebb_print_48uint(UINT8 data[6]);
void ebb_print_48int(UINT8 data[6]);
