/* 
 * File:   usbser.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 3:09 PM
 */

#ifndef USBSER_H
#define	USBSER_H

#define kTX_BUF_SIZE          64        // In bytes
#define kRX_BUF_SIZE          255       // In bytes
#define kRX_COMMAND_BUF_SIZE  10        // In bytes

#define kCR                     0x0D
#define kLF                     0x0A


extern unsigned char g_RX_buf[kRX_BUF_SIZE];
extern unsigned char g_TX_buf_out;
extern unsigned char g_RX_buf_out;
extern unsigned char g_RX_buf_in;
extern const rom char st_OK[];
extern const rom char st_LFCR[];

void usbser_Init(void);
void parse_packet (void);       // Take a full packet and dispatch it to the right function
signed char extract_digit (unsigned long * acc, unsigned char digits); // Pull a character out of the packet
void ProcessIO(void);
int _user_putc (char c);
void check_and_send_TX_data (void);

#endif	/* USBSER_H */

