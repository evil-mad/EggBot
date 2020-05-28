/* 
 * File:   analog.h
 * Author: Brian Schmalz
 *
 * Created on May 25, 2020, 8:50 PM
 */

#ifndef ANALOG_H
#define	ANALOG_H

#define ANALOG_INITATE_MS_BETWEEN_STARTS 5      // Number of ms between analog converts (all enabled channels)

extern unsigned char A_cur_channel;
extern unsigned char AnalogInitiate;
extern volatile unsigned int AnalogEnabledChannels;
extern volatile unsigned int ChannelBit;


/** P U B L I C  P R O T O T Y P E S *****************************************/
void AnalogConfigure (unsigned char Channel, unsigned char Enable);

#endif	/* ANALOG_H */

