/*********************************************************************
 *
 *   HID Device Bootloader Firmware for PIC18F87J50 Family Devices
 *
 *********************************************************************
 * FileName:        Boot87J50Family.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 3.12+
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Fritz Schlunder		04/12/08	Original.
 * Fritz Schlunder		02/17/09	Slight modifications for 
 *									MCHPFSUSB v2.4 release. 
 ********************************************************************/

/** C O N S T A N T S **********************************************************/

//Section defining the address range to erase for the erase device command, along with the valid programming range to be reported by the QUERY_DEVICE command.
#define StartPageToErase				4		 //The 1024 byte page starting at address 0x1000 will be erased.
#define ProgramMemStart					0x001000 //Beginning of application program memory (not occupied by bootloader).  **THIS VALUE MUST BE ALIGNED WITH 64 BYTE BLOCK BOUNDRY** Also, in order to work correctly, make sure the StartPageToErase is set to erase this section.
#define ConfigWordsSectionLength		0x08	//8 bytes worth of Configuration words on the PIC18F87J50 family devices

#if defined(__18F87J50)||defined(__18F67J50)
	#define MaxPageToEraseNoConfigs		126		 //Last page of flash on the PIC18F87J50, which does not contain the flash configuration words.
	#define MaxPageToEraseWithConfigs	127		 //Page 127 contains the flash configurations words on the PIC18F87J50.
	#define ProgramMemStopNoConfigs		0x01FC00 //**MUST BE WORD ALIGNED (EVEN) ADDRESS.  This address does not get updated, but the one just below it does: IE: If AddressToStopPopulating = 0x200, 0x1FF is the last programmed address (0x200 not programmed)**	
	#define ProgramMemStopWithConfigs	0x01FFF8 //**MUST BE WORD ALIGNED (EVEN) ADDRESS.  This address does not get updated, but the one just below it does: IE: If AddressToStopPopulating = 0x200, 0x1FF is the last programmed address (0x200 not programmed)**	
	#define ConfigWordsStartAddress		0x01FFF8 //0xXXXF8 is CONFIG1L on PIC18F87J50 family devices
#elif defined(__18F86J55)||defined(__18F66J55)
	#define MaxPageToEraseNoConfigs		94		 //Last page of flash on the PIC18F87J50, which does not contain the flash configuration words.
	#define MaxPageToEraseWithConfigs	95		 //Page 127 contains the flash configurations words on the PIC18F87J50.
	#define ProgramMemStopNoConfigs		0x017C00 //**MUST BE WORD ALIGNED (EVEN) ADDRESS.  This address does not get updated, but the one just below it does: IE: If AddressToStopPopulating = 0x200, 0x1FF is the last programmed address (0x200 not programmed)**	
	#define ProgramMemStopWithConfigs	0x017FF8 //**MUST BE WORD ALIGNED (EVEN) ADDRESS.  This address does not get updated, but the one just below it does: IE: If AddressToStopPopulating = 0x200, 0x1FF is the last programmed address (0x200 not programmed)**	
	#define ConfigWordsStartAddress		0x017FF8 //0xXXXF8 is CONFIG1L on PIC18F87J50 family devices
#elif defined(__18F86J50)||defined(__18F66J50)
	#define MaxPageToEraseNoConfigs		62		 //Last page of flash on the PIC18F87J50, which does not contain the flash configuration words.
	#define MaxPageToEraseWithConfigs	63		 //Page 127 contains the flash configurations words on the PIC18F87J50.
	#define ProgramMemStopNoConfigs		0x00FC00 //**MUST BE WORD ALIGNED (EVEN) ADDRESS.  This address does not get updated, but the one just below it does: IE: If AddressToStopPopulating = 0x200, 0x1FF is the last programmed address (0x200 not programmed)**	
	#define ProgramMemStopWithConfigs	0x00FFF8 //**MUST BE WORD ALIGNED (EVEN) ADDRESS.  This address does not get updated, but the one just below it does: IE: If AddressToStopPopulating = 0x200, 0x1FF is the last programmed address (0x200 not programmed)**	
	#define ConfigWordsStartAddress		0x00FFF8 //0xXXXF8 is CONFIG1L on PIC18F87J50 family devices
#elif defined(__18F85J50)||defined(__18F65J50)
	#define MaxPageToEraseNoConfigs		30		 //Last page of flash on the PIC18F87J50, which does not contain the flash configuration words.
	#define MaxPageToEraseWithConfigs	31		 //Page 127 contains the flash configurations words on the PIC18F87J50.
	#define ProgramMemStopNoConfigs		0x007C00 //**MUST BE WORD ALIGNED (EVEN) ADDRESS.  This address does not get updated, but the one just below it does: IE: If AddressToStopPopulating = 0x200, 0x1FF is the last programmed address (0x200 not programmed)**	
	#define ProgramMemStopWithConfigs	0x007FF8 //**MUST BE WORD ALIGNED (EVEN) ADDRESS.  This address does not get updated, but the one just below it does: IE: If AddressToStopPopulating = 0x200, 0x1FF is the last programmed address (0x200 not programmed)**	
	#define ConfigWordsStartAddress		0x007FF8 //0xXXXF8 is CONFIG1L on PIC18F87J50 family devices
#endif

//Switch State Variable Choices
#define	QUERY_DEVICE				0x02	//Command that the host uses to learn about the device (what regions can be programmed, and what type of memory is the region)
#define	UNLOCK_CONFIG				0x03	//Note, this command is used for both locking and unlocking the config bits (see the "//Unlock Configs Command Definitions" below)
#define ERASE_DEVICE				0x04	//Host sends this command to start an erase operation.  Firmware controls which pages should be erased.
#define PROGRAM_DEVICE				0x05	//If host is going to send a full RequestDataBlockSize to be programmed, it uses this command.
#define	PROGRAM_COMPLETE			0x06	//If host send less than a RequestDataBlockSize to be programmed, or if it wished to program whatever was left in the buffer, it uses this command.
#define GET_DATA					0x07	//The host sends this command in order to read out memory from the device.  Used during verify (and read/export hex operations)
#define	RESET_DEVICE				0x08	//Resets the microcontroller, so it can update the config bits (if they were programmed, and so as to leave the bootloader (and potentially go back into the main application)

//Unlock Configs Command Definitions
#define UNLOCKCONFIG				0x00	//Sub-command for the ERASE_DEVICE command
#define LOCKCONFIG					0x01	//Sub-command for the ERASE_DEVICE command

//Query Device Response "Types" 
#define	TypeProgramMemory			0x01	//When the host sends a QUERY_DEVICE command, need to respond by populating a list of valid memory regions that exist in the device (and should be programmed)
#define TypeEEPROM					0x02
#define TypeConfigWords				0x03
#define	TypeEndOfTypeList			0xFF	//Sort of serves as a "null terminator" like number, which denotes the end of the memory region list has been reached.


//BootState Variable States
#define	Idle						0x00
#define NotIdle						0x01

//OtherConstants
#define InvalidAddress				0xFFFFFFFF

//Application and Microcontroller constants
#define BytesPerAddressPIC18		0x01		//One byte per address.  PIC24 uses 2 bytes for each address in the hex file.

#define	TotalPacketSize				0x40
#define WORDSIZE					0x02	//PIC18 uses 2 byte words, PIC24 uses 3 byte words.
#define	FlashBlockSize				0x40	//For PIC18F87J50 family devices, a flash block is 64 bytes
#define RequestDataBlockSize 		0x3A	//Number of data bytes in a standard request to the PC.  Must be an even number from 2-58 (0x02-0x3A).  Larger numbers make better use of USB bandwidth and 
											//yeild shorter program/verify times, but require more micrcontroller RAM for buffer space.
#define BufferSize 					0x40

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "typedefs.h"
#include "usb.h"
#include "io_cfg.h"             // I/O pin mapping


typedef union 
{
		unsigned char Contents[64];
		
		struct{
			unsigned char Command;
			unsigned long Address;
			unsigned char Size;
//			unsigned char PadBytes[58-RequestDataBlockSize];	//Uncomment this if using a smaller than 0x3A RequestDataBlockSize.  Compiler doesn't like 0 byte array when using 58 byte data block size.
			unsigned char Data[RequestDataBlockSize];
		};
		
		struct{
			unsigned char Command;
			unsigned char PacketDataFieldSize;
			unsigned char BytesPerAddress;
			unsigned char Type1;
			unsigned long Address1;
			unsigned long Length1;
			unsigned char Type2;
			unsigned long Address2;
			unsigned long Length2;
			unsigned char EndOfTypes;
			unsigned char ExtraPadBytes[42];
		};
		
		struct{						//For lock/unlock config command
			unsigned char Command;
			unsigned char LockValue;
		};
} PacketToFromPC;		
	

/** V A R I A B L E S ********************************************************/
#pragma udata 
unsigned char MaxPageToErase;
unsigned short long ProgramMemStopAddress;
PacketToFromPC PacketFromPC;
PacketToFromPC PacketToPC;
unsigned char BootState;
unsigned char ErasePageTracker;
unsigned char ProgrammingBuffer[BufferSize];
unsigned char BufferedDataIndex;
unsigned short long ProgrammedPointer;


/***************** P R O T O T Y P E S ***************************************/
void BlinkUSBStatus(void);
void UserInit(void);
void EraseFlash(void);
void WriteFlashSubBlock(void);
void LongDelay(void);


/** D E C L A R A T I O N S **************************************************/
#pragma code
void UserInit(void)
{
	//Initialize bootloader state variables
	MaxPageToErase = MaxPageToEraseNoConfigs;		//Assume we will not allow erase/programming of config words (unless host sends override command)
	ProgramMemStopAddress = ProgramMemStopNoConfigs;
	BootState = Idle;
	ProgrammedPointer = InvalidAddress;	
	BufferedDataIndex = 0;
}//end UserInit


/******************************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user routines.
 *                  It is a mixture of both USB and non-USB tasks.
 *
 * Note:            None
 *****************************************************************************/
void ProcessIO(void)
{
	unsigned char i;

	if(BootState == Idle)
	{
		if(!mHIDRxIsBusy())	//Did we receive a command?
		{
			HIDRxReport((char *)&PacketFromPC, 64);
			BootState = NotIdle;
			
			for(i = 0; i < TotalPacketSize; i++)		//Prepare the next packet we will send to the host, by initializing the entire packet to 0x00.
				PacketToPC.Contents[i] = 0;				//This saves code space, since we don't have to do it independently in the QUERY_DEVICE and GET_DATA cases.
		}
	}
	else //(BootState must be NotIdle)
	{	
		switch(PacketFromPC.Command)
		{
			case QUERY_DEVICE:
			{
				//Prepare a response packet, which lets the PC software know about the memory ranges of this device.
				PacketToPC.Command = QUERY_DEVICE;
				PacketToPC.PacketDataFieldSize = RequestDataBlockSize;
				PacketToPC.BytesPerAddress = BytesPerAddressPIC18;
				PacketToPC.Type1 = TypeProgramMemory;
				PacketToPC.Address1 = (unsigned long)ProgramMemStart;
				PacketToPC.Length1 = (unsigned long)(ProgramMemStopAddress - ProgramMemStart);	//Size of program memory area
				PacketToPC.Type2 = TypeConfigWords;
				PacketToPC.Address2 = (unsigned long)ConfigWordsStartAddress;
				PacketToPC.Length2 = (unsigned long)ConfigWordsSectionLength;
				PacketToPC.EndOfTypes = TypeEndOfTypeList;
				//Init pad bytes to 0x00...  Already done after we received the QUERY_DEVICE command (just after calling HIDRxReport()).
	
				if(!mHIDTxIsBusy())
				{
					HIDTxReport((char *)&PacketToPC, 64);
					BootState = Idle;
				}
			}
				break;
			case UNLOCK_CONFIG:
			{
				if(PacketFromPC.LockValue == UNLOCKCONFIG)
				{
					MaxPageToErase = MaxPageToEraseWithConfigs;		//Assume we will not allow erase/programming of config words (unless host sends override command)
					ProgramMemStopAddress = ProgramMemStopWithConfigs;
				}
				else	//LockValue must be == LOCKCONFIG
				{
					MaxPageToErase = MaxPageToEraseNoConfigs;		
					ProgramMemStopAddress = ProgramMemStopNoConfigs;
				}
				BootState = Idle;
			}
				break;
			case ERASE_DEVICE:
			{
				for(ErasePageTracker = StartPageToErase; ErasePageTracker < (MaxPageToErase + 1); ErasePageTracker++)
				{
					ClrWdt();
					EraseFlash();
					USBDriverService(); 	//Call USBDriverService() periodically to prevent falling off the bus if any SETUP packets should happen to arrive.
				}
				BootState = Idle;				
			}
				break;
			case PROGRAM_DEVICE:
			{
				if(ProgrammedPointer == (unsigned short long)InvalidAddress)
					ProgrammedPointer = PacketFromPC.Address;
				
				if(ProgrammedPointer == (unsigned short long)PacketFromPC.Address)
				{
					for(i = 0; i < PacketFromPC.Size; i++)
					{
						ProgrammingBuffer[BufferedDataIndex] = PacketFromPC.Data[i+(RequestDataBlockSize-PacketFromPC.Size)];	//Data field is right justified.  Need to put it in the buffer left justified.
						BufferedDataIndex++;
						ProgrammedPointer++;
						if(BufferedDataIndex == RequestDataBlockSize)
						{
							WriteFlashSubBlock();
						}
					}
				}
				//else host sent us a non-contiguous packet address...  to make this firmware simpler, host should not do this without sending a PROGRAM_COMPLETE command in between program sections.
				BootState = Idle;
			}
				break;
			case PROGRAM_COMPLETE:
			{
				WriteFlashSubBlock();
				ProgrammedPointer = InvalidAddress;		//Reinitialize pointer to an invalid range, so we know the next PROGRAM_DEVICE will be the start address of a contiguous section.
				BootState = Idle;
			}
				break;
			case GET_DATA:
			{
				//Init pad bytes to 0x00...  Already done after we received the QUERY_DEVICE command (just after calling HIDRxReport()).
				PacketToPC.Command = GET_DATA;
				PacketToPC.Address = PacketFromPC.Address;
				PacketToPC.Size = PacketFromPC.Size;

				TBLPTR = (unsigned short long)PacketFromPC.Address;
				for(i = 0; i < PacketFromPC.Size; i++)
				{
					_asm
					tblrdpostinc
					_endasm
					PacketToPC.Data[i+((TotalPacketSize - 6) - PacketFromPC.Size)] = TABLAT;					
				}

				if(!mHIDTxIsBusy())
				{
					HIDTxReport((char *)&PacketToPC, 64);
					BootState = Idle;
				}
				
			}
				break;
			case RESET_DEVICE:
			{
				UCONbits.SUSPND = 0;		//Disable USB module
				UCON = 0x00;				//Disable USB module
//				//And wait awhile for the USB cable capacitance to discharge down to disconnected (SE0) state. 
//				//Otherwise host might not realize we disconnected/reconnected when we do the reset.
				LongDelay();
				Reset();
			}
				break;
		}//End switch
	}//End if/else

}//End ProcessIO()



void EraseFlash(void)
{
//Really want this: TBLPTR = ((unsigned short long)ErasePageTracker << 10); but compiler not very efficient at this, so instead do this:
	TBLPTRL = 0x00;
	TBLPTRH = ErasePageTracker;
	TBLPTRU = 0x00;
	_asm
	bcf		STATUS, 0, 0
	rlcf	TBLPTRH, 1, 0
	rlcf	TBLPTRU, 1, 0
	rlcf	TBLPTRH, 1, 0
	rlcf	TBLPTRU, 1, 0
	_endasm 

	EECON1 = 0b00010100;

	INTCONbits.GIE = 0;	//Make certain interrupts disabled for unlock process.
	_asm
	MOVLW 0x55
	MOVWF EECON2, 0
	MOVLW 0xAA
	MOVWF EECON2, 0
	BSF EECON1, 1, 0
	_endasm

	EECON1bits.WREN = 0;  //Good practice now to clear the WREN bit, as further protection against any future accidental activation of self write/erase operations.
	ClrWdt();
}	


void WriteFlashSubBlock(void)		//Use word writes to write code chunks less than a full 64 byte block size.
{
	unsigned char i = 0;

	while(BufferedDataIndex > 0)		//While data is still in the buffer.
	{
		TBLPTR = (ProgrammedPointer - BufferedDataIndex);
		//Below section will need to be modified if the WORDSIZE of your processor is not 2 bytes.
		TABLAT = ProgrammingBuffer[i];
		_asm
		tblwtpostinc
		_endasm
		i++;		
		TABLAT = ProgrammingBuffer[i];
		_asm
		tblwt					//Do not increment TBLPTR on the second write.  See datasheet.
		_endasm
		i++;
		
		EECON1 = 0b00100100;	//Word programming mode
		INTCONbits.GIE = 0;		//Make certain interrupts disabled for unlock process.
		_asm
		MOVLW 0x55
		MOVWF EECON2, 0
		MOVLW 0xAA
		MOVWF EECON2, 0
		BSF EECON1, 1, 0		//Initiates write operation (halts CPU execution until complete)
		_endasm		

		BufferedDataIndex = BufferedDataIndex - WORDSIZE;	//Used up one word from the buffer.
		ClrWdt();
	}
	EECON1bits.WREN = 0;  //Good practice now to clear the WREN bit, as further protection against any accidental activation of self write/erase operations.
}



void LongDelay(void)
{
	unsigned char i;
	//A basic for() loop decrementing a 16 bit number would be simpler, but seems to take more code space for
	//a given delay.  So do this instead:	
	for(i = 0; i < 0xFF; i++)
	{
		WREG = 0xFF;
		while(WREG)
		{
			WREG--;
			_asm
			bra	0	//Equivalent to bra $+2, which takes half as much code as 2 nop instructions
			bra	0	//Equivalent to bra $+2, which takes half as much code as 2 nop instructions
			clrwdt
			nop
			_endasm	
		}
	}
	//Delay is ~59.8ms at 48MHz.	
}	


/** EOF Boot87J50Family.c *********************************************************/
