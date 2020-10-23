#include "flash.h"
void EBBWriteBytesFlash(unsigned long startaddr, unsigned int num_bytes, unsigned char *flash_array)
{
unsigned char write_byte=0,flag=0;
DWORD_VAL flash_addr;
		
		flash_addr.Val = startaddr;

		startaddr /= FLASH_WRITE_BLOCK ;	// Align the starting address block
		startaddr *= FLASH_WRITE_BLOCK ;
		startaddr += FLASH_WRITE_BLOCK ;
		
		write_byte = startaddr - flash_addr.Val;
		
		while(num_bytes)
		{
				TBLPTRU = flash_addr.byte.UB;						//Load the address to Address pointer registers
				TBLPTRH = flash_addr.byte.HB;	
				TBLPTRL	= flash_addr.byte.LB;
				
				
				while(write_byte--)
				{
					TABLAT = *flash_array++;
					_asm  TBLWTPOSTINC 	_endasm
					if(--num_bytes==0)	break;
				}

				TBLPTRU = flash_addr.byte.UB;						//Load the address to Address pointer registers
				TBLPTRH = flash_addr.byte.HB;	
				TBLPTRL	= flash_addr.byte.LB;
			  //*********** Flash write sequence ***********************************
			  EECON1bits.WREN = 1;
			  if(INTCONbits.GIE)
			  {
				INTCONbits.GIE = 0;
				flag=1;
			  }		  
			  EECON2 = 0x55;
			  EECON2 = 0xAA;
			  EECON1bits.WR =1;
			  EECON1bits.WREN = 0 ; 
			  if(flag)
			  {
				INTCONbits.GIE = 1;	
				flag=0;
			  }
			 write_byte = FLASH_WRITE_BLOCK;
			 flash_addr.Val = flash_addr.Val + FLASH_WRITE_BLOCK;									//increment to one block of 64 bytes
		}
		
}

