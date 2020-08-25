
//#include <GenericTypeDefs.h>
#include "usbser.h"
//#include "utility.h"
//#include "usb_config.h"
//#include "Usb\usb.h"
//#include "Usb\usb_function_cdc.h"
//#include "parse.h"

// ROM strings
const char st_OK[] = {"OK\r\n"};
const char st_LFCR[] = {"\r\n"};

// USB Transmit buffer for packets (back to PC)
static unsigned char g_TX_buf[kTX_BUF_SIZE];

// Buffer for data received in one chunk from USB stack (from PC))
static unsigned char g_RX_command_buf[kRX_COMMAND_BUF_SIZE];

// As transferred from g_RX_command_buf[], each command is put here
// and then parsed when end of line is reached
unsigned char g_RX_buf[kRX_BUF_SIZE];

// Pointers to USB transmit (back to PC) buffer
unsigned char g_TX_buf_in;
unsigned char g_TX_buf_out;

// Pointers to USB receive (from PC) buffer
unsigned char g_RX_buf_in;
unsigned char g_RX_buf_out;

void usbser_Init(void)
{
  UINT16 i;
  
  // Initialize USB TX and RX buffer management
  g_RX_buf_in = 0;
  g_RX_buf_out = 0;
  g_TX_buf_in = 0;
  g_TX_buf_out = 0;

  for (i=0; i < kTX_BUF_SIZE; i++)
  {
    g_TX_buf[i] = 0;
  }
  for (i=0; i < kRX_COMMAND_BUF_SIZE; i++)
  {
    g_RX_command_buf[i] = 0;
  }
  for (i=0; i < kRX_BUF_SIZE; i++)
  {
    g_RX_buf[i] = 0;
  }
}

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
 * Overview:        In this function, we check for a new packet that just
 *                  arrived via USB. We do a few checks on the packet to see
 *                  if it is worthy of us trying to interpret it. If it is,
 *                  we go and call the proper function based upon the first
 *                  character of the packet.
 *                  NOTE: We need to see everything in one packet (i.e. we
 *                  won't treat the USB data as a stream and try to find our
 *                  start and end of packets within the stream. We just look 
 *                  at the first character of each packet for a command and
 *                  check that there's a CR as the last character of the
 *                  packet.
 *
 * Note:            None
 *****************************************************************************/
void ProcessIO(void)
{
  static BOOL in_esc = FALSE;
  static char esc_sequence[3] = {0};
  static BOOL in_cr = FALSE;
  unsigned char tst_char;
  static unsigned char button_state = 0;
  static unsigned int button_ctr = 0;
  char i;
  BOOL done = FALSE;
  unsigned char rx_bytes = 0;
  unsigned char byte_cnt = 0;

  BlinkUSBStatus();

  // Bail from here if we're not 'plugged in' to a PC or we're suspended
  if(
    (USBDeviceState < CONFIGURED_STATE)
    ||
    (USBSuspendControl==1)
  ) 
  {
    return;
  }

  // Pull in some new data if there is new data to pull in
  // And we aren't waiting for the current move to finish
  rx_bytes = getsUSBUSART((char *)g_RX_command_buf, kRX_COMMAND_BUF_SIZE);
  if (rx_bytes > 0)
  {
    for(byte_cnt = 0; byte_cnt < rx_bytes; byte_cnt++)
    {
      tst_char = g_RX_command_buf[byte_cnt];

      // Check to see if we are in a CR/LF situation
      if (
        !in_cr 
        && 
        (
          kCR == tst_char
          ||
          kLF == tst_char
        )
      )
      {
        in_cr = TRUE;
        g_RX_buf[g_RX_buf_in] = kCR;
        g_RX_buf_in++;

        // At this point, we know we have a full packet
        // of information from the PC to parse

        // Now, if we've gotten a full command (user send <CR>) then
        // go call the code that deals with that command, and then
        // keep parsing. (This allows multiple small commands per packet)
        parsePacket();
        g_RX_buf_in = 0;
        g_RX_buf_out = 0;
      }
      else if (tst_char == 27 && in_esc == FALSE)
      {
        in_esc = TRUE;
        esc_sequence[0] = 27;
        esc_sequence[1] = 0;
        esc_sequence[2] = 0;
      }
      else if (
        in_esc == TRUE 
        && 
        tst_char == 91 
        && 
        esc_sequence[0] == 27 
        && 
        esc_sequence[1] == 0
      )
      {
        /// TODO: What is this for?
        esc_sequence[1] = 91;
      }
      else if (tst_char == 8 && g_RX_buf_in > 0)
      {
        // Handle the backspace thing
        g_RX_buf_in--;
        g_RX_buf[g_RX_buf_in] = 0x00;
        printf((char *)" \b");
      }
      else if (
        tst_char != kCR
        &&
        tst_char != kLF
        &&
        tst_char >= 32
      )
      {
        esc_sequence[0] = 0;
        esc_sequence[1] = 0;
        esc_sequence[2] = 0;
        in_esc = FALSE;

        // Only add a byte if it is not a CR or LF
        g_RX_buf[g_RX_buf_in] = tst_char;
        in_cr = FALSE;
        g_RX_buf_in++;
      }
      // Check for buffer wraparound
      if (kRX_BUF_SIZE == g_RX_buf_in)
      {
        bitset (error_byte, kERROR_BYTE_RX_BUFFER_OVERRUN);
        g_RX_buf_in = 0;
        g_RX_buf_out = 0;
      }
    }
  }

  // Check for any errors logged in error_byte that need to be sent out
  if (error_byte)
  {
    if (bittst (error_byte, 0))
    {
      // Unused as of yet
      printf ((char *)"!0 \r\n");
    }
    if (bittst (error_byte, kERROR_BYTE_STEPS_TO_FAST))
    {
      // Unused as of yet
      printf ((char *)"!1 Err: Can't step that fast\r\n");
    }
    if (bittst (error_byte, kERROR_BYTE_TX_BUF_OVERRUN))
    {
      printf ((char *)"!2 Err: TX Buffer overrun\r\n");
    }
    if (bittst (error_byte, kERROR_BYTE_RX_BUFFER_OVERRUN))
    {
      printf ((char *)"!3 Err: RX Buffer overrun\r\n");
    }
    if (bittst (error_byte, kERROR_BYTE_MISSING_PARAMETER))
    {
      printf ((char *)"!4 Err: Missing parameter(s)\r\n");
    }
    if (bittst (error_byte, kERROR_BYTE_PRINTED_ERROR))
    {
      // We don't need to do anything since something has already been printed out
      //printf ((rom char *)"!5\r\n");
    }
    if (bittst (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT))
    {
      printf ((char *)"!6 Err: Invalid paramter value\r\n");
    }
    if (bittst (error_byte, kERROR_BYTE_EXTRA_CHARACTERS))
    {
      printf ((char *)"!7 Err: Extra parmater\r\n");
    }
    error_byte = 0;
  }

  // Go send any data that needs sending to PC
  check_and_send_TX_data();
}

// This is our replacement for the standard putc routine
// This enables printf() and all related functions to print to
// the USB output (i.e. to the PC) buffer
int _user_putc(char c)
{
  BYTE OldPtr = g_TX_buf_in;

  // Check to see if adding this byte will cause us to be full
  OldPtr++;
  if (kTX_BUF_SIZE == OldPtr)
  {
    OldPtr = 0;
  }
  // If so, then wait until some bytes go away first and make room
  if (OldPtr == g_TX_buf_out)
  {
    check_and_send_TX_data();
  }
  // Copy the character into the output buffer
  g_TX_buf[g_TX_buf_in] = c;
  g_TX_buf_in++;

  // Check for wrap around
  if (kTX_BUF_SIZE == g_TX_buf_in)
  {
    g_TX_buf_in = 0;
  }

  // Also check to see if we bumped up against our output pointer
  if (g_TX_buf_in == g_TX_buf_out)
  {
    bitset (error_byte, kERROR_BYTE_TX_BUF_OVERRUN);
  }
  return (c);
}

// In this function, we check to see if we have anything to transmit. 
// If so then we schedule the data for sending.
void check_and_send_TX_data(void)
{
  char temp;

  // Only send if there's something there to send
  if (g_TX_buf_out != g_TX_buf_in)
  {
    // Yes, Microchip says not to do this. We'll be blocking
    // here until there's room in the USB stack to send
    // something new. But without making our buffers huge,
    // I don't know how else to do it.
    while (!USBUSARTIsTxTrfReady())
    {
      CDCTxService();
#if defined(USB_POLLING)
      USBDeviceTasks();
#endif
    }
    // Now we know that the stack can transmit some new data

    // Now decide if we need to break it up into two parts or not
    if (g_TX_buf_in > g_TX_buf_out)
    {
      // Since IN is beyond OUT, only need one chunk
      temp = g_TX_buf_in - g_TX_buf_out;
      putUSBUSART((char *)&g_TX_buf[g_TX_buf_out], temp);
      // Now that we've scheduled the data for sending,
      // update the pointer
      g_TX_buf_out = g_TX_buf_in;
    }
    else
    {
      // Since IN is before OUT, we need to send from OUT to the end
      // of the buffer, then the next time around we'll catch
      // from the beginning to IN.
      temp = kTX_BUF_SIZE - g_TX_buf_out;
      putUSBUSART((char *)&g_TX_buf[g_TX_buf_out], temp);
      // Now that we've scheduled the data for sending,
      // update the pointer
      g_TX_buf_out = 0;
    }
    CDCTxService();
  }
}

