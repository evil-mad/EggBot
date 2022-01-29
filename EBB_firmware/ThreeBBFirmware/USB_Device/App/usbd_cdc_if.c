/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v3.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <stdbool.h>
#include "main.h"
#include "parse.h"
#include "utility.h"
#include "retarget.h"
#include "debug.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// ROM strings
const char st_OK[] = {"OK\n"};
const char st_LFCR[] = {"\n"};

// As each buffer is processed from the USB stack, each command is put here
// and then parsed when end of line is reached
uint8_t g_RX_buf[kRX_BUF_SIZE] = {0};

// Pointers to USB receive (from PC) buffer
uint8_t g_RX_buf_in = 0;
uint8_t g_RX_buf_out = 0;

bool RXDataAvailable = false;
uint8_t * RXDataBuf;
uint32_t * RXDataLen;

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  uint16_t i;

  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

  // Initialize USB TX and RX buffer management
  g_RX_buf_in = 0;
  g_RX_buf_out = 0;

  for (i=0; i < kRX_BUF_SIZE; i++)
  {
    g_RX_buf[i] = 0;
  }

  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  static bool LEDOn = false;

  DEBUG_G0_SET();

  RXDataAvailable = true;
  RXDataBuf = Buf;
  RXDataLen = Len;

  if (LEDOn)
  {
    USR_LED_GPIO_Port->BSRR =  (uint32_t)USR_LED_Pin;
    LEDOn = false;
  }
  else
  {
    USR_LED_GPIO_Port->BRR =  (uint32_t)USR_LED_Pin;
    LEDOn = true;
  }

  DEBUG_G0_RESET();

  return USBD_OK;
}

static void processRXData(void){
  /* USER CODE BEGIN 6 */
  static bool in_cr = false;
  static bool in_esc = false;
  static uint8_t esc_sequence[3] = {0};
  uint8_t byte_cnt = 0;
  uint8_t tst_char = 0;
  uint16_t dataLength = *RXDataLen;

  DEBUG_G2_SET();

  if (dataLength > 0)
  {
    for(byte_cnt = 0; byte_cnt < dataLength; byte_cnt++)
    {
      tst_char = RXDataBuf[byte_cnt];

      SWOprintf("%c", tst_char);

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
        in_cr = true;
        g_RX_buf[g_RX_buf_in] = kCR;
        g_RX_buf_in++;

        // At this point, we know we have a full packet
        // of information from the PC to parse

        // Now, if we've gotten a full command (user send <CR>) then
        // go call the code that deals with that command, and then
        // keep parsing. (This allows multiple small commands per packet)
        DEBUG_G1_SET();
        parsePacket();
        DEBUG_G1_RESET();

        g_RX_buf_in = 0;
        g_RX_buf_out = 0;

        /// TODO: Make this into a function
        // Check for any errors logged in error_byte that need to be sent out
        if (error_byte)
        {
          if (bittst (error_byte, 0))
          {
            // Unused as of yet
            printf ((char *)"!0 \n");
          }
          if (bittst (error_byte, kERROR_BYTE_STEPS_TO_FAST))
          {
            // Unused as of yet
            printf ((char *)"!1 Err: Can't step that fast\n");
          }
          if (bittst (error_byte, kERROR_BYTE_TX_BUF_OVERRUN))
          {
            printf ((char *)"!2 Err: TX Buffer overrun\n");
          }
          if (bittst (error_byte, kERROR_BYTE_RX_BUFFER_OVERRUN))
          {
            printf ((char *)"!3 Err: RX Buffer overrun\n");
          }
          if (bittst (error_byte, kERROR_BYTE_MISSING_PARAMETER))
          {
            printf ((char *)"!4 Err: Missing parameter(s)\n");
          }
          if (bittst (error_byte, kERROR_BYTE_PRINTED_ERROR))
          {
            // We don't need to do anything since something has already been printed out
            //printf ((rom char *)"!5\n");
          }
          if (bittst (error_byte, kERROR_BYTE_PARAMETER_OUTSIDE_LIMIT))
          {
            printf ((char *)"!6 Err: Invalid parameter value\n");
          }
          if (bittst (error_byte, kERROR_BYTE_EXTRA_CHARACTERS))
          {
            printf ((char *)"!7 Err: Extra parameter\n");
          }
          error_byte = 0;
        }
      }
      else if (tst_char == 27 && in_esc == false)
      {
        in_esc = true;
        esc_sequence[0] = 27;
        esc_sequence[1] = 0;
        esc_sequence[2] = 0;
      }
      else if (
        in_esc == true
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
        CDC_Transmit_FS((uint8_t*)" \b", 2);
      }
      else if (
        tst_char != kCR
        &&
        tst_char != kLF
        &&
        tst_char >= 32
        &&
        tst_char <= 127
      )
      {
        esc_sequence[0] = 0;
        esc_sequence[1] = 0;
        esc_sequence[2] = 0;
        in_esc = false;

        // Only add a byte if it is not a CR or LF
        g_RX_buf[g_RX_buf_in] = tst_char;
        in_cr = false;
        g_RX_buf_in++;
      }
      else
      {
        /// TODO: Signal an error here - that the data the PC sent is not in the range 32 to 127
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

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &RXDataBuf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  DEBUG_G2_RESET();

//  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  while (hcdc->TxState != 0)
  {
    // Sit and spin here until the previous USB transmission has finished. This is inefficient.
    /// TODO: It would be better to have a separate function in main() that looked at an output (TX)
    /// USB buffer and sent it out whenever it got full enough or enough time has passed rather than
    /// sitting and waiting for each command's response to be sent before sending the next one.

    /// TODO: Should this have some sort of timeout that expires after some amount of time has passed?
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_IsTxBusy(void)
{
  uint8_t result = USBD_OK;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0)
  {
    return USBD_BUSY;
  }
  return result;
}


void USB_Run(void)
{
  if (RXDataAvailable)
  {
    processRXData();
    RXDataAvailable = false;
  }
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
