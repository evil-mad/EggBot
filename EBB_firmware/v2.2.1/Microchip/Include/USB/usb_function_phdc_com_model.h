/************************************************************************
  File Information:
    FileName:       usb_function_phdc_com_model.h
    Dependencies:   See INCLUDES section
    Processor:      PIC18, PIC24 or PIC32 USB Microcontrollers
    Hardware:       The code is natively intended to be used on the following
                    hardware platforms: PICDEM™ FS USB Demo Board,
                    PIC18F87J50 FS USB Plug-In Module, or
                    Explorer 16 + PIC24 USB PIM.  The firmware may be
                    modified for use on other USB platforms by editing the
                    HardwareProfile.h file.
    Complier:       Microchip C18 (for PIC18) or C30 (for PIC24)
    Company:        Microchip Technology, Inc.

    Software License Agreement:

    The software supplied herewith by Microchip Technology Incorporated
    (the “Company”) for its PIC® Microcontroller is intended and
    supplied to you, the Company’s customer, for use solely and
    exclusively on Microchip PIC Microcontroller products. The
    software is owned by the Company and/or its supplier, and is
    protected under applicable copyright laws. All rights are reserved.
    Any use in violation of the foregoing restrictions may subject the
    user to criminal sanctions under applicable laws, as well as to
    civil liability for the breach of the terms and conditions of this
    license.

    THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
    WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
    TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
    IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
    CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

  Summary:
    This file contains all of functions, macros, definitions, variables,
    datatypes, etc. that are required for usage with the application software
	.

    This file is located in the "\<Install Directory\>\\Microchip\\Include\\USB"
    directory.

  Description:
    Application file

    This file contains all of functions, macros, definitions, variables,
    datatypes, etc. that are required for usage with the application software.

    This file is located in the "\<Install Directory\>\\Microchip\\Include\\USB"
    directory.



********************************************************************/

/********************************************************************
 Change History:
  Rev    Description
  ----   -----------
  1.0   Initial release
  2.0

********************************************************************/
#ifndef _PHD_COM_H
#define _PHD_COM_H

/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"


/** D E F I N I T I O N S ****************************************************/
#define BYTE_SWAP16(a) (UINT16)((((UINT16)(a)&0xFF00)>>8) | \
                                    (((UINT16)(a)&0x00FF)<<8))

/* callback function pointer structure for Application to handle events */
typedef void(* PHDC_APP_CB)(UINT8);


/* Application States */
#define PHD_DISCONNECTED  0x00
#define PHD_CONNECTING    0x01
#define PHD_CONNECTED     0x02
#define PHD_DISCONNECTING 0x03
#define PHD_MEASUREMENT_SENT 0x04
#define PHD_MEASUREMENT_SENDING 0x05


/* Agent states */
#define  PHD_COM_STATE_DISCONNECTED                  0x00
#define  PHD_COM_STATE_UNASSOCIATED                  0x01
#define  PHD_COM_STATE_ASSOCIATING                   0x02
#define  PHD_COM_STATE_ASSOC_CFG_SENDING_CONFIG      0x03
#define  PHD_COM_STATE_ASSOC_CFG_WAITING_APPROVAL    0x04
#define  PHD_COM_STATE_ASSOC_OPERATING               0x05
#define  PHD_COM_STATE_DISASSOCIATING                0x06

/* requests */
#define PHD_ASSOCIATION_REQUEST     0xE200
#define PHD_ASSOCIATION_RESPONSE    0xE300
#define PHD_RELEASE_REQUEST         0xE400
#define PHD_RELEASE_RESPONSE        0xE500
#define PHD_ABORT_REQUEST           0xE600
#define PHD_PRESET_APDU             0xE700


 /******************************************************************************
 * Function:
 *      void PHDConnect(void)
 *
 * Summary:
 *      This function is used to connect to the PHD Manager.
 *
 * Description:
 *       This function initiates connection to the PHD Manager by sending an
 *   Association request to manager.  The Agent doesn't get connected to
 *	 the Manager immediately after calling this function. Upon receiving
 *	 the association request from an Agent, the PHD Manager responds with
 *	 an association response. The association response tells whether Manager
 *	 accepting the request or rejecting it. The Association response from
 *	 the Manager is handled by the PHD stack. The PHD stack calls a callback
 *	 function (void(* PHDC_APP_CB)(UINT8)) to the application with status of
 *	 the connection.
 *	 The Manager should respond to the Agent within the specified timeout of
 *	 ASSOCIATION_REQUEST_TIMEOUT. The Agent should send the Association request
 *	 once more if no response is received from Manager and ASSOCIATION_REQUEST_TIMEOUT
 *   is expired. This function starts a Timer for the Association Timeout request.
 *   The timeout is handled by the PHDTimeoutHandler() function.
 *
 * Conditions:
 *       The agent should be in PHD_INITIALIZED state.
 *
 * Parameters:
 *	None
 *
 * Return:
 *	None
 *
 * Side Effects:
 *	None
 *
 * Remarks:
 *      None
 *
 *****************************************************************************/
void PHDConnect(void);

/******************************************************************************
 * Function:
 *      void PHDDisConnect(void)
 *
 * Summary:
 *      This function is used to disconnect from the PHD Manager.
 *
 * Description:
 *       This function initiates disconnection of the Agent from the PHD Manager by sending an
 *	 Release request to manager.  The Agent doesn't get disconnected from the Manager
 *	 immediately after calling this function. The PHD Manager sends back a release response
 *	 to the Agent. The Agent responds back with an Abort Message and the Agent moves to DISCONNECTED
 *	 state. The PHD stack calls a callback function (void(* PHDC_APP_CB)(UINT8)) to the application
 *	 with status of the connection. This function disables all timeout.
 *
 * Conditions:
 *    None.
 *
 * Parameters:
 *	None
 *
 * Return:
 *	None
 *
 * Side Effects:
 *	None
 *
 * Remarks:
 *      None
 *
 *****************************************************************************/
void PHDDisConnect(void);


/******************************************************************************
 * Function:
 *      void PHDAppInit(PHDC_APP_CB callback)
 *
 * Summary:
 *      This function is used to initialize the PHD stack.
 *
 * Description:
 *       This function initializes all the application related items.
 *       The input to the function is address of the callback function. This callback function
 *	 which will be called by PHD stack when there is a change in Agent's connection status.
 *
 * Conditions:
 *       None
 *
 * Parameters:
 *	PHDC_APP_CB callback - Pointer to application Call Back Function.
 *
 * Return:
 *	None
 *
 * Side Effects:
 *	None
 *
 * Remarks:
 *      None
 *
 *****************************************************************************/
void PHDAppInit(PHDC_APP_CB);

/******************************************************************************
 * Function:
 *      void PHDTimeoutHandler(void)
 *
 * Summary:
 *      This function is used to handle all timeout.
 *
 * Description:
 *       This function handles all timers. This function should be called once in every milli Second.
 *
 * Conditions:
 *       None
 *
 * Parameters:
 *	None
 *
 * Return:
 *	None
 *
 * Side Effects:
 *	None
 *
 * Remarks:
 *      If USB is used at the Transport layer then the USB SOF handler can call this function.
 *
 *****************************************************************************/
void PHDTimeoutHandler(void);

/******************************************************************************
 * Function:
 *      void PHDSendMeasuredData(void)
 *
 * Summary:
 *      This function is used to send measurement data to the PHD Manager.
 *
 * Description:
 *       This function sends measurement data to manager. Before calling this function
 *       the caller should fill the Application buffer with the data to send. The Agent
 *	 expects a Confirmation from the Manager for the data sent. This confirmation should
 *	 arrive at the Agent within a specified time of CONFIRM_TIMEOUT. The function starts
 *	 a Timer to see if the Confirmation from the Manager arrives within specified time.
 *	 The timeout is handled by the PHDTimeoutHandler() function.
 *
 * Conditions:
 *       Before calling this function the caller should fill the Application buffer with the data to send.
 *
 * Parameters:
 *	None
 *
 * Return:
 *	None
 *
 * Side Effects:
 *	None
 *
 * Remarks:
 *      None
 *
 *****************************************************************************/
void PHDSendMeasuredData(void);

/******************************************************************************
 * Function:
 *      void PHDSendAppBufferPointer(UINT8 * pAppBuffer)
 *
 * Summary:
 *      This function is used to send measurement data to the PHD Manager.
 *
 * Description:
 *       This function passes the application buffer pointer to the PHD stack. The PHD stack
 *       uses this pointer send and receive data through the transport layer.
 *
 * Conditions:
 *
 *
 * Parameters:
 *	UINT8 *pAppBuffer - Pointer to Application Buffer.
 *
 * Return:
 *	None
 *
 * Side Effects:
 *	None
 *
 * Remarks:
 *      None
 *
 *****************************************************************************/
void PHDSendAppBufferPointer(UINT8 * pAppBuffer);

#endif

