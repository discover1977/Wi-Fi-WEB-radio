/* This source file is part of the ATMEL QTouch Library 5.0.8 */

/*****************************************************************************
 *
 * \file
 *
 * \brief  This file contains the QDebug public API that can be used to
 * transfer data from a Touch Device to Atmel Studio.
 *
 *
 * - Userguide:          QTouch Library Peripheral Touch Controller User Guide.
 * - Support email:      www.atmel.com/design-support/
 *
 *
 * Copyright (c) 2013-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/

/* == include files ========================================================== */
#include "touch.h"
#if DEF_TOUCH_QDEBUG_ENABLE == 1
#if defined(QDEBUG_SERIAL)
#include <avr/io.h>
#include "Serial.h"
#include "QDebugTransport.h"

/* == #defines ================================================================ */

#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define PORT UART_PORT

/* == types ==================================================================== */

/* == constants ================================================================= */

/* == global constants ========================================================== */

/* == global variables ========================================================== */

/* == file-scope (static) variables ============================================= */

/* == file-scope (static) function prototypes =================================== */

/*! \brief Send  one byte using Uart Interface
 *
 */
static void Serial_Send_Byte(const uint8_t TxData);

/* == file-scope (static) functions defines =================================== */

/*! \brief Send and Read one byte using BitBangSPI Interface
 *
 * \notes  Called from BitBangSPI_Send_Message in this file
 * \param void
 * \return void
 */
static void Serial_Send_Byte(const uint8_t TxData)
{
    /* wait till Data Register Empty flag is set */
    while ((CONCAT(UCSR,PORT,A) & (1u << REG(UDRE,PORT))) == 0);

    /* Transmit the Data */
    REG(UDR,PORT) = TxData;

    /* wait till the Transmit Complete flag is set*/
    while ((CONCAT(UCSR,PORT,A) & (1u << REG(TXC,PORT))) == 0);

    /* clear the Transmit Complete flag */
    CONCAT(UCSR,PORT,A) |= (1 << REG(TXC,PORT));
}

/* == public function(global function) defines =================================== */

/*! \brief Initialize Serial Interface
 *
 * \notes  Called from QDebug_Init in QDebug.c
 * \param void
 * \return void
 */
void Serial_Init (void)
{
    CONCAT(UBRR,PORT,H) = BAUD_PRESCALE >> 8u;

    CONCAT(UBRR,PORT,L) = BAUD_PRESCALE & 0x00FF;

    /* enable RX complete Interrupt, Rx and TX module */
    CONCAT(UCSR,PORT,B) = ((1 << REG(RXCIE,PORT)) | (1 << REG(RXEN,PORT)) | (1 << REG(TXEN,PORT)));

    /* Use 8-bit character sizes */
    CONCAT(UCSR,PORT,C) =  ((1 << CONCAT(UCSZ,PORT,0)) | (1 << CONCAT(UCSZ,PORT,1)));

    /* Enable  Start of Frame Detect */
    CONCAT(UCSR,PORT,D) = (1 <<SFDE);

}

/*! \brief Send and Read one frame using BitBangSPI Interface
 *
 * \notes  Called from Send_Message in QDebugTransport.c
 * \param void
 * \return void
 */
void Serial_Send_Message(void)
{
    /* Send our message upstream */
    for (uint8_t index=0; index<= TX_index; index++)
    {
        Serial_Send_Byte(TX_Buffer[index]);
    }/*for*/
}/*Serial_Send_Message*/

/* == Interrupt =================================== */

/*! \brief ISR routine for Uart Rx
 *
 */
ISR(CONCAT(USART,PORT,_RX_vect))
{
    const volatile uint8_t ReceivedByte = REG(UDR,PORT);

    RxHandler(ReceivedByte);
}

#endif /* _DEBUG_INTERFACE_ */
#endif

