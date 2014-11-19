/**
 * \file template.c
 *
 * \brief Empty application template
 *
 * Copyright (C) 2012-2014, Atmel Corporation. All rights reserved.
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
 * Modification and other use of this code is subject to Atmel's Limited
 * License Agreement (license.txt).
 *
 * $Id: template.c 9267 2014-03-18 21:46:19Z ataradov $
 *
 */

/*- Includes ---------------------------------------------------------------*/
#include <stdbool.h>

#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "halUart.h"
#include "halLed.h"

/*- Definitions ------------------------------------------------------------*/
// Put your preprocessor definitions here
/*- Types ------------------------------------------------------------------*/
// Put your type definitions here
/*- Prototypes -------------------------------------------------------------*/
// Put your function prototypes here
/*- Variables --------------------------------------------------------------*/
char bytes_received[64];
int array_index = 0;
int message_length = 0;
volatile bool have_message = false;

NWK_DataReq_t nwkDataReq;
typedef void (*appDataConf_ptr_t)(NWK_DataReq_t *req);

enum states {
	AWAITING_DATA, RECEIVED_DATA, AWAITING_CONF
};
static uint8_t state = AWAITING_DATA;

/*- Implementations --------------------------------------------------------*/

// Put your function implementations here
void not_ready_to_receive()
{
	// set pin PB7 high when ready to receive data
	PORTB &= ~_BV(PB7);
}

void ready_to_receive()
{
	// set pin PB7 low when ready to receive data
	PORTB |= _BV(PB7);
}

void HAL_UartBytesReceived(uint16_t bytes)
{
//	for (int i=0;i<bytes;i++)
//			HAL_UartReadByte();
	//HAL_UartWriteByte('H');
	for (; array_index < bytes; array_index++)
	{
		//HAL_UartWriteByte('F');
		bytes_received[array_index] = HAL_UartReadByte();

		// get the length
		if (array_index == 0)
		{
			message_length = bytes_received[array_index];
		} else if (array_index == message_length) // the end
		{
			//raise rts high
			not_ready_to_receive();
			//and set a flag that we have a message
			state = RECEIVED_DATA;
		}
	}
	//state = RECEIVED_DATA;
	// after getting all bites set (not ready to send)
	// Do nothing, but make the compiler happy
}

static void appDataConf(NWK_DataReq_t *req)
{
	// empty bytes_received and set array_index to 0
	array_index = 0;
	// set ready_to_receive after message is handled?
	ready_to_receive();
	HAL_UartWriteByte('O');
	HAL_UartWriteByte('K');
	HAL_UartWriteByte('\n');
	state = AWAITING_DATA;
}

void send_message()
{
	nwkDataReq.dstAddr = 0;
	nwkDataReq.dstEndpoint = 0x0001;
	nwkDataReq.srcEndpoint = 0x0001;
	//nwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
	//nwkDataReq.options = NWK_OPT_ENABLE_SECURITY;
	nwkDataReq.size = array_index;
	nwkDataReq.data = (uint8_t *) bytes_received;
	nwkDataReq.confirm = appDataConf;

	NWK_DataReq(&nwkDataReq);
	state = AWAITING_CONF;
}

/*************************************************************************//**
 *****************************************************************************/
static void APP_TaskHandler(void)
{
	// Put your application code here
	switch (state)
	{
	case RECEIVED_DATA:
		// send message out?
//		bytes_received[0] = 'O';
//		bytes_received[1] = 'K';
//		bytes_received[2] = 'X';
//		bytes_received[3] = '\n';
//		HAL_UartWriteByte('X');
		//array_index = 4;
		state = AWAITING_DATA;
		send_message();
		break;
	}

}

/*************************************************************************//**
 *****************************************************************************/bool network_ind(
		NWK_DataInd_t *ind)
{

	return true;
}

void initNetwork()
{
	NWK_SetAddr(0x0001);
	NWK_SetPanId(0x1973);
	PHY_SetChannel(0x16);
	PHY_SetTxPower(0);
	PHY_SetRxState(true);
}

int main(void)
{
// set PB7 as output
	DDRB |= _BV(PB7);
// set pin high - not clear to receive
	not_ready_to_receive();

	SYS_Init();
	HAL_UartInit(38400);
	HAL_LedInit();
	HAL_LedOn(0);
	initNetwork();
// ready to recieve
	ready_to_receive();
	sei();
	while (1)
	{
		SYS_TaskHandler();
		HAL_UartTaskHandler();
		APP_TaskHandler();
	}
}
