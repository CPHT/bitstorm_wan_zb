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
#include <avr/boot.h>

#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "halUart.h"
#include "halLed.h"
#include "sysTimer.h"
#include "commands.h"

/*- Definitions ------------------------------------------------------------*/
// Put your preprocessor definitions here
/*- Types ------------------------------------------------------------------*/
// Put your type definitions here
/*- Prototypes -------------------------------------------------------------*/
void send_msg_status();
void send_message();
// Put your function prototypes here
/*- Variables --------------------------------------------------------------*/

char bytes_received[64];
uint8_t array_index = 0;
int frame_length = 0;
int message_length = 0;
uint64_t extAddr = 0;
int cmd = 0;
int ack_retry_count = 0;

NWK_DataReq_t nwkDataReq;
SYS_Timer_t timeoutTimer;

typedef struct {
	uint8_t command;
	uint64_t address;
	uint8_t cs;
} cmd_send_address_t;

typedef struct {
	uint8_t command;
	uint8_t cs;
} cmd_nwk_configured_t;

typedef struct {
	uint8_t command;
	uint8_t cs;
} msg_status_resp_t;

cmd_send_address_t cmd_send_address;
cmd_nwk_configured_t cmd_nwk_configured;
cmd_config_nwk_t cmd_config_ntw;
msg_status_resp_t msg_status_resp;

void initNetwork(cmd_config_nwk_t * nwk);

enum states {
	AWAITING_DATA, RECEIVED_DATA, AWAITING_CONF, RESEND_MSG
};
enum commands {
	SEND, ACK_SEND, CONFIG_NWK, GET_ADDRESS
};
static uint8_t state = AWAITING_DATA;
static uint8_t command;

/*- Implementations --------------------------------------------------------*/

// Put your function implementations here
void not_ready_to_receive()
{
	// set pin PB7 LOW when NOT ready to receive data
	PORTB &= ~_BV(PB7);
}

void ready_to_receive()
{
	// set pin PB7 HIGH when ready to receive data
	PORTB |= _BV(PB7);
}

void timeoutTimerHandler(SYS_Timer_t *timer)
{
	// handle timeout
	HAL_UartWriteByte('T'); // Send back a timeout error.
	array_index = 0;
	ready_to_receive();
	state = AWAITING_DATA;
}

void startTimeoutTimer()
{
	timeoutTimer.interval = 250;
	timeoutTimer.mode = SYS_TIMER_INTERVAL_MODE;
	timeoutTimer.handler = timeoutTimerHandler;
	SYS_TimerStart(&timeoutTimer);
}

void get_device_address()
{
	uint8_t b;
	b = boot_signature_byte_get(0x0102);
	extAddr = b;
	b = boot_signature_byte_get(0x0103);
	extAddr |= ((uint64_t) b << 8);
	b = boot_signature_byte_get(0x0104);
	extAddr |= ((uint64_t) b << 16);
	b = boot_signature_byte_get(0x0105);
	extAddr |= ((uint64_t) b << 24);
	b = boot_signature_byte_get(0x0106);
	extAddr |= ((uint64_t) b << 32);
	b = boot_signature_byte_get(0x0107);
	extAddr |= ((uint64_t) b << 40);
	b = boot_signature_byte_get(0x0108);
	extAddr |= ((uint64_t) b << 48);
	b = boot_signature_byte_get(0x0109);
	extAddr |= ((uint64_t) b << 56);

	cmd_send_address.address = extAddr;
}

void send_address()
{
	uint8_t frame[80];
	frame[0] = sizeof(cmd_send_address);

	int frame_index = 1;
	// message
	for (int i = 0; i < sizeof(cmd_send_address); i++)
	{
		frame[frame_index++] = ((uint8_t *) (&cmd_send_address))[i];
	}

	// send the bytes to the 1284
	for (int i = 0; i < frame_index; i++)
	{
		HAL_UartWriteByte(frame[i]);
	}

	array_index = 0;
	ready_to_receive();
	state = AWAITING_DATA;
}

void HAL_UartBytesReceived(uint16_t bytes)
{
	if (!NWK_Busy())
	{
		for (int i = 0; i < bytes; i++, array_index++)
		{
			//HAL_UartWriteByte('X');
			// get the bytes
			bytes_received[array_index] = HAL_UartReadByte();

			// get the length
			if (array_index == 0)
			{
				frame_length = bytes_received[array_index];

				// if it takes longer than .25 sec to process the message, bail.
				startTimeoutTimer();
			} else if (array_index == 1)
			{
				cmd = bytes_received[array_index];

			} else if (array_index == frame_length) // the end
			{
				// Good message, stop timer
				SYS_TimerStop(&timeoutTimer);

				switch (cmd)
				{
				case 1:
					command = SEND;
					break;
				case 2:
					command = ACK_SEND;
					break;
				case 3:
					command = CONFIG_NWK;
					break;
				case 4:
					command = GET_ADDRESS;
					break;
				default:
					{
						// we got an unknown command
						array_index = 0;
						ready_to_receive();
						state = AWAITING_DATA;
					}
				}

				//raise rts low
				not_ready_to_receive();
				//and set a flag that we have a message
				state = RECEIVED_DATA;
			} else if (array_index > frame_length)
			{
				HAL_UartWriteByte('E');
				array_index = 0;
				state = AWAITING_DATA;
			}
		}
	} else
	{
		for (int i = 0; i < bytes; i++)
			HAL_UartReadByte();
	}
}

static void appDataConf(NWK_DataReq_t *req)
{
	if (NWK_SUCCESS_STATUS == req->status)
	{
		array_index = 0;
		state = AWAITING_DATA;
		HAL_LedOff(0);
	} else
	{
		if (ack_retry_count <= 3)
		{
			// retry to send the message 3 times
			//nwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
			state = RESEND_MSG;
			ack_retry_count++;
		} else
		{
			// send message to 1284 that message was not received
			send_msg_status();
			array_index = 0;
			state = AWAITING_DATA;
			ack_retry_count = 0;
			HAL_LedOff(0);
		}
	}
}

void send_msg_status()
{
	msg_status_resp.command = 0x07;
	msg_status_resp.cs = 0xFF;
	// respond back
	uint8_t frame[80];
	frame[0] = sizeof(msg_status_resp);

	int frame_index = 1;
	// message
	for (int i = 0; i < sizeof(msg_status_resp); i++)
	{
		frame[frame_index++] = ((uint8_t *) (&msg_status_resp))[i];
	}

	// send the bytes to the 1284
	for (int i = 0; i < frame_index; i++)
	{
		HAL_UartWriteByte(frame[i]);
	}
}

void send_message()
{
	cmd_send_header_t * cmd;
	cmd = (cmd_send_header_t*) &bytes_received[1];
	nwkDataReq.dstAddr = 0;
	nwkDataReq.dstEndpoint = 0x0001;
	nwkDataReq.srcEndpoint = 0x0001;
	//if(command == ACK_SEND)
	//nwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
	//nwkDataReq.options = NWK_OPT_ENABLE_SECURITY;
	nwkDataReq.size = cmd->message_length;
	nwkDataReq.data = &(cmd->dummy_data);
	nwkDataReq.confirm = appDataConf;

	NWK_DataReq(&nwkDataReq);
	state = AWAITING_CONF;
	HAL_LedOn(0);
}

/*************************************************************************//**
 *****************************************************************************/

void initNetwork(cmd_config_nwk_t * nwk)
{

	NWK_SetAddr(nwk->short_id);
	NWK_SetPanId(nwk->pan_id);
	PHY_SetChannel(nwk->channel);
	PHY_SetTxPower(0);
	PHY_SetRxState(true);

	// respond back
	uint8_t frame[80];
	frame[0] = sizeof(cmd_nwk_configured);

	int frame_index = 1;
	// message
	for (int i = 0; i < sizeof(cmd_nwk_configured); i++)
	{
		frame[frame_index++] = ((uint8_t *) (&cmd_nwk_configured))[i];
	}

	// send the bytes to the 1284
	for (int i = 0; i < frame_index; i++)
	{
		HAL_UartWriteByte(frame[i]);
	}

	array_index = 0;
	state = AWAITING_DATA;
}

static void APP_TaskHandler(void)
{

	// Put your application code here
	switch (state)
	{
	case AWAITING_DATA:
		if (!NWK_Busy())
			ready_to_receive();
		break;
	case RESEND_MSG:
		if (!NWK_Busy())
			send_message();
		break;
	case RECEIVED_DATA:
		switch (command)
		{
		case SEND:
			// send message out
			send_message();
			break;
		case ACK_SEND:
			nwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
			send_message();
			break;
		case GET_ADDRESS:
			get_device_address();
			cmd_send_address.command = 0x04;
			cmd_send_address.cs = 0xFF;
			send_address();
			break;
		case CONFIG_NWK:
			cmd_nwk_configured.command = 0x03;
			cmd_nwk_configured.cs = 0xFF;
			initNetwork((cmd_config_nwk_t*) &bytes_received[1]);
			break;
		}

	}

}

/*************************************************************************//**
 *****************************************************************************/bool network_ind(
		NWK_DataInd_t *ind)
{

	return true;
}
void initNetwork2()
{
	NWK_SetAddr(0x0000);
	NWK_SetPanId(0x1973);
	PHY_SetChannel(0x16);
	PHY_SetTxPower(0);
	PHY_SetRxState(true);
}

int main(void)
{
	// set PB7 as output
	DDRB |= _BV(PB7);
	// set pin low - not clear to receive
	not_ready_to_receive();

	SYS_Init();
	HAL_UartInit(38400);
	HAL_LedInit();
	HAL_LedOff(0);
	//initNetwork2();

	// ready to recieve
	ready_to_receive();

	while (1)
	{
		SYS_TaskHandler();
		HAL_UartTaskHandler();
		APP_TaskHandler();
	}
}
