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
#include <string.h>
#include <util/delay.h>

/*- Definitions ------------------------------------------------------------*/
// Put your preprocessor definitions here
//#define ROUTER
#define COORDINATOR
typedef bool (*appDataInd_ptr_t)(NWK_DataInd_t *ind);
static bool appDataInd(NWK_DataInd_t *ind);
void initNetwork(cmd_config_nwk_t * nwk, appDataInd_ptr_t);
static size_t cobsEncode(uint8_t *input, uint8_t length, uint8_t *output);

/*- Types ------------------------------------------------------------------*/
// Put your type definitions here
typedef struct PACK AppMessage_t {
	uint8_t messageType;
	uint8_t nodeType;
	uint64_t extAddr;
	uint16_t shortAddr;
	uint64_t routerAddr;
	uint16_t panId;
	uint8_t workingChannel;
	uint16_t parentShortAddr;
	uint8_t lqi;
	int8_t rssi;
	uint8_t ackByte;

	int32_t battery;
	int32_t temperature;

	uint8_t cs;

} AppMessage_t;
/*- Prototypes -------------------------------------------------------------*/
void send_msg_status();
void send_message();
void bailReceivedMessage(char e);
void setSyncronizing();
void synchronizeTimerHandler(SYS_Timer_t *timer);
// Put your function prototypes here
/*- Variables --------------------------------------------------------------*/

char bytes_received[64];
uint8_t array_index = 0;
int frame_length = 0;
int message_length = 0;
uint64_t extAddr = 0;
int cmd = 0;
int ack_retry_count = 0;
bool configured = false;
uint8_t x_counter = 0;
uint8_t counter = 0;

typedef struct {
	uint8_t command;
	uint8_t *data;
	uint8_t cs;

} msg_def_t;

uint8_t cobs_buffer[sizeof(msg_def_t)];

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

enum states {
	AWAITING_DATA, RECEIVED_DATA, AWAITING_CONF, RESEND_MSG, SYNCHRONIZING
};
enum commands {
	SEND = 1, ACK_SEND = 2, CONFIG_NWK = 3, GET_ADDRESS = 4, CONFIG_DONE = 7
};
static uint8_t state = AWAITING_DATA;

static uint8_t ack;
//static uint8_t command;

/*- Implementations --------------------------------------------------------*/

//DEBUG DEBUG DEBUG DEBUG
//DEBUG DEBUG DEBUG DEBUG
//DEBUG DEBUG DEBUG DEBUG
//DEBUG DEBUG DEBUG DEBUG
//DEBUG DEBUG DEBUG DEBUG
//void not_ready_to_receive();
//SYS_Timer_t debugTimer;
//char debugString[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789CPHT";
//void debugTimerHandler(SYS_Timer_t *timer) {
//	bytes_received[0] = 1 + 4 + sizeof(debugString);
//	bytes_received[1] = 1;
//	bytes_received[2] = 0x00;
//	bytes_received[3] = 0x00;
//	bytes_received[4] = 0x00;
//	bytes_received[5] = sizeof(debugString);
//	for (int i = 0, j = 6; debugString[i]; bytes_received[j++] =
//			debugString[i++])
//		;
//	not_ready_to_receive();
//	state = RECEIVED_DATA;
//}
//void debugInit() {
//	debugTimer.interval = 10;
//	debugTimer.mode = SYS_TIMER_INTERVAL_MODE;
//	debugTimer.handler = debugTimerHandler;
//	SYS_TimerStart(&debugTimer);
//}
//DEBUG DEBUG DEBUG DEBUG
//DEBUG DEBUG DEBUG DEBUG
//DEBUG DEBUG DEBUG DEBUG
//DEBUG DEBUG DEBUG DEBUG
//DEBUG DEBUG DEBUG DEBUG
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

void config_nwk()
{
	PORTB &= ~_BV(PB6);
}

void config_nwk_done()
{
	// set pin PB HIGH when nwk configuration is done
	PORTB |= _BV(PB6);
}

void flushUartRx(void)
{
	while (HAL_UartGetRxFifoBytes() > 0)
		HAL_UartReadByte();
}

void setAwaitingData()
{
	array_index = 0;
	flushUartRx();
	state = AWAITING_DATA;
	ready_to_receive();
}

void timeoutTimerHandler(SYS_Timer_t *timer)
{
	// handle timeout
	HAL_UartWriteByte('T'); // Send back a timeout error.
	setAwaitingData();
}

void startTimeoutTimer()
{
	timeoutTimer.interval = 250;
	timeoutTimer.mode = SYS_TIMER_INTERVAL_MODE;
	timeoutTimer.handler = timeoutTimerHandler;
	SYS_TimerStart(&timeoutTimer);
}

void startSynchronizeTimer()
{
	timeoutTimer.interval = 500;
	timeoutTimer.mode = SYS_TIMER_INTERVAL_MODE;
	timeoutTimer.handler = synchronizeTimerHandler;
	SYS_TimerStart(&timeoutTimer);
}

void synchronizeTimerHandler(SYS_Timer_t *timer)
{
	// handle timeout
	HAL_LedOff(0);
	HAL_UartWriteByte('Z');
	setAwaitingData();
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

	setAwaitingData();
}

bool isValidMessage(void)
{
	uint8_t cs = 0;
	uint8_t cs_in = 0;

	for (int i = 0; i < frame_length; cs ^= bytes_received[i++])
		;

	cs_in = bytes_received[frame_length];

	if (cs == cs_in)
	{
		return true;
	} else
	{
		return false;
	}
}

void bailReceivedMessage(char e)
{
	HAL_UartWriteByte(e);
	array_index = 0;
	flushUartRx();
	//state = AWAITING_DATA;
	setSyncronizing();
}

void setSyncronizing()
{
	if (state == SYNCHRONIZING)
		return;

	HAL_LedOn(0);

	HAL_UartWriteByte('A');
	not_ready_to_receive();
	state = SYNCHRONIZING;
	array_index = 0;
	memset(bytes_received, 0, sizeof(bytes_received));
	SYS_TimerStop(&timeoutTimer);
	startSynchronizeTimer();
}

void HAL_UartBytesReceived(uint16_t bytes)
{
	for (int i = 0; i < bytes; i++, array_index++)
	{
		uint8_t data = HAL_UartReadByte();

		if (data == 'X')
		{
			x_counter++;
			if (x_counter >= 10)
			{
				setSyncronizing();
			}
		} else
			x_counter = 0;

		if (array_index >= sizeof(bytes_received))
			continue;

		if (state == AWAITING_DATA)
		{
			// get the bytes
			bytes_received[array_index] = data;

			// get the length
			if (array_index == 0)
			{
				frame_length = bytes_received[array_index];
				// if it takes longer than .25 sec to process the message, bail.
				startTimeoutTimer();

				// get the command
			} else if (array_index == 1)
			{
				cmd = bytes_received[array_index];

				// message completed
			} else if (array_index == frame_length)
			{
				SYS_TimerStop(&timeoutTimer);
				if (isValidMessage())
				{
					not_ready_to_receive();
					state = RECEIVED_DATA;
				} else
				{
					bailReceivedMessage('F');
				}

			} else if (array_index > frame_length)
			{
				// This should never happen ...
				bailReceivedMessage('G');
			}
		}
	}
}

static void appDataConf(NWK_DataReq_t *req)
{
	ack = req->control;
	if (NWK_SUCCESS_STATUS == req->status)
	{
		setAwaitingData();
		HAL_LedOff(0);
		//SYS_TimerStart(&debugTimer);	//DEBUG DEBUG DEBUG
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
			setAwaitingData();
			ack_retry_count = 0;
			HAL_LedOff(0);
			//SYS_TimerStart(&debugTimer);	//DEBUG DEBUG DEBUG
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
	nwkDataReq.dstAddr = 0x0000;
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

void initNetwork(cmd_config_nwk_t * nwk, appDataInd_ptr_t ind_ptr)
{
	uint8_t cobs_size = 0;
	uint8_t msg_size = 0;

	NWK_SetAddr(nwk->short_id);
	NWK_SetPanId(nwk->pan_id);
	PHY_SetChannel(nwk->channel);
	PHY_SetTxPower(0);
	PHY_SetRxState(true);
	NWK_OpenEndpoint(1, ind_ptr);

	// respond back to master
	uint8_t frame[10];
	msg_size = sizeof(cmd_nwk_configured) + 1;

	int frame_index = 0;
	// message
	for (int i = 0; i < sizeof(cmd_nwk_configured); i++)
	{
		frame[frame_index++] = ((uint8_t *) (&cmd_nwk_configured))[i];
	}

	// checksum
	uint8_t cs = 0;
	for (int i = 0; i < frame_index; cs ^= frame[i++])
		;
	frame[frame_index++] = cs;

	// Stuff bytes (remove all zeros)
	cobs_size = cobsEncode(frame, msg_size, cobs_buffer);

	for (uint8_t i = 0; i < cobs_size; i++)
	{
		//HAL_LedToggle(LED_DATA);
		HAL_UartWriteByte(cobs_buffer[i]);
	}

	// Zero becomes EOR marker
	HAL_UartWriteByte(0x00);

	setAwaitingData();
}

void config_done()
{
	config_nwk_done();
	setAwaitingData();
}

void send_test_data()
{
	if (!NWK_Busy())
	{
		uint8_t frame;
		// size of message

		frame = 0x06;
//		frame[1] = 0xBB;
//		frame[2] = 0xBB;

		nwkDataReq.dstAddr = 0x389C;
		nwkDataReq.dstEndpoint = 0x0001;
		nwkDataReq.srcEndpoint = 0x0001;
		nwkDataReq.size = sizeof(frame);
		nwkDataReq.data = &frame;
		nwkDataReq.confirm = appDataConf;

		NWK_DataReq(&nwkDataReq);
		state = AWAITING_CONF;
		HAL_LedOn(0);
		_delay_ms(250);
		HAL_LedOff(0);
	}
}

static void APP_TaskHandler(void)
{
#ifdef COORDINATOR
	//send_test_data();
#endif
	// Put your application code here
	switch (state)
	{
//	case AWAITING_DATA:
//		if (!NWK_Busy())
//			ready_to_receive();
//		break;
	case RESEND_MSG:
		if (!NWK_Busy())
			send_message();
		break;
	case RECEIVED_DATA:
		if (!NWK_Busy())
		{
			switch (cmd)
			{
			case SEND:
				// send message out
				send_message();
				break;
			case ACK_SEND:
				// send message with ack
				nwkDataReq.options = NWK_OPT_ACK_REQUEST | NWK_OPT_ENABLE_SECURITY;
				send_message();
				break;
			case CONFIG_NWK:
				cmd_nwk_configured.command = 0x03;
				cmd_nwk_configured.cs = 0xAA;
				initNetwork((cmd_config_nwk_t*) &bytes_received[1], appDataInd);
				break;
			case CONFIG_DONE:
				config_done();
				break;
			default:
				bailReceivedMessage('E');
			}
		}
	}

}

/*************************************************************************//**
 *****************************************************************************/bool network_ind(NWK_DataInd_t *ind)
{

	return true;
}

void initNetwork2(appDataInd_ptr_t ind_ptr)
{
	NWK_SetAddr(0x0000);
	NWK_SetPanId(0x1973);
	PHY_SetChannel(0x16);
	PHY_SetTxPower(0);
	PHY_SetRxState(true);
	NWK_OpenEndpoint(1, ind_ptr);
}

static size_t cobsEncode(uint8_t *input, uint8_t length, uint8_t *output)
{
	size_t read_index = 0;
	size_t write_index = 1;
	size_t code_index = 0;
	uint8_t code = 1;

	while (read_index < length)
	{
		if (input[read_index] == 0)
		{
			output[code_index] = code;
			code = 1;
			code_index = write_index++;
			read_index++;
		} else
		{
			output[write_index++] = input[read_index++];
			code++;
			if (code == 0xFF)
			{
				output[code_index] = code;
				code = 1;
				code_index = write_index++;
			}
		}
	}

	output[code_index] = code;

	return write_index;
}

// send message that we received from the radio out the uart
static void sendReceivedMsg(uint8_t *data, uint8_t size)
{
	uint8_t cobs_size = 0;
	// messages intended for the coordinator
	// regular pings and router status messages
	if (data[0] == 0x01 || data[0] == 0x08)
	{
		AppMessage_t * msg = (AppMessage_t *) data;

		// Calculate checksum...Couples us to AppMessage_t, but whatever...
		msg->cs = 0;
		for (uint8_t i = 0; i < sizeof(AppMessage_t) - 1; i++)
		{
			uint8_t x = ((uint8_t*) msg)[i];
			msg->cs ^= x;
		}
	}


	// Stuff bytes (remove all zeros)
	cobs_size = cobsEncode(data, size, cobs_buffer);

	for (uint8_t i = 0; i < cobs_size; i++)
	{
		//HAL_LedToggle(LED_DATA);
		HAL_UartWriteByte(cobs_buffer[i]);
	}

	// Zero becomes EOR marker
	HAL_UartWriteByte(0x00);
}

static bool appDataInd(NWK_DataInd_t *ind)
{
	HAL_LedToggle(0);
	sendReceivedMsg(ind->data, ind->size);
	//NWK_SetAckControl(ack);
	return true;
}

int main(void)
{

// set PB7 and PB6 as output
	DDRB |= _BV(PB7);
	DDRB |= _BV(PB6);
// set pin low - not clear to receive
	not_ready_to_receive();

	SYS_Init();
	HAL_UartInit(38400);
	HAL_LedInit();
	HAL_LedOff(0);
#ifdef COORDINATOR
	initNetwork2(appDataInd); // FOR TESTING
#endif

// set pin PB6 to low. This will tell the 1284 to configure zigbit nwk
#ifdef ROUTER
	config_nwk();
#endif
// ready to recieve
	setAwaitingData();

	while (1)
	{
		SYS_TaskHandler();
		HAL_UartTaskHandler();
		APP_TaskHandler();

	}
}
