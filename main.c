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
//#define SENDMSG_RAW
//#define SEND_TESTDATA
//#define HELLO_WORLD

#define ROUTER_ADDR 0x389C

typedef bool (*appDataInd_ptr_t)(NWK_DataInd_t *ind);
static bool appDataInd(NWK_DataInd_t *ind);
void initNetwork(cmd_config_nwk_t * nwk, appDataInd_ptr_t);
static size_t cobsEncode(uint8_t *input, uint8_t length, uint8_t *output);
void helloWordTimerHandler(SYS_Timer_t *timer);

/*- Types ------------------------------------------------------------------*/
// Put your type definitions here
/*- Prototypes -------------------------------------------------------------*/
void send_status_msg(uint8_t status);
void send_message();
void bailReceivedMessage(uint8_t e);
void setSyncronizing();
void synchronizeTimerHandler(SYS_Timer_t *timer);
void config_done();
void send_message_to();
// Put your function prototypes here
/*- Variables --------------------------------------------------------------*/

char bytes_received[64];
char rx_buffer[64];
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

uint8_t cobs_buffer[80];

NWK_DataReq_t nwkDataReq;
SYS_Timer_t timeoutTimer;

SYS_Timer_t helloWordTimer;


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
	uint8_t status;
} wan_status_msg_t;

cmd_send_address_t cmd_send_address;
cmd_nwk_configured_t cmd_nwk_configured;
cmd_config_nwk_t cmd_config_ntw;
wan_status_msg_t wan_status_msg;

enum states {
	AWAITING_DATA, RECEIVED_DATA, AWAITING_CONF, RESEND_MSG, SYNCHRONIZING
};
enum commands {
	SEND = 1, ACK_SEND = 2, CONFIG_NWK = 3, GET_ADDRESS = 4, CONFIG_DONE = 7
};
enum msg_types {
	PINGX = 0x01, ROUTER_STATUS = 0x08, CHANGESET = 0x09, WAN_STATUS = 0xEF
};
enum wan_statuses {
	MSG_TIMEOUT = 0x0A, CONF_RETRIES = 0x0B, MSG_ERROR = 0x0C, INVALID_MSG = 0x0D, SYNC_MSG = 0x0E
};
static uint8_t state = AWAITING_DATA;

static uint8_t ack;
//static uint8_t command;

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
	send_status_msg(MSG_TIMEOUT); // Send 1284 a timeout error.
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
	// sync done
	HAL_LedOff(0);
	send_status_msg(SYNC_MSG);
	setAwaitingData();
}

///////TESTING//////////////
void startHelloWordTimer()
{
	helloWordTimer.interval = 1000;
	helloWordTimer.mode = SYS_TIMER_INTERVAL_MODE;
	helloWordTimer.handler = helloWordTimerHandler;
	SYS_TimerStart(&helloWordTimer);
}

void helloWordTimerHandler(SYS_Timer_t *timer)
{
	HAL_UartWriteByte('H');
	HAL_UartWriteByte('E');
	HAL_UartWriteByte('L');
	HAL_UartWriteByte('L');
	HAL_UartWriteByte('O');
	HAL_UartWriteByte('W');
	HAL_UartWriteByte('O');
	HAL_UartWriteByte('R');
	HAL_UartWriteByte('L');
	HAL_UartWriteByte('D');

	HAL_LedToggle(0);
	SYS_TimerStart(&helloWordTimer);

}
////////////////////////////

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

void bailReceivedMessage(uint8_t e)
{
	send_status_msg(e);
	array_index = 0;
	flushUartRx();
	setSyncronizing();
}

void setSyncronizing()
{
	if (state == SYNCHRONIZING)
		return;

	HAL_LedOn(0);


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
				//ERIC: Check frame_length against max length
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
					bailReceivedMessage(INVALID_MSG);
				}

			} else if (array_index > frame_length)
			{
				// This should never happen ...
				//bailReceivedMessage('G');
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
		ack_retry_count = 0;
		HAL_LedOff(0);
	} else
	{
		if (ack_retry_count <= 3)
		{
			// retry to send the message 3 times
			state = RESEND_MSG;
			ack_retry_count++;
		} else
		{
			// send message to 1284 that message was not received
			send_status_msg(CONF_RETRIES);
			setAwaitingData();
			ack_retry_count = 0;
			HAL_LedOff(0);
		}
	}
}

void send_status_msg(uint8_t status)
{
	uint8_t cobs_size = 0;
	uint8_t msg_size = 0;
	wan_status_msg.command = WAN_STATUS;
	wan_status_msg.status = status;

	uint8_t frame[20];
	msg_size = sizeof(wan_status_msg_t) + 1;

	int frame_index = 0;
	// message
	for (int i = 0; i < sizeof(wan_status_msg); i++)
	{
		frame[frame_index++] = ((uint8_t *) (&wan_status_msg))[i];
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
		HAL_UartWriteByte(cobs_buffer[i]);
	}

	// Zero becomes EOR marker
	HAL_UartWriteByte(0x00);
}

void send_message()
{
	cmd_send_header_t * cmd;
	cmd = (cmd_send_header_t*) &bytes_received[1];
	nwkDataReq.dstAddr = 0x0000;
	nwkDataReq.dstEndpoint = 0x0001;
	nwkDataReq.srcEndpoint = 0x0001;
	nwkDataReq.size = cmd->message_length;
	nwkDataReq.data = &(cmd->dummy_data);
	nwkDataReq.confirm = appDataConf;

	NWK_DataReq(&nwkDataReq);
	state = AWAITING_CONF;
	HAL_LedOn(0);
}

void initNetwork(cmd_config_nwk_t * nwk, appDataInd_ptr_t ind_ptr)
{
	NWK_SetAddr(nwk->short_id);
	NWK_SetPanId(nwk->pan_id);
	PHY_SetChannel(nwk->channel);
	PHY_SetTxPower(0);
	PHY_SetRxState(true);
	NWK_OpenEndpoint(1, ind_ptr);

	config_done();
}

void config_done()
{
	config_nwk_done();
	setAwaitingData();
}

static void APP_TaskHandler(void)
{
#ifdef COORDINATOR
	//send_test_data();
#endif
	// Put your application code here
	switch (state)
	{
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
				cmd_nwk_configured.command = CONFIG_NWK;
				cmd_nwk_configured.cs = 0xAA;
				initNetwork((cmd_config_nwk_t*) &bytes_received[1], appDataInd);
				break;
			case CONFIG_DONE:
				config_done();
				break;
			case CHANGESET:
				if (frame_length <= sizeof(rx_buffer))
				{
					memcpy(rx_buffer, bytes_received, frame_length);
					send_message_to();
				}
			default:
				send_status_msg(MSG_ERROR);
			}
		}
	}

}

void send_message_to()
{
	cmd_send_header_t * cmd;
	cmd = (cmd_send_header_t*) &rx_buffer[1];

	// send this message to the requested router
	nwkDataReq.dstAddr = cmd->short_id;

	nwkDataReq.dstEndpoint = 1;
	nwkDataReq.srcEndpoint = 1;
	nwkDataReq.size = cmd->message_length;
	nwkDataReq.data = &(cmd->dummy_data);
	nwkDataReq.confirm = appDataConf;

	NWK_DataReq(&nwkDataReq);
	state = AWAITING_CONF;
	HAL_LedOn(0);
}

bool network_ind(NWK_DataInd_t *ind)
{
	return true;
}

void initNetwork2(appDataInd_ptr_t ind_ptr)
{
#ifdef HELLO_WORLD
	startHelloWordTimer();
#endif
#ifdef COORDINATOR
	NWK_SetAddr(0x0000);
#else
	NWK_SetAddr(0x1337);
#endif

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

#ifdef SENDMSG_RAW
static void sendReceivedMsgRAW(uint8_t *data, uint8_t size)
{
//	HAL_UartWriteByte(0xB0);
//	HAL_UartWriteByte(0x0B);
//	HAL_UartWriteByte(0xBE);

	for (uint8_t i = 0; i < size; i++)
	{
		//HAL_LedToggle(LED_DATA);
		HAL_UartWriteByte(data[i]);
	}
//	HAL_UartWriteByte(0xC0);
//	HAL_UartWriteByte(0xFF);
//	HAL_UartWriteByte(0xEE);
}
#else
// send message that we received from the radio out the uart
static void sendReceivedMsg(uint8_t *data, uint8_t size)
{
	uint8_t cobs_size = 0;

	if (size > sizeof(cobs_buffer))
		return;

	// Stuff bytes (remove all zeros)
	cobs_size = cobsEncode(data, size, cobs_buffer);

	for (uint8_t i = 0; i < cobs_size; i++)
	{
		HAL_UartWriteByte(cobs_buffer[i]);
	}

	// Zero becomes EOR marker
	HAL_UartWriteByte(0x00);
}
#endif

static bool appDataInd(NWK_DataInd_t *ind)
{
	HAL_LedOff(0);

	sendReceivedMsg(ind->data, ind->size);
	return true;
}

#ifdef SEND_TESTDATA

uint8_t frame[5];
void startTestDataTimer();
SYS_Timer_t testDataTimer;

void timeoutTestData(SYS_Timer_t *timer)
{
	if (!NWK_Busy())
	{
		frame[0] = 0xAA;
		frame[1] = 0xBB;
		frame[2] = 0xCC;
		frame[3] = 0xDD;
		frame[4] = 0xEE;

#if defined(COORDINATOR)
		nwkDataReq.dstAddr = ROUTER_ADDR;
#else
		nwkDataReq.dstAddr = 0x0000;
#endif
		nwkDataReq.dstEndpoint = 1;
		nwkDataReq.srcEndpoint = 1;
		nwkDataReq.size = sizeof(frame);
		nwkDataReq.data = frame;
		nwkDataReq.confirm = appDataConf;

		NWK_DataReq(&nwkDataReq);
		HAL_LedOn(0);
	}
	startTestDataTimer();
}

void startTestDataTimer()
{
	testDataTimer.interval = 1000;
	testDataTimer.mode = SYS_TIMER_INTERVAL_MODE;
	testDataTimer.handler = timeoutTestData;
	SYS_TimerStart(&testDataTimer);
}

#endif

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

#if defined(COORDINATOR)
	initNetwork2(appDataInd); // FOR TESTING
#else
	// set pin PB6 to low. This will tell the 1284 to configure zigbit nwk
	config_nwk();
#endif

	// ready to recieve
	setAwaitingData();

#ifdef SEND_TESTDATA
	startTestDataTimer();
#endif

	while (1)
	{
		SYS_TaskHandler();
		HAL_UartTaskHandler();
		APP_TaskHandler();

	}
}
