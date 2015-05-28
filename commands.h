/*
 * commands.h
 *
 *  Created on: Dec 5, 2014
 *      Author: titan
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_


typedef struct
{
	uint8_t command;
	uint16_t pan_id;
	uint8_t short_id;
	uint8_t message_length;
	uint8_t dummy_data; // used as place holder for payload/ not in original
}cmd_send_header_t;

typedef struct
{
	uint8_t message_length;
	uint8_t dummy_data; // used as place holder for payload/ not in original
}cmd_send_status_header_t;

typedef struct
{
	uint8_t command;
	uint16_t pan_id;
	uint16_t short_id;
	uint8_t channel;
}cmd_config_nwk_t;
#endif /* COMMANDS_H_ */
