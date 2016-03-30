/*
 * rpcHandlers.h
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

#ifndef SERVER_H_
#define SERVER_H_
//Standard includes
#include <stdint.h>
//IO include
#include "iomatrix.h"
#include "IObuffer.h"

// defined constants ----------------------------------------------------------

//	MAPI Protocol:
//  Host->Master:
//  1 byte|1 byte|any length up to MAX_MSG_LENGTH
//	length|opcode|parameters...
//  Notes:
//    The address will always be the first parameter
//
//  Master->Host:
//  1 byte|1 byte  |any length up to MAX_MSG_LENGTH
//	length|msg type|parameters...

// message offsets: Host->Master
#define OFFSET_LENGTH 0
#define OFFSET_OPCODE 1
#define OFFSET_ADDR 2

// message offsets: Master->Host
#define OFFSET_TYPE 1
#define OFFSET_RESULT 2
#define OFFSET_REPLY_PARAMS 3

// maximum length of a message
#define MAX_MSG_LENGTH   32

// length of reply without parameters
#define STD_REPLY_LENGTH 2 // type + result = 2 bytes

// message types
#define TYPE_REPLY 0xFEu // rpc reply
#define TYPE_ERROR 0xFCu // error

// opcodes
#define OPCODE_INIT_DEVICES	 	'i'
#define OPCODE_GET_DEVICE_TYPE  't'
#define OPCODE_GET_DEVICE_VALUE 'g'
#define OPCODE_GET_DEVICE_MULTI_VALUES 'h'
#define OPCODE_SET_DEVICE_VALUE 's'

// runs the server for a single remote procedure call
void server_init();
int serverEvent();

//IO define
#ifndef SERVER_BUF_SIZE
#define SERVER_BUF_SIZE 64;
#endif

#endif /* SERVER_H_ */
