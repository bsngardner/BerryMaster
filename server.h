/*
 * rpcHandlers.h
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

#ifndef SERVER_H_
#define SERVER_H_

#include <stdint.h>

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
#define MAX_MSG_LENGTH   64

// length of reply without parameters
#define STD_REPLY_LENGTH 2 // type + result = 2 bytes

// message types
#define TYPE_REPLY 0xFE // rpc reply
#define TYPE_DEBUG 0xFD // debug
#define TYPE_ERROR 0xFCu // error

// opcodes
#define OPCODE_INIT_DEVICES	 	'i'
#define OPCODE_GET_DEVICE_TYPE  't'
#define OPCODE_GET_DEVICE_VALUE 'g'
#define OPCODE_SET_DEVICE_VALUE 's'

// recv_request return types
#define RECV_REQ_SUCCESS 0
#define RECV_REQ_ERROR   1
#define MSG_NOT_SENT_YET 2

// runs the server for a single remote procedure call
int serverEvent();


#endif /* SERVER_H_ */