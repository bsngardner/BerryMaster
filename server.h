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
#define MSG_REPLY 		0xFEu // rpc reply
#define MSG_INTERRUPT	0xFD // interrupt
#define MSG_ERROR 		0xFCu // error

// opcodes
#define OP_INIT_DEVICES 0
#define OP_GET_DEV_TYPE 1
#define OP_GET_DEV_VAL  2
#define OP_SET_DEV_VAL  3
#define OP_GET_MUL_VALS 4
#define OP_SET_MUL_VALS 5
#define OP_SET_PROJ_KEY 6

// initializes the server
void server_init();

// runs the server for a single remote procedure call
int server_event();

// sends an "interrupt" message to the host
void interrupt_host(uint8_t intr_source, uint8_t *buff,	uint8_t count);

//IO define
#ifndef SERVER_BUF_SIZE
#define SERVER_BUF_SIZE 64;
#endif

#endif /* SERVER_H_ */
