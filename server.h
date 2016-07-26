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

// message types
#define MSG_REPLY       254 // rpc reply
#define MSG_INTERRUPT   253 // interrupt
#define MSG_ERROR       252 // error
#define MSG_LOG			251 // log
#define MSG_WRN			250 // warning

// initializes the server
void server_init();

// runs the server for a single remote procedure call
int server_event();

// sends an "interrupt" message to the host
void interrupt_host(uint8_t intr_source, uint8_t *buff,	uint8_t count);

#endif /* SERVER_H_ */
