/*
 * rpcHandlers.c
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

#include "server.h"
#include "BerryMaster.h"
#include "IObuffer.h"
#include "ft201x.h"
#include <msp430.h>

// Global variables
char sentMessage[MAX_MSG_LENGTH];
volatile int buffLocked = FALSE;

// Functions
int serverEvent() {

	static uint8_t message[MAX_MSG_LENGTH];
	static uint8_t len;
	static uint8_t opcode;

	// get the message:
	if (recv_request(message) == MSG_NOT_SENT_YET) {
		// todo: error! shouldn't service this event until the message
		// has been received
		handleError();
	}

	// get the length and opcode
	len = message[0];
	opcode = message[1];

	// todo: use len to error check.

	// handle the request:
	switch(opcode) {
	case OPCODE_INIT_DEVICES:
		// init the network (also sends a reply to the host)
		rpc_initDevices(message, len);
		break;
	case OPCODE_GET_DEVICE_TYPE:
		// get the device type (this also sends a reply to the host)
		rpc_getDeviceType(message, len);
		break;
	case OPCODE_GET_DEVICE_VALUE:
		// Get the specified register value in the berry.
		rpc_getDeviceValue(message, len);
		break;
	case OPCODE_SET_DEVICE_VALUE:
		// Set the specified register value in the berry.
		rpc_setDeviceValue(message, len);
		break;
	default:
		break;
	}

	return 0;
}

// calls master initDevices function
// and sends a reply back to the host
void rpc_initDevices(uint8_t* message, uint8_t len) {
	uint8_t result = initDevices();
	uint8_t length = STD_REPLY_LENGTH;
	message[OFFSET_LENGTH] = length;
	message[OFFSET_TYPE] = TYPE_REPLY;
	message[OFFSET_RESULT] = result;
	send_reply(message, length);
}

// gets the address from the message
// calls master getDeviceType
// and sends reply back to host
void rpc_getDeviceType(uint8_t* message, uint8_t len) {

	// get the address from the message
	uint8_t addr = message[OFFSET_ADDR];

	// call master getDeviceType
	uint8_t type;
	uint8_t result = getDeviceType(addr, &type);

	// reply back to host.
	uint8_t length = STD_REPLY_LENGTH + 1;
	message[OFFSET_LENGTH] = length;
	message[OFFSET_TYPE] = TYPE_REPLY;
	message[OFFSET_RESULT] = result;
	message[OFFSET_REPLY_PARAMS] = type;
	send_reply(message, length);
}

void rpc_getDeviceValue(uint8_t* message, uint8_t len) {

	reportError("test in getDeviceValue function", 32);
	// get the address from the message
	uint8_t addr = message[OFFSET_ADDR];

	// get the register from the message
	uint8_t reg = message[OFFSET_ADDR + 1];

	// get the value from the device
	uint8_t value;
	uint8_t result = getDeviceValue(addr, &value, reg);

	// reply back to the host.
	uint8_t length = STD_REPLY_LENGTH + 1;
	message[OFFSET_LENGTH] = length; // length of reply in bytes
	message[OFFSET_TYPE] = TYPE_REPLY;
	message[OFFSET_RESULT] = result; // success (0) or failed (non-zero)
	message[OFFSET_REPLY_PARAMS] = value; // the value in the register
	send_reply(message, length);
}

void rpc_setDeviceValue(uint8_t* message, uint8_t len) {
	// get the address from the message
	uint8_t addr = message[OFFSET_ADDR];

	// get the register from the message (1 byte)
	uint8_t reg = message[OFFSET_ADDR + 1];

	// get the value from the message (1 byte)
	uint8_t value = message[OFFSET_ADDR + 2];

	// set the value in the device
	uint8_t result = setDeviceValue(addr, value, reg);

	// reply back to the host
	uint8_t length = STD_REPLY_LENGTH;
	message[OFFSET_LENGTH] = length; // length of reply in bytes
	message[OFFSET_TYPE] = TYPE_REPLY;
	message[OFFSET_RESULT] = result; // success (0) or failed (non-zero)
	send_reply(message, length);
}

// Receive a message from the host
// @param message - the buffer to copy the message to
// @return RECV_REQ_SUCCESS = 0: successfully received message
// 		   RECV_REQ_ERROR = 1: error in receiving the message
//         MSG_NOT_SENT_YET = 2: haven't finished receiving the message yet
int recv_request(uint8_t* message) {
	// Has a message finished sending yet? (recv buffer is locked)
	if (buffLocked == TRUE) {
		// yes, copy the message, unlock the buffer, and return success
		memcpy((uint8_t*)message, (uint8_t*)sentMessage,
				((int) sentMessage[0]) + 1);
		buffLocked = FALSE;
		return RECV_REQ_SUCCESS;
	}
	else {
		return MSG_NOT_SENT_YET;
	}
}

// Reply back to host:
int send_reply(uint8_t* message, int replyLength) {
	int bytesLeft = replyLength + 1;
	char c;
	// send the message back to the host via Uart:
	while (bytesLeft-- > 0) {
		c = *message++;
		if (IOputc(c, io_usb_out)) {
			// todo: handle error adding character to io buffer
			// obviously can't send a message...
		}
	}
	return 0; // success
}

/*
 * puts an integer into the message array (LITTLE ENDIAN) at
 * the position specified by offset
 */
void putUInt16(uint8_t* message, uint16_t num, int offset) {
	message[offset] = num & 0xff;
	message[offset+1] = (num>>8) & 0xff;
}

/*
 * gets an integer from the message array (LITTLE ENDIAN) at
 * the position specified by offset
 */
uint16_t getUInt16(uint8_t* message, int offset) {
	uint16_t result = 0xff & message[offset];
	result |= (0xff00 & (message[offset+1]<<8));
	return result;
}
