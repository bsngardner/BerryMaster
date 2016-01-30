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
#include "pthreads.h"

// ----------------------------------------------------------------------------
// Global variables -----------------------------------------------------------
// ----------------------------------------------------------------------------

// The buffer to receive a message from the host.
uint8_t sentMessage[MAX_MSG_LENGTH];

// A signal for the sentMessage buffer - if FALSE, the buffer is open for
// writing; if TRUE, the buffer is locked and we have received a full message.
volatile int buffLocked = FALSE;

// The mutex for writing to or reading from the vine.
// TODO: eventually move this down to HAL code - the closer to the hardware,
// the better.
extern //pthread_mutex_t vineMutex;

// ----------------------------------------------------------------------------
// Function prototypes --------------------------------------------------------
// ----------------------------------------------------------------------------

// calls master initDevices function
// and sends a reply back to the host
void rpc_initDevices(uint8_t* message, uint8_t len);

// gets the address from the message
// calls master getDeviceType
// and sends reply back to host
void rpc_getDeviceType(uint8_t* message, uint8_t len);

void rpc_getDeviceValue(uint8_t* message, uint8_t len);

void rpc_getDeviceMultiValues(uint8_t* message, uint8_t len);

void rpc_setDeviceValue(uint8_t* message, uint8_t len);

int recv_request();
int send_reply(uint8_t* message, int size);

/*
 * puts an integer into the message array (LITTLE ENDIAN) at
 * the position specified by offset
 */
void putUInt16(uint8_t* message, uint16_t num, int offset);

/*
 * gets an integer from the message array (LITTLE ENDIAN) at
 * the position specified by offset
 */
uint16_t getUInt16(uint8_t* message, int offset);

// ----------------------------------------------------------------------------
// Functions ------------------------------------------------------------------
// ----------------------------------------------------------------------------
int serverEvent() {

	static uint8_t len;
	static uint8_t opcode;

	// get the message:
	if (recv_request() == MSG_NOT_SENT_YET) {
		// Error! Shouldn't service this event until the message has been rxed
		//reportError("recv_request err", RECV_REQ_ERROR, io_usb_out);
		handleError(); // todo: do this better
	}

	// get the length and opcode; todo: use len to error check.
	len = sentMessage[0];
	opcode = sentMessage[1];

	// handle the request:
	switch(opcode) {
	case OPCODE_INIT_DEVICES:
		// init the network (also sends a reply to the host)
		rpc_initDevices(sentMessage, len);
		break;
	case OPCODE_GET_DEVICE_TYPE:
		// get the device type (this also sends a reply to the host)
		rpc_getDeviceType(sentMessage, len);
		break;
	case OPCODE_GET_DEVICE_VALUE:
		// Get the specified register value in the berry.
		rpc_getDeviceValue(sentMessage, len);
		break;
	case OPCODE_GET_DEVICE_MULTI_VALUES:
		// Get specified register values in the berry.
		rpc_getDeviceMultiValues(sentMessage, len);
		break;
	case OPCODE_SET_DEVICE_VALUE:
		// Set the specified register value in the berry.
		rpc_setDeviceValue(sentMessage, len);
		break;
	default:
		break;
	}

	return 0;
}

// calls master initDevices function
// and sends a reply back to the host
void rpc_initDevices(uint8_t* message, uint8_t len) {
	// Call init devices
	//pthread_mutex_lock(&vineMutex);
	uint8_t result = initDevices();
	//pthread_mutex_unlock(&vineMutex);

	// Send reply
	const uint8_t length = STD_REPLY_LENGTH;
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
	//pthread_mutex_lock(&vineMutex);
	uint8_t result = getDeviceType(addr, &type);
	//pthread_mutex_unlock(&vineMutex);

	// reply back to host.
	const uint8_t length = STD_REPLY_LENGTH + 1;
	message[OFFSET_LENGTH] = length;
	message[OFFSET_TYPE] = TYPE_REPLY;
	message[OFFSET_RESULT] = result;
	message[OFFSET_REPLY_PARAMS] = type;
	send_reply(message, length);
}

void rpc_getDeviceValue(uint8_t* message, uint8_t len) {


	// get the address from the message
	uint8_t addr = message[OFFSET_ADDR];

	// get the register from the message
	uint8_t reg = message[OFFSET_ADDR + 1];

	// get the value from the device
	uint8_t value;
	//pthread_mutex_lock(&vineMutex);
	uint8_t result = getDeviceValue(addr, &value, reg);
	//pthread_mutex_unlock(&vineMutex);

	reportError("is value", value, io_usb_out);

	// reply back to the host.
	const uint8_t length = STD_REPLY_LENGTH + 1;
	message[OFFSET_LENGTH] = length; // length of reply in bytes
	message[OFFSET_TYPE] = TYPE_REPLY;
	message[OFFSET_RESULT] = result; // success (0) or failed (non-zero)
	message[OFFSET_REPLY_PARAMS] = value; // the value in the register
	send_reply(message, length);
}

void rpc_getDeviceMultiValues(uint8_t* message, uint8_t len) {
	// address of the berry
	uint8_t addr = message[OFFSET_ADDR];

	// register to start the read
	uint8_t reg = message[OFFSET_ADDR + 1];

	// number of bytes to read
	uint8_t count = message[OFFSET_ADDR + 2];

	// read from the berry
	uint8_t buff[4];
	//pthread_mutex_lock(&vineMutex);
	uint8_t result = getDeviceMultiValues(addr, reg, buff, count);
	//pthread_mutex_unlock(&vineMutex);

	// reply back to the host
	const uint8_t length = STD_REPLY_LENGTH + 4;
	message[OFFSET_LENGTH] = length;
	message[OFFSET_TYPE] = TYPE_REPLY;
	message[OFFSET_RESULT] = result;
	unsigned i;
	for (i = 0; i < 4; i++) {
		message[OFFSET_REPLY_PARAMS + i] = buff[i];
	}
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
	//pthread_mutex_lock(&vineMutex);
	uint8_t result = setDeviceValue(addr, value, reg);
	//pthread_mutex_unlock(&vineMutex);

	// reply back to the host
	const uint8_t length = STD_REPLY_LENGTH;
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
int recv_request() {
	// Has a message finished sending yet? (recv buffer is locked)
	if (buffLocked == TRUE) {
		// yes, return success
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
	// unlock the buffer so we can receive another message
	buffLocked = FALSE;
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
