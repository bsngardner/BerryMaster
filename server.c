/*
 * rpcHandlers.c
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

// Standard header files
#include <msp430.h>

// Local header files
#include "server.h"
#include "IObuffer.h"
#include "ft201x.h"
#include "berryMaster.h"

// BIG TODO: handle IOputc errors

// ----------------------------------------------------------------------------
// Global variables -----------------------------------------------------------
// ----------------------------------------------------------------------------
extern IObuffer* io_usb_in;
extern IObuffer* io_usb_out;

// ----------------------------------------------------------------------------
// Function prototypes --------------------------------------------------------
// ----------------------------------------------------------------------------

static int isValidMessage();
static void rpc_initDevices();
static void rpc_getDeviceType();
static void rpc_getDeviceValue();
static void rpc_getDeviceMultiValues();
static void rpc_setDeviceValue();
static void setReply(uint8_t replyLength, uint8_t result, uint8_t* buff,
		uint8_t count);

// ----------------------------------------------------------------------------
// Functions ------------------------------------------------------------------
// ----------------------------------------------------------------------------

/*
 * Services a pending request from the host.
 */
int serverEvent() {
	uint8_t opcode;
	int error;

	// get the message:
	if (error = isValidMessage()) {
		return error;
	}

	// get the opcode from the message
	if (error = IOgetc((char*)&opcode, io_usb_in)) {
		// TODO: handle this better
		return error;
	}

	// handle the request:
	switch(opcode) {
	case OPCODE_INIT_DEVICES:
		// init the network (also sends a reply to the host)
		rpc_initDevices();
		break;
	case OPCODE_GET_DEVICE_TYPE:
		// get the device type (this also sends a reply to the host)
		rpc_getDeviceType();
		break;
	case OPCODE_GET_DEVICE_VALUE:
		// Get the specified register value in the berry.
		rpc_getDeviceValue();
		break;
	case OPCODE_GET_DEVICE_MULTI_VALUES:
		// Get specified register values in the berry.
		rpc_getDeviceMultiValues();
		break;
	case OPCODE_SET_DEVICE_VALUE:
		// Set the specified register value in the berry.
		rpc_setDeviceValue();
		break;
	default:
		break;
	}

	return SUCCESS;
}

/*
 * Vaildates the message from the host - don't service if message is
 * invalid or incomplete.
 */
int isValidMessage() {
	int error;
	uint8_t length;

	// Get the length of the incoming message.
	if (error = IOgetc((char*)&length, io_usb_in)) {
		return error;
	}

	// Make sure we have the full message. If we don't, return an error.
	if (io_usb_in->count < length) {
		// Don't have the full message, return error.
		return -2;
	}

	// We have a full message.
	return SUCCESS;
}

/*
 * calls master initDevices function and sends a reply back to the host
 */
void rpc_initDevices() {

	// Call init devices
	uint8_t result = initDevices();

	// Put reply in output buffer.
	setReply(STD_REPLY_LENGTH, result, NULL, NULL);
}

// gets the address from the message
// calls master getDeviceType
// and sends reply back to host
void rpc_getDeviceType() {

	// Get the address from the message
	uint8_t addr;
	IOgetc((char*)&addr, io_usb_in);

	// Get the type of the berry
	uint8_t type[2];
	uint8_t result = getDeviceType(addr, type);
	type[1] = 0;

	// Put reply in output buffer.
	setReply(STD_REPLY_LENGTH + 1, result, type, 1);
}

void rpc_getDeviceValue() {

	// Get the address from the message
	uint8_t addr;
	IOgetc((char*)&addr, io_usb_in);

	// Get the register from the message
	uint8_t reg;
	IOgetc((char*)&reg, io_usb_in);

	// Get the value from the device
	uint8_t value[2];
	uint8_t result = getDeviceValue(addr, value, reg);
	value[1] = 0;

	// Put reply in output buffer.
	setReply(STD_REPLY_LENGTH + 1, result, value, 1);
}

void rpc_getDeviceMultiValues() {

	// Get the address from the message
	uint8_t addr;
	IOgetc((char*)&addr, io_usb_in);

	// Get the register from the message
	uint8_t reg;
	IOgetc((char*)&reg, io_usb_in);

	// Number of bytes to read
	uint8_t count;
	IOgetc((char*)&count, io_usb_in);

	// Read from the berry
	uint8_t buff[MAX_MSG_LENGTH];
	uint8_t result;
	if (count > MAX_MSG_LENGTH) {
		result = READ_BYTES_LENGTH_EXCEEDED; // return error
		int i;
		for (i = 0; i < MAX_MSG_LENGTH; i++)
			buff[i] = 0xff;
	}
	else {
		result = getDeviceMultiValues(addr, reg, buff, count);
		buff[count] = 0;
	}

	// Put reply in output buffer.
	setReply(STD_REPLY_LENGTH + count, result, buff, count);

//	IOputc(STD_REPLY_LENGTH + count, io_usb_out);
//	IOputc((uint8_t)TYPE_REPLY, io_usb_out);
//	IOputc(result, io_usb_out);
//	int i;
//	for (i = 0; i < count; i++) {
//		IOputc(buff[i], io_usb_out);
//	}
}

void rpc_setDeviceValue() {

	// Get the address from the message
	uint8_t addr;
	IOgetc((char*)&addr, io_usb_in);

	// Get the register from the message
	uint8_t reg;
	IOgetc((char*)&reg, io_usb_in);

	// Get the value from the message
	uint8_t value;
	IOgetc((char*)&value, io_usb_in);

	// Set the value in the device
	uint8_t result = setDeviceValue(addr, value, reg);

	// Put reply in output buffer.
	setReply(STD_REPLY_LENGTH, result, NULL, NULL);
}

/*
 * Places the reply in the output buffer.
 * A reply packet has the following form:
 * {length, type, result, [optional] values}
 *
 * Parameters:
 * replyLength - length of the reply
 * result - result of the reply
 * buff - additional data to send to the host
 */
void setReply(uint8_t replyLength, uint8_t result, uint8_t* buff,
		uint8_t count) {

	// Standard reply data: length, type, and result
	IOputc(replyLength, io_usb_out);
	IOputc((uint8_t)TYPE_REPLY, io_usb_out);
	IOputc(result, io_usb_out);

	// Addition values:
	if (buff) {
		int i;
		for (i = 0; i < count; i++) {
			IOputc(buff[i], io_usb_out);
		}
	}
}
