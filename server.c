/*
 * rpcHandlers.c
 *
 *  Created on: Dec 3, 2015
 *      Authors:
 *      Marshall Garey
 *      Broderick Gardner
 */

// Standard header files
#include <msp430.h>
#include <stdio.h>

// Local header files
#include "server.h"
#include "IObuffer.h"
#include "ft201x.h"
#include "berryMaster.h"

// BIG TODO: handle IOputc errors

// ----------------------------------------------------------------------------
// Global variables -----------------------------------------------------------
// ----------------------------------------------------------------------------
IObuffer* server_buffer;
IObuffer* server_slot;
uint8_t msg_id;

// ----------------------------------------------------------------------------
// Function prototypes --------------------------------------------------------
// ----------------------------------------------------------------------------

static int isValidMessage();
static void rpc_initDevices();
static void rpc_getDeviceMultiValues();
static void rpc_setDeviceMultiValues();
static void rpc_setProjectKey();
static void setReply(uint8_t replyLength, uint8_t result,
		uint8_t* buff, uint8_t count);
static void rpc_enableInterrupt();

// ----------------------------------------------------------------------------
// Macros ---------------------------------------------------------------------
// ----------------------------------------------------------------------------
#define READ(dest) IOgetc((char*)&(dest), server_buffer)
#define WRITE(src) IOputc((char)(src),server_slot);
#define WRITE_S(src) IOputs((const char*)(src),server_slot)
#define WRITE_N(src,n) IOnputs((const char*)(src),(n),server_slot)

// ----------------------------------------------------------------------------
// defined constants ----------------------------------------------------------
// ----------------------------------------------------------------------------

//	Protocol:
//  Host->Master:
//  1 byte|1 byte|1 byte|any length up to MAX_MSG_LENGTH
//	length|opcode|msg id|parameters...
//  Notes:
//    The address will always be the first parameter
//
//  Master->Host:
//  1 byte|1 byte  |1 byte|any length up to MAX_MSG_LENGTH
//	length|msg type|msg id|parameters...


// length of reply without parameters
#define STD_REPLY_LENGTH 3 // msg_type + msg_id + result = 3 bytes

// opcodes
#define OP_INIT_DEVICES 0
#define OP_GET_MUL_VALS 4
#define OP_SET_MUL_VALS 5
#define OP_SET_PROJ_KEY 6
#define OP_EN_INTERRUPT 7

// ----------------------------------------------------------------------------
// Function definitions -------------------------------------------------------
// ----------------------------------------------------------------------------

void server_init()
{
	//Allocate buffer.  This function must be called before assigning any slots
	//	to server buffer.  Clearly.
	//Size is defined, in priority order, by
	//	any define before iomatrix.h, by iomatrix.h and by server.h
	server_buffer = IObuffer_create(SERVER_BUF_SIZE);
	if (server_buffer == 0)
	{
		while (1)
			;	//Report error somehow
	}
}

/*
 * Services a pending request from the host.
 */
int server_event()
{
	uint8_t opcode;
	int error;

	// get the message:
	if (error = isValidMessage())
	{
		char msg[40];
		sprintf(msg, "err%d @top of server_ev", error);
		send_log_msg(msg, error_msg);
		return error;
	}

	// get the opcode from the message
	if (error = READ(opcode))
	{
		// TODO: handle this better
		return error;
	}

	// get the message id
	if (error = READ(msg_id))
	{
		return error;
	}

	// handle the request:
	switch (opcode)
	{
	case OP_INIT_DEVICES:
		// Initialize the network
		rpc_initDevices();
		break;
	case OP_GET_MUL_VALS:
		// Get specified register values in the berry.
		rpc_getDeviceMultiValues();
		break;
	case OP_SET_MUL_VALS:
		// Set multiple registers in the berry.
		rpc_setDeviceMultiValues();
		break;
	case OP_SET_PROJ_KEY:
		// Update the project key
		rpc_setProjectKey();
		break;
	case OP_EN_INTERRUPT:
		// Enable an interrupt on the berry
		rpc_enableInterrupt();
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
int isValidMessage()
{
	int error;
	static uint8_t length;

	// Get the length of the incoming message.
	if (!length && (error = READ(length)))
	{
		return error;
	}

	// Make sure we have the full message. If we don't, return an error.
	if (server_buffer->count < length)
	{
		// Don't have the full message, return error.
		return -2;
	}
	length = 0;
	// We have a full message.
	return SUCCESS;
}

/*
 * calls master initDevices function and sends a reply back to the host
 */
void rpc_initDevices()
{
	// Get the project key from the message
	uint16_t project_key;
	uint8_t lo, hi, hot_swap_en;
	READ(lo);
	READ(hi);
	READ(hot_swap_en);
	project_key = lo | (hi << 8);

	// Call init devices
	uint8_t result = init_devices(project_key, hot_swap_en);

	// Put reply in output buffer.
	setReply(STD_REPLY_LENGTH, result, NULL, NULL);
}

void rpc_getDeviceMultiValues()
{
	// Get the address from the message
	uint8_t addr;
	READ(addr);

	// Get the register from the message
	uint8_t reg;
	READ(reg);

	// Number of bytes to read
	uint8_t count;
	READ(count);

	// Read from the berry
	uint8_t buff[25];
	int8_t result;
	if (count > 25)
	{
		result = READ_BYTES_OVERFLOW; // return error
		int i;
		for (i = 0; i < 25; i++)
			buff[i] = 0xff;
	}
	else
	{
		result = get_device_multi_values(addr, reg, buff, count);
		buff[count] = 0;
	}

	// Put reply in output buffer.
	setReply(STD_REPLY_LENGTH + count, result, buff, count);
}

void rpc_setDeviceMultiValues()
{
	// Get the address from the message
	uint8_t addr;
	READ(addr);

	// Get the register from the message
	uint8_t reg;
	READ(reg);

	// Get count of values to set from message
	uint8_t count;
	READ(count);

	// Get the value from the message
	uint8_t value_buff[25];
	int8_t result;

	if (count > 25)
	{
		result = SET_BYTES_OVERFLOW; // return error
	}
	else
	{
		int i;
		for (i = 0; i < count; i++)
			READ(value_buff[i]);
		result = set_device_multi_values(addr, reg, value_buff, count);
	}

	// Put reply in output buffer.
	setReply(STD_REPLY_LENGTH, result, NULL, NULL);
}

static void rpc_setProjectKey()
{
	// Get the project key from the message - 2 bytes
	uint16_t new_proj_key;
	uint16_t lo, hi;
	READ(lo);
	READ(hi);
	new_proj_key = (hi << 8) | lo;

	// Update the project key
	uint8_t result = update_proj_key(new_proj_key);

	// Put reply in output buffer
	setReply(STD_REPLY_LENGTH, result, NULL, NULL);
}

static void rpc_enableInterrupt()
{
	uint8_t addr;
	READ(addr);

	// Interrupt type
	uint8_t int_type;
	READ(int_type);

	// Enable the interrupt on the berry
	uint8_t result = enable_interrupt(addr, int_type);

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
 * id - the unique reply id
 * result - result of the reply
 * buff - additional data to send to the host
 */
void setReply(uint8_t replyLength, uint8_t result,
		uint8_t* buff, uint8_t count)
{
	// Standard reply data: length, type, id, and result
	WRITE(replyLength);
	WRITE(MSG_REPLY);
	WRITE(msg_id);
	WRITE(result);

	if (buff)
		WRITE_N(buff, count);
	return;
}

/*
 * Sends a message to the host of type interrupt
 * count is the size of buff in bytes
 */
void interrupt_host(uint8_t intr_source, uint8_t *buff, uint8_t count)
{
	WRITE(count + 2); // length = count + 1 (MSG_INT) + 1 (intr source)
	WRITE(MSG_INTERRUPT); // msg type
	WRITE(intr_source); // interrupt source (0 for master, nonzero for berry)

	// any additional information from the master or berry
	if (buff)
	{
		WRITE_N(buff, count);
	}
}
