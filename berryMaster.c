/*
 * berryMaster.c
 *
 *  Created on: Dec 3, 2015
 *      Author: Marshall Garey
 */

// Standard headers
#include <string.h>
#include <stdio.h>
#include <msp430.h>
#include <stdlib.h>

// Local headers
#include "berryMaster.h"
#include "hal.h"
#include "server.h"
#include "timer.h"

/******************************************************************************
 Macros ***********************************************************************
 *****************************************************************************/

#define MAX_NUM_DEVICES 32u
#define MAX_I2C_ADDR	127
#define INT_EN_REG 		-9
#define INT_REG 		-10

/******************************************************************************
 Variables ********************************************************************
 *****************************************************************************/

// Berry struct
typedef struct Berry
{
	uint8_t i2c_addr;	// vine i2c address
	uint8_t int_en;		// interrupt enable
} Berry_t;

// Berry network. The index is the i2c address.
static Berry_t berry_list[MAX_NUM_DEVICES] =
{ 0 };

// Self-explanatory
static uint8_t num_connected_berries = 0;

// Bit array - each bit represents whether or not the associated i2c address
// is taken or not.
static uint8_t used_i2c_addresses[(MAX_I2C_ADDR+1)/8];

// Self-explanatory
static uint8_t hot_swapping_enabled = FALSE;

/******************************************************************************
 Function prototypes **********************************************************
 *****************************************************************************/
static void reset_network();
static int is_valid_dev_num(uint8_t addr);
static int find_all_new_devices();
static int find_new_device();
static uint8_t get_available_address();
static int add_berry(uint8_t i2c_addr);
static int remove_berry(uint8_t device_number);
static int ping_berry(uint8_t i2c_addr);

/******************************************************************************
 API functions ****************************************************************
 *****************************************************************************/

/*
 * Initialize master and berry network with project key
 * TODO: think about / discuss with Broderick & Kristian what exactly
 * this function should do
 */
int init_devices(uint8_t hot_swap_en)
{
	int error;

	// Disable hot-swapping while initializing the berry network.
	hot_swapping_enabled = FALSE;

	reset_network();

	// Look for new devices on the network.
	if (error = find_all_new_devices())
	{
		char msg[40];
		sprintf(msg, "init_devices err%d", error);
		send_log_msg(msg, error_msg);
		return error;
	}

	// Conditionally enable hot-swapping
	hot_swapping_enabled = hot_swap_en;

	// We're done.
	return SUCCESS; // success
}

/*
 * read count bytes from a berry, beginning at register reg
 * store the read data in buff
 */
int get_device_multi_values(uint8_t dev_num, int8_t reg, uint8_t* buff,
		uint8_t count)
{
	int error;
	if (error = is_valid_dev_num(dev_num))
		return error;
	else
		return hal_getDeviceMultiRegs(berry_list[dev_num].i2c_addr, reg, buff, count);
}

/*
 * write count bytes from buffer buff to a berry, starting at register reg
 */
int set_device_multi_values(uint8_t dev_num, int8_t reg, uint8_t* buff,
		uint8_t count)
{
	int error;
	if (error = is_valid_dev_num(dev_num))
		return error;
	else
		return hal_setDeviceMultiRegs(berry_list[dev_num].i2c_addr, reg, buff, count);
}

/*
 * Enable a berry interrupt
 */
int enable_interrupt(uint8_t dev_num, uint8_t int_type)
{
	int error;
	uint8_t int_reg;

	// Read the berry's interrupt register
	if (error = get_device_multi_values(dev_num, INT_EN_REG, &int_reg, 1))
		return error;

	// Set the int_type bit
	int_reg |= int_type;

	// Write it back
	if (error = set_device_multi_values(dev_num, INT_EN_REG, &int_reg, 1))
		return error;

	// Set interrupt enable flag on master so that it will be polled in the
	// vine interrupt event
	berry_list[dev_num].int_en = TRUE;
	return SUCCESS;
}

/******************************************************************************
 Master functions *************************************************************
 *****************************************************************************/

/*
 * Poll each berry to find which one interrupted.
 * Read its interrupt register - the berry will release the interrupt line.
 * Send the interrupt register to the host.
 * Repeat until interrupt line is released.
 */
void vine_interrupt_event()
{
	int error;

	// Static because we don't want to start over at 0 every time.
	// i is the index into the berry list (which is the device number).
	static int i = 0;

	// Loop while interrupt line is asserted (low)
	int loops = 0;
	while ((P1IN & BINT) == 0)
	{
		// Next index
		++i;
		if (i >= MAX_NUM_DEVICES)
			i = 0;

		// Only read the berry's interrupt register if interrupts are enabled
		if (berry_list[i].int_en)
		{
			uint8_t int_reg = 0;
			if ((error = get_device_multi_values(i, INT_REG,
					&int_reg, 1)) == SUCCESS)
			{
				// if the interrupt register is nonzero, that berry interrupted
				if (int_reg)
				{
					// send the interrupt register back to host
					interrupt_host(i, &int_reg, 1);
					return;
				}
			}
			else
			{
				// report the error
				char msg[40];
				sprintf(msg, "err%d vine_int_ev read int_reg", error);
				send_log_msg(msg, error_msg);
				return;
			}
		}

		// If we've checked every berry and the interrupt line is still
		// asserted, there might be a problem.
		++loops;
		if (loops >= MAX_NUM_DEVICES)
		{
			send_log_msg("vine intr still asserted", warning_msg);
			return;
		}
	}
}

/*
 * successively pings each device and looks for new devices on the network
 * interrupts the host if a device is missing or if a new device was plugged in
 */
void hot_swap_event()
{
	static uint8_t curr_device_num = 0;
	static uint16_t num_events = 0;

	if (!hot_swapping_enabled)
	{
		return;
	}
	// Every 4th call, check for a new device.
	if ((num_events & 3) == 0)
	{

//		return;

		int device_number = find_new_device();
		if (device_number < 0)
		{
			// Found a berry, but had an error adding it (network full)
			char msg[25];
			sprintf(msg, "Err%d adding berry", device_number);
			send_log_msg(msg, error_msg);
		}
	}
	// 3 out of 4 calls, ping a device.
	else
	{
		uint8_t addr = berry_list[curr_device_num].i2c_addr;

		// Ping if it's a berry.
		if (addr != 0)
		{
			if (ping_berry(addr) != SUCCESS)
			{
				// No answer - delete the berry and notify the host.
				remove_berry(curr_device_num);
			}
		}

		// Next device index
		++curr_device_num;
		if (curr_device_num > MAX_NUM_DEVICES)
		{
			curr_device_num = 0;
		}
	}

	// keep track of how many times this function has been called;
	// rollover is okay
	++num_events;
}

/*
 * Sends a null-terminated error, log, or warning message to the host
 */
void send_log_msg(char *msg, enum log_type_e log_type)
{
	char *ptr = msg;
	int n = 0;
	while (*ptr != NULL)
	{
		ptr++;
		n++;
	}
	n++; // add one for null terminating byte
	IOputc((char)n+1, log_slot); // length (add one for the type byte)
	switch(log_type) // type
	{
	case error_msg:
		IOputc((char)MSG_ERROR, log_slot);
		break;
	case log_msg:
		IOputc((char)MSG_LOG, log_slot);
		break;
	case warning_msg:
		/* fall through */
	default:
		IOputc((char)MSG_WRN, log_slot);
		break;
	}
	IOnputs(msg, n, log_slot); // message
}

/******************************************************************************
 Local helper functions *******************************************************
 *****************************************************************************/

/*
 * Clear the device table and reset all connected berries
 */
static void reset_network()
{
	memset(berry_list, 0, sizeof(berry_list));
	hal_resetAllDevices();
	num_connected_berries = 0;
	memset(used_i2c_addresses, 0, sizeof(used_i2c_addresses));
}

/*
 * 1. Checks that dev_num is valid.
 * 2. Checks if a berry has the associated device number.
 * Return 0 if both checks pass, negative error code otherwise.
 */
static int is_valid_dev_num(uint8_t dev_num)
{
	if (dev_num > MAX_NUM_DEVICES)
	{
		return INVALID_ADDR;
	}
	else if (berry_list[dev_num].i2c_addr == 0)
	{
		return DEVICE_NOT_FOUND;
	}
	else
	{
		return SUCCESS;
	}
}

/*
 * Add a berry to the list and notify the host.
 * Parameter i2c_addr is its i2c address.
 * We need to give it a device number (its index in the list). Find an index
 * where the i2c address is zero - that entry is empty, so we can add it there.
 * Return the positive device number if the berry was added,
 * a negative error code otherwise.
 */
static int add_berry(uint8_t i2c_addr)
{
	uint8_t device_number;
	for (device_number = 0; device_number < MAX_NUM_DEVICES; device_number++)
	{
		if (berry_list[device_number].i2c_addr == 0)
		{
			berry_list[device_number].i2c_addr = i2c_addr;
			berry_list[device_number].int_en = 0;
			++num_connected_berries;

			// Add the address to the used i2c addresses.
			uint8_t index = i2c_addr >> 3;			// /8
			uint8_t bit = 1 << (i2c_addr & 0x07);	// %8
			used_i2c_addresses[index] |= bit;

			// Notify the host of the new berry.
			uint8_t buff[2] = { INTR_TYPE_NEW_BERRY, device_number};
			interrupt_host(INTR_SRC_MASTER, buff, 2);
			return device_number;
		}
	}
	return NETWORK_FULL;
}

/*
 * Remove a berry from the list and notify the host.
 * Parameter dev_num is the berry device number (its index in the list).
 * Return 0 if successful, a negative error code if there was no berry
 * with the specified device number.
 */
static int remove_berry(uint8_t device_number)
{
	// Check if the berry exists (it should, since this is an internal
	// function, assuming we're using this correctly).
	uint8_t addr = berry_list[device_number].i2c_addr;
	if (addr == 0)
	{
		send_log_msg("Err in remove_berry", error_msg);
		return DEVICE_NOT_FOUND;
	}

	// Remove the address from the used i2c addresses list.
	uint8_t index = addr >> 3;			// /8
	uint8_t bit = 1 << (addr & 0x07);	// %8
	used_i2c_addresses[index] &= ~bit;

	// Remove the berry from the list.
	berry_list[device_number].i2c_addr = 0;
	berry_list[device_number].int_en = 0;
	--num_connected_berries;

	// Notify the host with an interrupt.
	uint8_t buff[2] = { INTR_TYPE_MISSING_BERRY, device_number };
	interrupt_host(INTR_SRC_MASTER, buff, 2);
	return SUCCESS;
}

/*
 * Ping a berry at the specified i2c address.
 * This tries up to 3 times with a ~2 ms delay between pings.
 * Return 0 if the berry responds, negative error code if it doesn't.
 */
static int ping_berry(uint8_t i2c_addr)
{
	int tries = 3; // number of times to ping the berry until we give up.
	for (tries = 3; tries > 0; tries--)
	{
		if (hal_pingDevice(i2c_addr) == 0)
			return 0;

		// The berry didn't answer. Wait a few ms and try again.
		timer_delay_ms(2);
	}
	return DEVICE_NOT_FOUND;
}

/*
 * Assign addresses to all new devices on the network.
 * Iterate until find_new_device returns 0 (no more devices) or negative (error)
 */
static int find_all_new_devices()
{
	int error;
	while ((error = find_new_device()) > 0);
	return error;
}

/*
 * Assigns an address to a new device on the network.
 * Also adds the berry to the network list and notifies the host.
 * Returns the (positive) device number on success.
 * Returns negative error code when there are no available addresses.
 * Returns 0 if there was no new device.
 */
static int find_new_device()
{
	uint8_t addr;
//	return 0;
	addr = get_available_address();
	if (addr == 0)
	{
		return NETWORK_FULL;
	}
	// Offer a new address to an open device.
	else if (hal_discoverNewDevice(addr) == SUCCESS)
	{
		// Only add the berry if it responds to pinging.
		if (ping_berry(addr) == SUCCESS)
		{
			// The berry responded. Add it to the list and return.
			return add_berry(addr);
		}
	}

	return 0; // no new device
}

/*
 * Pick the next i2c address. Rather than assign them contiguously, we
 * assign them with a difference of primes (we start with 7, increment
 * by 11). Every possible address (1-127) will be used before repeating.
 */
static void increment_next_i2c_addr(uint8_t *next_i2c_addr)
{
	*next_i2c_addr += 11;
	if (*next_i2c_addr > MAX_I2C_ADDR)
	{
		*next_i2c_addr -= MAX_I2C_ADDR;
	}
}

/*
 * Return an available vine i2c address, or 0 if there are none available.
 */
static uint8_t get_available_address()
{
	static uint8_t next_i2c_addr = 7;
//	return next_i2c_addr;

	uint8_t loops;
	for (loops = 0; loops < MAX_I2C_ADDR; ++loops)
	{
		// Check if we've already used the next_i2c_addr.
		uint8_t index = next_i2c_addr >> 3;			// /8
		uint8_t bit = 1 << (next_i2c_addr & 0x07);	// %8
		if (used_i2c_addresses[index] & bit)
		{
			// This i2c address has been used, pick another one.
			increment_next_i2c_addr(&next_i2c_addr);
		}
		else
		{
			return next_i2c_addr;
		}
	}

	// There are no more valid i2c addresses.
	// TODO: When this happens, we want to make sure that this function isn't
	// called over and over again by hot_swap_event(); this function might
	// eat up all the processor time.
	return 0;
}
