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

// Local headers
#include "berryMaster.h"
#include "hal.h"
#include "server.h"
#include "timer.h"

/******************************************************************************
 Macros ***********************************************************************
 *****************************************************************************/

// Maximum number of devices allowed on the network - limited to 127 because
// the vine uses 7 address bits, no device will be allowed to use address 0.
#define MAX_NUM_DEVICES 32u
#define MAX_I2C_ADDR	127
#define INT_EN_REG 		-4
#define INT_REG 		-5

/******************************************************************************
 Variables ********************************************************************
 *****************************************************************************/

// Berry struct
typedef struct Device
{
	uint8_t i2c_addr;	// vine i2c address
	uint8_t int_en;		// interrupt enable
} Device_t;

// Berry network. The index is the i2c address.
static Device_t berry_list[MAX_NUM_DEVICES] =
{ 0 };

static uint8_t used_i2c_addresses[MAX_NUM_DEVICES/8];

static uint8_t hot_swapping_enabled = FALSE;

/******************************************************************************
 Function prototypes **********************************************************
 *****************************************************************************/
static void reset_network();
static int check_addr(uint8_t addr);
static int find_all_new_devices();
static int find_new_device();
static uint8_t get_available_address();
static void add_berry(uint8_t addr);
static void remove_berry(uint8_t dev_num);
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
	if (error = check_addr(dev_num))
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
	if (error = check_addr(dev_num))
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
	static uint8_t curr_device = 0;
	static uint16_t num_events = 0;

	if (!hot_swapping_enabled)
	{
		return;
	}
	// Every 4th call, check for a new device.
	if ((num_events & 3) == 0)
	{
		uint8_t addr;
		if (addr = find_new_device())
		{
			// Interrupt the host with address of new device
			uint8_t buff[2] =
			{ INTR_TYPE_NEW_BERRY, addr};
			interrupt_host(INTR_SRC_MASTER, buff, 2);
		}
	}
	// 3 out of 4 calls, ping a device.
	else
	{
		uint8_t addr = berry_list[curr_device].i2c_addr;
		uint8_t dev_num = curr_device;

		// Loop until (1) we've found a berry (nonzero i2c address), or
		// (2) we've looped through every device (we don't want an infinite loop).
		// If (2) happens, then there are no berries connected.
		while ((addr == 0) && ((dev_num + 1) != curr_device))
		{
			++dev_num;
			if (dev_num > MAX_NUM_DEVICES)
			{
				dev_num = 0;
			}
			addr = berry_list[dev_num].i2c_addr;
		}
		curr_device = dev_num;

		// Return if we didn't find a berry.
		if (addr == 0)
			return;

		// Otherwise, ping the berry.
		if (hal_pingDevice(curr_device) != SUCCESS)
		{
			// No answer - notify host of the missing berry.
			uint8_t buff[2] =
			{
					INTR_TYPE_MISSING_BERRY,
					curr_device
			};
			interrupt_host(INTR_SRC_MASTER, buff, 2);

			// Delete the berry from the list.
			remove_berry(curr_device);
		}

		// Next device index
		++curr_device;
		if (curr_device > MAX_NUM_DEVICES)
		{
			curr_device = 1;
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
 * clear the device table and reset all connected berries
 */
static void reset_network()
{
	memset(berry_list, 0, sizeof(berry_list));
	hal_resetAllDevices();
}

/*
 * checks if address is valid and the berry is still connected
 */
static int check_addr(uint8_t dev_num)
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
 * Add a berry to the list.
 * Parameter addr is its i2c address.
 * We need to give it a device number (its index in the list). Find an index
 * where the i2c address is zero - that entry is empty, so we can add it there.
 */
static void add_berry(uint8_t addr)
{
	uint8_t i;
	for (i = 0; i < MAX_NUM_DEVICES; i++)
	{
		if (berry_list[i].i2c_addr == 0)
		{
			berry_list[i].i2c_addr = addr;
			berry_list[i].int_en = 0;

			// Add the address to the used i2c addresses.
			uint8_t index = addr >> 3;			// /8
			uint8_t bit = 1 << (addr & 0xff);	// %8
			used_i2c_addresses[index] |= bit;
		}
	}
}

/*
 * Remove a berry from the list.
 * Parameter dev_num is the berry device number (its index in the list).
 */
static void remove_berry(uint8_t dev_num)
{
	// Remove the address to the used i2c addresses.
	uint8_t addr = berry_list[dev_num].i2c_addr;
	uint8_t index = addr >> 3;			// /8
	uint8_t bit = 1 << (addr & 0xff);	// %8
	used_i2c_addresses[index] |= bit;

	// Remove the berry from the list.
	berry_list[dev_num].i2c_addr = 0;
	berry_list[dev_num].int_en = 0;
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
 * Returns the address (positive number) on success.
 * Returns negative error code when there are no available addresses.
 * Returns 0 if there was no new device.
 */
static int find_new_device()
{
	uint8_t addr;
	addr = get_available_address();
	if (addr == 0)
	{
		return NETWORK_FULL;
	}
	// Offer a new address to an open device.
	else if (!(hal_discoverNewDevice(addr)))
	{
		// Only add the berry if it responds to pinging.
		if (ping_berry(addr) == SUCCESS)
		{
			// The berry responded. Add it to the list and return.
			add_berry(addr);
			return addr;
		}
	}

	return 0; // no new device
}

/*
 * Return an available vine i2c address, or 0 if there are none available.
 * TODO: don't get the lowest available address. Change this (talk to Kristian -
 * he had a good idea).
 */
static uint8_t get_available_address()
{
	static uint8_t next_i2c_addr = 1;

	uint8_t loops;
	for (loops = 0; loops < MAX_I2C_ADDR; ++loops)
	{
		// Check if we've already used the next_i2c_addr.
		uint8_t index = next_i2c_addr >> 3;			// /8
		uint8_t bit = 1 << (next_i2c_addr & 0xff);	// %8
		if (used_i2c_addresses[index] & bit)
		{
			// This i2c address has been used, pick another one.
			++next_i2c_addr;
			if (next_i2c_addr == MAX_I2C_ADDR)
			{
				next_i2c_addr = 1;
			}
		}
		else
		{
			// Return a valid and unused i2c address.
			return next_i2c_addr;
		}
	}

	// There are no more valid i2c addresses.
	return 0;
}
