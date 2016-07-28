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

/******************************************************************************
 variables ********************************************************************
 *****************************************************************************/
#pragma PERSISTENT(myDeviceList)
DeviceList_t myDeviceList =
{ 0 };

#pragma PERSISTENT(fram_proj_key)
uint16_t fram_proj_key = 0;

static uint8_t proj_initialized = FALSE;

/******************************************************************************
 function prototypes and macros ***********************************************
 *****************************************************************************/

// return 1 if a berry has that address
#define addrIsUsed(addr) (myDeviceList.devices[addr].address > 0)

#define INT_EN_REG 	-4
#define INT_REG 	-5

static void reset_network();
static int check_addr(uint8_t addr);
static int find_all_new_devices();
static int find_new_device();
static uint8_t get_available_address();
static inline int get_device_type(uint8_t addr, uint8_t* value);

/******************************************************************************
 API functions ****************************************************************
 *****************************************************************************/

/*
 * Initialize master and berry network with project key
 */
int init_devices(uint16_t project_key)
{
	int error;
	proj_initialized = FALSE;

	// If the project key is different, reset the network and update the
	// project key
	if (project_key != fram_proj_key)
	{
		reset_network();
		fram_proj_key = project_key;
	}

	hal_check_proj_key(project_key);

	// Look for new devices on the network.
	if (error = find_all_new_devices())
	{
		char msg[40];
		sprintf(msg, "init_devices err%d", error);
		send_log_msg(msg, error_msg);
		return error;
	}

	// We're done.
	proj_initialized = TRUE;
	return SUCCESS; // success
}

/*
 * read count bytes from a berry, beginning at register reg
 * store the read data in buff
 */
int get_device_multi_values(uint8_t addr, int8_t reg, uint8_t* buff,
		uint8_t count)
{
	int error;
	if (error = check_addr(addr))
		return error;
	else
		return hal_getDeviceMultiRegs(addr, reg, buff, count);
}

/*
 * write count bytes from buffer buff to a berry, starting at register reg
 */
int set_device_multi_values(uint8_t addr, int8_t reg, uint8_t* buff,
		uint8_t count)
{
	int error;
	if (error = check_addr(addr))
		return error;
	else
		return hal_setDeviceMultiRegs(addr, reg, buff, count);
}

/*
 * Update the project key on the master and on the berries without changing
 * their addresses
 */
int update_proj_key(uint16_t new_proj_key)
{
	// update master's project key
	fram_proj_key = new_proj_key;

	// update project key on all berries
	return hal_update_proj_key(new_proj_key);
}

/*
 * Enable a berry interrupt
 */
int enable_interrupt(uint8_t addr, uint8_t int_type)
{
	int error;
	uint8_t int_reg;

	// Read the berry's interrupt register
	if (error = get_device_multi_values(addr, INT_EN_REG, &int_reg, 1))
		return error;

	// Set the int_type bit
	int_reg |= int_type;

	// Write it back
	if (error = set_device_multi_values(addr, INT_EN_REG, &int_reg, 1))
		return error;

	// Set interrupt enable flag on master so that it will be polled in the
	// vine interrupt event
	myDeviceList.devices[addr].int_en = TRUE;
	return SUCCESS;
}

/******************************************************************************
 Master functions *************************************************************
 *****************************************************************************/

/*
 * poll each berry to find which one interrupted
 * read its interrupt register - the berry will release the interrupt line
 * send the interrupt register to the host
 * repeat until interrupt line is released
 */
void vine_interrupt_event()
{
	// Static because we don't want to start over at 0 every time
	static int i = 0;

	int error;
	Device_t *list = myDeviceList.devices;

	// Loop while interrupt line is asserted (low)
	int loops = 0;
	while ((P1IN & BINT) == 0)
	{
		// Next index
		++i;
		if (i >= DEVICES_ARRAY_SIZE)
			i = 0;

		// Only read the berry's interrupt register if interrupts are enabled
		if (list[i].int_en && !list[i].missing)
		{
			uint8_t int_reg = 0;
			if ((error = get_device_multi_values(list[i].address, INT_REG,
					&int_reg, 1)) == SUCCESS)
			{
				// if the interrupt register is nonzero, that berry interrupted
				if (int_reg)
				{
					// send the interrupt register back to host
					interrupt_host(list[i].address, &int_reg, 1);
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
		// asserted, there's a problem.
		++loops;
		if (loops >= DEVICES_ARRAY_SIZE)
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
	static uint16_t curr_device = 0;
	static uint8_t num_events = 0;
	// If the initialization hasn't happened yet, return immediately
	if (!proj_initialized)
	{
		return;
	}
	// Every 4th call, check for new devices
	if ((num_events & 3) == 0)
	{
		uint8_t addr;
		hal_check_proj_key(fram_proj_key);
		if (addr = find_new_device())
		{
			// There's a new device! Interrupt the host with address and type
			// of new device
			uint8_t type = 0;
			int error;
			if (error = get_device_type(addr, &type))
			{
				// error - failed to get the type from the device
				myDeviceList.devices[addr].address = 0;
				char msg[40];
				sprintf(msg, "Err%d get new device type.", error);
				send_log_msg(msg, error_msg);
			}
			uint8_t buff[3] =
			{ INTR_TYPE_NEW_BERRY, addr, type };
			interrupt_host(INTR_SRC_MASTER, buff, 3);
		}
	}
	// 3 out of 4 calls, ping a device
	else
	{
		Device_t *device_list = myDeviceList.devices;
		uint8_t addr;
		// Grab the next address
		addr = device_list[curr_device].address;
		// Only ping if address is nonzero (there's actually a berry there)
		if (addr != 0)
		{
			// Ping the device
			if (hal_pingDevice(addr) != SUCCESS)
			{
				// Only interrupt if the device wasn't already missing
				if (!device_list[curr_device].missing)
				{
					// notify host of the missing berry and its address
					uint8_t buff[2] =
					{ INTR_TYPE_MISSING_BERRY, addr };
					interrupt_host(INTR_SRC_MASTER, buff, 2);
					device_list[curr_device].missing = 1;
				}
			}
			else
			{
				// It answered. If it was missing, then notify the host
				// that it's back again.
				if (device_list[curr_device].missing)
				{
					uint8_t buff[2] =
					{ INTR_TYPE_FOUND_BERRY, addr };
					interrupt_host(INTR_SRC_MASTER, buff, 2);
					device_list[curr_device].missing = 0;
				}
			}
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
	myDeviceList.size = 0;
	memset(myDeviceList.devices, 0, sizeof(myDeviceList.devices));
	hal_resetAllDevices();
}

/*
 * checks if address is valid and the berry is still connected
 */
static int check_addr(uint8_t addr)
{
	if (addr <= 0 || addr > MAX_NUM_DEVICES)
	{
		return INVALID_ADDR;
	}
	else if (!addrIsUsed(addr) || myDeviceList.devices[addr].missing)
	{
		return DEVICE_NOT_FOUND;
	}
	else
	{
		return SUCCESS;
	}
}

/*
 * Assign addresses to all new devices on the network
 */
static int find_all_new_devices()
{
	int error;
	while ((error = find_new_device()) > 0);
	return error;
}

/*
 * Assigns an address to a new device on the network
 * Returns the address (positive number) on success
 * Returns negative error code when there are no available addresses
 * Returns 0 if there was no new device
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
		myDeviceList.devices[addr].address = addr;
		myDeviceList.devices[addr].missing = 0;
		myDeviceList.size++;
		return addr;
	}
	else
		return 0; // no new device
}

/*
 * Return the lowest available address, or 0 if there are none available
 */
static uint8_t get_available_address()
{
	int i = 1;
	while (addrIsUsed(i) && i < DEVICES_ARRAY_SIZE)
		++i;
	if (i >= DEVICES_ARRAY_SIZE)
		return 0;
	else
		return i;
}

/*
 * Wrapper to make code cleaner
 */
static inline int get_device_type(uint8_t addr, uint8_t* value)
{
#define REG_TYPE 0
	return get_device_multi_values(addr, REG_TYPE, value, 1);
}

