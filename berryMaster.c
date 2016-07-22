/*
 * BerryMaster.c
 *
 *  Created on: Dec 3, 2015
 *      Author: Marshall Garey
 */

#include <string.h>
#include "berryMaster.h"
#include "hal.h"
#include "server.h"
#include <stdio.h>

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
/* addrIsUsed
 * checks if a device is currently configured at the specified address
 * addr - the address to check
 * return 1 if a device is configured at the address; 0 otherwise
 */
#define addrIsUsed(addr) (myDeviceList.devices[addr].address > 0)

/* clearNetwork
 * resets device table and all connected devices
 */
static void clearNetwork();

/* check_addr
 * checks to make sure the address is okay and the device is still there
 */
static int check_addr(uint8_t addr);

/* find_all_new_devices
 * while(1):
 *   get an available address
 *   call hal_getNewDevice(addr)
 *     if a device grabbed the address (hal_getNewDevice returned true),
 *       initialize that device in the network
 *     else return
 * return 0 if successful, non-zero if failed
 */
static int find_all_new_devices();

/* find_new_device
 * Looks for a new device
 * Returns the address
 * If the address is 0, then there was no new device
 */
static int find_new_device();

/* findUnusedAddress
 * finds an unused address in the network
 * return the lowest num unused address in the network;
 *     0 if there are no open addresses
 */
static uint8_t get_available_address();

/* getDeviceTypeFromHal
 * addr - the address of the device on the network
 * value - store the type here
 * return 0 on success, nonzero otherwise
 */
static inline int getDeviceTypeFromHal(uint8_t addr, uint8_t* value);

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
		clearNetwork();
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

/* getDeviceMultiValues
 * gets multiple bytes from the requested berry
 * addr - address of the device
 * reg - the register number where we begin to read
 * buff - pointer to store the values
 * count - number of bytes to read
 */
int get_device_multi_values(uint8_t addr, uint8_t reg, uint8_t* buff,
		uint8_t count)
{
	int error;
	if (error = check_addr(addr))
	{
		return error;
	}
	// valid device
	else
	{
		return hal_getDeviceMultiRegs(addr, reg, buff, count);
	}
}

/* setDeviceValue
 * sets the device's value to the specified value
 * addr - the address of the device
 * reg - the register to write
 * buff - the buffer pointer to values
 * count - number of values to write out from buffer
 * return SUCCESS if successful, non-zero if failed
 */
int set_device_multi_values(uint8_t addr, uint8_t reg, uint8_t* buff,
		uint8_t count)
{
	int error;
	if (error = check_addr(addr))
	{
		return error;
	}
	else
	{
		return hal_setDeviceMultiRegs(addr, reg, buff, count);
	}
}

/* update_proj_key
 * Updates the project key on the master and on the berries without changing
 * their addresses
 */
int update_proj_key(uint16_t new_proj_key)
{
	// update master's project key
	fram_proj_key = new_proj_key;

	// update project key on all berries
	return hal_update_proj_key(new_proj_key);
}

/******************************************************************************
 Master functions *************************************************************
 *****************************************************************************/

/*
 * poll each berry to find which one interrupted
 * read its interrupt register - the berry will release the interrupt line
 * send to host
 * repeat until interrupt line is released
 */
void vine_interrupt_event()
{

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
			if (error = getDeviceTypeFromHal(addr, &type))
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

/* send_log_msg
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

/* clearNetwork
 * resets the device table
 * resets all devices
 */
static void clearNetwork()
{
	myDeviceList.currNumDevices = 0;
	memset(myDeviceList.devices, 0, sizeof(myDeviceList.devices));
	hal_resetAllDevices();
}

/* check_addr
 * checks to make sure the address is okay and the device is still there
 */
static int check_addr(uint8_t addr)
{
	// Is it a valid address?
	if (addr <= 0 || addr > MAX_NUM_DEVICES)
	{
		return INVALID_ADDR;
	}
	// Is there a device at this address?
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
//	uint8_t addr;
//	uint8_t type;
//
//	while (1)
//	{
//		addr = findUnusedAddress();
//		if (addr == 0)
//		{
//			// error - full network
//			// todo: Note - If we have exactly the
//			// maximum number of allowed devices then this will return an
//			// error. Should we say there can only be one less device (126)
//			// than the maximum number of available addresses (127)?
//			return NETWORK_FULL;
//		}
//		else if (!(hal_discoverNewDevice(addr)))
//		{
//			myDeviceList.devices[addr].address = addr;
//			if (error = getDeviceTypeFromHal(addr, &type))
//			{
//				// error - failed to get the type from the device
//				myDeviceList.devices[addr].type = UNKNOWN;
//				myDeviceList.devices[addr].address = 0;
//				return error;
//			}
//			myDeviceList.devices[addr].type = type;
//			myDeviceList.devices[addr].missing = 0;
//			myDeviceList.currNumDevices++;
//		}
//		else
//			return SUCCESS; // no devices grabbed the address - we're done
//	}
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
		myDeviceList.currNumDevices++;
		return addr;
	}
	else
		return 0; // no new device
}

/*
 * return the lowest available address
 *     0 if there are no open addresses
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

/* getDeviceTypeFromHal
 * addr - the address of the device on the network
 * value - store the type here
 * return 0 on success, nonzero otherwise
 */
static inline int getDeviceTypeFromHal(uint8_t addr, uint8_t* value)
{
#define REG_TYPE 0
	return get_device_multi_values(addr, REG_TYPE, value, 1);
}

