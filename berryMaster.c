/*
 * BerryMaster.c
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

#include <string.h>
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
/* addrIsUsed
 * checks if a device is currently configured at the specified address
 * @param addr the address to check
 * @return 1 if a device is configured at the address; 0 otherwise
 */
#define addrIsUsed(addr) (myDeviceList.devices[addr].deviceAddress > 0)

/* clearNetwork
 * resets device table and all connected devices
 */
static void clearNetwork();

/* check_addr
 * checks to make sure the address is okay and the device is still there
 */
static int check_addr(uint8_t addr);

/* validateDeviceList
 *  pings each device in the list
 *  if the device doesn't respond, take this device out of the list
 * @return SUCCESS if successful, non-zero if failed
 */
static int validateDeviceList();

/* find_all_new_devices
 * while(1):
 *   get an available address
 *   call hal_getNewDevice(addr)
 *     if a device grabbed the address (hal_getNewDevice returned true),
 *       initialize that device in the network
 *     else return
 * @return SUCCESS if successful, non-zero if failed
 */
static int find_all_new_devices();

/* find_new_device
 * Looks for a new device
 * Returns the address
 * If the address is 0, then there was no new device
 */
static int find_new_device();

/* findUnusedAddress
 * @return the iterator to an uncofigured device in the list
 */
static uint8_t findUnusedAddress();

/* getDeviceTypeFromHal(addr)
 * gets the device type from the hal
 * @param addr the address of the device on the network
 * @return type the type of the device;
 *         notADevice if the device is not on the network
 */
static inline int getDeviceTypeFromHal(uint8_t addr, uint8_t* value);

/******************************************************************************
 API functions ****************************************************************
 *****************************************************************************/
/* connectToMaster
 * here for the sake of portability of host code
 * just returns 0 for success
 */
int connect_to_master()
{
	return SUCCESS;
}

/* disconnectFromMaster
 * here for the sake of portability of host code
 * just returns 0 for success
 */
int disconnect_from_master()
{
	return SUCCESS;
}

/* init
 * project_hash - the hash of the project
 * initializes master:
 * calls hal_initDevices
 * validates current list of berries by pinging each one
 * discovers new devices and assigns them addresses
 * 	 iterates hal_getNewDevice(newDevAddr)
 */
int init_devices(uint16_t project_key)
{
	int error;
	proj_initialized = FALSE;

	// If project hash is different, clear the device network and
	// update the project hash.
	if (project_key != fram_proj_key)
	{
		clearNetwork();
		fram_proj_key = project_key;
		hal_check_proj_key(project_key);

		// Look for new devices on the network.
		if (error = find_all_new_devices())
			return error;
	}
	// Project key is the same. Check to see if the devices are still there.
	else
	{
		// Validate the device list we have.
		if (error = validateDeviceList())
			return error;
	}

	// We're done.
	proj_initialized = TRUE;
	return SUCCESS; // success
}

/* getDeviceMultiValues
 * gets multiple bytes from the requested berry
 * @param address - address of the device
 * @param reg - the register number where we begin to read
 * @param ret_val - pointer to store the value
 * @param count - number of bytes to read
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
 * @param the address of the device
 * @param the register to write
 * @param the buffer pointer to values
 * @param count - number of values to write out from buffer
 * @return SUCCESS if successful, non-zero if failed
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
 * @param the new project key
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
 * hot_swap_event
 * successively pings each device and looks for new devices on the network
 * interrupts the host if a device is missing or if a new device was plugged in
 */
void hot_swap_event()
{
	static uint16_t curr_device = 0;
	static uint8_t num_events = 0;
	// If the initialization hasn't happened yet, don't do this.
	if (!proj_initialized)
	{
		return;
	}
	// Every 4th call, check for new devices
	if ((num_events & 4) == 0)
	{
		uint8_t addr;
		hal_check_proj_key(fram_proj_key);
		if (addr = find_new_device())
		{
			// There's a new device! Interrupt the host with address and type
			// of new device
			uint8_t buff[3] =
			{ INTR_TYPE_NEW_BERRY, addr, myDeviceList.devices[addr].deviceType };
			interrupt_host(INTR_SRC_MASTER, buff, 3);
		}
	}
	// 3 out of 4 calls, ping a device
	else
	{
		Device_t *device_list = myDeviceList.devices;
		uint8_t addr;
		// Grab the next address
		addr = device_list[curr_device].deviceAddress;
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

	// debugging - send garbage interrupt to the host
	//uint8_t buff[2] =
	//{ 0xFE, 0xED };
	//interrupt_host(0, buff, 2);

	// debugging

	if (num_events & 4)
	{
		// search for the switch berry - get its address
		int i;
		uint8_t addr = 0;
		for (i = 0; i < DEVICES_ARRAY_SIZE; i++)
		{
			if (myDeviceList.devices[i].deviceType == 6)
			{
				addr = myDeviceList.devices[i].deviceAddress;
				break;
			}
		}
		if (addr)
		{
			// send interrupt to the host pretending the switch berry was pressed
			interrupt_host(addr, 0, 0);
		}
	}

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

/* validateDeviceList
 * iterates through the list of devices
 * if the device was configured (address is not zero), then ping it
 * if it doesn't respond, take it off the network list.
 * @return SUCCESS if successful, non-zero if failed
 */
static int validateDeviceList()
{
	int i = 0;
	int error;
	int addr = 1;
	int tempNumDevices = myDeviceList.currNumDevices;

	// First, send a general call to the berries to make sure they have
	// the same project hash
	hal_check_proj_key(fram_proj_key);

	// Second, iterate through the device table and ping each device
	// i keeps track of how many devices we've checked
	// addr iterates through the device array
	// exit the loop when
	//   (a) we've checked all devices previously on the network, or
	//   (b) we've iterated through the entire array of devices
	//		 (there's a problem if this happens)
	while (i < tempNumDevices && addr < DEVICES_ARRAY_SIZE)
	{
		// is the device currently configured?
		if (addrIsUsed(addr))
		{ // yes,
			i++;
			// ping the device to see if it's still there:
			error = hal_pingDevice(myDeviceList.devices[addr].deviceAddress);
			if (error)
			{
				// no device on this address
				// erase the device from the network list
				Device_t *device = &myDeviceList.devices[addr];
				device->deviceAddress = 0;
				device->deviceType = UNKNOWN;
				device->missing = 0;
				myDeviceList.currNumDevices--;
			}
			else
			{ // make sure that the missing flag is updated in the device table
				myDeviceList.devices[addr].missing = 0;
			}
		}
		addr++;
	}
	if (i < tempNumDevices)
	{
		// error - didn't check all previously connected berries
		return VALIDATE_LIST_ERR; // return failed
	}
	return SUCCESS; // success
}

/* find_all_new_devices
 * while(1):
 *   get an available address
 *   call hal_getNewDevice(addr)
 *     if a device grabbed the address (hal_discoverNewDevice returned 0),
 *       initialize that device in the network
 *     else return
 */
static int find_all_new_devices()
{
	int error;
	uint8_t addr;
	uint8_t type;

	while (1)
	{
		addr = findUnusedAddress();
		if (addr == 0)
		{
			// error - full network
			// todo: Note - If we have exactly the
			// maximum number of allowed devices then this will return an
			// error. Should we say there can only be one less device (126)
			// than the maximum number of available addresses (127)?
			return NETWORK_FULL;
		}
		// Offer a new address to an open device.
		else if (!(hal_discoverNewDevice(addr)))
		{
			// A new device grabbed the requested address!
			// Record the address of the device.
			myDeviceList.devices[addr].deviceAddress = addr;
			// get the type of the device and assign the type
			if (error = getDeviceTypeFromHal(addr, &type))
			{
				// error - failed to get the type from the device
				myDeviceList.devices[addr].deviceType = UNKNOWN;
				return error;
			}
			// Successfully got the type
			myDeviceList.devices[addr].deviceType = type;

			// Not missing
			myDeviceList.devices[addr].missing = 0;

			// Increment number of devices on the network.
			myDeviceList.currNumDevices++;
		}
		else
			return SUCCESS; // no devices grabbed the address - we're done
	}
}

/* find_new_device
 * Looks for a new device
 * Returns the address
 * If the address is 0, then there was no new device
 */
static int find_new_device()
{
	uint8_t addr, type;
	addr = findUnusedAddress();
	if (addr == 0)
	{
		return 0;
	}
	// Offer a new address to an open device.
	else if (!(hal_discoverNewDevice(addr)))
	{
		// A new device grabbed the requested address!
		// Record the address of the device.
		myDeviceList.devices[addr].deviceAddress = addr;

		// Get the type of the device and assign the type
		if (getDeviceTypeFromHal(addr, &type))
		{
			// error - failed to get the type from the device
			myDeviceList.devices[addr].deviceType = UNKNOWN;
		}
		else
		{
			// Successfully got the type
			myDeviceList.devices[addr].deviceType = type;
		}

		// Not missing
		myDeviceList.devices[addr].missing = 0;

		// Increment number of devices on the network.
		myDeviceList.currNumDevices++;
		return addr;
	}
	else
		return 0; // no new device
}

/* findUnusedAddress
 * finds an unused address in the network
 * @return the lowest num unused address in the network;
 *     0 if there are no open addresses
 */
static uint8_t findUnusedAddress()
{
	int i = 1;
	while (addrIsUsed(i) && i < DEVICES_ARRAY_SIZE)
		i++;
	if (i >= DEVICES_ARRAY_SIZE)
		return 0;
	else
		return i;
}

/* getDeviceTypeFromHal(addr)
 * simply a special case of getDeviceValue - for getting the type
 * @param addr the address of the device on the network
 * @param the variable in which we want to stor the value
 */
static inline int getDeviceTypeFromHal(uint8_t addr, uint8_t* value)
{
#define REG_TYPE 0
	return get_device_multi_values(addr, REG_TYPE, value, 1);
}

