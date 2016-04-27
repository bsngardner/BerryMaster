/*
 * BerryMaster.c
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

#include "berryMaster.h"
#include "hal.h"

/******************************************************************************
 variables ********************************************************************
 *****************************************************************************/
#pragma PERSISTENT(myDeviceList)
DeviceList_t myDeviceList = { 0 };

#pragma PERSISTENT(fram_proj_hash)
uint8_t fram_proj_hash = 0;

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

/* validateDeviceList
 *  pings each device in the list
 *  if the device doesn't respond, take this device out of the list
 * @return SUCCESS if successful, non-zero if failed
 */
static int validateDeviceList();

/* getNewDevices
 * while(1):
 *   get an available address
 *   call hal_getNewDevice(addr)
 *     if a device grabbed the address (hal_getNewDevice returned true),
 *       initialize that device in the network
 *     else return
 * @return SUCCESS if successful, non-zero if failed
 */
static int getNewDevices();

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
int connectToMaster()
{
	return SUCCESS;
}

/* disconnectFromMaster
 * here for the sake of portability of host code
 * just returns 0 for success
 */
int disconnectFromMaster()
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
int initDevices(uint8_t project_hash)
{
	int error;
	// Init communications over the vine (the HAL)
	if (error = hal_init())
		return error;

	// If project hash is different, clear the device network and
	// update the project hash.
	if (project_hash != fram_proj_hash)
	{
		clearNetwork();
		fram_proj_hash = project_hash;
	}
	// Validate the device list we have.
	if (error = validateDeviceList())
		return error;
	// Look for new devices on the network.
	if (error = getNewDevices())
		return error;

	// We're done.
	return SUCCESS; // success
}

/* getDeviceType
 * @param the address of the device
 * @param a pointer to store the type of the device at the specified address;
 * 	       unknown (0) if device is not on network
 * @return SUCCESS for success; 1 if the device is not on the network.
 */
int getDeviceType(uint8_t addr, uint8_t* deviceType)
{
	// Is it a valid address?
	if (addr <= 0 || addr > MAX_NUM_DEVICES)
	{
		*deviceType = UNKNOWN;
		return INVALID_ADDR;
	}
	// Is there a device at this address?
	if (!addrIsUsed(addr))
	{
		// No - return failed.
		*deviceType = UNKNOWN;
		return DEVICE_NOT_FOUND;
	}
	else
	{
		// Yes, there is a device - set the type and return success.
		*deviceType = myDeviceList.devices[addr].deviceType;
		return SUCCESS;
	}
}

/* getDeviceValue
 * gets a value in the specified register of the device at the specified address
 * @param the address of the device
 * @param a pointer to store the value
 * @param the register to read
 * @return SUCCESS if successful, non-zero if failed
 */
int getDeviceValue(uint8_t addr, uint8_t* value, uint8_t reg)
{
	int error;
	// invalid address for a device
	if (addr == 0 || addr > MAX_NUM_DEVICES)
	{
		*value = 0xff;
		return INVALID_ADDR;
	}
	// valid address, but there is no device on the network at this address
	else if (!addrIsUsed(addr))
	{
		*value = 0xff;
		return DEVICE_NOT_FOUND;
	}
	// valid device
	else
	{
		// call hal to get the value.
		if (error = hal_getDeviceRegister(addr, reg, value))
		{
			// operation failed
			*value = 0xff;
			return error;
		}
		else
		{
			// operation successful
			return SUCCESS;
		}
	}
}

/* getDeviceMultiValues
 * gets multiple bytes from the requested berry
 * @param address - address of the device
 * @param reg - the register number where we begin to read
 * @param ret_val - pointer to store the value
 * @param count - number of bytes to read
 */
int getDeviceMultiValues(uint8_t addr, uint8_t reg, uint8_t* buff,
		uint8_t count)
{
	int error;
	// invalid address for a device
	if (addr == 0 || addr > MAX_NUM_DEVICES)
	{
		buff[0] = 0xff;
		return INVALID_ADDR;
	}
	// valid address, but there is no device on the network at this address
	else if (!addrIsUsed(addr))
	{
		buff[0] = 0xff;
		return DEVICE_NOT_FOUND;
	}
	// valid device
	else
	{
		// call hal to get the value.
		if (error = hal_getDeviceMultiRegs(addr, reg, buff, count))
		{
			// operation failed
			buff[0] = 0xff;
			return error;
		}
		else
		{
			// operation successful
			return SUCCESS;
		}
	}
}

/* setDeviceValue
 * sets the device's value to the specified value
 * @param the address of the device
 * @param the value to set
 * @param the register to write
 * @return SUCCESS if successful, non-zero if failed
 */
int setDeviceValue(uint8_t addr, uint8_t value, uint8_t reg)
{
	// invalid address for a device
	if (addr == 0 || addr > MAX_NUM_DEVICES)
	{
		return 1;
	}
	// valid address, but device doesn't exist
	else if (!addrIsUsed(addr))
	{
		return 2;
	}
	// valid address for a device
	else
	{
		// call hal to set the value to the device.
		if (hal_setDeviceRegister(addr, reg, value))
		{
			// operation failed
			return 3;
		}
		else
		{
			// operation successful
			return SUCCESS;
		}
	}
}

/******************************************************************************
 internal functions ***********************************************************
 *****************************************************************************/

/* clearNetwork
 * resets the device table
 * resets all devices
 */
static void clearNetwork()
{
	int i;
	for (i = 0; i < DEVICES_ARRAY_SIZE; i++)
	{
		myDeviceList.devices[i].deviceAddress = 0;
		myDeviceList.devices[i].deviceType = UNKNOWN;
	}
	myDeviceList.currNumDevices = 0;

//	hal_resetAllDevices();
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
	hal_check_proj_hash(fram_proj_hash);

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
				myDeviceList.devices[addr].deviceAddress = 0;
				myDeviceList.devices[addr].deviceType = UNKNOWN;
				// one less device on the network
				myDeviceList.currNumDevices--;
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

/* getNewDevices
 * while(1):
 *   get an available address
 *   call hal_getNewDevice(addr)
 *     if a device grabbed the address (hal_discoverNewDevice returned 0),
 *       initialize that device in the network
 *     else return
 */
static int getNewDevices()
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

			// Increment number of devices on the network.
			myDeviceList.currNumDevices++;
		}
		else
			return SUCCESS; // no devices grabbed the address - we're done
	}
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
	return getDeviceValue(addr, value, REG_TYPE);
}

