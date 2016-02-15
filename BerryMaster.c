/*
 * BerryMaster.c
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

#include "BerryMaster.h"
#include "hal.h"

/******************************************************************************
 variables ********************************************************************
 *****************************************************************************/
static DeviceList_t myDeviceList;
const char* typeStrings[] = { "unknown", "LED", "Switch" };

/******************************************************************************
 function prototypes and macros ***********************************************
 *****************************************************************************/
/* addrIsUsed
 * checks if a device is currently configured at the specified address
 * @param addr the address to check
 * @return 1 if a device is configured at the address; 0 otherwise
 */
#define addrIsUsed(addr) (myDeviceList.devices[addr].deviceAddress > 0)

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
int connectToMaster() {
	return SUCCESS;
}

/* disconnectFromMaster
 * here for the sake of portability of host code
 * just returns 0 for success
 */
int disconnectFromMaster() {
	return SUCCESS;
}

/* init
 * initializes master:
 * calls hal_initDevices
 * validates current list of berries by pinging each one
 * discovers new devices and assigns them addresses
 * 	 iterates hal_getNewDevice(newDevAddr)
 * @return SUCCESS for success.
 */
int initDevices() {
	int error;
	if (error = hal_init())
		return error;
	if (error = validateDeviceList())
		return error;
	if (error = getNewDevices())
		return error;
	return SUCCESS; // success
}

/* getDeviceType
 * @param the address of the device
 * @param a pointer to store the type of the device at the specified address;
 * 	       notADevice (0) if device is not on network
 * @return SUCCESS for success; 1 if the device is not on the network.
 */
int getDeviceType(uint8_t addr, uint8_t* deviceType) {
	// Is there a device at this address?
	if (!addrIsUsed(addr)) {
		// No - return failed.
		*deviceType = (int)UNKNOWN;
		return 1;
	}
	else {
		// Yes, there is a device - set the type and return success.
		*deviceType = (int)myDeviceList.devices[addr].deviceType;
		return SUCCESS;
	}
}

/* getDeviceValue
 * gets the value (or current state) of the device at the specified address
 * @param the address of the device
 * @param a pointer to store the value
 * @param the register to read
 * @return SUCCESS if successful, non-zero if failed
 */
int getDeviceValue(uint8_t addr, uint8_t* value, uint8_t reg) {
	int error;
	// invalid address for a device
	if (addr == 0 || addr > MAX_NUM_DEVICES) {
		*value = 0xff;
		return INVALID_ADDR;
	}
	// valid address, but there is no device on the network at this address
	else if (!addrIsUsed(addr)) {
		*value = 0xff;
		return DEVICE_NOT_FOUND;
	}
	// valid device
	else {
		// call hal to get the value.
		if (error = hal_getDeviceRegister(addr, reg, (uint8_t*)value)) {
			// operation failed
			*value = 0xff;
			return error;
		}
		else {
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
int setDeviceValue(uint8_t addr, uint8_t value, uint8_t reg) {
	// invalid address for a device
	if (addr == 0 || addr > MAX_NUM_DEVICES) {
		return 1;
	}
	// valid address, but device doesn't exist
	else if (!addrIsUsed(addr)) {
		return 2;
	}
	// valid address for a device
	else {
		// call hal to set the value to the device.
		if (hal_setDeviceRegister(addr, reg, value)) {
			// operation failed
			return 3;
		}
		else {
			// operation successful
			return SUCCESS;
		}
	}
}

/******************************************************************************
 possible API functions *******************************************************
 *****************************************************************************/

/* clearNetwork
 * clears all info about devices in the network
 */
void clearNetwork() {
	int i;
	for (i = 1; i < DEVICES_ARRAY_SIZE; i++) {
		// TODO: will probably call a hal function that
		// wipes the device from the network
		myDeviceList.devices[i].deviceAddress = 0;
		myDeviceList.devices[i].deviceType = UNKNOWN;
	}
	myDeviceList.currNumDevices = 0;
}

/******************************************************************************
 internal functions ***********************************************************
 *****************************************************************************/

/* validateDeviceList
 * iterates through the list of devices
 * if the device was configured (address is not zero), then ping it
 * if it doesn't respond, take it off the network list.
 * @return SUCCESS if successful, non-zero if failed
 */
static int validateDeviceList() {
	int i = 0;
	int error;
	int addr = 1;
	int tempNumDevices = myDeviceList.currNumDevices;

	// i keeps track of how many devices we've checked
	// addr iterates through the device array
	// exit the loop when
	//   (a) we've checked all devices previously on the network, or
	//   (b) we've iterated through the entire array of devices
	//		 (there's a problem if this happens)
	while (i < tempNumDevices && addr < DEVICES_ARRAY_SIZE) {
		// is the device currently configured?
		if (addrIsUsed(addr)) { // yes,
			i++;
			// ping the device to see if it's still there:
			error = hal_pingDevice(myDeviceList.devices[addr].deviceAddress);
			if (error) {
				// no device on this address
				// erase the device from the network list
				myDeviceList.devices[addr].deviceAddress = 0;
				myDeviceList.devices[addr].deviceType = UNKNOWN;
				// one less device on the network
				myDeviceList.currNumDevices--;
				return error;
			}
		}
		addr++;
	}
	if (i < tempNumDevices) {
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
static int getNewDevices() {
	int error;
	uint8_t addr;
	uint8_t type;

	while(1) {
		addr = findUnusedAddress();
		if (addr == 0) {
			// error - full network
			// fixme: this is actually wrong: if we have exactly the
			// maximum number of allowed devices then this will return an
			// error when it shouldn't
			return NETWORK_FULL;
		}
		// todo: error check hal_discoverNewDevice
		else if (!(hal_discoverNewDevice(addr))) {
			// No error, record the address of the device
			myDeviceList.devices[addr].deviceAddress = addr;
			// get the type of the device and assign the type
			if (error = getDeviceTypeFromHal(addr, &type)) {
				// error - failed to get the type from the device
				myDeviceList.devices[addr].deviceType = UNKNOWN;
				return error;
			}
			// Successfully got the type
			myDeviceList.devices[addr].deviceType = type;
		}
		else return SUCCESS; // no devices grabbed the address - we're done
	}
}

/* findUnusedAddress
 * finds an unused address in the network
 * @return the lowest num unused address in the network;
 *     0 if there are no open addresses
 */
static uint8_t findUnusedAddress() {
	int i = 1;
	while (addrIsUsed(i) && i < DEVICES_ARRAY_SIZE) i++;
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
static inline int getDeviceTypeFromHal(uint8_t addr, uint8_t* value) {
	return getDeviceValue(addr, value, REG_TYPE);
}

