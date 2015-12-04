/*
 * hal.c
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

#include "hal.h"


// simulate values in devices
#define MAX_DEVICES 19
static int devValues[MAX_DEVICES] =
	{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 };

//Initialize the data bus
uint8_t hal_init() {
	return 0;
}

//Discover new device, return 0 on accept, 1 on no response, 3 on reject
uint8_t hal_discoverNewDevice(uint8_t new_address) {
	// simulate finding 3 devices
	static int i = 0;
	i++;
	if (i < 4) {
		return 0;
	}
	else if (i >= 4) {
		i = 0;
	}

	return 1;
}

//Return 0 on ping success, 1 on ping failure
uint8_t hal_pingDevice(uint8_t address) {
	return 0;
}

uint8_t hal_setDeviceRegister(uint8_t address, uint8_t reg,
		uint8_t value) {
	devValues[address-1] = value;
	return 0;
}

// store the value in the parameter value
uint8_t hal_getDeviceRegister(uint8_t address, uint8_t reg, uint8_t* value) {
	*value = devValues[address-1];
	return 0;
}

