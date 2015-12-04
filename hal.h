/*
 * hal.h
 *
 *  Created on: Dec 3, 2015
 *      Author: Berry_Admin
 */

#ifndef HAL_H_
#define HAL_H_

#include <stdint.h>

//Initialize the data bus
uint8_t hal_init();
//Discover new device, return 0 on accept, 1 on no response, 3 on reject
uint8_t hal_discoverNewDevice(uint8_t new_address);
//Return 0 on ping success, 1 on ping failure
uint8_t hal_pingDevice(uint8_t address);

uint8_t hal_setDeviceRegister(uint8_t address, uint8_t reg,
		uint8_t value);
uint8_t hal_getDeviceRegister(uint8_t address, uint8_t reg, uint8_t* ret_val);


#endif /* HAL_H_ */
