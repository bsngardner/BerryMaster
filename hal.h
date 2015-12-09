/*
 * hal.h
 *
 *  Created on: Oct 6, 2015
 *      Author: Broderick
 */

#ifndef HAL_H_
#define HAL_H_

#include <stdint.h>

#define SCL_PIN BIT7
#define SDA_PIN BIT6

#define ADDR_ACCEPTED 0
#define ADDR_REJECTED 2
#define NO_RESPONSE	1
#define PING 0
#define SUCCESS 0

//Initialize the data bus
// sets up SDA and SCL, inits eUSCI module, enables interrupt vector
extern uint8_t hal_init();

//Discover new device, return 0 on accept, 1 on rejected
extern uint8_t hal_discoverNewDevice(uint8_t new_address);

//Sends command to all berries to reset slave addr
extern uint8_t hal_resetAllDevices();

//Return 0 on ping success, 1 on ping failure
extern uint8_t hal_pingDevice(uint8_t address);

//Sets register (reg) on device (@address) to value (value)
extern uint8_t hal_setDeviceRegister(uint8_t address, uint8_t reg,
		uint8_t value);

//Stores data from device (@address) register (reg) in variable (ret_val)
//														passed by reference
extern uint8_t hal_getDeviceRegister(uint8_t address, uint8_t reg,
		uint8_t* ret_val);

#endif /* HAL_H_ */
