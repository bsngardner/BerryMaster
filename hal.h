/*
 * hal.h
 *
 *  Created on: Oct 6, 2015
 *      Author: Broderick
 */

#ifndef HAL_H_
#define HAL_H_

#define SCL_PIN BIT7
#define SDA_PIN BIT6

#define NO_RESPONSE	1
#define TIMEOUT 6
#define SUCCESS 0

//Initialize the data bus sets up SDA and SCL, inits eUSCI module
uint8_t hal_init();
//Discover new device, return 0 on accept, 1 on rejected
uint8_t hal_discoverNewDevice(uint8_t new_address);
//Sends command to all berries to reset slave addr
uint8_t hal_resetAllDevices();
//Return 0 on ping success, 1 on ping failure
uint8_t hal_pingDevice(uint8_t address);
uint8_t hal_check_proj_key(uint8_t proj_hash);
uint8_t hal_update_proj_key(uint8_t proj_hash);
//Sets register (reg) on device (@address) to value (value)
uint8_t hal_setDeviceRegister(uint8_t address, uint8_t reg, uint8_t value);
//Sets registers, starting at reg
uint8_t hal_setDeviceMultiRegs(uint8_t address, uint8_t reg, uint8_t* buf,
		uint8_t count);
//Stores data from device (@address) register (reg) in variable (ret_val)
uint8_t hal_getDeviceRegister(uint8_t address, uint8_t reg, uint8_t* ret_val);
//Reads registers from device starting at reg
uint8_t hal_getDeviceMultiRegs(uint8_t address, uint8_t reg, uint8_t* ret_val,
		uint8_t count);

#endif /* HAL_H_ */
