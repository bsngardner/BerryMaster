//	i2c.h	02/08/2014
#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

//******************************************************************************
//	i2c equates
//
#define APDS9300_ADR	0x29		// Light Sensor
#define MPL115A2_ADR	0x60		// Barometer
#define MPL3115A2_ADR	0x60		// Barometer
#define FM24VN10_ADR	0x50		// FRAM
#define FRAM_ADR		0x50		// FRAM

#define MPU9250_ADR		0x68		//Gyro/Accelerometer

uint8_t i2c_init(void);
void i2c_write(uint16_t address, uint8_t* data, int16_t bytes);
uint8_t i2c_read(uint16_t address, uint8_t* buffer, int16_t bytes);
uint8_t i2c_reg_read(uint16_t address, uint8_t reg, uint8_t* buffer, int16_t bytes);

// TODO: These shouldn't be public!! Fix FRAM sleep
void i2c_clocklow(void);
void i2c_clockhigh(void);
void i2c_out_bit(uint8_t bit);
void i2c_start_address(uint16_t address, uint8_t rwFlag);
void i2c_out_stop(void);
int i2c_out8bits(uint8_t c);

//#define I2C_FSCL	100							// ~100kHz
//#define I2C_FSCL	200							// ~200kHz
#define I2C_FSCL	4800							// ~400kHz

void wait(uint16_t time);

#endif /* I2C_H_ */
