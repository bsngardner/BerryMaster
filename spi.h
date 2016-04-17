/*
 * spi.h
 *
 *  Created on: Feb 8, 2014
 *      Author: proper
 */

#ifndef SPI_H_
#define SPI_H_

enum SPI_MODE {
	SPI_SINGLE, SPI_BUFFERED
};
enum SPI_DIVISOR {
	SPI_SLOW = 0x80, SPI_FAST = 0x10
};

//Function
int spi_init(void);
unsigned char spi_read(void);
void spi_write(unsigned char byte);
unsigned char spi_transfer(unsigned char byte);
void spi_start_read();

void spi_set_mode(enum SPI_MODE mode);
void spi_set_divisor(enum SPI_DIVISOR divisor);

unsigned char mSD_write_byte(unsigned char byte);

//Variables
extern volatile int spi_reading;

#endif /* SPI_H_ */
