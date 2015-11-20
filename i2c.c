//	RBX430_i2c.c - RBX430-1 REV D board system functions
//******************************************************************************
//******************************************************************************
//
//	Author:			Paul Roper, Brigham Young University
//	Revision:		1.0		02/01/2012
//
//	Built with CCSv5.1 w/cgt 3.0.0
//*******************************************************************************
//
//	                       MSP430G2553
//                  .----------------------.
//          CS_SD<--|P1.0             ^P2.0|-->LED03
//           MOSI<--|P1.1             ^P2.1|-->LED02
//           SCLK<--|P1.2              P2.2|-->LED01
//           MISO<--|P1.3              P2.3|-->LED04
//         LED_IR<--|P1.4              P2.4|-->SW1/Tx
//           ~INT-->|P1.5              P2.5|-->SW2/Rx
//        i2c_SCL<--|P1.6 (UCB0SCL)    P2.6|-->XIN
//        i2c_SDA<->|P1.7 (UCB0SDA)    P2.7|-->XOUT
//                  '----------------------'
//
//******************************************************************************
//******************************************************************************

#include <msp430.h>
#include <setjmp.h>
#include "BerryMaster.h"
#include "i2c.h"

//******************************************************************************
//******************************************************************************
//	LCD Global variables (NOT ZERO'D!!!)
//
jmp_buf i2c_context;				// error context
uint16_t i2c_delay;

//******************************************************************************

#define I2C_CLOCK_LOW		P1OUT &= ~SCL		// put clock low FIXME
#define I2C_CLOCK_HIGH		P1OUT |= SCL		// put clock high FIXME
// driving the clock this way breaks clock stretching

#define I2C_DATA_LOW		P1DIR |= SDA		// put data low
#define I2C_DATA_HIGH		P1DIR &= ~SDA		// put data high (pull-up)

//******************************************************************************
#define I2C_DELAY	0

// 1.2 MHz		i2c_fSCL = (1200/I2C_FSCL) = 12 / 30 = 0
// 8 MHz		i2c_fSCL = (8000/I2C_FSCL) = 80 / 30 = 2
// 12 MHz		i2c_fSCL = (12000/I2C_FSCL) = 120 / 30 = 4
// 16 MHz		i2c_fSCL = (16000/I2C_FSCL) = 160 / 30 = 5

uint16_t i2c_fSCL;				// i2c timing constant

//******************************************************************************
//	Init Universal Synchronous Controller
//
uint8_t i2c_init()
{
	uint16_t i;

	i2c_delay = 0;//i2c_fSCL;
//	i2c_delay = I2C_DELAY;

	P1SEL0 &= ~(SDA | SCL);			// GPIO Mode
	P1SEL1 &= ~(SDA | SCL);

	P1DIR &= ~SDA;					// set SDA as input (high)
	P1OUT &= ~SDA;					// setup SDA for low

	P1DIR |= SCL;					// set SCL as output
	P1OUT |= SCL;					// set SCL high

	// output 9 clocks with SDA high
	for (i = 9; i > 0; i--)
	{
		i2c_clocklow();				// clock SCL
		i2c_clockhigh();
	}

	// send stop condition
	i2c_out_stop();
	return 0;
} // init_i2c


//******************************************************************************
//
void i2c_clocklow()
{
//	volatile uint16_t delay = i2c_delay;
	volatile uint16_t delay = i2c_delay;
	I2C_CLOCK_LOW;					// put clock low
	while (delay--);				// delay
	return;
} // end clocklow

void i2c_clockhigh()
{
	volatile uint16_t delay = i2c_delay;
//	uint16_t delay = i2c_delay;
	I2C_CLOCK_HIGH;					// put clock high
	while (delay--);				// delay
	return;
} // end clockhigh

//******************************************************************************
//
//	.         .__delay__.
//	|         |
//	|__delay__|
//	 ^
//	 ^--> 0/1 SDA
//
void i2c_out_bit(uint8_t bit)
{
	i2c_clocklow();					// drop clock
	if (bit) I2C_DATA_HIGH;			// set SDA high
	else I2C_DATA_LOW;				// or SDA low
	i2c_clockhigh();				// raise clock
	return;
} // end i2c_out_bit


//	.         .__delay__.      .          .__delay__.
//	|         |            x8  |          |^
//	|__delay__|                |__delay __|^
//	 ^                          ^          ^--> check for ack
//	 ^                          ^
//	 ^--> 0/1 SDA               ^--> 0/1 SDA
//
//	exit high impedance
//
int i2c_out8bits(uint8_t c)
{
	uint8_t shift = 0x80;
	volatile int delay;

	// output 8 bits during SDA low
	while (shift)
	{
		i2c_out_bit(c & shift);
		shift >>= 1;				// adjust mask
	}
	// look for slave ack, if not low, then error

	//	i2c_clocklow();
	I2C_CLOCK_LOW;					// put clock low
	I2C_DATA_HIGH;					// turn SDA to input (high impedance)
	delay = i2c_delay;
	while (delay--);				// delay

	i2c_clockhigh();				// put clock high
	if (P1IN & SDA)
	{
		// try again
		delay = 8;//i2c_delay;
		while (delay--);				// delay
		if (P1IN & SDA) return SYS_ERR_I2C_ACK;
	}
	I2C_CLOCK_LOW; // Is this needed to end the ACK?
	return 0;
} // end out8bits


//******************************************************************************
//
//	exit w/high impedance SDA
//
void i2c_start_address(uint16_t address, uint8_t rwFlag)
{
	volatile uint16_t delay = i2c_delay;
	int error;

	// output start
	I2C_DATA_HIGH;
	I2C_CLOCK_HIGH;					// w/SCL & SDA high

	I2C_DATA_LOW;					// output start (SDA high to low while SCL high)
	while (delay--);				// delay

	// output (address * 2 + read/write bit)
	if (error = i2c_out8bits((address << 1) + rwFlag))
	{
		i2c_out_stop();					// output stop 1st
		longjmp(i2c_context, error);	//return error;
	}
	return;
} // end i2c_out_address


//******************************************************************************
//
void i2c_out_stop()
{
	volatile uint16_t delay = i2c_delay;

	i2c_clocklow();					// put clock low
	I2C_DATA_LOW;					// make sure SDA is low
	i2c_clockhigh();				// clock high
	I2C_DATA_HIGH;					// stop = low to high
	while (delay--);
	return;
} // end i2c_out_stop


//******************************************************************************
//
void i2c_write(uint16_t address, uint8_t* data, int16_t bytes)
{
	int error;
	i2c_start_address(address, 0);	// output write address
	while (bytes--)					// write 8 bits
	{
		if (error = i2c_out8bits(*data++)) longjmp(i2c_context, error);	//return error
	}
	i2c_out_stop();					// output stop
	return;							// return success
} // end i2c_write


//******************************************************************************
//	read bytes into buffer using i2c
//
//	IN:		address	i2c address
//			buffer	pointer to input buffer
//			bytes	# of bytes to read
//	OUT:	last byte read
//
uint8_t i2c_read(uint16_t address, uint8_t* buffer, int16_t bytes)
{
	uint16_t i, data;
	i2c_start_address(address, 1);	// output read address

	while (bytes--)					// read 8 bits
	{
		for (i = 8; i > 0; i--)
		{
			i2c_clocklow();			// I2C_CLOCK_LOW;
			I2C_DATA_HIGH;			// high impedance
			i2c_clockhigh();		// I2C_CLOCK_HIGH;
			data <<= 1;				// assume 0
			if (P1IN & SDA) data++;
		}
		// save data
		*buffer++ = data;

		// output ack or nack
		i2c_clocklow();				// I2C_CLOCK_LOW;
		if (bytes) I2C_DATA_LOW;	// ack (0)
		else I2C_DATA_HIGH;			// nack (1)
		i2c_clockhigh();			// I2C_CLOCK_HIGH;
	}
	i2c_out_stop();					// output stop
	return data;
} // end i2c_read


uint8_t i2c_reg_read(uint16_t address, uint8_t reg, uint8_t* buffer, int16_t bytes)
{
	int error;

	i2c_start_address(address, 0);
	if (error = i2c_out8bits(reg)) longjmp(i2c_context, error);	//return error
	return i2c_read(address, buffer, bytes);

}
