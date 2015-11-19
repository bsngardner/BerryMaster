//	ft201x.c - library for USB chip FT201X
//******************************************************************************
//******************************************************************************
//
//	Author:			Kristian Sims, Brigham Young University
//	Revision:		0.1		12/19/2014	First created
//					0.2		03/19/2015	Works, but not ready/optimized
//
//*******************************************************************************
//******************************************************************************

#include <msp430.h>
#include <setjmp.h>
#include "BDL.h"
#include "ft201x.h"
#include "commands.h"
#include "printf.h"
#include <string.h>

// Global variables
jmp_buf usb_i2c_context;				// error context
IObuffer* io_usb_out;

volatile uint8 USB_mode;
extern volatile SYS_MODE sys_mode;
int insertDC(uint16 event, uint16 time);

extern volatile command_st my_cmd;

//******************************************************************************
//	Define port macros for the current board
#if defined(HW_VERSION_3C) || defined(HW_VERSION_4A) || defined(HW_VERSION_4B)
#define FT201X_I2C_CLOCK_LOW		PJDIR |= ASCL		// put clock low
#define FT201X_I2C_CLOCK_HIGH		PJDIR &= ~ASCL		// put clock high

#define FT201X_I2C_DATA_LOW			PJDIR |= ASDA		// put data low
#define FT201X_I2C_DATA_HIGH		PJDIR &= ~ASDA		// put data high (pull-up)

#define FT201X_I2C_READ_DATA		PJIN & ASDA		// read state of data pin
#elif defined(HW_VERSION_3A) || defined(HW_VERSION_3B)
#define FT201X_I2C_CLOCK_LOW		P2DIR |= ASCL		// put clock low
#define FT201X_I2C_CLOCK_HIGH		P2DIR &= ~ASCL		// put clock high

#define FT201X_I2C_DATA_LOW			P2DIR |= ASDA		// put data low
#define FT201X_I2C_DATA_HIGH		P2DIR &= ~ASDA		// put data high (pull-up)

#define FT201X_I2C_READ_DATA		P2IN & ASDA		// read state of data pin
#else
#error 666
#endif

#define FT201X_READ_ADDR	0x45
#define FT201X_WRITE_ADDR	0x44

//*****************************************************************************
//	Check if USB is connected (pullups are powered by the USB chip)
//
uint8 usb_check_connected()
{
	//TODO are the pins configured?

	if (FT201X_I2C_READ_DATA)
	{
		USB_mode = 1;
	}

	return USB_mode;
}

void USB_IO_callback()
{
	sys_event |= USB_IO;
}

//*****************************************************************************
//	Right now this just sets up the ports, nothing else...
//
int ft201x_init()
{
#if defined(HW_VERSION_3C) || defined(HW_VERSION_4A) || defined(HW_VERSION_4B)
	PJSEL0 &= ~(ASDA & ASCL);
	PJSEL1 &= ~(ASDA & ASCL);
	PJOUT &= ~(ASDA & ASCL);
	PJDIR &= ~(ASDA & ASCL);
#elif defined(HW_VERSION_3A) || defined(HW_VERSION_3B)
	P2SEL0 &= ~(ASDA & ASCL);
	P2SEL1 &= ~(ASDA & ASCL);
	P2OUT &= ~(ASDA & ASCL);
	P2DIR &= ~(ASDA & ASCL);
#endif

	io_usb_out = IObuffer_create(128);
	io_usb_out->bytes_ready = USB_IO_callback;

//	io_usb_in = IObuffer_create(16);
//	io_usb_in->bytes_ready = Henrys_USB_Trap;

	insertDC(USB_IO, 127); //TODO this doesn't belong here!

	return 0;
}
//TODO: There should be an init, open, and close--or really an open and close
// and let the BDL_init take care of the ports.

//*****************************************************************************
//	Inline function for a single bit. Could use some heavy optimization, but
//	I'm not sure if the bus could handle full speed at higher clock rates.
//
inline void ft201x_i2c_out_bit(uint8 bit)
{
	FT201X_I2C_CLOCK_LOW;				// drop clock
	if (bit)
		FT201X_I2C_DATA_HIGH;		// set SDA high
	else
		FT201X_I2C_DATA_LOW;			// or SDA low
	FT201X_I2C_CLOCK_HIGH;				// raise clock
	return;
}

//*****************************************************************************
//	Outputs one byte and reads the N/ACK. Currently returns an error on NACK,
//	that probably could be handled a little more civilly, since NACK is used
//	to represent no data available on the FT201X.
//
int ft201x_i2c_out8bits(uint8 c)
{
	uint8 shift = 0x80;

	// output 8 bits during SDA low
	while (shift)
	{
		ft201x_i2c_out_bit(c & shift);
		shift >>= 1;				// adjust mask
	}

	FT201X_I2C_CLOCK_LOW;				// put clock low
	FT201X_I2C_DATA_HIGH;				// put data high
	FT201X_I2C_CLOCK_HIGH;				// put clock high

	// look for slave ack, if not low, then error
	if (FT201X_I2C_READ_DATA)
	{
		// wait & try again
		__delay_cycles(20);
		if (FT201X_I2C_READ_DATA)
			return SYS_ERR_I2C_ACK;
	}
	FT201X_I2C_CLOCK_LOW;
	return 0;
}

//*****************************************************************************
//	Output the stop condition
//
inline void ft201x_i2c_out_stop()
{
	FT201X_I2C_CLOCK_LOW;					// put clock low
	FT201X_I2C_DATA_LOW;					// make sure SDA is low
	FT201X_I2C_CLOCK_HIGH;					// clock high
	FT201X_I2C_DATA_HIGH;					// stop = low to high
	return;
}

//*****************************************************************************
//	Output start condition and address
//
int ft201x_i2c_start_address(uint8 address)
{
	int error;

	// output start
	FT201X_I2C_DATA_HIGH;
	FT201X_I2C_CLOCK_HIGH;			// w/SCL & SDA high
	FT201X_I2C_DATA_LOW;		// output start (SDA high to low while SCL high)

	if ((error = ft201x_i2c_out8bits(address)))
	{
		ft201x_i2c_out_stop();		// output stop 1st
		return error;				//return error
	}
	return 0;
}

//*****************************************************************************
//	Write a number of bytes to the FT201X over I2C
//
void ft201x_i2c_write(char* data, int16 bytes)
{
	int error;
	if ((error = ft201x_i2c_start_address(FT201X_WRITE_ADDR)))// output write address
		longjmp(usb_i2c_context, error);

	while (bytes--)									// write 8 bits
		if ((error = ft201x_i2c_out8bits(*data++)))
			longjmp(usb_i2c_context, error);		//return error
	ft201x_i2c_out_stop();							// output stop
	return;											// return success
}

//*****************************************************************************
//	Read a number of bytes from FT201X over I2C. Returns the number of bytes
//	read, which can easily be less than the number requested.
//
uint8 ft201x_i2c_read(char* buffer, int16 bytes)
{
	uint8 i, data;
	uint16 error;

	if ((error = ft201x_i2c_start_address(FT201X_READ_ADDR)))// output read address
	{
		if (error == SYS_ERR_I2C_ACK)
			return 0;							// no bytes to read
		else
			longjmp(usb_i2c_context, error);	// unrecognized error
	}

	while (bytes--)								// read 8 bits
	{
		for (i = 8; i > 0; i--)
		{
			FT201X_I2C_CLOCK_LOW;	// I2C_CLOCK_LOW;
			FT201X_I2C_DATA_HIGH;	// high impedance
			FT201X_I2C_CLOCK_HIGH;	// I2C_CLOCK_HIGH;
			data <<= 1;				// assume 0
			if (FT201X_I2C_READ_DATA)
				data++;
		}
		// save data
		*buffer++ = data; // is there a byte/word problem here?

		// output ack or nack
		FT201X_I2C_CLOCK_LOW;				// I2C_CLOCK_LOW;
		if (bytes)
			FT201X_I2C_DATA_LOW;		// ack (0)
		else
			FT201X_I2C_DATA_HIGH;			// nack (1)
		FT201X_I2C_CLOCK_HIGH;				// I2C_CLOCK_HIGH;
	}
	ft201x_i2c_out_stop();					// output stop
	return data; // TODO: this is not the number of bytes you are looking for! (not the documented behavior but any chnges will affect commands.c)
}

void USB_event()
{
	//static int usb_in_timeout = 0;
	int i;
	char buf[16];
	if ((i = setjmp(usb_i2c_context)))
		return;

	command_retrieve(((command_st*) (&my_cmd)));
	if (my_cmd.size != 0)
	{
		if (!strcmp(my_cmd.cmd, "config"))
		{
			USB_echo("Enter USB Config");
			start_blocking_command_sequence();

		}
		else
		{
			command_echo(((command_st*) (&my_cmd)));
		}
	}
	for (i = 0; i < 16; ++i)
		if (IOgetc(buf + i, io_usb_out))
			break;
//	ft201x_i2c_write("\r\nAAAAAA", 8);
	if (i)
	{
		LED1_ON;
		ft201x_i2c_write(buf, i);
		sys_event |= USB_IO;
		LED1_OFF;
	}
}

