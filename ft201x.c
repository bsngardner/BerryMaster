//	ft201x.c - library for USB chip FT201X
//*****************************************************************************
//*****************************************************************************
//
//	Authors:		Kristian Sims, Brigham Young University
//					Marshall Garey
//	Revision:		0.1		12/19/2014	First created
//					0.2		03/19/2015	Works, but not ready/optimized
//					0.3		11/19/2015	Adapted for new Berry Master
//*****************************************************************************
//*****************************************************************************

#include <msp430.h>
#include <setjmp.h>
#include "server.h"
#include "ft201x.h"

#include "berryMaster.h"
#include "events.h"

// Global variables
jmp_buf usb_i2c_context;	// error context
IObuffer* usb_buffer;		// MSP430 to USB buffer
IObuffer* usb_slot;		// USB to MSP430 buffer

int insertDC(uint16_t event, uint16_t time);

extern volatile uint16_t sys_event;
extern char sentMessage[MAX_MSG_LENGTH]; // buffer used for the message
extern volatile int buffLocked;

// local function prototypes
inline void ft201x_i2c_out_bit(uint8_t bit);
int ft201x_i2c_out8bits(uint8_t c);
inline void ft201x_i2c_out_stop();
int ft201x_i2c_start_address(uint8_t address);
void addToMessage(char c);

//*****************************************************************************
//	Define port macros for the current board
#if defined(MAMA_REV_A)
#define FT201X_I2C_CLOCK_LOW		PJDIR |= ASCL		// put clock low
#define FT201X_I2C_CLOCK_HIGH		PJDIR &= ~ASCL		// put clock high

#define FT201X_I2C_DATA_LOW			PJDIR |= ASDA		// put data low
#define FT201X_I2C_DATA_HIGH		PJDIR &= ~ASDA		// put data high (pull-up)

#define FT201X_I2C_READ_DATA		PJIN & ASDA		// read state of data pin
#else
#error 666
#endif

#define FT201X_READ_ADDR	0x45
#define FT201X_WRITE_ADDR	0x44

//*****************************************************************************

void ft201x_setUSBCallback(void (*callback)(void))
{
	usb_buffer->bytes_ready = callback;
}

// Flush the ft201x tx and rx buffers
int ft201x_flushBuffers()
{
	// Make a global i2c call
	ft201x_i2c_start_address(0);

	// 0x0E is the command to clear buffers (see ft201x datasheet)
	ft201x_i2c_out8bits(0x0E);

	// Generate stop condiditon
	ft201x_i2c_out_stop();

	// Successfully return
	return 0;
}

//*****************************************************************************
//
//
int ft201x_init()
{

	// Create an output buffer to send to host
	usb_buffer = IObuffer_create(USB_BUF_SIZE);
	if (usb_buffer == NULL)
	{
		// Error creating the USB IO buffer
		return SYS_ERR_430init;
	}

	usb_slot = 0;

	ft201x_flushBuffers(); // flush the tx and rx buffers on ft201x chip

	// return success
	return 0;
}

//*****************************************************************************
//  For now, just free the io buffers
//
void ft201x_close()
{
	IObuffer_destroy(usb_buffer);
}

//*****************************************************************************
//	Inline function for a single bit. Could use some heavy optimization, but
//	I'm not sure if the bus could handle full speed at higher clock rates.
//
inline void ft201x_i2c_out_bit(uint8_t bit)
{
	FT201X_I2C_CLOCK_LOW;				// drop clock
	if (bit)
		FT201X_I2C_DATA_HIGH;			// set SDA high
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
int ft201x_i2c_out8bits(uint8_t c)
{
	uint8_t shift = 0x80;

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
int ft201x_i2c_start_address(uint8_t address)
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
//	Write entire output io buffer to the FT201X over I2C
//
void ft201x_i2c_write()
{
	int error;
	char c;

	// Output write address
	if ((error = ft201x_i2c_start_address(FT201X_WRITE_ADDR)))
		longjmp(usb_i2c_context, error);

	// Write entire output io buffer to usb
	while (usb_buffer->count)
	{
		// get character from io buffer
		if (error = IOgetc(&c, usb_buffer))
		{
			longjmp(usb_i2c_context, error); // return error
		}
		// write 8 bits
		if ((error = ft201x_i2c_out8bits((uint8_t) c)))
		{
			longjmp(usb_i2c_context, error); //return error
		}
	}
	ft201x_i2c_out_stop(); // output stop
	return;	// return success
}

//*****************************************************************************
//	Read a number of bytes from FT201X over I2C. Returns the number of bytes
//	read, which can easily be less than the number requested.
//
// TODO: do this right
int ft201x_i2c_read()
{
	uint8_t i, data;
	int16_t error;
	uint16_t bytes = 1;

	if ((error = ft201x_i2c_start_address(FT201X_READ_ADDR)))// output read address
	{
		if (error == SYS_ERR_I2C_ACK)
			return -1;							// no bytes to read
		else
			longjmp(usb_i2c_context, error);	// unrecognized error
	}

	// TODO: when there is a CBUS pin
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
		if (error = IOputc(data, usb_slot))
		{
			ft201x_i2c_out_stop();
			longjmp(usb_i2c_context, error);
		}

		// output ack or nack
		FT201X_I2C_CLOCK_LOW;				// I2C_CLOCK_LOW;
		if (bytes)
			FT201X_I2C_DATA_LOW;			// ack (0)
		else
			FT201X_I2C_DATA_HIGH;			// nack (1)
		FT201X_I2C_CLOCK_HIGH;				// I2C_CLOCK_HIGH;
	}
	ft201x_i2c_out_stop();					// output stop

	// TODO: this is not the number of bytes you are looking for!
	// (not the documented behavior)
	return 0;
}

// Poll the USB input buffer.
void USBInEvent()
{
	// Read input characters from computer into io_usb_in
	if (!(setjmp(usb_i2c_context)))
	{
		// Read 1 byte until there are no more bytes to be read
		// io_usb_in will signal the server event if it has data
		while (!ft201x_i2c_read())
			;
	}
	else
	{
		// TODO: ERROR
		return;
	}
}

// Write the passed in io buffer to the usb
int USBOutEvent()
{
	int err;

	// set context restore point
	if (!(err = _setjmp(usb_i2c_context)))
	{
		// write buffer out to usb
		ft201x_i2c_write();
	}
	else
	{
		// Error!
		// todo: Is there any way to handle this better?
		return err;
	}
	return 0; // done with the buffer
}
