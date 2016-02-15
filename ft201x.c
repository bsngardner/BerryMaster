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
#include "BerryMaster.h"
#include "ft201x.h"
#include "events.h"

#define OUT_BUFFER_SIZE 160

// Global variables
jmp_buf usb_i2c_context;	// error context
IObuffer* io_usb_out;		// MSP430 to USB buffer
IObuffer* io_usb_in;		// USB to MSP430 buffer

extern volatile int buffLocked;

// local function prototypes
inline void ft201x_i2c_out_bit(uint8_t bit);
int ft201x_i2c_out8bits(uint8_t c);
inline void ft201x_i2c_out_stop();
int ft201x_i2c_start_address(uint8_t address);
void addToMessage(char c);

//******************************************************************************
//	Define port macros for the current board
#define FT201X_I2C_CLOCK_LOW		PJDIR |= ASCL		// put clock low
#define FT201X_I2C_CLOCK_HIGH		PJDIR &= ~ASCL		// put clock high

#define FT201X_I2C_DATA_LOW			PJDIR |= ASDA		// put data low
#define FT201X_I2C_DATA_HIGH		PJDIR &= ~ASDA		// put data high (pull-up)

#define FT201X_I2C_READ_DATA		PJIN & ASDA		// read state of data pin

#define FT201X_READ_ADDR	0x45
#define FT201X_WRITE_ADDR	0x44

//*****************************************************************************

// Queues USB out event
void USB_Out_callback() {
	sys_event |= USB_O_EVENT;
}

// Does nothing for now
void USB_In_callback() {
	return;
}

// Flush the ft201x tx and rx buffers
int ft201x_flushBuffers() {
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
int ft201x_init() {

	// Create an output buffer to send to host
	io_usb_out = IObuffer_create(OUT_BUFFER_SIZE);
	if (io_usb_out == 0) {
		// Error creating the IO buffer
		return SYS_ERR_430init;
	}

	// Set custom callback function.
	io_usb_out->bytes_ready = USB_Out_callback;

	// Need this input buffer to read data from host.
	io_usb_in = IObuffer_create(OUT_BUFFER_SIZE);
	if (io_usb_in == 0) {
		// Error creating the IO buffer
		return SYS_ERR_430init;
	}
	io_usb_in->bytes_ready = 0; // not using a callback for now

	ft201x_flushBuffers(); // flush the tx and rx buffers

	// return success
	return 0;
}

//*****************************************************************************
//  For now, just free the io buffers
//
void ft201x_close() {
	IObuffer_destroy(io_usb_in);
	IObuffer_destroy(io_usb_out);
}

//*****************************************************************************
//	Inline function for a single bit. Could use some heavy optimization, but
//	I'm not sure if the bus could handle full speed at higher clock rates.
//
inline void ft201x_i2c_out_bit(uint8_t bit) {
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
int ft201x_i2c_out8bits(uint8_t c) {
	uint8_t shift = 0x80;

	// output 8 bits during SDA low
	while (shift) {
		ft201x_i2c_out_bit(c & shift);
		shift >>= 1;				// adjust mask
	}

	FT201X_I2C_CLOCK_LOW;				// put clock low
	FT201X_I2C_DATA_HIGH;				// put data high
	FT201X_I2C_CLOCK_HIGH;				// put clock high

	// look for slave ack, if not low, then error
	if (FT201X_I2C_READ_DATA) {
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
inline void ft201x_i2c_out_stop() {
	FT201X_I2C_CLOCK_LOW;					// put clock low
	FT201X_I2C_DATA_LOW;					// make sure SDA is low
	FT201X_I2C_CLOCK_HIGH;					// clock high
	FT201X_I2C_DATA_HIGH;					// stop = low to high
	return;
}

//*****************************************************************************
//	Output start condition and address
//
int ft201x_i2c_start_address(uint8_t address) {
	int error;

	// output start
	FT201X_I2C_DATA_HIGH;
	FT201X_I2C_CLOCK_HIGH;			// w/SCL & SDA high
	FT201X_I2C_DATA_LOW;		// output start (SDA high to low while SCL high)

	if ((error = ft201x_i2c_out8bits(address))) {
		ft201x_i2c_out_stop();		// output stop 1st
		return error;				//return error
	}
	return 0;
}

//*****************************************************************************
//	Write a number of bytes to the FT201X over I2C
//
void ft201x_i2c_write(char* data, int16_t bytes) {
	int error;
	if ((error = ft201x_i2c_start_address(FT201X_WRITE_ADDR)))// output write address
		longjmp(usb_i2c_context, error);

	while (bytes--)									// write 8 bits
		if ((error = ft201x_i2c_out8bits(*data++)))
			longjmp(usb_i2c_context, error);		//return error
	ft201x_i2c_out_stop();							// output stop
	return;											// return success
}

void ft201x_i2c_write_io(IObuffer* iob) {
	int error;
	uint8_t data;
	if ((error = ft201x_i2c_start_address(FT201X_WRITE_ADDR)))// output write address
		longjmp(usb_i2c_context, error);

	while (iob->count) {									// write 8 bits
		IOgetc((char*) &data, iob);
		if ((error = ft201x_i2c_out8bits(data)))
			longjmp(usb_i2c_context, error);		//return error
	}
	ft201x_i2c_out_stop();							// output stop
	return;
}

//*****************************************************************************
//	Read a number of bytes from FT201X over I2C. Returns the number of bytes
//	read, which can easily be less than the number requested.
//
// TODO: do this right
int ft201x_i2c_read(int bytes) {
	uint8_t i, data;
	int16_t error;

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
		for (i = 8; i > 0; i--) {
			FT201X_I2C_CLOCK_LOW;	// I2C_CLOCK_LOW;
			FT201X_I2C_DATA_HIGH;	// high impedance
			FT201X_I2C_CLOCK_HIGH;	// I2C_CLOCK_HIGH;
			data <<= 1;				// assume 0
			if (FT201X_I2C_READ_DATA)
				data++;
		}
		// save data
		if (error = IOputc(data, io_usb_in)) {
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

int IOcopy(IObuffer* iodst, IObuffer* iosrc) {
	__disable_interrupt();
	char data;
	while (!IOgetc(&data, iosrc)) {
		IOputc(data, iodst);
	}
	//IOreset_peek(iosrc);
	return 0;
}

// Output the out buffer to the usb
void USBInEvent() {
	int err;
	int bytesRead = 0;
	// Read input characters from computer into io_usb_in
	if (!(err = setjmp(usb_i2c_context))) {
		// todo: I need to fix ft201x_i2c_read so I can read multiple bytes
		// Read 1 byte until there are no more bytes to be read
		while (!ft201x_i2c_read(1)) {
			LED1_ON;
			//bytesRead++;
		}
//		if (bytesRead) {
//			//IOcopy(io_usb_out, io_usb_in);
//		}

	} else {
		// Error! The error value is the var err
		// todo: for now, I'll echo the error;
		// later I want to handle this differently
		IOputc((char) (err + '0'), io_usb_out);
		return;
	}
	send_bytes(io_usb_in);
	LED1_OFF;
	return;
}

// Output the out buffer to the usb
int USBOutEvent() {
	ft201x_i2c_write_io(io_usb_out);
	return 0; // done with the buffer
}
