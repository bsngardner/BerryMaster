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

// Global variables
jmp_buf usb_i2c_context;	// error context
IObuffer* io_usb_out;		// MSP430 to USB buffer
IObuffer* io_usb_in;		// USB to MSP430 buffer

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
	io_usb_out = IObuffer_create(MAX_MSG_LENGTH);
	if (io_usb_out == NULL) {
		// Error creating the IO buffer
		return SYS_ERR_430init;
	}

	// Set custom callback function.
	io_usb_out->bytes_ready = USB_Out_callback;

	// Need this input buffer to read data from host.
	io_usb_in = IObuffer_create(MAX_MSG_LENGTH);
	if (io_usb_in == NULL) {
		// Error creating the IO buffer
		return SYS_ERR_430init;
	}
	io_usb_in->bytes_ready = NULL; // not using a callback for now

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

// Poll the USB input buffer.
void USBInEvent() {
	int err = 0;
	char data = 0;
	int bytesRead = 0;

	// receive message FSM vars
	static enum states_e {
		getSize, getMsg
	} state = getSize;

	// private message size variable
	static int msgBytesLeft = 0;

	// the rx buffer index - index of next empty location
	static unsigned int sentMessage_i = 0;

	// Only read characters if we're ready to read them:
	if (buffLocked == TRUE) {
		// the receive buffer is locked, just return
		return;
	}

	// Read input characters from computer into io_usb_in
	if (!(err = setjmp(usb_i2c_context))) {
		// todo: I need to fix ft201x_i2c_read so I can read multiple bytes
		// Read 1 byte until there are no more bytes to be read
		while (!ft201x_i2c_read(1))
			bytesRead++;
	} else {
		// Error! The error value is the var err
		// todo: for now, I'll echo the error;
		// later I want to handle this differently
		IOputc((char) (err + ASCII_ZERO), io_usb_out);
		return;
	}

	// check if any data was actually read
	if (!bytesRead) {
		return; // no bytes read
	}

	// Read the data from the IObuffer until there is no more left to read
	while (bytesRead) {
		bytesRead--;
		// Read the next char
		if (err = IOgetc(&data, io_usb_in)) {
			// Error getting character
			reportError("IOgetc USBInEvent err", SYS_ERR_IOBUFFER);
			handleError();
		}
		LED1_TOGGLE;
		IOputc(data, io_usb_out);
		continue;

		// receive message FSM:
		switch (state) {
		case getSize:
			// is the buffer locked?
			if (buffLocked == FALSE) {
				// no, get number of bytes of the message
				msgBytesLeft = (int) (data); // todo: check this!!!
				// put into buffer:
				sentMessage[0] = data;

				// prep to store data in buffer:
				sentMessage_i = 1; // reset buffer index
				state = getMsg; // go to the get message state
			}
			break;
		case getMsg:
			// put message into buffer
			sentMessage[sentMessage_i++] = data;

			// do we expect any more data?
			msgBytesLeft--;
			if (msgBytesLeft == 0) {
				// no, lock the buffer, signal server event,
				// and go back to initial state
				buffLocked = TRUE;
				sys_event |= SERVER_EVENT;
				state = getSize;
				// We should have read the whole message by now - check
				if (bytesRead != 0) {
					// Host sent more data than it promised, error
					reportError("Host msg err", SYS_ERR_RX_HOST_MSG);
					handleError(); // spins in an infinite loop
				}
			}
			break;
		default:
			// shouldn't ever get here - default to initial state
			state = getSize;
			break;
		} // end state machine
	}

	/*
	 // just for testing - echo the rxed char.
	 char c;
	 // Try to read a character from io_usb_in
	 if (IOgetc(&c, io_usb_in) == 0) {
	 // Got it. Now try to put it into io_usb_out.
	 if (IOputc(c, io_usb_out) != 0) {
	 // error
	 }
	 }
	 */

}

// Output the out buffer to the usb
int USBOutEvent() {
#define BUF_SIZE 32 // was 16
	int i; // byte count
	char buf[BUF_SIZE]; // out buffer (double buffering is inefficient here...)

	// Copy from IOBuffer into our temp buffer
	// (TODO: make this direct in ft201x write)
	for (i = 0; i < BUF_SIZE; ++i) {
		if (IOgetc(buf + i, io_usb_out)) {
			break;
		}
	}

	// Write the temp buffer out to USB (to computer)
	if (i) {
		// todo: do a setjmp here
//		LED1_ON;
		ft201x_i2c_write(buf, i);
//		LED1_OFF;
	}

	// If we haven't read everything out of the buffer yet,
	// come back to this event.
	if (i == BUF_SIZE) {
		sys_event |= USB_O_EVENT; // queue up this event again
		return -1; // signal that we're not done yet
	}
	return 0; // done with the buffer
}
