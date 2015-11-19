//	ft201x.h 2015/02/06
#ifndef FT201X_H_
#define FT201X_H_

#include "IObuffer.h"
#include <setjmp.h>

// Global function prototypes
int ft201x_init();
void ft201x_i2c_write(char*, int16);
uint8 ft201x_i2c_read(char*, int16);
void USB_event();

// Global variables
extern jmp_buf usb_i2c_context;				// error context
extern IObuffer* io_usb_out;

#endif /* FT201X_H_ */
