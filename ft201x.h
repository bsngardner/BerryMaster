//	ft201x.h 2015/02/06
#ifndef FT201X_H_
#define FT201X_H_

//Standard includes
#include <setjmp.h>
#include <stdint.h>
//Project includes
#include "IObuffer.h"
#include "iomatrix.h"

// Global function prototypes
int ft201x_init();
void ft201x_close();
void ft201x_i2c_write();
int ft201x_i2c_read();
int ft201x_flushBuffers();
void ft201x_setUSBCallback(void (*callback)(void));
void usb_in_event();
int usb_out_event();

// Global variables
extern jmp_buf usb_i2c_context;	// error context

//Defines
#ifndef USB_BUF_SIZE
#define USB_BUF_SIZE 64
#endif

#endif /* FT201X_H_ */
