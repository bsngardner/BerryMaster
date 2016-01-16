/*
 * printf library for the MSP432
 *
 * Largely taken from and inspired from:
 * 	http://www.msp430launchpad.com/2012/06/using-printf.html
 *	http://www.43oh.com/forum/viewtopic.php?f=10&t=1732
 * 
 * See http://www.samlewis.me for an example implementation.
 */

//#include "stdarg.h"
#include <stdint.h>
#include <stdio.h>

#include "ioprintf.h"
#include "IOBuffer.h"

#define PRINT_BUF_SIZE 80
static char printfBuffer[PRINT_BUF_SIZE];
IObuffer* defaultIObuffer;

void ioprintf_init(IObuffer* init_buffer) {
	defaultIObuffer = init_buffer;
}

int ioprintf(char *format, ...) {
	va_list args;
	va_start(args, format);
	vsnprintf(printfBuffer, PRINT_BUF_SIZE, (const char *) format, args);
	va_end(args);
	return IOputs(printfBuffer, defaultIObuffer);
}

int iofprintf(IObuffer* io_buffer, char *format, ...) {
	va_list args;
	va_start(args, format);
	vsnprintf(printfBuffer, PRINT_BUF_SIZE, (const char *) format, args);
	va_end(args);
	return IOputs(printfBuffer, io_buffer);
}
