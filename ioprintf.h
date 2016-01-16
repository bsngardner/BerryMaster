#ifndef BERRYMASTER_PRINTF_H_
#define BERRYMASTER_PRINTF_H_
#include <stdint.h>
#include "IObuffer.h"

// change to zero to disable all printf debug statements
#define DEBUG 0
#define DEFAULT_MODULE EUSCI_A0_MODULE

#if DEBUG
#define debug(format,...) ioprintf(format,##__VA_ARGS__)
#else
#define debug(format,...)
#endif

int ioprintf(char* format, ...);
int iofprintf(IObuffer* io_buffer, char *format, ...);
void ioprintf_init(IObuffer* init_buffer);

#endif /* BERRYMASTER_PRINTF_H_ */
