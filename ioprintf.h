#ifndef BERRYMASTER_PRINTF_H_
#define BERRYMASTER_PRINTF_H_
#include <stdint.h>
#include "IObuffer.h"

int ioprintf(char* format, ...);
int iofprintf(IObuffer* io_buffer, char *format, ...);
void ioprintf_init(IObuffer* init_buffer);

#endif /* BERRYMASTER_PRINTF_H_ */
