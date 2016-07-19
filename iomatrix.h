/*
 * iomatrix.h
 *
 *  Created on: Mar 29, 2016
 *      Author: Broderick
 */

#ifndef IOMATRIX_H_
#define IOMATRIX_H_

#include "IObuffer.h"

//Size defines
#ifndef USB_BUF_SIZE
#define USB_BUF_SIZE 256
#endif
#ifndef SERVER_BUF_SIZE
#define SERVER_BUF_SIZE 256
#endif

//Buffers
extern IObuffer* usb_buffer;
extern IObuffer* nrf_buffer;
extern IObuffer* server_buffer;

//Slots
extern IObuffer* usb_slot;
extern IObuffer* nrf_slot;
extern IObuffer* server_slot;
extern IObuffer* log_slot;
extern IObuffer* error_slot;

#endif /* IOMATRIX_H_ */
