/*
 * events.h
 *
 *  Created on: Mar 25, 2016
 *      Author: Berry_Admin
 */

#ifndef EVENTS_H_
#define EVENTS_H_

#include <stdint.h>

// Events
#define USB_I_EVENT 	0x01
#define USB_O_EVENT 	0x02
#define SERVER_EVENT 	0x04
#define NRF_EVENT		0x08
#define VINE_EVENT		0x10
#define HOT_SWAP_EVENT	0x40
#define HEARTBEAT_EVENT 0x80

extern volatile uint16_t sys_event; // holds all events

/*
 * The main loop of the program
 */
void eventsLoop();
int events_tick();
void events_init();
int timer_init();

#endif /* EVENTS_H_ */
