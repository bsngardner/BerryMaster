/*
 * events.h
 *
 *  Created on: Mar 25, 2016
 *      Author: Berry_Admin
 */

#ifndef EVENTS_H_
#define EVENTS_H_

// Events
#define USB_I_EVENT 	0x01
#define USB_O_EVENT 	0x02
#define SERVER_EVENT 	0x04
#define HOT_SWAP_EVENT	0x08
#define HEARTBEAT_EVENT 0x80

/*
 * The main loop of the program
 */
void eventsLoop();
void events_init();

/*
 * Initialize the Watchdog Timer
 */
int WDT_init();

#endif /* EVENTS_H_ */
