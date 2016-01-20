/*
 * streamer.c
 *
 *  Created on: Jan 15, 2016
 *      Author: Berry_Admin
 */

#include <msp430.h>
#include "BerryMaster.h"
#include "pthreads.h"

// The list of devices on the network
extern DeviceList_t myDeviceList;

// The vine mutex
extern pthread_mutex_t vineMutex;

/*
 * streamerThread
 * This is a thread to be created using the pthreads library.
 * It continuously reads data from sensor berries and stores this data
 * in the network list on the master berry.
 */
void* streamerThread() {
	// Variables
	// i is an iterator
	// err holds the return status of a function (an error value or success)
	// notifyHost is 0 if no values were changed; 1 if there was an update
	int i, err, notifyHost;
	Device_t* dev;
	volatile int cnt = 0;
	while(1) {
		notifyHost = FALSE;
		// toggle LED every 50th iteration (I just made that number up)
		if (++cnt == 50) {
			cnt = 0;
			LED1_TOGGLE;
		}

		// Iterate through the network
		for (i = 0; i < MAX_NUM_DEVICES; i++) {
			// Use our local pointer - we'll be using this a lot.
			dev = &myDeviceList.devices[i];
			// Make sure there is a device (address is nonzero)
			if (dev->deviceAddress) {
				// Do something depending on the type of device.
				// We only really care about sensor devices.
				switch(dev->deviceType) {
				case UNKNOWN:
					break;
				case 1:
					break;
				case TYPE_LED:
					pthread_mutex_lock(&vineMutex);
					getDeviceValue(dev->deviceAddress, &dev->vals[0], REG_LED0);
					pthread_mutex_unlock(&vineMutex);
					//reportError(dev->vals[0], "led value");
					break;
				case 3:
					break;
				case 4:
					break;
				case 5:
					break;
				case TYPE_BUTTON:
					// Read the state of the button
					// todo: grab vine mutex
					/*if (err = getDeviceValue(dev->deviceAddress,
							&dev->vals[0], REG_BTN)) {
						// todo: Error
					}
					// todo: release vine mutex
					else {
						// todo: Success
						notifyHost = TRUE;
					}*/
					break;
				default:
					break;
				}
			}
		} // end for-loop

		// Finished reading the berries
		if (notifyHost) {
			// todo: values changed - send the host a message
		}
		//pthread_yield();
	}
}
