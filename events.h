/*
 * events.h
 *
 *  Created on: Jan 13, 2016
 *      Author: Broderick
 */

#ifndef EVENTS_H_
#define EVENTS_H_
#include <stdint.h>
#include "IObuffer.h"

//Defines
// Events
#define USB_I_EVENT 0x01
#define USB_O_EVENT 0x02
#define SERVER_EVENT 0x04
#define TEST_EVENT 0x08

// Macros
#define WDT_CLKS_PER_SEC	512				// 512 Hz WD clock (@32 kHz)
#define WDT_CTL				WDT_ADLY_1_9	// 1.95 ms
#define DEBOUNCE_CNT		20
#define USB_POLL_CNT		32
#define MAX_EVENT_ERRORS	10

//Extern variables
extern volatile uint16_t sys_event;
//WDT
extern volatile int WDT_cps_cnt;
extern volatile int usb_poll_cnt;
extern volatile int debounceCnt;
extern volatile int hal_cnt;

//Functions
extern void event_loop();
extern void test_event();
extern void send_bytes(IObuffer*);

#endif /* EVENTS_H_ */
