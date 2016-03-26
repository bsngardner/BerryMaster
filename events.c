/*
 * events.c
 *
 *  Created on: Mar 25, 2016
 *      Author: Berry_Admin
 */

// Standard includes
#include <msp430.h>

// Local includes
#include "BerryMaster.h"
#include "events.h"
#include "IObuffer.h"
#include "server.h"
#include "ft201x.h"

// Macros
#define WDT_CLKS_PER_SEC	512				// 512 Hz WD clock (@32 kHz)
#define WDT_CTL				WDT_ADLY_1_9	// 1.95 ms
#define DEBOUNCE_CNT		80
#define USB_POLL_CNT		1
#define MAX_EVENT_ERRORS	10

// Global variables
static volatile int WDT_cps_cnt; // one second counter
static volatile int usb_poll_cnt; // when 0, the usb is polled for data
static volatile int debounceCnt; // debounce counter
volatile uint16_t sys_event; // holds all events
extern IObuffer* io_usb_out; // MSP430 to USB buffer

void eventsLoop() {
	int numEventErrors = 0;

	// Wait for an interrupt
    while(1) {
		// disable interrupts before check sys_event
    	__disable_interrupt();
    	if (!sys_event) {
			// no events pending, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(LPM0_bits | GIE);
			continue;
		}
    	else {
			// at least 1 event is pending, enable interrupts before servicing
			__enable_interrupt();

			// Service any pending events.
			if (sys_event & USB_I_EVENT) {
				sys_event &= ~USB_I_EVENT;
				USBInEvent();
			}
			else if (sys_event & USB_O_EVENT) {
				sys_event &= ~USB_O_EVENT;
				if (USBOutEvent(io_usb_out)) {
					// We're not finished, queue up this event again.
					sys_event |= USB_O_EVENT;
				}
			}
			else if (sys_event & SERVER_EVENT) {
				sys_event &= ~SERVER_EVENT;
				serverEvent();
			}
			else {
				// ERROR. Unrecognized event. Report it.
				//reportError("UnrecognizedEvent", SYS_ERR_EVENT, io_usb_out);

				// Clear all pending events -
				// attempt to let the system correct itself.
				sys_event = 0;

				// If the number of event errors reaches a predefined value,
				// stop the program
				numEventErrors++;
				if (numEventErrors >= MAX_EVENT_ERRORS) {
					handleError();
				}
			}
    	}
    }
}

// Reports an error to the user
void reportError(char* msg, int err, IObuffer* buff) {
	int byteCount;
	buff = io_usb_out;
	// Fill up the buffer - it's easier for us to fill up the buffer all the
	// way than to try to count the size of each error message.
	// Because the length byte isn't part of the message, put size-1 as length.
	IOputc((char)(buff->size-1), buff);
	// put the type of message (error) in the buffer
	IOputc((char)(TYPE_ERROR), buff);
	// put the error code in
	IOputc((char)err, buff);
	// put the message in - count how many bytes that is
	byteCount = IOputs(msg, buff);
	// is there space left in the buffer
	if (byteCount > 0) {
		// yes, fill up the remaining space with nulls
		while(IOputc(0, buff) == SUCCESS);
	}
	// no, buffer is full - didn't finish putting message into buffer.
	// just send the message as is.
	while (USBOutEvent(buff)); // keep calling until it returns done.
}

// Initialize the Watchdog Timer
int WDT_init() {
	WDTCTL = WDT_CTL; // Set Watchdog interval
	SFRIE1 |= WDTIE; // Enable WDT interrupt

	WDT_cps_cnt = WDT_CLKS_PER_SEC;	// set WD 1 second counter
	usb_poll_cnt = USB_POLL_CNT;
	return 0;
}

//-----------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {

	// One second elapsed
	--WDT_cps_cnt;
	if (WDT_cps_cnt == 0) {
		WDT_cps_cnt = WDT_CLKS_PER_SEC;
		LED0_TOGGLE; // toggle heartbeat LED
	}

	// Should we poll the USB?
	--usb_poll_cnt;
	if (usb_poll_cnt == 0) {
		sys_event |= USB_I_EVENT; // poll the usb chip (ft201x)
		usb_poll_cnt = USB_POLL_CNT; // 1/16 sec
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}

	// Are we currently debouncing the switch?
	if (debounceCnt) {
		// Yes; are we finished?
		--debounceCnt;
		if (debounceCnt == 0) {
			// Toggle LED1. At a later time, we may want
			// to signal some kind of button event.
			LED1_TOGGLE;
		}
	}
}

//-----------------------------------------------------------------------------
//	Port 1 ISR
//
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void)
{
	// USB interrupt?
	if (P1IFG & USBINT) {
		// Clear the pending interrupt.
		P1IFG &= ~USBINT;

		// Signal USB input event
		sys_event |= USB_I_EVENT;

		// Wake up the processor
		__bic_SR_register_on_exit(LPM0_bits);
	}

	// Switch1 interrupt?
	if (P1IFG & SW1) {
		debounceCnt = DEBOUNCE_CNT; // set debounce count
		P1IFG &= ~SW1; // clear interrupt flag
	}

	// Radio interrupt?
//	if (P1IFG & INT) {
//		P1IFG &= ~INT;
//		if (bdl_config.radio_cfg.radio_mode)
//			rf_irq |= RF24_IRQ_FLAGGED;
//		sys_event |= INTERRUPT;
//		__bic_SR_register_on_exit(LPM0_bits);
//	}


}
