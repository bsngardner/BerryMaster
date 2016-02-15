/*
 * events.c
 *
 *  Created on: Jan 13, 2016
 *      Author: Broderick
 */

#include "msp430.h"
#include "events.h"
#include "BerryMaster.h"
#include <stdint.h>
#include "hal.h"
#include "ft201x.h"
#include "server.h"
#include "ioprintf.h"
#include "spi.h"
#include "IObuffer.h"
#include "nrfradio.h"

volatile uint16_t sys_event;

//Defines
#define TEST_COUNT 0
#define USB_POLL_COUNT 10

//Variables
volatile int test_cnt = 0;
volatile int usb_poll_cnt = 0;

// Global Variables
volatile int WDT_cps_cnt;
volatile int debounceCnt;
volatile uint16_t WDT_delay;

void event_loop() {
	int numEventErrors = 0;
	usb_poll_cnt = USB_POLL_COUNT;
	test_cnt = TEST_COUNT;
	iofprintf(io_usb_out, "Listen!\n\r");
	// Wait for an interrupt
	while (1) {
		// disable interrupts before check sys_event
		__disable_interrupt();

		if (sys_event) {
			// if there's something pending, enable interrupts before servicing
			__enable_interrupt();
		} else {
			// otherwise, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(LPM0_bits | GIE);
			continue;
		}

		// Service any pending events.
		if (sys_event & USB_I_EVENT) {
			sys_event &= ~USB_I_EVENT;
			USBInEvent();
		} else if (sys_event & USB_O_EVENT) {
			sys_event &= ~USB_O_EVENT;
			USBOutEvent();
		} else if (sys_event & SERVER_EVENT) {
			sys_event &= ~SERVER_EVENT;
			serverEvent();
		} else if (sys_event & TEST_EVENT) {
			sys_event &= ~TEST_EVENT;
			test_event();
		} else {
			// ERROR. Unrecognized event. Report it.
			reportError("UnrecognizedEventErr", SYS_ERR_EVENT);

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

#define TYPE_REG 0
#define STATUS_REG 1

#define LED_T 0x02
#define SW_T 0x06

static struct {
	uint8_t addr;
	uint8_t type;
	uint8_t reg;
	uint8_t val;
} devices[8] = { 0 };

int i, k;
uint8_t addr = 0x1;
uint8_t new = 0;
uint8_t reset = 0;

void test_event() {
	LED0_OFF;
}

void send_bytes(IObuffer* iob) {
	radio_sendBytes(0, iob);
}

void WDTdelay(uint16_t time) {
	WDT_delay = time;
	while (WDT_delay)
		LPM0;
}
//-----------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {

	if (test_cnt && !(--test_cnt)) {
		test_cnt = TEST_COUNT;
		sys_event |= TEST_EVENT;
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}

	if (usb_poll_cnt && !(--usb_poll_cnt)) {
		usb_poll_cnt = USB_POLL_COUNT;
		sys_event |= USB_I_EVENT;
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}
//
//	if (!(P1IN & USBINT)) {
//		sys_event |= USB_I_EVENT;
//		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
//
//	}

	--WDT_cps_cnt;
	// One second elapsed
	if (WDT_cps_cnt == 10) {
		LED0_ON;
	}
	if (WDT_cps_cnt == 0) {
		WDT_cps_cnt = WDT_CLKS_PER_SEC;
		sys_event |= TEST_EVENT;
	}

	// Are we currently debouncing the switch?
	if (debounceCnt) {
		// Yes; are we finished?
		--debounceCnt;
		if (debounceCnt == 0) {
			//test_cnt = TEST_COUNT;
			// TODO: Signal button event.
		}
	}

	if (WDT_delay && !(--WDT_delay))
		__bic_SR_register_on_exit(LPM0_bits);
}

//-----------------------------------------------------------------------------
//	Port 1 ISR
//
enum {
	NONE = 0x00, IV1_0 = 0x02, //bit0,0x01
	IV1_1 = 0x04, //bit1,0x02
	IV1_2 = 0x06, //bit2,0x04
	IV1_3 = 0x08, //bit3,0x08
	IV1_4 = 0x0a, //bit4,0x10
	IV1_5 = 0x0c, //bit5,0x20
	IV1_6 = 0x0e, //bit6,0x40
	IV1_7 = 0x10  //bit7,0x80
};
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void) {

	switch (__even_in_range(P1IV, 0x10)) {
	case NONE:
		break;
	case IV1_0: 	//INT
		spi_start_read();
		P1IFG &= ~INT0;
		break;
	case IV1_3: //SW1
		debounceCnt = DEBOUNCE_CNT;
		P1IFG &= ~SW1;
		break;
	default:
		// USB interrupt?
		if (P1IFG & USBINT) {
			// Clear the pending interrupt.
			P1IFG &= ~USBINT;

			// Signal USB input event
			//sys_event |= USB_I_EVENT;

			// Wake up the processor
			//__bic_SR_register_on_exit(LPM0_bits);
		}

		break;
	}

}

