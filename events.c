/*
 * events.c
 *
 *  Created on: Mar 25, 2016
 *      Author: Berry_Admin
 */

// Standard includes
#include <msp430.h>

// Local includes
#include "events.h"

#include "berryMaster.h"
#include "IObuffer.h"
#include "iomatrix.h"
#include "ioprintf.h"
#include "server.h"
#include "nrf.h"
#include "ft201x.h"

// Macros
#define WDT_CLKS_PER_SEC	512				// 512 Hz WD clock (@32 kHz)
#define WDT_CTL				WDT_ADLY_1_9	// 1.95 ms
#define DEBOUNCE_CNT		80
#define USB_POLL_CNT		1
#define TEST_COUNT			50
#define MAX_EVENT_ERRORS	10

// Global variables
static volatile int WDT_cps_cnt; // one second counter
static volatile int usb_poll_cnt; // when 0, the usb is polled for data
static volatile int debounceCnt; // debounce counter
static volatile int test_cnt; // test counter
volatile uint16_t sys_event; // holds all events

//Function prototypes
void events_usbCallback();
void events_serverCallback();
void events_nrfCallback();
void events_testCallback();
void events_test();

IObuffer* test_slot;
IObuffer* test_buffer;

#define MASTER 0

//Init all buffer and slot connections
void events_init() {
	nrf_init(120, 1);
	nrf_open(0);
	nrf_buffer->bytes_ready = events_nrfCallback;
	test_cnt = TEST_COUNT;
	//server_init();
	test_buffer = IObuffer_create(120);
	test_buffer->bytes_ready = events_testCallback;

	usb_poll_cnt = 0;

	nrf_slot = test_buffer;
	test_slot = nrf_buffer;

//	usb_buffer->bytes_ready = events_usbCallback;
//	server_buffer->bytes_ready = events_serverCallback;

}

void eventsLoop() {
	int numEventErrors = 0;

	// Wait for an interrupt
	while (1) {
		// Are there events available?
		__disable_interrupt();
		if (!sys_event) {
			// no events pending, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(LPM0_bits | GIE);
			continue;
		} else {
			// At least 1 event is pending
			__enable_interrupt();

			// Output is ready to be sent to the host:
			if (sys_event & USB_O_EVENT) {
				sys_event &= ~USB_O_EVENT;
				if (USBOutEvent()) {
					// We're not finished, queue up this event again.
					sys_event |= USB_O_EVENT;
				}
				LED0_OFF;
			}

			//Radio data ready to send
			else if (sys_event & NRF_EVENT) {
				nrf_sendBytes();
				if (!nrf_buffer->count){
					sys_event &= ~NRF_EVENT;
					LED1_OFF;
				}
			}

			// Input is available from the host:
			else if (sys_event & USB_I_EVENT) {
				sys_event &= ~USB_I_EVENT;
				USBInEvent();
			}

			// Ready to servic a pending request from the host:
			else if (sys_event & SERVER_EVENT) {
				sys_event &= ~SERVER_EVENT;
				serverEvent();
			}

			else if (sys_event & TEST_RX_EVENT) {
				sys_event &= ~TEST_RX_EVENT;
				events_testRX();
			}

			//event for testing regular stuff
			else if (sys_event & TEST_EVENT) {
				sys_event &= ~TEST_EVENT;
				events_test();
			}

			// Ready to servic a pending request from the host:
			else if (sys_event & HEARTBEAT_EVENT) {
				sys_event &= ~HEARTBEAT_EVENT;
			}

			// Error - Unrecognized event.
			else {
				// TODO: do this right.
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

static char sharedVar = 0;
#define TIMEOUT_CNT 3
static char timeout = 1;

void events_test() {
	static char lshared = 1;
#if MASTER
	if(lshared == sharedVar && !(--timeout)){
		timeout = 1;
		LED0_OFF;
	}else{
		lshared = sharedVar;
	}
#else
	if(lshared != sharedVar){
		lshared = sharedVar;
		IOputc(lshared,nrf_buffer);
		LED0_OFF;
	}
#endif
}

void events_testRX() {
	static char lrx;
#if MASTER
	IOgetc(&lrx,test_buffer);
	LED0_ON;
	timeout = TIMEOUT_CNT;
	IOputc(lrx,nrf_buffer);
#else
	IOgetc(&lrx,test_buffer);
	if(test_buffer->count)
		sys_event |= TEST_RX_EVENT;
	if(lrx == sharedVar){
		sharedVar++;
		LED0_ON;
	}
#endif
}

void events_testCallback() {
	sys_event |= TEST_RX_EVENT;
}

void events_usbCallback() {
	sys_event |= USB_O_EVENT;
	SFRIFG1 |= WDTIFG;
}

void events_serverCallback() {
#if MASTER

#else
	sys_event |= SERVER_EVENT;
	SFRIFG1 |= WDTIFG;
#endif

}

void events_nrfCallback() {
	LED1_ON;
	sys_event |= NRF_EVENT;
}

// Reports an error to the user
void reportError(char* msg, int err, IObuffer* buff) {
	int byteCount;
	//buff = io_usb_out;
	// Fill up the buffer - it's easier for us to fill up the buffer all the
	// way than to try to count the size of each error message.
	// Because the length byte isn't part of the message, put size-1 as length.
	IOputc((char) (buff->size - 1), buff);
	// put the type of message (error) in the buffer
	IOputc((char) (TYPE_ERROR), buff);
	// put the error code in
	IOputc((char) err, buff);
	// put the message in - count how many bytes that is
	byteCount = IOputs(msg, buff);
	// is there space left in the buffer
	if (byteCount > 0) {
		// yes, fill up the remaining space with nulls
		while (IOputc(0, buff) == SUCCESS)
			;
	}
	// no, buffer is full - didn't finish putting message into buffer.
	// just send the message as is.
	while (USBOutEvent(buff))
		; // keep calling until it returns done.
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

	if (test_cnt && !(--test_cnt)) {
		test_cnt = TEST_COUNT;
		sys_event |= TEST_EVENT;
	}

	// One second elapsed
	--WDT_cps_cnt;
	if (WDT_cps_cnt == WDT_CLKS_PER_SEC / 4)
		//LED0_ON; // toggle heartbeat LED
		if (WDT_cps_cnt == 0) {
			WDT_cps_cnt = WDT_CLKS_PER_SEC;
			//sys_event |= HEARTBEAT_EVENT;
		}

	// Should we poll the USB?
	if (usb_poll_cnt && !(--usb_poll_cnt)) {
		sys_event |= USB_I_EVENT; // poll the usb chip (ft201x)
		usb_poll_cnt = USB_POLL_CNT; // 1/16 sec
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}

	// Are we currently debouncing the switch?
	if (debounceCnt) {
		// Yes; are we finished?
		--debounceCnt;
		if (debounceCnt == 0) {
			sharedVar = 0;
			// Toggle LED1. At a later time, we may want
			// to signal some kind of button event.
			//LED1_TOGGLE;
		}
	}

	if (sys_event)
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
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
//		if (P1IFG & USBINT) {
//			// Clear the pending interrupt.
//			P1IFG &= ~USBINT;
//		}
		break;
	}

}
