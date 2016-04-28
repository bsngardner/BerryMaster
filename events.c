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
#define NRF_PING_MAX		2
#define NRF_PING_MIN		1
#define TEST_COUNT			0
#define MAX_EVENT_ERRORS	10

// Global variables
static volatile int WDT_cps_cnt; // one second counter
static volatile int usb_poll_cnt; // when 0, the usb is polled for data
static volatile int nrf_ping_cnt = 0;
static volatile int nrf_ping_timeout = 0;
static volatile int debounceCnt; // debounce counter
static volatile int test_cnt; // test counter
volatile uint16_t sys_event; // holds all events

//Function prototypes
void events_usbCallback();
void events_serverCallback();
void events_nrfCallback();

void events_test();
void events_testRX();

IObuffer* test_slot;
IObuffer* test_buffer;

#define MASTER 1

//Init all buffer and slot connections
void events_init()
{
	nrf_open(!MASTER);
	test_cnt = TEST_COUNT;

#if MASTER
	server_init();
	nrf_ping_cnt = nrf_ping_timeout = NRF_PING_MIN;
	usb_poll_cnt = 0;
	nrf_slot = server_buffer;
	server_slot = nrf_buffer;
	server_buffer->bytes_ready = events_serverCallback;
#else
	usb_poll_cnt = USB_POLL_CNT;
	nrf_slot = usb_buffer;
	usb_slot = nrf_buffer;
	usb_buffer->bytes_ready = events_usbCallback;
#endif

	nrf_buffer->bytes_ready = events_nrfCallback;
}

void eventsLoop()
{
	int numEventErrors = 0;

	// Wait for an interrupt
	while (1)
	{
		// Are there events available?
		__disable_interrupt();
		if (!sys_event)
		{
			// no events pending, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(LPM0_bits | GIE);
			continue;
		}
		// At least 1 event is pending
		__enable_interrupt();

		if (sys_event & NRF_PING_EVENT)
		{
			sys_event &= ~NRF_PING_EVENT;
			nrf_sendPing();
		}

		// Input is available from the host:
		else if (sys_event & USB_I_EVENT)
		{
			sys_event &= ~USB_I_EVENT;
			if (ft201x_powered())
				USBInEvent();
		}

		// Ready to servic a pending request from the host:
		else if (sys_event & SERVER_EVENT)
		{
			sys_event &= ~SERVER_EVENT;
			serverEvent();
		}

		//Radio data ready to send
		else if (sys_event & NRF_EVENT)
		{
			nrf_sendPacket();
			if (!nrf_buffer->count)
			{
				sys_event &= ~NRF_EVENT;
				LED1_OFF;
			}
		}

		// Output is ready to be sent to the host:
		else if (sys_event & USB_O_EVENT)
		{
			sys_event &= ~USB_O_EVENT;
			if (USBOutEvent())
			{
				// We're not finished, queue up this event again.
				sys_event |= USB_O_EVENT;
			}
		}

		//event for testing regular stuff
		else if (sys_event & TEST_EVENT)
		{
			sys_event &= ~TEST_EVENT;
			events_test();
		}

		// Ready to servic a pending request from the host:
		else if (sys_event & HEARTBEAT_EVENT)
		{
			sys_event &= ~HEARTBEAT_EVENT;
			LED0_OFF;
		}

		// Error - Unrecognized event.
		else
		{
			// TODO: do this right.
			//reportError("UnrecognizedEvent", SYS_ERR_EVENT, io_usb_out);

			// Clear all pending events -
			// attempt to let the system correct itself.
			sys_event = 0;

			// If the number of event errors reaches a predefined value,
			// stop the program
			numEventErrors++;
			if (numEventErrors >= MAX_EVENT_ERRORS)
			{
				handleError();
			}
		}

	}
}

void events_test()
{
	static long count = 0;
	iofprintf(nrf_buffer, "%ld, ", ++count);
	LED0_OFF;
}

void events_usbCallback()
{
	sys_event |= USB_O_EVENT;
}

void events_serverCallback()
{
	sys_event |= SERVER_EVENT;
#if MASTER
	nrf_ping_timeout = nrf_ping_cnt = NRF_PING_MIN;
	sys_event |= NRF_PING_EVENT;
#endif
}

void events_nrfCallback()
{
	LED1_ON;
	sys_event |= NRF_EVENT;
}

// Reports an error to the user
void reportError(char* msg, int err, IObuffer* buff)
{
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
	if (byteCount > 0)
	{
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
int WDT_init()
{
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
__interrupt void WDT_ISR(void)
{
	//If there is a ping count going and it decrements to 0,
	//	trigger ping event if buffer is empty,
	//	double timeout if less than max,
	//	set ping count to timeout.
	if (nrf_ping_timeout && !(--nrf_ping_cnt))
	{
		if (nrf_buffer->count == 0)
			sys_event |= NRF_PING_EVENT;
		if (nrf_ping_timeout < NRF_PING_MAX)
			nrf_ping_timeout <<= 1;
		nrf_ping_cnt = nrf_ping_timeout;
	}

	if (test_cnt && !(--test_cnt))
	{
		LED0_ON;
		test_cnt = TEST_COUNT;
		sys_event |= TEST_EVENT;
	}

// One second elapsed
	--WDT_cps_cnt;
	if (WDT_cps_cnt == WDT_CLKS_PER_SEC / 4)
	{
		LED0_ON; // toggle heartbeat LED
	}
	if (WDT_cps_cnt == 0)
	{
		WDT_cps_cnt = WDT_CLKS_PER_SEC;
		sys_event |= HEARTBEAT_EVENT;
	}

// Should we poll the USB?
	if (usb_poll_cnt && !(--usb_poll_cnt))
	{
		sys_event |= USB_I_EVENT; // poll the usb chip (ft201x)
		usb_poll_cnt = USB_POLL_CNT; // 1/16 sec
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}

// Are we currently debouncing the switch?
	if (debounceCnt)
	{
		// Yes; are we finished?
		--debounceCnt;
		if (debounceCnt == 0)
		{
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
enum
{
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
__interrupt void Port_1_ISR(void)
{
	switch (__even_in_range(P1IV, 0x10))
	{
	case NONE:
		break;
	case IV1_0: 	//INT
		spi_int();
		break;
	case IV1_3: //SW1
		WDTCTL = 0; //Reset the proc with a PUC
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
