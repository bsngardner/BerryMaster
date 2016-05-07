/*
 * events.c
 *
 *  Created on: Mar 25, 2016
 *      Authors:
 *      Marshall Garey
 *      Broderick Gardner
 */

// Standard includes
#include <msp430.h>

// Local includes
#include "events.h"
#include "berryMaster.h"
#include "IObuffer.h"
#include "server.h"
#include "ft201x.h"
#include "nrf.h"

// Macros
#define MAX_EVENT_ERRORS	10
#define TIMERA_INTERVAL		32 // @32 kHz, that's ~1ms
#define HEARTBEAT_CNT		2 // ms
#define DEBOUNCE_CNT		50 // ms
#define HOT_SWAP_POLL_CNT	250 // ms
#define USB_POLL_CNT		1 // ms
#define NRF_PING_MAX		16
#define NRF_PING_MIN		1
#define SLEEP_MODE			LPM1_bits

// Pet the watchdog:
// WDTSSEL0 - select ACLK;
// WDTCNTCL - clear timer
/* WDTIS (lowest 3 bits):
 000b = Watchdog clock source / (2^(31)) (18:12:16 at 32.768 kHz)
 001b = Watchdog clock source /(2^(27)) (01:08:16 at 32.768 kHz)
 010b = Watchdog clock source /(2^(23)) (00:04:16 at 32.768 kHz)
 011b = Watchdog clock source /(2^(19)) (00:00:16 at 32.768 kHz)
 100b = Watchdog clock source /(2^(15)) (1 s at 32.768 kHz)
 101b = Watchdog clock source / (2^(13)) (250 ms at 32.768 kHz)
 110b = Watchdog clock source / (2^(9)) (15.625 ms at 32.768 kHz)
 111b = Watchdog clock source / (2^(6)) (1.95 ms at 32.768 kHz)
 */
#define WDT_15MS (WDTIS2_L | WDTIS1_L | WDTIS0_L)
#define WDT_250MS (WDTIS2_L | WDTIS0_L)
#define WDT_1S (WDTIS2_L)
#define PET_WATCHDOG WDTCTL = (WDTPW | WDTSSEL0 | WDTCNTCL | WDT_1S | WDTHOLD)
//TODO Reenable watchdog

// Global variables
static volatile int heartbeat_cnt; // when 0, trigger heartbeat event
static volatile int hot_swap_cnt; // when 0, check on the berries
static volatile int usb_poll_cnt = 0; // when 0, the usb is polled for data
static volatile int nrf_ping_cnt = 0;
static volatile int nrf_ping_timeout = 0;
static volatile int debounceCnt; // debounce counter
volatile uint16_t sys_event; // holds all events

//Function prototypes
void events_usb_callback();
void events_server_callback();
void events_nrf_callback();

IObuffer* log_slot = 0;

//#define USB_SOURCE
#define NRF_SOURCE

//Init all buffer and slot connections
void events_init()
{
	heartbeat_cnt = HEARTBEAT_CNT;
	hot_swap_cnt = HOT_SWAP_POLL_CNT;
	server_init();

#ifdef NRF_SOURCE
	nrf_open(0); //open a a pipe, 0 indicates primary TX device
	server_slot = nrf_buffer;
	nrf_slot = server_buffer;
	log_slot = usb_buffer;
	//Start ping at high freq, it decays to min freq
	nrf_ping_cnt = nrf_ping_timeout = NRF_PING_MIN;

#elif USB_SOURCE
	server_slot = usb_buffer;
	usb_slot = server_buffer;
	log_slot = 0;
	usb_poll_cnt = USB_POLL_CNT;

#else
	//Bad, no source defined for server
#endif

	usb_buffer->bytes_ready = events_usb_callback;
	server_buffer->bytes_ready = events_server_callback;
	nrf_buffer->bytes_ready = events_nrf_callback;
}

void eventsLoop()
{
	int numEventErrors = 0;

	PET_WATCHDOG; // Pet the watchdog - 250 ms
	// Wait for an interrupt
	while (1)
	{
		// Are there events available?
		__disable_interrupt();
		if (!sys_event)
		{
			// no events pending, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(SLEEP_MODE | GIE);
			continue;
		}
		else
		{
			// At least 1 event is pending
			__enable_interrupt();

			PET_WATCHDOG; // Pet the watchdog - 250 ms

			// Output is ready to be sent to the host:
			if (sys_event & USB_O_EVENT)
			{
				sys_event &= ~USB_O_EVENT;
				if (usb_out_event())
				{
					// We're not finished, queue up this event again.
					sys_event |= USB_O_EVENT;
				}
				LED1_OFF;
			}

			// Input is available from the host:
			else if (sys_event & USB_I_EVENT)
			{
				sys_event &= ~USB_I_EVENT;
				usb_in_event();
			}

			// Ready to service a pending request from the host:
			else if (sys_event & SERVER_EVENT)
			{
				sys_event &= ~SERVER_EVENT;
				server_event();
			}

			// Radio
			else if (sys_event & NRF_EVENT)
			{
				sys_event &= ~NRF_EVENT;
				nrf_sendPacket();
			}

			// Verify that berries are still connected and look for new ones
			else if (sys_event & HOT_SWAP_EVENT)
			{
				sys_event &= ~HOT_SWAP_EVENT;
				hot_swap_event();
			}

			// We're still alive
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
}

void events_usb_callback()
{
	sys_event |= USB_O_EVENT;
}

void events_server_callback()
{
	sys_event |= SERVER_EVENT;
}

void events_nrf_callback()
{
	sys_event |= NRF_EVENT;
}

int events_tick()
{
	// Heartbeat
	--heartbeat_cnt;
	if (heartbeat_cnt == HEARTBEAT_CNT/2)
	{
		LED0_ON; // toggle heartbeat LED
	}
	if (heartbeat_cnt == 0)
	{
		heartbeat_cnt = HEARTBEAT_CNT;
		sys_event |= HEARTBEAT_EVENT; // trigger heartbeat event
	}

	// Should we check on the berries?
	--hot_swap_cnt;
	if (hot_swap_cnt == 0)
	{
		hot_swap_cnt = HOT_SWAP_POLL_CNT;
		sys_event |= HOT_SWAP_EVENT;
	}

	// Should we poll the USB?
	--usb_poll_cnt;
	if (usb_poll_cnt == 0)
	{
		sys_event |= USB_I_EVENT; // poll the usb chip (ft201x)
		usb_poll_cnt = USB_POLL_CNT; // 1/16 sec
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
			LED1_TOGGLE;
		}
	}
	return sys_event;
}

// Initialize the Watchdog Timer
int WDT_init()
{
	PET_WATCHDOG;
	return SUCCESS;
}

//-----------------------------------------------------------------------------
//	Port 1 ISR
//
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void)
{
	// USB interrupt?
	if (P1IFG & USBINT)
	{
		// Clear the pending interrupt.
		P1IFG &= ~USBINT;

		// Signal USB input event
		sys_event |= USB_I_EVENT;

		// Wake up the processor
		__bic_SR_register_on_exit(LPM0_bits);
	}

	// Switch1 interrupt?
	if (P1IFG & SW1)
	{
		debounceCnt = DEBOUNCE_CNT; // set debounce count
		P1IFG &= ~SW1; // clear interrupt flag
	}
}
