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

// Macros
#define WDT_CLKS_PER_SEC	512				// 512 Hz WD clock (@32 kHz)
#define WDT_CTL				WDT_ADLY_1_9	// 1.95 ms
#define DEBOUNCE_CNT		80
#define HOT_SWAP_POLL_CNT	(WDT_CLKS_PER_SEC>>1) // 1/2 second
#define USB_POLL_CNT		1
#define MAX_EVENT_ERRORS	10

// Global variables
static volatile int WDT_cps_cnt; // one second counter
static volatile int hot_swap_cnt; // when 0, check on the berries
static volatile int usb_poll_cnt; // when 0, the usb is polled for data
static volatile int debounceCnt; // debounce counter
volatile uint16_t sys_event; // holds all events

//Function prototypes
void events_usb_callback();
void events_server_callback();

//Init all buffer and slot connections
void events_init()
{
	server_init();
	server_slot = usb_buffer;
	usb_slot = server_buffer;

	ft201x_setUSBCallback(events_usb_callback);
	server_buffer->bytes_ready = events_server_callback;
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
		else
		{
			// At least 1 event is pending
			__enable_interrupt();

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

			// Verify that berries are still connected and look for new ones
			else if (sys_event & HOT_SWAP_EVENT)
			{
				sys_event &= ~HOT_SWAP_EVENT;
//				hot_swap_event();
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
	LED1_ON;
}

// Initialize the Watchdog Timer
int WDT_init()
{
	WDTCTL = WDT_CTL; // Set Watchdog interval
	SFRIE1 |= WDTIE; // Enable WDT interrupt

	WDT_cps_cnt = WDT_CLKS_PER_SEC;	// set WD 1 second counter
	usb_poll_cnt = USB_POLL_CNT;
	hot_swap_cnt = HOT_SWAP_POLL_CNT;
	return 0;
}

//-----------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	// One second elapsed
	--WDT_cps_cnt;
	if (WDT_cps_cnt == WDT_CLKS_PER_SEC / 4)
		LED0_ON; // toggle heartbeat LED
	if (WDT_cps_cnt == 0)
	{
		WDT_cps_cnt = WDT_CLKS_PER_SEC;
		sys_event |= HEARTBEAT_EVENT;
	}

	// Should we check on the berries?
	--hot_swap_cnt;
	if (hot_swap_cnt)
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

	// Wake up if a system event is pending
	if (sys_event)
	{
		__bic_SR_register_on_exit(LPM0_bits);
	}
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
