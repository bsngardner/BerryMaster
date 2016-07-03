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
#include <stdio.h>

// Local includes
#include "events.h"
#include "berryMaster.h"
#include "IObuffer.h"
#include "server.h"
#include "ft201x.h"
#include "nrf.h"

// Macros
#define MAX_EVENT_ERRORS	10
#define HEARTBEAT_CNT		1000	 // ms
#define DEBOUNCE_CNT		50 // ms
#define HOT_SWAP_POLL_CNT	100 // ms
#define USB_POLL_CNT		16 // ms
#define USB_ACTIVE			((PJIN & ASDA) && (PJIN & ASCL))
#define NRF_PING_MAX		16
#define NRF_PING_MIN		1
#define LED_CNT				10

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
#define WDT_16S (WDTIS1_L | WDTIS0_L)
#define PET_WATCHDOG (WDTCTL = WDTPW | WDTSSEL0 | WDTCNTCL | WDT_16S)

// Global variables
static volatile int heartbeat_cnt; // when 0, trigger heartbeat event
static volatile int hot_swap_cnt; // when 0, check on the berries
static volatile int usb_poll_cnt = 0; // when 0, the usb is polled for data
static volatile int nrf_ping_cnt = 0;
static volatile int nrf_ping_timeout = 0;
static volatile int led_cnt = 0;
static volatile int debounceCnt; // debounce counter
volatile uint16_t sys_event; // holds all events

//Function prototypes
void events_usb_callback();
void events_server_callback();
void events_nrf_callback();

IObuffer* log_slot = 0;

#define USB_SOURCE
//#define NRF_SOURCE

//#define DEBOUNCE_SW

//Init all buffer and slot connections
void events_init()
{
	heartbeat_cnt = HEARTBEAT_CNT;
	hot_swap_cnt = HOT_SWAP_POLL_CNT;
	server_init();

#if defined(NRF_SOURCE)
	nrf_open(0); //open a a pipe, 0 indicates primary TX device
	server_slot = nrf_buffer;
	nrf_slot = server_buffer;
	log_slot = usb_buffer;
	//Start ping at high freq, it decays to min freq
	nrf_ping_cnt = nrf_ping_timeout = NRF_PING_MIN;

#elif defined(USB_SOURCE)
	server_slot = usb_buffer;
	usb_slot = server_buffer;
	log_slot = usb_buffer;
	usb_poll_cnt = USB_POLL_CNT;

#else
	nrf_open(0); //open a a pipe, 0 indicates primary TX device
	usb_slot = nrf_buffer;
	nrf_slot = usb_buffer;
	log_slot = usb_buffer;
	nrf_ping_cnt = nrf_ping_timeout = NRF_PING_MIN;
	usb_poll_cnt = USB_POLL_CNT;
	//Bad, no source defined for server
#endif

	usb_buffer->bytes_ready = events_usb_callback;
	server_buffer->bytes_ready = events_server_callback;
	nrf_buffer->bytes_ready = events_nrf_callback;

	PET_WATCHDOG;
}

void eventsLoop()
{
	char msg[80];
	// Wait for an interrupt
	while (1)
	{
		// Are there events available?
		__disable_interrupt();
		PET_WATCHDOG; // Pet the watchdog
		if (!sys_event)
		{
			// no events pending, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(SLEEP_MODE | GIE);
			continue;
		}
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
			else
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
			//sendPacket returns sem_fifo value; if nonzero and
			//	buffer is not empty, resignal radio event
			if (nrf_sendPacket() && nrf_buffer->count)
				sys_event |= NRF_EVENT;
			else
				LED1_OFF;
		}

		// Verify that berries are still connected and look for new ones
		else if (sys_event & HOT_SWAP_EVENT)
		{
			sys_event &= ~HOT_SWAP_EVENT;
			hot_swap_event();
			// debugging:
//			sprintf(msg, "In Hotswap %d", 32767);
//			send_log_msg(msg, log);
//			sprintf(msg, "2nd in hotswap");
//			send_log_msg(msg, warning);
		}

		// We're still alive
		else if (sys_event & HEARTBEAT_EVENT)
		{
			sys_event &= ~HEARTBEAT_EVENT;
			LED0_OFF;
			// debugging:
//			sprintf(msg, "In Heartbeat %d", 1235);
//			send_log_msg(msg, error);
		}

		// Error - Unrecognized event.
		else
		{
			sprintf(msg, "Unrecognized event");
			send_log_msg(msg, error_msg);
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

void events_nrf_callback()
{
	sys_event |= NRF_EVENT;
}

int events_tick()
{
	//If there is a ping count going and it decrements to 0,
	//	trigger ping event if buffer is empty,
	//	double timeout if less than max,
	//	set ping count to timeout.
	if (nrf_ping_timeout && !(--nrf_ping_cnt))
	{
		if (nrf_buffer->count == 0)
			sys_event |= NRF_EVENT;
		if (nrf_ping_timeout < NRF_PING_MAX)
			nrf_ping_timeout <<= 1;
		nrf_ping_cnt = nrf_ping_timeout;
	}

// Heartbeat
#define BLINK_TIME 200
	--heartbeat_cnt;
	if (heartbeat_cnt == BLINK_TIME)
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
	if (usb_poll_cnt && !(--usb_poll_cnt))
	{
		if (USB_ACTIVE)
			sys_event |= USB_I_EVENT; // poll the usb chip (ft201x)
		usb_poll_cnt = USB_POLL_CNT; // 1/16 sec
	}

#ifdef DEBOUNCE_SW
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
#endif

	return sys_event;
}

// Initialize the Watchdog Timer
int WDT_init()
{
	PET_WATCHDOG; // enable watchdog
	return SUCCESS;
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
#ifdef DEBOUNCE_SW
	debounceCnt = DEBOUNCE_CNT;
	P1IFG &= ~SW1;
#else
		WDTCTL = 0; //Reset the proc with a PUC
#endif
		break;
	default:
		break;
	}
}
