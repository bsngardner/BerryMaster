/*
 * main.c
 * author: Marshall Garey
 * date created: Nov 18, 2015
 *
 * This project is for the Berry Master, which uses the MSP430FR5738.
 *
 *
 * Pinout:
 * Pin#	Dir	Part		Pin#		Dir	Part
 * Xin		Xin		|	2.4/A7		I	MIBO (Master in Berry out)
 * Xout		Xout	| 	2.3			O	MOBI (Master out Berry in)
 * AVSS		GND		|	DVCC		X	VDD
 * AVCC		VDD		|	DVSS		X	Gnd
 * 1.0	I	Int0	|	VCore		X	X
 * 1.1	I	Int1	|	1.7/UCB0SCL	O	SCL
 * 1.2	O	BCLK	|	1.6/UCB0SDA	X	SDA
 * 1.3	I	SW1		|	2.2			I	BCHG
 * 1.4	I	USBInt	|	2.1/UCA0RX0	I	MISO
 * 1.5	O	SCLK	|	2.0/UCA0TX0	O	MOSI
 * J.0	O	LED0	|	Rst			I	Rst
 * J.1	O	LED1	|	Test			Test
 * J.2	O	ASDA	|	2.6			O	RFCE (Radio chip enable)
 * J.3	O	ASCL	|	2.5			O	RFCS (Radio chip select)
 */

// TODO's:
// Develop streamer thread
// Malloc memory for berries rather than preallocating memory
// Add Prof Roper's ERROR2() for handling errors in initialization code
// Fix up ft201x code
// Add in SPI init code for radio
// Fix ft201x hardware interrupt - it is always pulling the line low, even
//   after we empty the hardware buffers (which should let the line float high)


// Standard includes
#include <msp430.h>
#include <stdint.h>

// Local includes
#include "BerryMaster.h"
#include "ft201x.h"
#include "IObuffer.h"
#include "i2c.h"
#include "server.h"
#include "pthreads.h"
#include "streamer.h"

// Macros
#define WDT_CLKS_PER_SEC	512				// 512 Hz WD clock (@32 kHz)
#define WDT_CTL				WDT_ADLY_1_9	// 1.95 ms
#define DEBOUNCE_CNT		80
#define USB_POLL_CNT		4
#define MAX_EVENT_ERRORS	10

// Local function prototypes
static int WDT_init();
static int setClock();
static int msp430init();
static int threadsInit();

// Global Variables
static volatile int WDT_cps_cnt; // one second counter
static volatile int usb_poll_cnt; // when 0, the usb is polled for data
static volatile int debounceCnt; // debounce counter
extern IObuffer* io_usb_out; // MSP430 to USB buffer
extern uint16_t i2c_fSCL; // i2c timing constant
volatile uint16_t sys_event; // holds all events
pthread_t pthreadHandle_server; // handle to server thread
pthread_t pthreadHandle_streamer; // handle to streamer thread
pthread_mutex_t vineMutex; // mutex for the vine communication
pthread_mutex_t hostMutex; // mutex for the host communication (USB)

/*
 * main.c
 */
int main(void) {
	int numEventErrors = 0;

	static int cnt = 0; // for debugging

	// initialize the board
	if (msp430init()) {
		// error initializing the board - spin in an idle loop
		while(1) handleError();
	}

	// initialize threads
	if (threadsInit()) {
		while(1) handleError();
	}

    // Enable global interrupts after all initialization is finished.
    __enable_interrupt();

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

				/* if (++cnt >= 50) {
					// just for debugging
					reportError("50 server events", 50, io_usb_out);
					while (cnt++);
					cnt = 0;
				} */
			}
			else {
				// ERROR. Unrecognized event. Report it.
				reportError("UnrecognizedEvent", SYS_ERR_EVENT, io_usb_out);

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

// Initialize the msp430 clock and crystal
static int setClock() {
	CSCTL0 = CSKEY; 				// Enable Clock access
	CSCTL1 &= ~(DCORSEL | 0x0006); 	// Clear clock control bits
	CSCTL3 = 0;						// Clear all dividers
	CSCTL1 |= DCORSEL | 0x0006; 	// Set clock to 24 MHz
	i2c_fSCL = (24000 / I2C_FSCL);	// fSCL

	// Initialize the crystal
	CSCTL4 |= (XT1DRIVE1 | XT1DRIVE0);
// Wait for XT1 to start by trying to clear fault flag until it succeeds
	CSCTL4 &= ~XT1OFF; // Turn XT1 on--this might be done by itself.
	do {
		SFRIFG1 &= ~OFIFG;
		CSCTL5 &= ~XT1OFFG; // Will HANG if Xtal doesn't work
	} while (SFRIFG1 & OFIFG);

	CSCTL0_H = 0x00; // Disable Clock access

	return 0; // success
}

// Initialize the Watchdog Timer
static int WDT_init() {
	WDTCTL = WDT_CTL; // Set Watchdog interval
	SFRIE1 |= WDTIE; // Enable WDT interrupt

	WDT_cps_cnt = WDT_CLKS_PER_SEC;	// set WD 1 second counter
	usb_poll_cnt = USB_POLL_CNT; // 1/16 sec
	return 0;
}

// Initialize ports, timers, clock, etc. for the msp430
static int msp430init() {
	int err;

	// Initialize Port 1
	P1SEL0 = P1SEL1 = 0; // Port 1 is GPIO
	P1DIR = BCLK; // output pins = 1; input = 0
	// Init button interrupt on port 1:
	P1REN = SW1 | USBINT; // enable resistors on switch1 and USBINT
	P1IE = SW1 | USBINT; // enable interrupt for sw1 and usb
	P1IES = SW1 | USBINT; // interrupt on falling edge
	P1OUT = SW1 | INT0 | INT1 | USBINT;

	// Initialize Port 2
	P2SEL0 = P2SEL1 = 0; // Port 2 is GPIO
	P2DIR = MOSI | MOBI | RFCS | RFCE; // these are outputs, others are inputs
	P2REN = BCHG; // pull-up on battery charger
	P2OUT = RFCS | BCHG; // high on radio chip select and battery charger

	// Initialize Port J
	// first 4 pins are GPIO, J.4 and J.5 are crystal pins, J.6 and J.7 unused
	// Set J.4 and J.5 PJSEL1 to 0, PJSEL0 to 1 for use as a crystal
	PJSEL0 = 0x30;
	PJSEL1 = 0;
	PJREN &= ~0x0f; // no pull-up resistors
    PJDIR = LED0 | LED1 | 0xC0; // LEDs and unused pins are output
	PJOUT &= ~0x0f; // all output pins low

    // Special init functions for the peripherals - if an error occurs
    // (a function returns non-zero), then print the error to the console:

	if (err = WDT_init()) { // init the watchdog timer
		return err;
	}

    if (err = setClock()) { // init the clock (also on port J)
    	return err;
    }

    if (err = ft201x_init()) { // init the USB comm chip (ft201x)
    	return err;
    }

    if (err = i2c_init()) { // init i2c
    	return err;
    }

	return 0;
}

// Initialize pthreads. Main is the server thread.
// Create a streamer thread.
// Create and initialize any mutexes.
// Return 0 if successful, nonzero if failed.
static int threadsInit() {
	int error;

	// Initialize pthreads. Use defaults (NULL argument).
	if (error = pthread_init(NULL)) {
		reportError("Err-pthread_init", error, io_usb_out);
		return error;
	}

	// Create server and streamer threads
	pthreadHandle_server = pthread_self();
	if (error = pthread_create(&pthreadHandle_streamer, NULL,
			streamerThread, NULL)) {
		reportError("Err-pthread_create(streamer)", error, io_usb_out);
		return error;
	}

	// Initialize mutexes
	pthread_mutex_init(&vineMutex, NULL); // vine mutex, unblocked
	pthread_mutex_init(&hostMutex, NULL); // host mutex, unblocked
	return 0;
}

// Reports an error to the user
// todo: this is a problem. it could be that because of
// this the master is missing a request from the host...
// but I don't think so. look into this.
// This function isn't working properly...
void reportError(char* msg, int err, IObuffer* buff) {
	int byteCount;
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

// Just spins in an infinite loop
void handleError() {
	volatile int i = 0;
	volatile int j = 0;
#define DELAY -30000
#define DELAY2 5
	// todo: I don't know what to do besides blink LEDs.
	// I may want to do some kind of system reset.
	// Include some close functions on the various peripherals, make sure
	// that I free memory

	__disable_interrupt(); // Disable interrupts
	WDTCTL = WDTPW | WDTHOLD; // Turn off watchdog
	ft201x_close();
	// todo: pthread_mutex_destroy() on each mutex
	// todo: pthread_join(pthreadHandle_streamer);

	// Loop forever - short delay between toggling LEDs
	LED0_OFF;
	LED1_ON;
	while (1) {
		j = DELAY2;
		while (j-- != 0) {
			i = DELAY;
			while (i-- != 0);
		}
		LEDS_TOGGLE;
	}
}

//-----------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {

	--WDT_cps_cnt;
	--usb_poll_cnt;

	// One second elapsed
	if (WDT_cps_cnt == 0) {
		WDT_cps_cnt = WDT_CLKS_PER_SEC;
		LED0_TOGGLE; // toggle heartbeat LED
	}

	// Should we poll the USB?
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

