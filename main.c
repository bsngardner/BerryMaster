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
// Modify USB input event to accept commands from host
// Modify USB output event to correctly output a reply to the host
// Port over server code
// Add in SPI init code for radio
// Fix ft201x hardware interrupt - it is always pulling the line low, even
//   after we empty the hardware buffers (which should let the line float high)
// TODO: COULD USE ENTIRE LENGTH OF MESSAGE (ALL 64 BYTES) FOR DEBUG MESSAGE
// SO THE LENGTH IS THE SAME EVERY TIME - THIS WILL SIMPLIFY PRINTING DEBUG OR
// ERROR MESSAGES ON THE MASTER
// Standard includes
#include <msp430.h>
#include <stdint.h>

// Local includes
#include "BerryMaster.h"
#include "ft201x.h"
#include "IObuffer.h"
#include "i2c.h"
#include "server.h"
#include "hal.h"
#include "events.h"

// Local function prototypes
static int WDT_init();
static int setClock();
static int msp430init();

// Global Variables
static volatile int WDT_cps_cnt;
static volatile int usb_poll_cnt;
static volatile int debounceCnt;
static volatile int hal_cnt = HAL_COUNT;

extern IObuffer* io_usb_out; // MSP430 to USB buffer
extern uint16_t i2c_fSCL; // i2c timing constant

/*
 * main.c
 */
int main(void) {
	int numEventErrors = 0;

	// initialize the board
	if (msp430init()) {
		// error initializing the board - spin in an idle loop
		while (1)
			handleError();
	}
	hal_init();
	event_loop();
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
		reportError("WDTinit err", err);
		return err;
	}

	if (err = setClock()) { // init the clock (also on port J)
		reportError("setClk err", err);
		return err;
	}

	if (err = ft201x_init()) { // init the USB comm chip (ft201x)
		reportError("ft201x err", err);
		return err;
	}

	if (err = i2c_init()) { // init i2c
		reportError("i2c err", err);
		return err;
	}

	// Enable global interrupts
	__enable_interrupt();

	return 0;
}

// Reports an error to the user
void reportError(char* msg, int err) {
	IOputs(msg, io_usb_out);
	IOputc((char) (err + ASCII_ZERO), io_usb_out);
	IOputs("\n\r", io_usb_out);
	while (USBOutEvent())
		; // keep calling until it returns done.
}

// Just spins in an infinite loop
void handleError() {
	volatile int i = 0;
	volatile int j = 0;
#define DELAY -30000
#define DELAY2 5
	// todo: I don't know what to do if this happens besides blink LEDs
	// I may want to do some kind of system reset.
	// Include some close functions on the various peripherals, make sure
	// that I free memory

	__disable_interrupt(); // Disable interrupts
	WDTCTL = WDTPW | WDTHOLD; // Turn off watchdog
	ft201x_close();

	// Loop forever - short delay between toggling LEDs
	LED0_OFF;
	LED1_ON;
	while (1) {
		j = DELAY2;
		while (j-- != 0) {
			i = DELAY;
			while (i-- != 0)
				;
		}
		LEDS_TOGGLE;
	}
}
