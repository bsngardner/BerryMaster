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

// Standard includes
#include <msp430.h>
#include <stdint.h>

// Local includes
#include "berryMaster.h"
#include "events.h"
#include "ft201x.h"
#include "nrf.h"
#include "hal.h"

// Local function prototypes
static int setClock();
static int msp430init();
static void error();

// Global Variables
extern uint16_t i2c_fSCL; // i2c timing constant

/*
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD; // Turn off watchdog
	// initialize the board
	__enable_interrupt();
	if (msp430init())
	{
		// error initializing the board - spin in an idle loop
		while (1)
			error();
	}

	events_init(); // also enables watchdog

	while (1)
	{
		eventsLoop();
	}
}

// Initialize the msp430 clock and crystal
static int setClock()
{
	CSCTL0 = CSKEY; 				// Enable Clock access
	CSCTL1 &= ~(DCORSEL | 0x0006); 	// Clear clock control bits
	CSCTL3 = 0;						// Clear all dividers
	CSCTL1 |= DCORSEL | 0x0006; 	// Set clock to 24 MHz

	// Initialize the crystal
	CSCTL4 |= (XT1DRIVE1 | XT1DRIVE0);
	// Wait for XT1 to start by trying to clear fault flag until it succeeds
	CSCTL4 &= ~XT1OFF; // Turn XT1 on--this might be done by itself.
	do
	{
		SFRIFG1 &= ~OFIFG;
		CSCTL5 &= ~XT1OFFG; // Will HANG if Xtal doesn't work
	} while (SFRIFG1 & OFIFG);

	CSCTL0_H = 0x00; // Disable Clock access

	return SUCCESS;
}

// Initialize ports, timers, clock, etc. for the msp430
static int msp430init()
{
	int err;

	// Initialize Port 1
	P1SEL0 = P1SEL1 = 0; // Port 1 is GPIO
	P1DIR = 0; // output pins = 1; input = 0
	// Init button interrupt on port 1:
	P1REN = SW1 | INT0 | BINT; // pull-up resistors: switch1, radio, vine intr
	P1IE = SW1 | INT0 | BINT; // enable interrupts
	P1IES = SW1 | INT0 | BINT; // interrupt on falling edge
	P1OUT = SW1 | INT0 | BINT; // initially high

	// Initialize Port 2
	P2SEL0 = P2SEL1 = 0; // Port 2 is GPIO
	P2DIR = RFCS | RFCE; // these are outputs, others are inputs
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

	// Special init functions for the peripherals:

	if (err = setClock())
	{ // init the clock (also on port J)
		return err;
	}

	if (err = timer_init())
	{ // init interval timer
		return err;
	}

	if (err = nrf_init(40, 2))
	{ //init radio
	  // (ends with radio powered down, pipe must be opened)
		return err;
	}

	if (err = ft201x_init())
	{ // init the USB comm chip (ft201x)
		return err;
	}

	if (err = hal_init())
	{ // init vine communication
		return err;
	}

	return SUCCESS;
}

// Just spins in an infinite loop
void error()
{
	volatile int i = 0;
	volatile int j = 0;
#define DELAY -30000
#define DELAY2 5
	// todo: Change this back to Professor Roper's error, where the
	// LED's blink the error code.
	// I may want to do some kind of system reset.
	// Include some close functions on the various peripherals, make sure
	// that I free memory

	__disable_interrupt(); // Disable interrupts
	WDTCTL = WDTPW | WDTHOLD; // Turn off watchdog
	ft201x_close();

	// Loop forever - short delay between toggling LEDs
	LED0_OFF;
	LED1_ON;
	while (1)
	{
		j = DELAY2;
		while (j-- != 0)
		{
			i = DELAY;
			while (i-- != 0)
				;
		}
		LEDS_TOGGLE;
	}
}
