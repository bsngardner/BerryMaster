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
#include "BerryMaster.h"
#include "ft201x.h"
#include "IObuffer.h"
#include "i2c.h"

// Macros
#define WDT_CLKS_PER_SEC	512				// 512 Hz WD clock (@32 kHz)
#define WDT_CTL				WDT_ADLY_1_9	// 1.95 ms
#define DEBOUNCE_CNT		20

// Local function prototypes
static int WDT_init();
static int setClock();
static int msp430init();

// Global Variables
static volatile int WDT_cps_cnt;
static volatile int usb_poll_cnt;
static volatile int debounceCnt;
volatile uint16_t sys_event;
extern IObuffer* io_usb_out; // MSP430 to USB buffer
extern uint16_t i2c_fSCL; // i2c timing constant

#define STRING "HELLO WORLD!!\r\n"

/*
 * main.c
 */
int main(void) {

	// initialize the board
	msp430init();

	sys_event |= USB_I_EVENT;

	// Wait for an interrupt
    while(1) {
		// disable interrupts before check sys_event
		__disable_interrupt();

		if (sys_event) {
			// if there's something pending, enable interrupts before servicing
			__enable_interrupt();
		}
		else {
			// otherwise, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(LPM0_bits + GIE);
			continue;
		}

		// Service any pending events.
		if (sys_event & USB_I_EVENT) {
			sys_event &= ~USB_I_EVENT;
			USBInEvent();
		}
		else if (sys_event & USB_O_EVENT) {
			sys_event &= ~USB_O_EVENT;
			USBOutEvent();
		}
		// TODO: Get rid of this event. It was just for testing writing
		// output to the computer.
		else if (sys_event & WRITE_DEBUG_EVENT) {
			sys_event &= ~WRITE_DEBUG_EVENT;
			// Write to the IO buffer and send info to computer
			IOputs(STRING, io_usb_out);
			IOputc(0, io_usb_out);
		}
		else {
			ERROR(SYS_ERR_EVENT); // ERROR. Unrecognized event.
		}
    }

//	return 0;
}

static int setClock() {
	CSCTL0 = CSKEY; 				// Enable Clock access
	CSCTL1 &= ~(DCORSEL | 0x0006); 	// Clear clock control bits
	CSCTL3 = 0;						// Clear all dividers
	CSCTL1 |= DCORSEL | 0x0006; 	// Set clock to 24 MHz
	i2c_fSCL = (24000 / I2C_FSCL);	// fSCL

	// Initialize the crystal
	PJSEL0 |= 0x10;					// Xtal mode for pin 4
	PJSEL1 &= 0x10;					// Xtal mode for pin 4
// It might be good to add code for pin 5

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
	usb_poll_cnt = WDT_CLKS_PER_SEC >> 4; // 1/16 sec
	return 0;
}

// Initialize ports, timers, clock, etc. for the msp430
static int msp430init() {
	ERROR(WDT_init()); // init the watchdog timer

	// Initialize Port 1
	P1SEL0 = P1SEL1 = 0; // Port 1 is GPIO
	P1DIR = BCLK; // output pins = 1; input = 0
	// Init button interrupt on port 1:
	P1REN = SW1 | USBINT; // enable resistors on switch1 and USBINT
	P1IE = SW1 | USBINT; // enable interrupt for sw1 and usb
	P1IES = SW1 | USBINT; // interrupt on falling edge
	P1OUT = SW1 | INT0 | INT1 | USBINT;

	// TODO: Initialize Port 2

	// Initialize Port J
	PJSEL0 &= ~0x0f; // first 4 pins are GPIO (J.4 and J.5 are crystal pins)
	PJSEL1 &= ~0x0f; // GPIO
	PJREN &= ~0x0f; // no pull-up resistors
    PJDIR = LED0 | LED1; // set LED pins as output, all others as input
	PJOUT &= ~0x0f; // all pins low
    LED0_OFF; // LED0 off
    LED1_OFF; // LED1 off

    // Special init functions for the peripherals
    ERROR(setClock()); // init the clock (also on port J)
    ERROR(ft201x_init()); // Initialize the USB comm chip (ft201x)
    ERROR(i2c_init());

    // Enable global interrupts
    __enable_interrupt();

	return 0;
}

//******************************************************************************
//	report hard error
//
void ERROR(int error)
{
	ERROR2(0, error);
} // end ERROR

void ERROR2(int class, int error)
{
	volatile int i, j, k;

// return if no error
	if (error == 0)
		return;

	__disable_interrupt();
	//todo: BDL_init(_1MHZ); // system reset @1 MHz

	k = 10;
	while (k--)
	{
		// pause
		LED0_OFF;
		for (i = 4; i > 0; --i)
			for (j = -1; j; --j)
				;

		// flash LED's 10 times
		i = 10;
		while (i--)
		{
			LED0_TOGGLE;
			for (j = 8000; j > 0; --j)
				;
		}

		// pause
		LED0_OFF;
		for (i = 1; i > 0; --i)
			for (j = -1; j; --j)
				;

		// now blink class
		for (i = class; i > 0; --i)
		{
			LED0_ON;
			for (j = -1; j; --j)
				;
			LED0_OFF;
			for (j = -1; j; --j)
				;
		}

		// pause again
		LED0_OFF;
		for (i = 1; i > 0; --i)
			for (j = -1; j; --j)
				;
		for (i = 1; i > 0; --i)
			for (j = -1; j; --j)
				;

		// now blink error #
		for (i = error; i > 0; --i)
		{
			LED0_ON;
			for (j = -1; j; --j)
				;
			LED0_OFF;
			for (j = -1; j; --j)
				;
		}
	}
	WDTCTL = 0; //die
} // end ERROR2

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

//		// Signal write-debug event just for fun. Wake up to service it.
//    	sys_event |= WRITE_DEBUG_EVENT;
//    	__bic_SR_register_on_exit(LPM0_bits);
	}

	// Should we poll the USB?
	if (usb_poll_cnt == 0) {
		sys_event |= USB_I_EVENT; // poll the usb chip (ft201x)
		usb_poll_cnt = WDT_CLKS_PER_SEC >> 4; // 1/16 sec
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}

	// Are we currently debouncing the switch?
	if (debounceCnt) {
		// Yes; are we finished?
		--debounceCnt;
		if (debounceCnt == 0) {
			// TODO: Signal button event. Remove toggling the LED.
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

		// Signal write-debug event just for fun. Wake up to service it.
    	// sys_event |= WRITE_DEBUG_EVENT;
    	__bic_SR_register_on_exit(LPM0_bits);

    	sys_event |= USB_I_EVENT;

//		IOputc('a', io_usb_out);
//		IOputc('\r', io_usb_out);
//		IOputc(0, io_usb_out);

//		LED1_TOGGLE;

		// Signal USB input event
//		sys_event |= USB_I_EVENT;

		// Wake up the processor
//		__bic_SR_register_on_exit(LPM0_bits);

	}

	// Switch1 interrupt?
	if (P1IFG & SW1) {
		debounceCnt = DEBOUNCE_CNT;
		P1IFG &= ~SW1;
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

