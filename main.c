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
static void setClock();
static int msp430init();

// Global Variables
static volatile int WDT_cps_cnt;
static volatile int debounceCnt;
volatile uint16_t sys_event;
extern IObuffer* io_usb_out; // MSP430 to USB buffer
extern uint16_t i2c_fSCL; // i2c timing constant

#define STRING "HELLO WORLD\r"

/*
 * main.c
 */
int main(void) {

	// initialize the board
	msp430init();

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
			// todo: ERROR. Unrecognized event.
		}
    }

//	return 0;
}

static void setClock() {
	CSCTL0 = CSKEY; 				// Enable Clock access
	CSCTL1 &= ~(DCORSEL | 0x0006); 	// Clear clock control bits
	CSCTL3 = 0;						// Clear all dividers
	CSCTL1 |= DCORSEL | 0x0006; 	// Set clock to 24 MHz
	i2c_fSCL = (24000 / I2C_FSCL);	// fSCL

	// Initialize the crystal
	PJSEL0 |= 0x10;					// Xtal mode for pin 4
	PJSEL1 &= 0x10;					// Xtal mode for pin 4
// TODO: It might be good to add code for pin 5

	CSCTL4 |= (XT1DRIVE1 | XT1DRIVE0);
// Wait for XT1 to start by trying to clear fault flag until it succeeds
	CSCTL4 &= ~XT1OFF; // Turn XT1 on--this might be done by itself.
	do
	{
		SFRIFG1 &= ~OFIFG;
		CSCTL5 &= ~XT1OFFG; // Will HANG if Xtal doesn't work
	} while (SFRIFG1 & OFIFG);

	CSCTL0_H = 0x00; // Disable Clock access
}

// Initialize the Watchdog Timer
static int WDT_init() {
	WDTCTL = WDT_CTL; // Set Watchdog interval
	SFRIE1 |= WDTIE; // Enable WDT interrupt

	WDT_cps_cnt = WDT_CLKS_PER_SEC;	// set WD 1 second counter
	return 0;
}

// Initialize ports, timers, clock, etc. for the msp430
static int msp430init() {
	WDT_init(); // init the watchdog timer

	// Initialize Port 1
	P1SEL0 = P1SEL1 = 0; // Port 1 is GPIO
	P1DIR = (BCLK | SCLK | SCL); // output pins = 1; input = 0
	// Init button interrupt on port 1:
	P1REN = SW1 | USBINT; // pull-up resistors on switch1 and USBINT
	P1IE = SW1 | USBINT; // enable interrupt for sw1 and usb
	P1IES = SW1 | USBINT; // interrupt on falling edge

	// TODO: Initialize Port 2

	// Initialize Port J
	PJSEL0 &= ~0x0f; // first 4 pins are GPIO (J.4 and J.5 are crystal pins)
	PJSEL1 &= ~0x0f; // GPIO
	PJOUT &= ~0x0f; // all pins low
	PJREN &= ~0x0f; // no pull-up resistors
    PJDIR = LED0 | LED1 | ASDA | ASCL; // set LED pins and ASDA/ASCL as output
    LED0_OFF; // LED0 off
    LED1_OFF; // LED1 off
	setClock(); // init the clock (also on port J)
    ft201x_init(); // Initialize the USB comm chip (ft201x)

    // Enable global interrupts
    __enable_interrupt();

	return 0;
}

//-----------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {

	--WDT_cps_cnt;

	// One second elapsed
	if (WDT_cps_cnt == 0) {
		WDT_cps_cnt = WDT_CLKS_PER_SEC;
		LED0_TOGGLE; // toggle heartbeat LED

//		// Signal write-debug event just for fun. Wake up to service it.
//    	sys_event |= WRITE_DEBUG_EVENT;
//    	__bic_SR_register_on_exit(LPM0_bits);
	}

	// Are we currently debouncing the switch?
	if (debounceCnt) {
		// Yes; are we finished?
		if (--debounceCnt == 0) {
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
    	sys_event |= WRITE_DEBUG_EVENT;
    	__bic_SR_register_on_exit(LPM0_bits);


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

