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
 * Xin		Xin		|	2.4/A7		I	MIBO
 * Xout		Xout	| 	2.3			O	MOBI
 * AVSS		GND		|	DVCC		X	VDD
 * AVCC		VDD		|	DVSS		X	Gnd
 * 1.0	I	Int0	|	VCore		X	X
 * 1.1	I	Int1	|	1.7/UCB0SCL	O	SCL
 * 1.2	O	BCLK	|	1.6/UCB0SDA	X	SDA
 * 1.3	I	SW1		|	2.2			I	BCHG
 * 1.4	?	??		|	2.1/UCA0RX0	I	MISO
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
#define USB_IN_CNT			51				// ~10 ms using WDT

// Local function prototypes
static int WDT_init();
static void setClock();
static int msp430init();

// Global Variables
static volatile int WDT_cps_cnt;
static volatile int USBInCnt;
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
	USBInCnt = USB_IN_CNT*2; // set interval to poll USB Input
	return 0;
}

// Initialize ports, timers, clock, etc. for the msp430
static int msp430init() {
	WDT_init(); // init the watchdog timer
	setClock(); // init the clock

	// Initialize Port 1
	P1SEL0 = P1SEL1 = 0; // Port 1 is GPIO

    // Initialize LED output pins.
    PJDIR |= LED0 | LED1; // set direction to output (high)
    PJOUT |= LED0; // LED0 on
    PJOUT &= ~LED1; // LED1 off

    // Initialize the USB comm chip - ft201x
    ft201x_init();

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

		// Signal write-debug event just for fun. Wake up to service it.
//    	sys_event |= WRITE_DEBUG_EVENT;
//    	__bic_SR_register_on_exit(LPM0_bits);
	}

	// 1/10th second elapsed
//	--USBInCnt;
//	if (USBInCnt == 0) {
//		USBInCnt = USB_IN_CNT*2;
//		sys_event |= USB_I_EVENT;
//
//	}
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

		IOputc('a', io_usb_out);
		IOputc('\r', io_usb_out);
		IOputc(0, io_usb_out);

		LED1_TOGGLE;

		// Signal USB input event
		sys_event |= USB_I_EVENT;

		// Wake up the processor
		__bic_SR_register_on_exit(LPM0_bits);

	}

	// Switch1 interrupt?
//	if (P1IFG & SW1) {
//		WDT_debounce_cnt = DEBOUNCE_CNT;
//		P1IFG &= ~SW1;
//	}

	// Radio interrupt?
//	if (P1IFG & INT) {
//		P1IFG &= ~INT;
//		if (bdl_config.radio_cfg.radio_mode)
//			rf_irq |= RF24_IRQ_FLAGGED;
//		sys_event |= INTERRUPT;
//		__bic_SR_register_on_exit(LPM0_bits);
//	}


}

