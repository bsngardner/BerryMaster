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

#include <msp430.h>

// Port J
#define LED0 0x01
#define LED1 0x02
#define LED_MASK 0x03

// Port 1

// Port 2

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    // Initialize LED output pins.
    PJDIR |= LED0 | LED1; // direction
    PJOUT |= LED0;
    PJOUT &= ~LED1;

    while(1) {
    	__delay_cycles(100000);
    	PJOUT = (PJOUT ^ LED_MASK); // toggle
    }


	return 0;
}
