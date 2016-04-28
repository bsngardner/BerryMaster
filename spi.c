/*
 * spi.c
 *
 *  Created on: Feb 8, 2014
 *      Author: proper
 */
#include <msp430.h>
#include <setjmp.h>
#include "BerryMaster.h"
#include "spi.h"
#include "IObuffer.h"
#include "nrfradio.h"

//#define TRUE	0xff

#ifndef __MSP430_HAS_USCI__
//	ERROR2(20);	//#error "Error! This MCU doesn't have a USCI peripheral"
#endif

static volatile unsigned char spi_busy;
volatile unsigned char RXData;
volatile unsigned char TXData;

int spi_init(void) {
	//P1DIR |= SDCS; //SD card CS
	P1SEL1 |= SCLK;
	P2SEL1 |= MOSI + MISO;
	UCA0CTLW0 |= UCSWRST;					// **Put state machine in reset**
	UCA0CTLW0 |= UCMST + UCSYNC + UCCKPH + UCMSB;// 3-pin, 8-bit, master, polarity high, MSB
	// TODO: We changed this from UCCKPL to UCCKPH for the Radio--does that break SD???
	UCA0CTLW0 |= UCSSEL__SMCLK;				// SMCLK
	UCA0BR0 = 3;							// /3
	UCA0BR1 = 0;							//
	UCA0MCTLW = 0;							// No modulation
	UCA0CTLW0 &= ~UCSWRST;					// **Initialize USCI state machine**

	UCA0IE |= UCRXIE;						// Enable USCI_A0 RX interrupt

	spi_busy = 0;

	return 0;
} // end spi_init

void spi_set_divisor(enum SPI_DIVISOR divisor) {
	UCA0BR0 = divisor;
	UCA0BR1 = 0;							//
	return;
} // end spi_set_divisor

#if 0
void spi_write(unsigned char byte) {
//    while(!spi_done);							// wait until done
	_disable_interrupts();
	spi_busy = TRUE;
	TXData = byte;// set transmit data
	UCA0IE |= UCTXIE;
	_enable_interrupts();
	while (spi_busy)
	;// wait until done
	return;
} // end spi_write

unsigned char spi_read() {
	_disable_interrupts();
	spi_busy = TRUE;
	TXData = 0xff;								// set transmit data
	UCA0IE |= UCTXIE;
	_enable_interrupts();
	while (spi_busy)
	;// wait for reply
	return RXData;
} // end spi_read
#endif

unsigned char spi_transfer(unsigned char byte) {
	_disable_interrupts();
	spi_busy = TRUE;
	TXData = byte;								// set transmit data
	UCA0IE |= UCTXIE;
	_enable_interrupts();
	while (spi_busy)
		;							// wait for reply
	return RXData;
}

#define NONE 0x00
#define RX_IV 0x02
#define TX_IV 0x04

#pragma vector=USCI_A0_VECTOR
__interrupt
void USCI_A0_ISR(void) {
	switch (__even_in_range(UCA0IV, 0x04)) {
	case NONE:
		break;                          // Vector 0 - no interrupt

		// transmission done, received character in UCA0RXBUF
	case RX_IV:						// UCRXIE = rx buffer received character
	{
		UCA0IFG &= ~UCRXIFG;				// clear UCRXIFG
		RXData = UCA0RXBUF;				// read rx buffer
		spi_busy = 0;					// done
		if (spi_reading)
			nrf_rx_handle(RXData);
		break;
	}

	case TX_IV:									// UCTXIE = tx buffer empty
	{
		UCA0IE &= ~UCTXIE;					// clear UCTXIE
		UCA0TXBUF = TXData;             // transmit character
		spi_busy = 1;						// busy
	}
		/* no break */

	default:
		break;
	}
}
