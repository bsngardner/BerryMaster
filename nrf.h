/*
 * nrf.h
 *
 *  Created on: Feb 15, 2016
 *      Author: Broderick
 */

#ifndef NRF_H_
#define NRF_H_

#include <stdint.h>
#include <stdlib.h>
#include "BerryMaster.h"
#include "nRF24L01.h"
#include "IObuffer.h"
#include "spi.h"

/* SPI port--Select which USCI port we're using.
 * Applies only to USCI devices.  USI users can keep these
 * commented out.
 */
//#define SPI_DRIVER_USCI_A 1
#define SPI_DRIVER_USCI_B 1

// available functions
int nrf_init(int freq, int speed);
void nrf_close();
int nrf_open();
int nrf_flush();
int nrf_sendBytes();
int nrf_TXEvent();

void spi_start_read();

/* Settings for 16MHz MCLK */
#define MPU_HZ 24000000
#define DELAY_CYCLES_5MS       MPU_HZ/200
#define DELAY_CYCLES_130US     MPU_HZ/7692
#define DELAY_CYCLES_15US      MPU_HZ/66667
#define DELAY_CYCLES_250US	DELAY_CYCLES_5MS/20

#define NRF_BUF_SIZE 96

/* IRQ */
#define nrfIRQport 1
#define nrfIRQpin INT0 // P2.2

/* CSN SPI chip-select */
#define nrfCSNport 2
#define nrfCSNportout P2OUT
#define nrfCSNpin RFCS // P2.6

/* CE Chip-Enable (used to put RF transceiver on-air for RX or TX) */
#define nrfCEport 2
#define nrfCEportout P2OUT
#define nrfCEpin RFCE // P2.6

#define INT_EN	P1IE |= nrfIRQpin
#define INT_DIS P1IE &= ~nrfIRQpin
#define TEST_INT (P1IN & nrfIRQpin)

#endif /* NRF_H_ */
