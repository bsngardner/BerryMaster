/*
 * hal.c
 *
 *  Created on: Oct 6, 2015
 *      Author: Broderick
 */

//#include <ioprintf.h>
#include <stdint.h>
#include "hal.h"
#include "msp430.h"

//enum used to track state and communicate between sending functions and isr
typedef enum
{
	START, DATA, SENT, STOP, NACK
} I2C_STATUS;

//Global variables
static volatile uint8_t byte_count, load_byte_count;
static volatile uint8_t txData[16];
static volatile uint8_t * txPtr;

static volatile uint8_t rxData[16];
static volatile uint8_t * rxPtr;

static volatile uint8_t vineSleep = 0;
static volatile uint16_t repeat_start = 0;
volatile static I2C_STATUS status = STOP;

//Defines
//Commands
#define NEW_ADDR_CMD 0
#define RESET_ALL_CMD 1
#define PROJ_HASH_CMD 2

#define CLOCK_SPEED 24000000
#define I2C_SPEED 1000000

//Addresses
#define GEN_CALL 0x00

#define FORCE_GPIO P1SEL1 &= ~(SDA_PIN | SCL_PIN)
#define SET_I2C	P1SEL1 |= (SDA_PIN | SCL_PIN)
//Initialize the data bus
// sets up SDA and SCL, inits eUSCI module, enables interrupt vector
uint8_t hal_init()
{

	SET_I2C;

	UCB0CTLW0 |= UCSWRST;                          // put eUSCI_B in reset state
	UCB0CTLW0 &= ~UCSSEL_2;
	UCB0CTLW0 |= (UCMODE_3 | UCMST | UCTR | UCSSEL_2); // I2C master mode, SMCLK
	UCB0BRW = CLOCK_SPEED / I2C_SPEED;        // baudrate = SMCLK /400,000
	UCB0CTLW0 &= ~UCSWRST;                            // clear reset register
	return 0;
} //End hal_init()

void configure_i2c(uint8_t num_bytes)
{

	UCB0CTLW0 |= UCSWRST;

	UCB0CTLW0 |= UCTR;
	UCB0CTLW1 &= ~UCASTP_3;
	UCB0CTLW1 |= UCASTP_2;
	UCB0TBCNT = num_bytes;

	UCB0CTLW0 &= ~UCSWRST;
}

//Sends 0x00 on general call followed by new address.  Returns 0 if address accepted,
//	1 if address not accepted
uint8_t hal_discoverNewDevice(uint8_t new_address)
{

	UCB0I2CSA = GEN_CALL;
	txPtr = txData;
	txPtr[0] = NEW_ADDR_CMD;
	txPtr[1] = new_address;
	configure_i2c(2);
	UCB0IE |= UCTXIE0 | UCNACKIE | UCSTPIE;

	vineSleep = 1;
	status = DATA;
	UCB0CTLW0 |= UCTXSTT;

	while (vineSleep)
		LPM0;

	if (status == STOP)
	{
		return ADDR_ACCEPTED;
	}
	if (status == NACK)
	{
		return ADDR_REJECTED;
	}
	return 1;
} //End hal_discoverNewDevice()

uint8_t hal_resetAllDevices()
{
	UCB0I2CSA = GEN_CALL;
	txPtr = txData;
	txPtr[0] = RESET_ALL_CMD;

	configure_i2c(1);
	UCB0IE |= UCTXIE0 | UCNACKIE | UCSTPIE;

	vineSleep = 1;
	status = DATA;
	UCB0CTLW0 |= UCTXSTT;

	while (vineSleep)
		LPM0;

	if (status == STOP)
	{
		return ADDR_ACCEPTED;
	}
	if (status == NACK)
	{
		return ADDR_REJECTED;
	}
	return ERROR;
}

// Sends 0x01 to device address, returns 0 if device ACKed, 1 if NACK
uint8_t hal_pingDevice(uint8_t address)
{

	UCB0I2CSA = address;
	configure_i2c(0);
	UCB0IE |= UCNACKIE | UCSTPIE; //

	vineSleep = 1;
	status = DATA;
	UCB0CTLW0 |= UCTXSTT;
	UCB0CTLW0 |= UCTXSTP;

	while (vineSleep)
		LPM0;

	if (status == STOP)
	{
		return PING;
	}
	if (status == NACK)
	{
		return NO_RESPONSE;
	}
	__no_operation();
	//oh no
	return 2;

} //End hal_pingDevice

uint8_t hal_check_proj_hash(uint8_t proj_hash)
{
	UCB0I2CSA = GEN_CALL;
	txPtr = txData;
	txPtr[0] = PROJ_HASH_CMD;
	txPtr[1] = proj_hash;
	configure_i2c(2);
	UCB0IE |= UCTXIE0 | UCNACKIE | UCSTPIE;

	vineSleep = 1;
	status = DATA;
	UCB0CTLW0 |= UCTXSTT;

	while (vineSleep)
		LPM0;

	if (status == STOP)
	{
		return 1;
	}
	if (status == NACK)
	{
		return 2;
	}
	return ERROR;
}

//Sets register (reg) on device (@address) to value (value)
uint8_t hal_setDeviceRegister(uint8_t address, uint8_t reg, uint8_t value)
{

	UCB0I2CSA = address;
	txPtr = txData;
	txPtr[0] = reg;
	txPtr[1] = value;

	configure_i2c(2);
	UCB0IFG = 0;
	UCB0IE |= UCTXIE0 | UCNACKIE | UCSTPIE;

	vineSleep = 1;
	status = DATA;
	UCB0CTLW0 |= UCTXSTT;

	while (vineSleep)
		LPM0;

	if (status == STOP)
	{
		return PING;
	}
	if (status == NACK)
	{
		return NO_RESPONSE;
	}
	return 1;
}

#define DMA_CTL (DMADSTINCR0 | DMADSTINCR1 | DMADSTBYTE | DMASRCBYTE)
#define DMA_DIS DMA0CTL = 0

inline void dma_config(uint8_t * dest_addr, uint8_t count)
{
	DMACTL0 = DMA0TSEL__UCB0RXIFG0;
	DMA0CTL = DMA_CTL;

	__data16_write_addr((unsigned int) &DMA0SA, (unsigned long) &UCB0RXBUF);
	__data16_write_addr((unsigned int) &DMA0DA, (unsigned long) dest_addr);
	DMA0SZ = count;
	DMA0CTL |= DMAEN;
}

//Stores data from device (@address) register (reg) in variable (ret_val)
uint8_t hal_getDeviceRegister(uint8_t address, uint8_t reg, uint8_t* ret)
{

	return hal_getDeviceMultiRegs(address, reg, ret, 1);
}

//Stores data from device (@address) register (reg) in variable (ret_val)
uint8_t hal_getDeviceMultiRegs(uint8_t address, uint8_t reg, uint8_t* ret,
		uint8_t count)
{

	UCB0I2CSA = address;
	txPtr = txData;
	*txPtr = reg;
	rxPtr = ret;

	UCB0CTLW0 |= UCSWRST;

	UCB0CTLW0 |= UCTR;
	UCB0CTLW1 &= ~UCASTP_3;
	UCB0CTLW1 |= UCASTP_2;
	UCB0TBCNT = 1;

	UCB0CTLW0 &= ~UCSWRST;
	UCB0IE |= UCTXIE0 | UCNACKIE | UCSTPIE;

	repeat_start = 1;
	byte_count = count;

	dma_config(rxPtr, count);

	vineSleep = 1;
	status = DATA;
	UCB0CTLW0 |= UCTXSTT;

	while (vineSleep)
		LPM0;

	if (status == NACK)
	{
		return NO_RESPONSE;
	}
	if (status == STOP)
	{
		return SUCCESS;
	}
	return 3;
}

//Defines for all eUSCIB interrupt vectors
#define NONE 0x00
#define UCAL_IV 0x02
#define NACK_IV 0x04
#define STT_IV	0x06
#define STP_IV	0x08
#define RX3_IV	0x0A
#define TX3_IV	0x0C
#define RX2_IV	0x0E
#define TX2_IV	0x10
#define RX1_IV	0x12
#define TX1_IV	0x14
#define RX0_IV	0x16
#define TX0_IV	0x18
#define BCNT_IV	0x1A
#define CLTO_IV	0x1C
#define BIT9_IV	0x1E

#pragma vector=USCI_B0_VECTOR
__interrupt void euscib0_isr(void)
{

	switch (UCB0IV)
	{
	case NACK_IV:
		UCB0IFG &= ~(UCNACKIFG);
		UCB0CTLW0 |= UCTXSTP;                  // I2C stop condition.
		repeat_start = 0;
		status = NACK;
		break;
	case STP_IV:
		UCB0IFG &= ~UCSTPIFG;
		if (repeat_start)
		{
			UCB0CTLW0 |= UCSWRST;
			UCB0CTLW0 &= ~UCTR;
			UCB0TBCNT = byte_count;
			UCB0CTLW0 &= ~UCSWRST;
			UCB0IE = UCNACKIE | UCSTPIE;
			//UCB0IE |= UCRXIE0;
			UCB0CTLW0 |= UCTXSTT;

			repeat_start = 0;
		}
		else
		{
			if (status != NACK)
				status = STOP;
			vineSleep = 0;
			__bic_SR_register_on_exit(LPM0_bits);
		}
		break;
	case TX0_IV:
		UCB0TXBUF = *txPtr++;
		break;
	case RX0_IV:
		*rxPtr++ = UCB0RXBUF;
		break;
	default:
		break;
	}
}
