/*
 * nrfradio.c
 *
 *  Created on: May 18, 2015
 *      Author: Broderick Gardner
 *      based heavily on code by Spirilis
 *
 */

#include <msp430.h>
#include "nrf.h"
#include "nrfradio.h"

/* Status variable updated every time SPI I/O is performed */
volatile uint8_t rf_status;

/* Configuration parameters used to set-up the RF configuration */
uint8_t rf_crc;
uint8_t rf_addr_width;
uint8_t rf_speed_power;
uint8_t rf_channel;
uint8_t rf_feature;  // Used to track which features have been enabled
/* IRQ state is stored in here after msprf24_get_irq_reason(), RF24_IRQ_FLAGGED raised during
 * the IRQ port ISR--user application issuing LPMx sleep or polling should watch for this to
 * determine if the wakeup reason was due to nRF24 IRQ.
 */
volatile uint8_t rf_irq;

// private variables
IObuffer* nrf_slot = 0;
IObuffer* nrf_buffer = 0;

static const uint8_t pipe_addr[6][5] =
{
{ 0xDE, 0xAD, 0xBE, 0xEF, 0x00 },
{ 0xFA, 0xDE, 0xDA, 0xCE, 0x51 },
{ 0xFA, 0xDE, 0xDA, 0xCE, 0x52 },
{ 0xFA, 0xDE, 0xDA, 0xCE, 0x53 },
{ 0xFA, 0xDE, 0xDA, 0xCE, 0x54 },
{ 0xFA, 0xDE, 0xDA, 0xCE, 0x55 } };

static uint8_t nrf_prx = 0;
volatile uint16_t sem_fifo = 3;
volatile int tx_failed = 0;

void (*nrf_txCallback)();
void (*nrf_txFailCallback)();

// Initialize radio, pass NULL for default values:
// frequency=		speed=
int nrf_init(int freq, int speed)
{
	spi_init();
	// Need this input buffer to read data from host.
	if (!(nrf_buffer = IObuffer_create(NRF_BUF_SIZE)))
		return -1;

	const uint8_t speeds[3] =
	{ RF24_SPEED_250KBPS, RF24_SPEED_1MBPS,
	RF24_SPEED_2MBPS };

	if (freq)
		rf_channel = freq;
	else
		rf_channel = 120;

	rf_speed_power = speeds[speed] | RF24_POWER_0DBM;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
	rf_addr_width = 5;

	CSN_DIS;
	CE_DIS;
	INT_DIS;

	//port_init();
	P2DIR |= (RFCS | RFCE);
	// Wait 100ms for RF transceiver to initialize.
	__delay_cycles(DELAY_CYCLES_100MS);

	msprf24_powerdown();

	// Configure RF transceiver with current value of rf_* configuration variables
	msprf24_close_pipe_all(); /* Start off with no pipes enabled, let the user open as needed.  This also
	 * clears the DYNPD register.
	 */
	msprf24_irq_clear(RF24_IRQ_MASK);  // Forget any outstanding IRQs
	msprf24_set_retransmit_delay(800);  // A default I chose
	msprf24_set_retransmit_count(6);    // A default I chose
	msprf24_set_speed_power();
	msprf24_set_channel();
	msprf24_set_address_width();
	rf_feature = 0x00;  // Initialize this so we're starting from a clean slate
	msprf24_enable_feature(RF24_EN_DPL); // Dynamic payload size capability (set with msprf24_set_pipe_packetsize(x, 0))
	//msprf24_enable_feature(RF24_EN_DYN_ACK); // Ability to use w_tx_payload_noack()  //----////somehow getting rid of this line fixes everything...!?!?!????

	// the following line enables ack payloads
	msprf24_enable_feature(RF24_EN_ACK_PAY);

	flush_tx();
	flush_rx();

	return !msprf24_is_alive();

}

#define PIPE 0
// ... open given pipe
int nrf_open(uint8_t is_prx)
{
	msprf24_standby();
	msprf24_set_pipe_packetsize(PIPE, 0); //Pipe 0

	uint8_t rxen, enaa;

//	if (pipe > 5)
//		return 0;

	rxen = r_reg(RF24_EN_RXADDR);
	enaa = r_reg(RF24_EN_AA);

	enaa |= (1 << PIPE);
	rxen |= (1 << PIPE);
	w_reg(RF24_EN_RXADDR, rxen);
	w_reg(RF24_EN_AA, enaa);

	write_tx_address(pipe_addr[0]);
	write_rx_address(0, pipe_addr[0]);

	w_reg(RF24_STATUS, RF24_RX_DR);
	nrf_prx = is_prx;
	if (nrf_prx)
	{
		//If PRX device, set to receive mode and enable interrupt
		msprf24_set_config(RF24_PWR_UP | RF24_PRIM_RX);
		CE_EN;
		INT_EN;
	}
	return 0;
}

// close given pipe
void nrf_close()
{
	uint8_t rxen, enaa;

//	if (pipe > 5)
//		return;

	rxen = r_reg(RF24_EN_RXADDR);
	enaa = r_reg(RF24_EN_AA);

	rxen &= ~(1 << PIPE);
	enaa &= ~(1 << PIPE);

	w_reg(RF24_EN_RXADDR, rxen);
	w_reg(RF24_EN_AA, enaa);
}

#define PAYLOAD_SIZE 32
#define MIN_RAND 3000	//Added to range to give 3000 - 27576 delay (24MHz clock)
#define RAND_1 (~(2^14)) // Random numbers added to get range of 0-24576
#define RAND_2 (~(2^13))
#define MAX_RETRIES 5

#define FIFO_FULL -1

int nrf_sendPacket()
{
	if (!sem_fifo)
		return FIFO_FULL;

	INT_DIS;	//Disable INT interrupt
	while (spi_reading)
		;	//Wait until any RX is done

	uint8_t count = nrf_buffer->count; //Holds the number of bytes in packet
	if (count > PAYLOAD_SIZE)
	{
		//Send 32 bytes of data if > 32
		count = PAYLOAD_SIZE;
	}

	CSN_EN; //Start radio command
	if (nrf_prx) //If primery receive radio,
		//Load packet as ack payload
		rf_status = spi_transfer(RF24_W_ACK_PAYLOAD);
	else
		//if primary transmit, load packet as normal
		rf_status = spi_transfer(RF24_W_TX_PAYLOAD); //CMD to send TX payload
	char data; //Temp variable for reading data from IObuffer
	//This may look confusing, but we injext another 0 into the packet if the
	//	first byte is 0 because the first byte will be ignored if it is 0;
	IOgetc(&data, nrf_buffer);
	count--;
	spi_transfer(data);
	if (!data)
	{
		if (count)
			count--;
		spi_transfer(0);
	}
	while (count-- > 0)
	{ //Loop to send data to radio over SPI
		IOgetc(&data, nrf_buffer);
		spi_transfer(data);
	}
	CSN_DIS; //End radio command

	--sem_fifo; //Decrement fifo semafore
	INT_EN; //Reenable interrupt for IRQs

	//If transmitter and fifo has at least 1 packet,
	//	raise CE (or keep raised) to send packets
	//CE will be lowered when a success interrupt is received
	//	and fifo is empty
	if (!nrf_prx && sem_fifo < 3)
		CE_EN;
	return sem_fifo;
}

void nrf_sendPing()
{
	if (sem_fifo!=3 || nrf_prx)
		return;
	INT_DIS;	//Disable INT interrupt
	while (spi_reading)
		;	//Wait until any RX is done

	CSN_EN; //Start radio command
	rf_status = spi_transfer(RF24_W_TX_PAYLOAD); //CMD to send TX payload

	spi_transfer(0);
	CSN_DIS; //End radio command
	--sem_fifo; //Decrement fifo semafore
	INT_EN; //Reenable interrupt for IRQs

	//If transmitter and fifo has at least 1 packet,
	//	raise CE (or keep raised) to send packets
	//CE will be lowered when a success interrupt is received
	//	and fifo is empty
	if (!nrf_prx && sem_fifo < 3)
		CE_EN;
}

void msprf24_close_pipe_all()
{
	w_reg(RF24_EN_RXADDR, 0x00);
	w_reg(RF24_EN_AA, 0x00);
	w_reg(RF24_DYNPD, 0x00);
}

// Set the rx pipe address, sending to the radio
void write_rx_address(uint8_t pipe, const uint8_t *addr)
{
	int i;

	if (pipe > 5)
		return;  // Only 6 pipes available
	CSN_EN;
	rf_status = spi_transfer((RF24_RX_ADDR_P0 + pipe) | RF24_W_REGISTER);
	if (pipe > 1)
	{  // Pipes 2-5 differ from pipe1's addr only in the LSB.
		spi_transfer(addr[rf_addr_width - 1]);
	}
	else
	{
		for (i = rf_addr_width - 1; i >= 0; i--)
		{
			spi_transfer(addr[i]);
		}
	}
	CSN_DIS;
}

// Set the tx pipe address, used to burst send
void write_tx_address(const uint8_t *addr)
{
	int i;

	CSN_EN;
	rf_status = spi_transfer(RF24_TX_ADDR | RF24_W_REGISTER);
	for (i = rf_addr_width - 1; i >= 0; i--)
	{
		spi_transfer(addr[i]);
	}
	CSN_DIS;
}

/* Clear IRQ flags */
void msprf24_irq_clear(uint8_t irqflag)
{
	uint8_t fifostat;

	rf_irq = 0x00; // Clear IRQs; afterward analyze RX FIFO to see if we should re-set RX IRQ flag.
	CSN_EN;
	rf_status = spi_transfer(RF24_STATUS | RF24_W_REGISTER);
	spi_transfer(irqflag);
	CSN_DIS;

// Per datasheet procedure, check FIFO_STATUS to see if there's more RX FIFO data to process.
	if (irqflag & RF24_IRQ_RX)
	{
		CSN_EN;
		rf_status = spi_transfer(RF24_FIFO_STATUS | RF24_R_REGISTER);
		fifostat = spi_transfer(RF24_NOP);
		CSN_DIS;
		if (!(fifostat & RF24_RX_EMPTY))
			rf_irq |= RF24_IRQ_RX | RF24_IRQ_FLAGGED; // Signal to user that there is remaining data, even if it's not "new"
	}
}

// Get IRQ flag status
uint8_t msprf24_get_irq_reason()
{
	uint8_t rf_irq_old = rf_irq;

//rf_irq &= ~RF24_IRQ_FLAGGED;  -- Removing in lieu of having this check determined at irq_clear() time
	CSN_EN;
	rf_status = spi_transfer(RF24_NOP);
	CSN_DIS;
	rf_irq = (rf_status & RF24_IRQ_MASK) | rf_irq_old;
	return rf_irq;
}

/* Evaluate state of TX, RX FIFOs
 * Compare this with RF24_QUEUE_* #define's from msprf24.h
 */
uint8_t msprf24_queue_state()
{
	return r_reg(RF24_FIFO_STATUS);
}

void msprf24_set_pipe_packetsize(uint8_t pipe, uint8_t size)
{
	uint8_t dynpdcfg;

	if (pipe > 5)
		return;

	dynpdcfg = r_reg(RF24_DYNPD);
	if (size < 1)
	{
		if (!(rf_feature & RF24_EN_DPL)) // Cannot set dynamic payload if EN_DPL is disabled.
			return;
		if (!((1 << pipe) & dynpdcfg))
		{
			// DYNPD not enabled for this pipe, enable it
			dynpdcfg |= 1 << pipe;
		}
	}
	else
	{
		dynpdcfg &= ~(1 << pipe);  // Ensure DynPD is disabled for this pipe
		if (size > 32)
			size = 32;
		w_reg(RF24_RX_PW_P0 + pipe, size);
	}
	w_reg(RF24_DYNPD, dynpdcfg);
}

void msprf24_set_retransmit_delay(uint16_t us)
{
	uint8_t c;

// using 'c' to evaluate current RF speed
	c = rf_speed_power & RF24_SPEED_MASK;
	if (us > 4000)
		us = 4000;
	if (us < 1500 && c == RF24_SPEED_250KBPS)
		us = 1500;
	if (us < 500)
		us = 500;

// using 'c' to save current value of ARC (auto-retrans-count) since we're not changing that here
	c = r_reg(RF24_SETUP_RETR) & 0x0F;
	us = (us - 250) / 250;
	us <<= 4;
	w_reg(RF24_SETUP_RETR, c | (us & 0xF0));
}

// Power down device, 0.9uA power draw
void msprf24_powerdown()
{
	CE_DIS;
	msprf24_set_config(0);  // PWR_UP=0
}

// Enable Standby-I, 26uA power draw
void msprf24_standby()
{
	uint8_t state = msprf24_current_state();
	if (state == RF24_STATE_NOTPRESENT || state == RF24_STATE_STANDBY_I)
		return;
	CE_DIS;
	msprf24_set_config(RF24_PWR_UP);  // PWR_UP=1, PRIM_RX=0
	if (state == RF24_STATE_POWERDOWN)
	{ // If we're powering up from deep powerdown...
//CE_EN;  // This is a workaround for SI24R1 chips, though it seems to screw things up so disabled for now til I can obtain an SI24R1 for testing.
		__delay_cycles(DELAY_CYCLES_5MS); // Then wait 5ms for the crystal oscillator to spin up.
		//CE_DIS;
	}
}

uint8_t r_reg(uint8_t addr)
{
	uint8_t i;

	CSN_EN;
	rf_status = spi_transfer(addr & RF24_REGISTER_MASK);
	i = spi_transfer(RF24_NOP);
	CSN_DIS;
	return i;
}

void w_reg(uint8_t addr, uint8_t data)
{
	CSN_EN;
	rf_status = spi_transfer((addr & RF24_REGISTER_MASK) | RF24_W_REGISTER);
	spi_transfer(data);
	CSN_DIS;
}

inline uint8_t _msprf24_crc_mask()
{
	return (rf_crc & 0x0C);
}

inline uint8_t _msprf24_irq_mask()
{
	return ~(RF24_MASK_RX_DR | RF24_MASK_TX_DS | RF24_MASK_MAX_RT);
}

void msprf24_set_retransmit_count(uint8_t count)
{
	uint8_t c;

	c = r_reg(RF24_SETUP_RETR) & 0xF0;
	w_reg(RF24_SETUP_RETR, c | (count & 0x0F));
}

uint8_t msprf24_get_last_retransmits()
{
	return r_reg(RF24_OBSERVE_TX) & 0x0F;
}

uint8_t msprf24_get_lostpackets()
{
	return (r_reg(RF24_OBSERVE_TX) >> 4) & 0x0F;
}

void msprf24_set_address_width()
{
	if (rf_addr_width < 3 || rf_addr_width > 5)
		return;
	w_reg(RF24_SETUP_AW, ((rf_addr_width - 2) & 0x03));
}

void msprf24_set_channel()
{
	if (rf_channel > 125)
		rf_channel = 0;
	w_reg(RF24_RF_CH, (rf_channel & 0x7F));
}

void msprf24_set_speed_power()
{
	if ((rf_speed_power & RF24_SPEED_MASK) == RF24_SPEED_MASK) // Speed setting RF_DR_LOW=1, RF_DR_HIGH=1 is reserved, clamp it to minimum
		rf_speed_power = (rf_speed_power & ~RF24_SPEED_MASK) | RF24_SPEED_MIN;
	w_reg(RF24_RF_SETUP, (rf_speed_power & 0x2F));
}

void msprf24_enable_feature(uint8_t feature)
{
	if ((rf_feature & feature) != feature)
	{
		rf_feature |= feature;
		rf_feature &= 0x07;  // Only bits 0, 1, 2 allowed to be set
		w_reg(RF24_FEATURE, rf_feature);
	}
}

void msprf24_disable_feature(uint8_t feature)
{
	if ((rf_feature & feature) == feature)
	{
		rf_feature &= ~feature;
		w_reg(RF24_FEATURE, rf_feature);
	}
}

uint8_t msprf24_set_config(uint8_t cfgval)
{
	uint8_t previous_config;

	previous_config = r_reg(RF24_CONFIG);
	w_reg(RF24_CONFIG, (_msprf24_crc_mask() | cfgval) & _msprf24_irq_mask());
	return previous_config;
}

uint8_t msprf24_is_alive()
{
	uint8_t aw;

	aw = r_reg(RF24_SETUP_AW);
	return ((aw & 0xFC) == 0x00 && (aw & 0x03) != 0x00);
}

uint8_t msprf24_current_state()
{
	uint8_t config;

	if (!msprf24_is_alive()) // Can't read/detect a valid value from SETUP_AW? (typically SPI or device fault)
		return RF24_STATE_NOTPRESENT;
	config = r_reg(RF24_CONFIG);
	if ((config & RF24_PWR_UP) == 0x00)  // PWR_UP=0?
		return RF24_STATE_POWERDOWN;
	if (!(nrfCEportout & nrfCEpin))      // PWR_UP=1 && CE=0?
		return RF24_STATE_STANDBY_I;
	if (!(config & RF24_PRIM_RX))
	{      // PWR_UP=1 && CE=1 && PRIM_RX=0?
		if ((r_reg(RF24_FIFO_STATUS) & RF24_TX_EMPTY))  // TX FIFO empty?
			return RF24_STATE_STANDBY_II;
		return RF24_STATE_PTX; // If TX FIFO is not empty, we are in PTX (active transmit) mode.
	}
	if (r_reg(RF24_RF_SETUP) & 0x90)     // Testing CONT_WAVE or PLL_LOCK?
		return RF24_STATE_TEST;
	return RF24_STATE_PRX;           // PWR_UP=1, PRIM_RX=1, CE=1 -- Must be PRX
}

inline void pulse_ce()
{
	CE_EN;
	__delay_cycles(DELAY_CYCLES_15US);
	CE_DIS;
}

void flush_tx()
{
	CSN_EN;
	rf_status = spi_transfer(RF24_FLUSH_TX);
	CSN_DIS;
}

void flush_rx()
{
	CSN_EN;
	rf_status = spi_transfer(RF24_FLUSH_RX);
	CSN_DIS;
}

void tx_reuse_lastpayload()
{
	CSN_EN;
	rf_status = spi_transfer(RF24_REUSE_TX_PL);
	CSN_DIS;
	pulse_ce();
}

uint8_t r_rx_peek_payload_size()
{
	uint8_t i;

	CSN_EN;
	rf_status = spi_transfer(RF24_R_RX_PL_WID);
	i = spi_transfer(RF24_NOP);
	CSN_DIS;
	return i;
}

volatile int spi_reading = 0;
volatile int rx_count = 0;
extern volatile unsigned char RXData;
extern volatile unsigned char TXData;

//Interrupt functions

enum
{
	IDLE = 0x00,
	INIT = 0x02,
	READ_SIZE = 0x04,
	PREP = 0x06,
	START = 0x08,
	READ_FIRST = 0x0a,
	READ = 0x0c,
	CLEAR_IRQ = 0x0e,
	CMD_FIFO = 0x10,
	FIFO = 0x12,
	READ_FIFO = 0x14
} rx_state = IDLE;

//Begin spi by enabling tx interrupt
#define START_SPI UCA0IE |= UCTXIE

inline void spi_int()
{
	if (!spi_reading)
	{
		spi_reading = 1;
		TXData = (RF24_NOP); //Send nop to read status
		CSN_EN; //Initiates command sequence to radio
		rx_state = INIT;
		START_SPI; //Begin spi transfer
		INT_DIS; //Disable irq interrupt so as to not be interrupted
	}
	return;
}

inline void nrf_rx_handle()
{
	static uint8_t irq_clear; //Used to clear specific irq's

	switch (__even_in_range(rx_state, 0x14))
	{
	//State when no interrupt is received
	case IDLE:
		break;

//Read status (read initiated prior), respond to IRQ sources
	case INIT:
		CSN_DIS;
		rf_status = RXData; //Read in status

		//Check for each possible interrupt source
		if (rf_status & RF24_IRQ_TX)
		{ //TX success
		  //If tx succes, increment fifo semafore and
		  //	clear irq source bit.
			tx_failed = 0;
			sem_fifo++;

			//If transmitter, deassert CE pin when fifo is empty
			if (!nrf_prx)
				CE_DIS;

			TXData = (RF24_STATUS | RF24_W_REGISTER);
			irq_clear = RF24_IRQ_TX;
			rx_state = CLEAR_IRQ;
		}
		else if (rf_status & RF24_IRQ_TXFAILED)
		{ //TX failed irq, toggle CE and call callback if available
		  //If tx failed,
			tx_failed = 1;
			if (!nrf_prx)
				CE_DIS;
			if (nrf_txFailCallback)
				nrf_txFailCallback();
			TXData = (RF24_STATUS | RF24_W_REGISTER);
			irq_clear = RF24_IRQ_TXFAILED;
			rx_state = CLEAR_IRQ;
		}
		else if (rf_status & RF24_IRQ_RX)
		{ //RX IRQ, begin by sending command to read byte count
			TXData = RF24_R_RX_PL_WID;
			rx_state = READ_SIZE;
		}
		else
		{ //No IRQ source, return to idle
			spi_reading = 0;
			rx_state = IDLE;
			INT_EN;
			if (!nrf_prx)
			{
				if (sem_fifo < 3)
					CE_EN;
				else
					CE_DIS;
			}
			return; //Return to not start another spi read
		}

		CSN_EN;
		START_SPI;
		break;

//We need to read the size of the available bytes in the RX FIFO
	case READ_SIZE:
		rf_status = RXData; //Command sent, send nop to receive size
		TXData = (RF24_NOP);
		START_SPI;
		rx_state = PREP;
		break;

//Finish reading byte count, prep reading payload
	case PREP:
		CSN_DIS;
		rx_count = RXData;
		TXData = RF24_R_RX_PAYLOAD;
		CSN_EN;
		START_SPI;
		rx_state = START;
		break;

//Command sent, initiate reading from fifo
	case START:
		rf_status = RXData;
		TXData = RF24_NOP;
		START_SPI;
		rx_state = READ_FIRST;
		break;

//Read the first byte, ignore if 0, pass to buffer if non zero
	case READ_FIRST:
		rx_state = READ;
		if (!RXData)
			break;

//Main reading state, read rx_count bytes
	case READ:
		IOputc(RXData, nrf_slot);
		if (!(--rx_count))
		{
			CSN_DIS;
			rx_state = CLEAR_IRQ;
			irq_clear = RF24_IRQ_RX;
			TXData = (RF24_STATUS | RF24_W_REGISTER);
			CSN_EN;
		}
		START_SPI;
		break;

//Send command to clear IRQ in irq_clear variable
	case CLEAR_IRQ:
		rf_status = RXData;
		TXData = irq_clear;
		START_SPI;
		rx_state = CMD_FIFO;
		break;

//Send command to read fifo state
	case CMD_FIFO:
		CSN_DIS;

		TXData = (RF24_FIFO_STATUS | RF24_R_REGISTER);
		CSN_EN;
		START_SPI;
		rx_state = FIFO;
		break;

//Command sent, begin reading state
	case FIFO:
		rf_status = RXData;
		TXData = RF24_NOP;
		START_SPI;
		rx_state = READ_FIFO;
		break;

//Read fifo state, return to idle if empty; restart if not
	case READ_FIFO:
		CSN_DIS;
		if (RXData & RF24_RX_EMPTY)
		{
			TXData = (RF24_NOP);
			rx_state = INIT; //Reread status to check for IRQ again
			START_SPI;
			CSN_EN;
		}
		else
		{
			TXData = RF24_R_RX_PL_WID;
			CSN_EN;
			START_SPI;
			rx_state = READ_SIZE; //Begin again, there is more data in rx fifo
		}
		break;

//Abandon all hope ye who enter here.  Or return to idle.
	default:
		rx_state = IDLE;
		break;
	}
}

inline void port_init()
{
#if nrfIRQport == 1
	P1DIR &= ~nrfIRQpin;  // IRQ line is input
	P1OUT |= nrfIRQpin;  // Pull-up resistor enabled
	P1REN |= nrfIRQpin;
	P1IES |= nrfIRQpin;  // Trigger on falling-edge
	P1IFG &= ~nrfIRQpin;  // Clear any outstanding IRQ
	P1IE |= nrfIRQpin;  // Enable IRQ interrupt
#elif nrfIRQport == 2
			P2DIR &= ~nrfIRQpin;  // IRQ line is input
			P2OUT |= nrfIRQpin;// Pull-up resistor enabled
			P2REN |= nrfIRQpin;
			P2IES |= nrfIRQpin;// Trigger on falling-edge
			P2IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P2IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 3
			P3DIR &= ~nrfIRQpin;  // IRQ line is input
			P3OUT |= nrfIRQpin;// Pull-up resistor enabled
			P3REN |= nrfIRQpin;
			P3IES |= nrfIRQpin;// Trigger on falling-edge
			P3IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P3IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 4
			P4DIR &= ~nrfIRQpin;  // IRQ line is input
			P4OUT |= nrfIRQpin;// Pull-up resistor enabled
			P4REN |= nrfIRQpin;
			P4IES |= nrfIRQpin;// Trigger on falling-edge
			P4IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P4IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 5
			P5DIR &= ~nrfIRQpin;  // IRQ line is input
			P5OUT |= nrfIRQpin;// Pull-up resistor enabled
			P5REN |= nrfIRQpin;
			P5IES |= nrfIRQpin;// Trigger on falling-edge
			P5IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P5IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 6
			P6DIR &= ~nrfIRQpin;  // IRQ line is input
			P6OUT |= nrfIRQpin;// Pull-up resistor enabled
			P6REN |= nrfIRQpin;
			P6IES |= nrfIRQpin;// Trigger on falling-edge
			P6IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P6IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 7
			P7DIR &= ~nrfIRQpin;  // IRQ line is input
			P7OUT |= nrfIRQpin;// Pull-up resistor enabled
			P7REN |= nrfIRQpin;
			P7IES |= nrfIRQpin;// Trigger on falling-edge
			P7IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P7IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 8
			P8DIR &= ~nrfIRQpin;  // IRQ line is input
			P8OUT |= nrfIRQpin;// Pull-up resistor enabled
			P8REN |= nrfIRQpin;
			P8IES |= nrfIRQpin;// Trigger on falling-edge
			P8IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P8IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 9
			P9DIR &= ~nrfIRQpin;  // IRQ line is input
			P9OUT |= nrfIRQpin;// Pull-up resistor enabled
			P9REN |= nrfIRQpin;
			P9IES |= nrfIRQpin;// Trigger on falling-edge
			P9IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P9IE |= nrfIRQpin;// Enable IRQ interrupt
#elif nrfIRQport == 10
			P10DIR &= ~nrfIRQpin;  // IRQ line is input
			P10OUT |= nrfIRQpin;// Pull-up resistor enabled
			P10REN |= nrfIRQpin;
			P10IES |= nrfIRQpin;// Trigger on falling-edge
			P10IFG &= ~nrfIRQpin;// Clear any outstanding IRQ
			P10IE |= nrfIRQpin;// Enable IRQ interrupt
#endif

// Setup CSN/CE ports
#if nrfCSNport == 1
	P1DIR |= nrfCSNpin;
#elif nrfCSNport == 2
	P2DIR |= nrfCSNpin;
#elif nrfCSNport == 3
	P3DIR |= nrfCSNpin;
#elif nrfCSNport == 4
	P4DIR |= nrfCSNpin;
#elif nrfCSNport == 5
	P5DIR |= nrfCSNpin;
#elif nrfCSNport == 6
	P6DIR |= nrfCSNpin;
#elif nrfCSNport == 7
	P7DIR |= nrfCSNpin;
#elif nrfCSNport == 8
	P8DIR |= nrfCSNpin;
#elif nrfCSNport == 9
	P9DIR |= nrfCSNpin;
#elif nrfCSNport == 10
	P10DIR |= nrfCSNpin;
#elif nrfCSNport == J
	PJDIR |= nrfCSNpin;
#endif
	CSN_DIS;

#if nrfCEport == 1
	P1DIR |= nrfCEpin;
#elif nrfCEport == 2
	P2DIR |= nrfCEpin;
#elif nrfCEport == 3
	P3DIR |= nrfCEpin;
#elif nrfCEport == 4
	P4DIR |= nrfCEpin;
#elif nrfCEport == 5
	P5DIR |= nrfCEpin;
#elif nrfCEport == 6
	P6DIR |= nrfCEpin;
#elif nrfCEport == 7
	P7DIR |= nrfCEpin;
#elif nrfCEport == 8
	P8DIR |= nrfCEpin;
#elif nrfCEport == 9
	P9DIR |= nrfCEpin;
#elif nrfCEport == 10
	P10DIR |= nrfCEpin;
#elif nrfCEport == J
	PJDIR |= nrfCEpin;
#endif
	CE_DIS;

	/* Straw-man spi_transfer with no Chip Select lines enabled; this is to workaround errata bug USI5
	 * on the MSP430G2452 and related (see http://www.ti.com/lit/er/slaz072/slaz072.pdf)
	 * Shouldn't hurt anything since we expect no CS lines enabled by the user during this function's execution.
	 */
	spi_transfer(RF24_NOP);
}
