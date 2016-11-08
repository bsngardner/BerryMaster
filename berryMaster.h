/*
 * BerryMaster.h
 *
 *  Created on: Nov 19, 2015
 *   Author(s): Marshall Garey
 */

#ifndef BERRYMASTER_H_
#define BERRYMASTER_H_

#include <stdint.h>
#include "IObuffer.h"

// Hardware version
#define MASTER_REV_A

#ifdef MASTER_REV_A
/*
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

// Port J
#define LED0 0x01 // Used as heartbeat LED
#define LED1 0x02 // Alternate led used for anything
#define ASDA 0x04 // Serial data line for ft201x i2c
#define ASCL 0x08 // Serial clock for ft201x i2c

// Port J LED operations:
#define LED_MASK 0x03
#define LED0_OFF (PJOUT &= ~LED0)
#define LED0_ON (PJOUT |= LED0)
#define LED0_TOGGLE (PJOUT ^= LED0)
#define LED1_OFF (PJOUT &= ~LED1)
#define LED1_ON (PJOUT |= LED1)
#define LED1_TOGGLE (PJOUT ^= LED1)
#define LEDS_TOGGLE (PJOUT ^= LED_MASK)

// Port 1
#define INT0 0x01 // Radio interrupt
#define BINT 0x02 // Berry interrupt (on the vine)
#define BCLK 0x04 // Clock for berry i2c
#define SW1  0x08 // The button
#define USBINT 0x10 // ft201x USB interrupt
#define SCLK 0x20
#define SDA  0x40 // data for radio spi
#define SCL  0x80

// Port 2
#define MOSI 0x01 // mosi for radio spi
#define MISO 0x02 // miso for radio spi
#define BCHG 0x04 // battery charger
#define MOBI 0x08 // master out for berry i2c
#define MIBO 0x10 // master in for berry i2c
#define RFCS 0x20 // radio chip select
#define RFCE 0x40 // radio chip enable
// pin 2.7 (0x80) not on chip
#endif

// System errors
enum SYS_ERRORS
{
	SYS_ERR_430init = 1,	// 1 initialize
	SYS_ERR_USCB_RX,		// 2 USCB receive timeout
	SYS_ERR_I2C_TO,			// 3 i2c timeout
	SYS_ERR_I2C_ACK,		// 4 i2c ACK timeout
	SYS_ERR_I2C_DEV,		// 5 i2c device error
	SYS_ERR_EVENT,			// 6 event error
	SYS_ERR_ISR,			// 7 unrecognized interrupt
	SYS_ERR_IOBUFFER,		// 8 iobuffer error
	SYS_ERR_RX_HOST_MSG,	// 9 error receiving message from host
	SYS_ERR_MALLOC			// 10 error using malloc
};

// Other macros
#define ASCII_ZERO 48
#define TRUE 1
#define FALSE 0
#define NULL 0

// Device interrupts
#define INTR_SRC_MASTER 0
#define INTR_TYPE_MISSING_BERRY 0
#define INTR_TYPE_NEW_BERRY 1
#define INTR_TYPE_FOUND_BERRY 2

// Errors
#define NETWORK_FULL		-1
#define INVALID_ADDR 		-2
#define DEVICE_NOT_FOUND	-3
#define VALIDATE_LIST_ERR	-4
#define READ_BYTES_OVERFLOW -5
#define SET_BYTES_OVERFLOW	-6
#define SUCCESS				0

/******************************************************************************
 * Data structures ************************************************************
 *****************************************************************************/

/*
 * if the device address is zero, the device is not configured
 */
typedef struct Device
{
	uint8_t address;
	uint8_t missing;
	uint8_t int_en;
} Device_t;

// Maximum number of devices allowed on the network - limited to 127 because
// the vine uses 7 address bits, no device will be allowed to use address 0.
#define MAX_NUM_DEVICES 30u
#define DEVICES_ARRAY_SIZE (MAX_NUM_DEVICES+1)

/******************************************************************************
 * master API function prototypes *********************************************
 *****************************************************************************/

int init_devices(uint16_t project_key, uint8_t hot_swap_en);
int get_device_multi_values(uint8_t addr, int8_t reg, uint8_t* buff,
		uint8_t count);
int set_device_multi_values(uint8_t addr, int8_t reg, uint8_t* buff,
		uint8_t count);
int update_proj_key(uint16_t new_proj_key);
int enable_interrupt(uint8_t addr, uint8_t int_type);
void vine_interrupt_event();
void hot_swap_event();

enum log_type_e { error_msg, log_msg, warning_msg };
void send_log_msg(char *msg, enum log_type_e log_type);


#endif /* BERRYMASTER_H_ */
