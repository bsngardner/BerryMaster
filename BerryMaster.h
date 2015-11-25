/*
 * BerryMaster.h
 *
 *  Created on: Nov 19, 2015
 *   Author(s): Marshall Garey
 *
 * Contains system macros: hardware version, pins, events, errors, IO messages
 */

#ifndef BERRYMASTER_H_
#define BERRYMASTER_H_

// Hardware version
#define MAMA_REV_A

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
#define LED0 0x01
#define LED1 0x02
#define ASDA 0x04
#define ASCL 0x08

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
#define INT0 0x01
#define INT1 0x02
#define BCLK 0x04
#define SW1  0x08
#define USBINT 0x10 // USB interrupt
#define SCLK 0x20
#define SDA  0x40
#define SCL  0x80

// Port 2
#define MOSI 0x01
#define MISO 0x02
#define BCHG 0x04
#define MOBI 0x08
#define MIBO 0x10
#define RFCS 0x20
#define RFCE 0x40
// pin 2.7 (0x80) not on chip

// System errors
enum SYS_ERRORS {
	SYS_ERR_430init = 1,	// 1 eZ430X initialize
	SYS_ERR_USCB_RX,		// 2 USCB receive timeout
	SYS_ERR_I2C_TO,			// 3 i2c timeout
	SYS_ERR_I2C_ACK,		// 4 i2c ACK timeout
	SYS_ERR_I2C_DEV,		// 5 i2c device error
	SYS_ERR_EVENT,			// 6 event error
	SYS_ERR_ISR,			// 7 unrecognized interrupt
	SYS_ERR_NO_CMD,			// 8 no CMD file
	SYS_ERR_NO_DATA,		// 9 no DATA.CSV
	SYS_ERR_WDT_MUTEX,		// 10 WDT mutex fault
	SYS_ERR_DC_MAX,			// 11 too many events in DC
};

// Maximum size of a message to or from the Master
#define MAX_MSG_LENGTH 64

// Events
#define USB_I_EVENT 0x01
#define USB_O_EVENT 0x02
#define WRITE_DEBUG_EVENT 0x04

#endif /* BERRYMASTER_H_ */
