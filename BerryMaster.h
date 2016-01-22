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

#include <stdint.h>

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
#define INT0 0x01
#define INT1 0x02
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

// System errors
enum SYS_ERRORS {
	SYS_ERR_430init = 1,	// 1 initialize
	SYS_ERR_USCB_RX,		// 2 USCB receive timeout
	SYS_ERR_I2C_TO,			// 3 i2c timeout
	SYS_ERR_I2C_ACK,		// 4 i2c ACK timeout
	SYS_ERR_I2C_DEV,		// 5 i2c device error
	SYS_ERR_EVENT,			// 6 event error
	SYS_ERR_ISR,			// 7 unrecognized interrupt
	SYS_ERR_IOBUFFER,		// 8 iobuffer error
	SYS_ERR_RX_HOST_MSG		// 9 error receiving message from host
};

// Maximum size of a message to or from the Master
#define MAX_MSG_LENGTH 220

// Other macros
#define ASCII_ZERO 48
#define TRUE 1
#define FALSE 0
#define NULL 0

// Device register mapping:
#define MAX_VALS 3 // the max number of value registers in a device

// General registers
#define REG_TYPE 0
#define REG_STATUS 1

// LED device
#define REG_LED0 2
#define REG_LED1 3
#define REG_LED2 4
#define NUM_LEDS 3

// Button device
#define REG_BTN 2

// Device types
#define MAX_DEVICE_TYPES 3
#define UNKNOWN 0
#define TYPE_LED 2
#define TYPE_SW 6
#define TYPE_BUTTON 6
extern const char* typeStrings[];

// Errors
#define NETWORK_FULL		255
#define INVALID_ADDR 		254
#define DEVICE_NOT_FOUND	253
#define VALIDATE_LIST_ERR	252
#define SUCCESS				0

/******************************************************************************
 * Data structures ************************************************************
 *****************************************************************************/

/* Device
 * the device types are listed above in deviceTypes_e
 * the device address is also its number
 * if the device address is zero, the device is not configured
 */
typedef struct Device {
	uint8_t deviceType;
	uint8_t deviceAddress;
	uint8_t vals[MAX_VALS];
} Device_t;

// Maximum number of devices allowed on the network - limited to 127 because
// the vine uses 7 address bits, no device will be allowed to use address 0.
#define MAX_NUM_DEVICES 6u
#define DEVICES_ARRAY_SIZE (MAX_NUM_DEVICES+1)

/* deviceList
 * devices: An array of devices
 * currNumDevices: the current number of devices
 *
 * BIG-O:
 * Insertion and deletion are linear time
 * Referencing is constant time
 */
typedef struct DeviceList {
	// NOTE: don't access index 0, just access 1-127 and the address is
	// 		 also the position of the berry in the array
	Device_t devices[DEVICES_ARRAY_SIZE];
	uint8_t currNumDevices;
} DeviceList_t;

/******************************************************************************
 master API function prototypes ***********************************************
 *****************************************************************************/

/* connectToMaster
 * here for the sake of portability of host code
 * just returns 0 for success
 */
int connectToMaster();

/* disconnectFromMaster
 * here for the sake of portability of host code
 * just returns 0 for success
 */
int disconnectFromMaster();

/* init
 * initializes master:
 * calls hal_init
 * validates current list of berries by pinging each one
 * discovers new devices and assigns them addresses
 * 	 iterates hal_discover(newDevAddr)
 * @return 0 for success.
 */
int initDevices();

/* getDeviceType
 * @param the address of the device
 * @param a pointer to store the type of the device at the specified address;
 * 	       notADevice (0) if device is not on network
 * @return 0 for success; 1 if the device is not on the network.
 */
int getDeviceType(uint8_t addr, uint8_t* deviceType);

/* getDeviceValue
 * gets the value (or current state) of the device at the specified address
 * @param the address of the device
 * @param a pointer to store the value
 * @param the register to read
 * @return 0 if successful; 1 otherwise
 */
int getDeviceValue(uint8_t addr, uint8_t* value, uint8_t reg);

/* setDeviceValue
 * sets the device's value to the specified value
 * @param the address of the device
 * @param the value to set
 * @param the register to write
 * @return 0 if successful; 1 otherwise
 */
int setDeviceValue(uint8_t addr, uint8_t value, uint8_t reg);

/* clearNetwork
 * clears all info about devices in the network
 */
void clearNetwork();

/* printNetworkInfo
 * prints all the info about all configured devices on the network
 * (all other addresses are not configured)
 */

/*
 * Other function prototypes
 */

/* reportError
 * prints an error message and number to the console
 * @param msg - the error message to print
 * @param err - the error number to print
 */
void reportError(char* msg, int err);

/*
 * spins in an infinite loop, toggling LEDs 0 and 1
 */
void handleError();

#endif /* BERRYMASTER_H_ */
