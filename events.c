/*
 * events.c
 *
 *  Created on: Jan 13, 2016
 *      Author: Broderick
 */

#include "events.h"

volatile uint16_t sys_event;

void event_loop() {
	// Wait for an interrupt
	while (1) {
		// disable interrupts before check sys_event
		__disable_interrupt();

		if (sys_event) {
			// if there's something pending, enable interrupts before servicing
			__enable_interrupt();
		} else {
			// otherwise, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(LPM0_bits | GIE);
			continue;
		}

		// Service any pending events.
		if (sys_event & USB_I_EVENT) {
			sys_event &= ~USB_I_EVENT;
			USBInEvent();
		} else if (sys_event & USB_O_EVENT) {
			sys_event &= ~USB_O_EVENT;
			USBOutEvent();
		} else if (sys_event & SERVER_EVENT) {
			sys_event &= ~SERVER_EVENT;
			serverEvent();
		} else if (sys_event & TEST_EVENT) {
			sys_event &= ~TEST_EVENT;
			test_event();
		} else {
			// ERROR. Unrecognized event. Report it.
			reportError("UnrecognizedEventErr", SYS_ERR_EVENT);

			// Clear all pending events -
			// attempt to let the system correct itself.
			sys_event = 0;

			// If the number of event errors reaches a predefined value,
			// stop the program
			numEventErrors++;
			if (numEventErrors >= MAX_EVENT_ERRORS) {
				handleError();
			}
		}
	}
}

#define TYPE_REG 0
#define STATUS_REG 1

#define LED_T 0x02
#define SW_T 0x06

static struct {
	uint8_t addr;
	uint8_t type;
	uint8_t reg;
	uint8_t val;
} devices[8] = { 0 };

int i;
uint8_t addr = 0x1;
uint8_t new = 0;
uint8_t reset = 0;

void test_event() {
	if (!reset) {
		hal_resetAllDevices();
		reset = 1;
	}

	if (!hal_discoverNewDevice(addr)) {
		devices[new].addr = addr;
		hal_getDeviceRegister(devices[new].addr, TYPE_REG,
				&(devices[new].type));
		switch (devices[new].type) {
		case LED_T:
			hal_setDeviceRegister(devices[new].addr, 2, 1);
			hal_setDeviceRegister(devices[new].addr, 3, 1);
			hal_setDeviceRegister(devices[new].addr, 4, 1);
			hal_setDeviceRegister(devices[new].addr, 5, 1);
			hal_setDeviceRegister(devices[new].addr, 2, 0);
			hal_setDeviceRegister(devices[new].addr, 3, 0);
			hal_setDeviceRegister(devices[new].addr, 4, 0);
			hal_setDeviceRegister(devices[new].addr, 5, 0);
			devices[new].reg = devices[new].addr + 2;
			break;
		}
		for (i = 0; i < 8; i++) {
			if (!devices[i].addr) {
				new = i;
				break;
			}
		}
		addr++;
	}

	for (i = 0; i < 8; i++) {

		if (devices[i].addr) {
			if (hal_pingDevice(devices[i].addr)) {
				devices[i].addr = 0;
			} else {
				switch (devices[i].type) {
				case LED_T:
					devices[i].val ^= 0x01;
					hal_setDeviceRegister(devices[i].addr, devices[i].reg,
							devices[i].val);
					break;
				case SW_T:
					hal_getDeviceRegister(devices[i].addr, devices[i].reg,
							&(devices[i].val));
					if (devices[i].val)
						LED1_ON;
					else
						LED1_OFF;
					break;
				default:
					break;
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {

	if (!(--hal_cnt)) {
		hal_cnt = TEST_COUNT;
		sys_event |= TEST_EVENT;
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}

	--WDT_cps_cnt;
	--usb_poll_cnt;

	// One second elapsed
	if (WDT_cps_cnt == 0) {
		WDT_cps_cnt = WDT_CLKS_PER_SEC;
		LED0_TOGGLE; // toggle heartbeat LED
	}
//
//	// Should we poll the USB?
//	if (usb_poll_cnt == 0) {
//		sys_event |= USB_I_EVENT; // poll the usb chip (ft201x)
//		usb_poll_cnt = USB_POLL_CNT; // 1/16 sec
//		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
//	}

	// Are we currently debouncing the switch?
	if (debounceCnt) {
		// Yes; are we finished?
		--debounceCnt;
		if (debounceCnt == 0) {
			// TODO: Signal button event.
		}
	}
}

//-----------------------------------------------------------------------------
//	Port 1 ISR
//
#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void) {
	// USB interrupt?
	if (P1IFG & USBINT) {
		// Clear the pending interrupt.
		P1IFG &= ~USBINT;

		// Signal USB input event
		sys_event |= USB_I_EVENT;

		// Wake up the processor
		__bic_SR_register_on_exit(LPM0_bits);
	}

	// Switch1 interrupt?
	if (P1IFG & SW1) {
		debounceCnt = DEBOUNCE_CNT; // set debounce count
		P1IFG &= ~SW1; // clear interrupt flag
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

