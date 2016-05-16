/*
 * timer.c
 *
 *  Created on: May 5, 2016
 *      Author: Broderick
 */

#include "msp430.h"
#include "events.h"
#include "iomatrix.h"
#include "ioprintf.h"

//accum gate range: -11.72 to 11.72 us
//Max value: ~30.52 us, round up to 32
//Mapped range: 16 to 65519
//Scaling factor: (65519-16)/32e-6
//Offset: 16 + 11.72us*scaling factor

//#define DEBUG
#define NEW_SCALE

#ifdef NEW_SCALE
#define MAX_POS_SKEW 50332u
#define ERR_MID_POINT 25166u
#define NATURAL_ERR 15204u
#define CORRECTION_ERR 50332u
#define NATURAL_FREQ 33
#define CORRECTION_FREQ 32

#else
#define MAX_POS_SKEW 47992u
#define ERR_MID_POINT 24004u
#define NATURAL_ERR 14493u
#define CORRECTION_ERR 47975u
#define NATURAL_FREQ 33
#define CORRECTION_FREQ 32

#endif

volatile unsigned int err_accum = 0;
volatile unsigned long sys_time = 0;
volatile unsigned int timer_delay_cnt = 0;

int timer_init()
{
	err_accum = ERR_MID_POINT + NATURAL_ERR;
	TA1CCR0 = NATURAL_FREQ; // TA1 period in clock cycles, ~32kHz / 32 = 1 kHz => 1 ms

	TA1CCTL0 = CCIE; // TA1CCR0 interrupt enabled
	TA1CTL = TACLR | TASSEL_1 | MC_2 | TAIE; // ACLK, continuous mode

	return 0;
}

void timer_delay_ms(int count)
{
	timer_delay_cnt = count;
	LPM1;
}

enum
{
	TA_NONE = 0x00,
	TA_CCR1 = 0x02,
	TA_CCR2 = 0x04,
	TA_CCR3 = 0x06,
	TA_CCR4 = 0x08,
	TA_CCR5 = 0x0A,
	TA_CCR6 = 0x0C,
	TA_IFG = 0x0E,
};

#pragma vector = TIMER1_A1_VECTOR
__interrupt void TimerA1_isr(void)
{
	switch (__even_in_range(TA1IV, 0x0E))
	{
	case TA_NONE:
		break;
	case TA_CCR1:
		break;
	case TA_CCR2:
		break;
	case TA_CCR3:
		break;
	case TA_CCR4:
		break;
	case TA_CCR5:
		break;
	case TA_CCR6:
		break;
	case TA_IFG:
		err_accum = ERR_MID_POINT + NATURAL_ERR;
		TA1CCR0 = NATURAL_FREQ; // TA1 period in clock cycles, ~32kHz / 32 = 1 kHz => 1 ms
		break;
	default:
		break;
	}
}

#define GATED

//-----------------------------------------------------------------------------
// Timer A0 interrupt service routine
//
#pragma vector = TIMER1_A0_VECTOR
__interrupt void TimerA1_CCR0_isr(void)
{
#ifdef GATED
	//If accumulated error is greater than the max,
	// switch to correction freq for one period
	//This keeps the average time per interrupt within
	//	the gate range of 1ms, the gate range being +/-12us
	if (err_accum < MAX_POS_SKEW)
	{
		TA1CCR0 += NATURAL_FREQ;
		err_accum += NATURAL_ERR;
	}
	else
	{
		TA1CCR0 += CORRECTION_FREQ;
		err_accum -= CORRECTION_ERR;
	}
#else
	TA1CCR0 += NATURAL_FREQ;

#endif

	++sys_time;

	//Delay timer
	if (timer_delay_cnt && !(--timer_delay_cnt))
	{
		__bic_SR_register_on_exit(LPM1_bits);
	}

	if (events_tick())
	{
		__bic_SR_register_on_exit(LPM1_bits);
	}

}

#pragma vector=UNMI_VECTOR
__interrupt void unmi_isr(void)
{
	do
	{
		SFRIFG1 &= ~OFIFG;                         // Clear OSCFault flag
		CSCTL5 &= ~XT1OFFG;
	} while (SFRIFG1 & OFIFG);                   // OSCFault flag still set?
}
