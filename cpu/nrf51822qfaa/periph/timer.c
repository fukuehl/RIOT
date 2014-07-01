/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     driver_periph
 * @{
 *
 * @file        timer.c
 * @brief       Low-level timer driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdlib.h>
#include <stdio.h>

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/timer.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"

typedef struct {
	void (*cb)(int);
} timer_conf_t;

/**
 * Timer state memory
 */

/*
 * page 90 in nrf51822.pdf
 * TimerFrequency = fTimer = HFCLK / (2 ^ Prescaler)
 * nrf51822 has 4 Timer Capture/Compare Registers
 * and 3 Hardware Timers
 */
timer_conf_t config[TIMER_NUMOF];

int timer_init(tim_t dev, unsigned int ticks_per_us, void (*callback)(int))
{

	volatile NRF_TIMER_Type * p_timer;

/*
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

*/

	switch (dev) {

#if TIMER_0_EN
	case (TIMER_0):
		p_timer = NRF_TIMER0;
		NVIC_SetPriority(TIMER0_IRQn, 1);
		NVIC_EnableIRQ(TIMER0_IRQn);
		break;
#endif

#if TIMER_1_EN
	case (TIMER_1):
		p_timer = NRF_TIMER1;
		NVIC_SetPriority(TIMER1_IRQn, 1);
		NVIC_EnableIRQ(TIMER1_IRQn);
		break;
#endif

#if TIMER_2_EN
	case (TIMER_2):
		p_timer = NRF_TIMER2;
		NVIC_SetPriority(TIMER2_IRQn, 1);
		NVIC_EnableIRQ(TIMER2_IRQn);
		break;
#endif
    case TIMER_UNDEFINED:
        break;


	}

	/* save callback */
	config[dev].cb = callback;

	p_timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;	//32 Bit Mode
	p_timer->MODE    = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.
	p_timer->TASKS_CLEAR    = 1;                    // clear the task first to be usable for later.

	switch (ticks_per_us) {
		case 1:
			p_timer->PRESCALER = 0;
			break;
		case 2:
			p_timer->PRESCALER = 1;
			break;
		case 4:
			p_timer->PRESCALER = 2;
			break;
		case 8:
			p_timer->PRESCALER = 3;
			break;
		case 16:
			p_timer->PRESCALER = 4;
			break;

		default:
			p_timer->PRESCALER = 1;
			break;
	}


	return 1;
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
	volatile NRF_TIMER_Type * p_timer;

	/* get timer base register address */
	switch (dev) {

#if TIMER_0_EN
	case (TIMER_0):
		p_timer = NRF_TIMER0;
		break;
#endif

#if TIMER_1_EN
	case (TIMER_1):
		p_timer = NRF_TIMER1;
		break;
#endif

#if TIMER_2_EN
	case (TIMER_2):
		p_timer = NRF_TIMER2;
		break;
#endif
    case TIMER_UNDEFINED:
        break;


	}

	/* set timeout value */

	switch (channel) {
		case 0:
			p_timer->CC[0] = timeout;
			p_timer->INTENSET |= TIMER_INTENSET_COMPARE0_Msk;
			//p_timer->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);

			break;
		case 1:
			p_timer->CC[1] = timeout;
			p_timer->INTENSET |= TIMER_INTENSET_COMPARE1_Msk;
			//p_timer->INTENSET = (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);

			break;
		case 2:
			p_timer->CC[2] = timeout;
			p_timer->INTENSET |= TIMER_INTENSET_COMPARE2_Msk;
			//p_timer->INTENSET = (TIMER_INTENSET_COMPARE2_Enabled << TIMER_INTENSET_COMPARE2_Pos);

			break;
		case 3:
			p_timer->CC[3] = timeout;
			p_timer->INTENSET |= TIMER_INTENSET_COMPARE3_Msk;
			//p_timer->INTENSET = (TIMER_INTENSET_COMPARE3_Enabled << TIMER_INTENSET_COMPARE3_Pos);

			break;
		default:
			break;
	}
	return 1;
}

int timer_clear(tim_t dev, int channel)
{
	volatile NRF_TIMER_Type * p_timer;

	/* get timer base register address */
	switch (dev) {

#if TIMER_0_EN
	case (TIMER_0):
		p_timer = NRF_TIMER0;
		break;
#endif

#if TIMER_1_EN
	case (TIMER_1):
		p_timer = NRF_TIMER1;
		break;
#endif

#if TIMER_2_EN
	case (TIMER_2):
		p_timer = NRF_TIMER2;
		break;
#endif
    case TIMER_UNDEFINED:
        break;

	}

	p_timer->TASKS_CLEAR = 1;

	/* set timeout value */

	switch (channel) {
		case 0:
			p_timer->CC[0] = 0;

			break;
		case 1:
			p_timer->CC[1] = 0;

			break;
		case 2:
			p_timer->CC[2] = 0;

			break;
		case 3:
			p_timer->CC[3] = 0;

			break;
		default:
			break;
	}

	return 1;
}

unsigned int timer_read(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
            return NRF_TIMER0->CC[0];
#endif
#if TIMER_1_EN
        case TIMER_1:
        	return NRF_TIMER1->CC[0];
#endif
#if TIMER_2_EN
        case TIMER_2:
        	return NRF_TIMER2->CC[0];
#endif
        case TIMER_UNDEFINED:
        default:
            return 0;
    }
}

void timer_start(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
            NRF_TIMER0->TASKS_START = 1;
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
        	NRF_TIMER1->TASKS_START = 1;
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
        	NRF_TIMER2->TASKS_START = 1;
            break;
#endif
        case TIMER_UNDEFINED:
            break;
    }
}

void timer_stop(tim_t dev) {
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
            NRF_TIMER0->TASKS_STOP = 1;
            NRF_TIMER0->TASKS_SHUTDOWN = 1;
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
        	NRF_TIMER1->TASKS_STOP = 1;
        	NRF_TIMER1->TASKS_SHUTDOWN = 1;
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
        	NRF_TIMER2->TASKS_STOP = 1;
        	NRF_TIMER2->TASKS_SHUTDOWN = 1;
            break;
#endif
        case TIMER_UNDEFINED:
            break;
    }
}

void timer_irq_enable(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
            NRF_TIMER0->INTENSET = 1;
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
        	NRF_TIMER1->INTENSET = 1;
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
        	NRF_TIMER2->INTENSET = 1;
            break;
#endif
        case TIMER_UNDEFINED:
            break;
    }
}

void timer_irq_disable(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
            NRF_TIMER0->INTENCLR = 1;
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
        	NRF_TIMER1->INTENCLR = 1;
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
        	NRF_TIMER2->INTENCLR = 1;
            break;
#endif
        case TIMER_UNDEFINED:
            break;
    }
}

void timer_reset(tim_t dev)
{
	/* TODO */
}

void isr_timer0(void)
{
	for(int i = 0; i<4; i++){
		if(NRF_TIMER0->EVENTS_COMPARE[i] == 1){
			config[0].cb(i);
			NRF_TIMER0->INTENCLR &= ~TIMER_INTENCLR_COMPARE0_Msk;
		}
	}
}

void isr_timer1(void)
{
	for(int i = 0; i<4; i++){
		if(NRF_TIMER1->EVENTS_COMPARE[i] == 1){
			config[1].cb(i);
			NRF_TIMER1->INTENCLR &= ~TIMER_INTENCLR_COMPARE1_Msk;
		}
	}
}

void isr_timer2(void)
{
	for(int i = 0; i<4; i++){
		if(NRF_TIMER2->EVENTS_COMPARE[i] == 1){
			config[2].cb(i);
			NRF_TIMER2->INTENCLR &= ~TIMER_INTENCLR_COMPARE2_Msk;
		}
	}
}
