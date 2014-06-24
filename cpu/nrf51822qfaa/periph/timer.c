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
 */
timer_conf_t config[TIMER_NUMOF];

int timer_init(tim_t dev, unsigned int ticks_per_us, void (*callback)(int)) {

	volatile NRF_TIMER_Type * p_timer;
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

	}

	/* save callback */
	config[dev].cb = callback;

	p_timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;	//32 Bit Mode
	p_timer->MODE    = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.

	switch (ticks_per_us) {
		case 1:
			p_timer->PRESCALER = 32;
			break;
		case 2:
			p_timer->PRESCALER = 16;
			break;
		case 3:
			p_timer->PRESCALER = 8;
			break;
		case 4:
			p_timer->PRESCALER = 4;
			break;
		case 5:
			p_timer->PRESCALER = 2;
			break;

		default:
			break;
	}


	return 1;
}

int timer_set(tim_t dev, int channel, unsigned int timeout) {
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

	}

	/* set timeout value */

	switch (channel) {
		case 0:
			p_timer->CC[0] = timeout;

			break;
		case 1:
			p_timer->CC[1] = timeout;

			break;
		case 2:
			p_timer->CC[2] = timeout;

			break;
		case 3:
			p_timer->CC[3] = timeout;

			break;
		default:
			break;
	}

	return 1;
}

int timer_clear(tim_t dev, int channel) {
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

	}

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

unsigned int timer_read(tim_t dev) {
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

void timer_start(tim_t dev) {
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
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
        	NRF_TIMER1->TASKS_STOP = 1;
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
        	NRF_TIMER2->TASKS_STOP = 1;
            break;
#endif
        case TIMER_UNDEFINED:
            break;
    }
}

void timer_irq_enable(tim_t dev) {
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

void timer_irq_disable(tim_t dev) {
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

void timer_reset(tim_t dev) {
	/* TODO */
}