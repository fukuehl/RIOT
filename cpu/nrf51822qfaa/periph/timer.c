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



int timer_init(tim_t dev, unsigned int ticks_per_us, void (*callback)(int))
{

    volatile NRF_TIMER_Type * p_timer;

    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

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

#if TIMER_3_EN
    case (TIMER_3):
		 p_timer = NRF_TIMER3;
		break;
#endif

    }

    //p_timer->


    p_timer->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    p_timer->PRESCALER      = 9;                            // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    p_timer->BITMODE        = TIMER_BITMODE_BITMODE_16Bit;  // 16 bit mode.
    p_timer->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.

    // With 32 us ticks, we need to multiply by 31.25 to get milliseconds.
    //p_timer->CC[0]          = number_of_ms * 31;
    //p_timer->CC[0]         += number_of_ms / 4;
    p_timer->CC[0] 			= ticks_per_us;
    p_timer->TASKS_START    = 1;                    // Start timer.

    while (p_timer->EVENTS_COMPARE[0] == 0)
    {
        // Do nothing.
    }

    p_timer->EVENTS_COMPARE[0]  = 0;
    p_timer->TASKS_STOP         = 1;                // Stop timer.


    return -1;
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
    return -1;
}

int timer_clear(tim_t dev, int channel)
{
    return -1;
}

unsigned int timer_read(tim_t dev)
{
    return 0;
}

void timer_start(tim_t dev)
{
    /* TODO */
}

void timer_stop(tim_t dev)
{
    /* TODO */
}

void timer_irq_enable(tim_t dev)
{
    /* TODO */
}

void timer_irq_disable(tim_t dev)
{
    /* TODO */
}

void timer_reset(tim_t dev)
{
    /* TODO */
}
