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

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/timer.h"



typedef struct {
    void (*cb)(int);
} timer_conf_t;

/**
 * Timer state memory
 */
timer_conf_t config[TIMER_NUMOF];



int timer_init(tim_t dev, unsigned int ticks_per_us, void (*callback)(int))
{
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
