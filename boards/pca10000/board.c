/*
 * Copyright (C) 2013 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_pca10000
 * @{
 *
 * @file        board.c
 * @brief       Board specific implementations for the nRF51822 evaluation board pca10000
 *
 * @author      Christian Kühling <kuehling@zedat.fu-berlin.de>
 * @author      Timo Ziegler <timo.ziegler@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "board.h"
#include "cpu.h"
#include "nrf51.h"
#include "periph/uart.h"

extern void SystemInit(void);

void leds_init(void);

void board_init(void)
{
    /* initialize the boards LEDs */
    leds_init();

    /* initialize core clocks via Nordic-lib given function */
    SystemInit();

    /* initialize the CPU */
    cpu_init();

    // uart_init_blocking(UART_0, 115200);

    // while (1) {
    //     uart_write_blocking(UART_0, 'A');
    //     uart_write_blocking(UART_0, '\n');
    //     for (int i = 0; i < 1000000; i++) {
    //         asm("nop");
    //     }
    // }

    // /*initialize UART */
    // uart_init_blocking(0, 115200);
}

/**
 * @brief Initialize the boards on-board RGB LED
 *
 * The LED initialization is hard-coded in this function.
 *
 * The LED channels are connected to the following pins:
 * - RED:   21
 * - GREEN: 22
 * - BLUE:  23
 */
void leds_init(void)
{
    /* set LED pins to function as output */
    GPIO_DEV->DIRSET = (LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN);

    /* turn all LEDs off (low active) */
    GPIO_DEV->OUTSET = (LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN);
}
