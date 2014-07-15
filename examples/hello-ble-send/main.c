/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Bluetooth Low Energy send application
 *
 * @author      Christian Kühling <kuehling@zedat.fu-berlin.de>>
 * @author      Timo Ziegler <timo.ziegler@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "periph/timer.h"
#include "board.h"
#include "cpu.h"
#include "nrf51.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "radio.h"

void delay(uint32_t microseconds);

/**
 * This example assumes that on GPIO pins 1 and 6 are LEDs connected.
 * This is just necessary to check the current status of a board without using the UART.
 *
 * If using the pca10000, gpio_sets are not disturbing, but LED_{COLOR}_{ON/OFF} is executed,
 * to use the onboard LEDs.
 **/
int main(void)
{
    gpio_init_out(GPIO_6, GPIO_NOPULL); 
    gpio_init_out(GPIO_1, GPIO_NOPULL);

    gpio_set(GPIO_1);
    LED_RED_ON;

    radioConfig();
    char msg = 'Q';
    while(1) {

        // sending all the time some message

        sendPacket(1, msg);

        delay(100*1000);
    }
    return 0;
}

void delay(uint32_t microseconds){
    /* perform busy-waiting for specified number of microseconds  */
    uint32_t cycles = microseconds ;
    for (int i = 0; i < cycles; i++) {
        asm("nop");
    }

}
