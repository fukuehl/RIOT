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
 * @brief       Bluetooth Low Energy receive application
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
    int status;

    radioConfig();
    char msg[255];
	while(1) {
		//char msg = receivePacket();
        //printf("message received: %c", msg);

		status = receivePacketTowards(4,0,msg);

		if (status == 1){
			printf("Received Message: %s\r\n",msg);
		} else if(status == 0) {
			printf("Received no Packet\r\n");
		} else if(status == -1){
			printf("Received Packet but CRC-Check failed\r\n");
		} else {
			printf("Error receiving Packet\r\n");
		}
	}

    return 0;
}
