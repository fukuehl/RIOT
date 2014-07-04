/*
 * Copyright (C) 2013 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_stm32f0discovery
 * @{
 *
 * @file        board.c
 * @brief       Board specific implementations for the STM32F0Discovery evaluation board
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "periph/timer.h"
#include "board.h"
#include "cpu.h"
#include "nrf51.h"
#include "nrf_delay.h"
//#include "nrf51_bitfields.h"
#include "periph/uart.h"

//#include "hwtimer_arch.h"
//#include "thread.h"
#include "periph/gpio.h"

//#include "radio.h"

extern void SystemInit(void);
void leds_init(void);

void printInt(int i){
	printf("timer0: channel: %d\n",i);
}
void printInt2(int i){
	printf("timer1: channel: %d\n",i);
}



void board_init(void)
{
	char charUART = '\0';
	int i = 0;
    /* initialize core clocks via STM-lib given function */
    SystemInit();

    /* initialize the CPU */
    cpu_init();

    /* initialize the boards LEDs */
    leds_init();

    /*initialize UART */
    uart_init_blocking(0, 115200);
    char* output = "Hello User!\r\n";
    char outputchar = output[i++];
	while ( outputchar != '\0')
	{
		uart_write_blocking(0, outputchar);
		outputchar = output[i++];
	}
	LED_RED_ON;
	i = 0;


	timer_init(TIMER_0, 1, printInt);
	timer_set(TIMER_0,0,2*1000*1000);
	timer_set(TIMER_0,1,4*1000*1000);
	timer_set(TIMER_0,2, 6*1000*1000);
	timer_set(TIMER_0,3, 8*1000*1000);


	timer_init(TIMER_1, 1, printInt2);
	timer_set(TIMER_1,0,1*1000*1000);
	timer_set(TIMER_1,1,3*1000*1000);
	timer_set(TIMER_1,2,5*1000*1000);
	timer_set(TIMER_1,3,7*1000*1000);


	int k = 0;
	while(1){
		k++;
		for(int i=0; i <= 1 * 1000 * 1000;i++){
			asm("nop");
		}
		//int a = timer_read(TIMER_0);

		//if ( a <= 8*1000*1000){
			//int k = NRF_TIMER0->TASKS_CAPTURE[1];
			//printf("k : %i\n", k);

		//}

	}


    /* blink stuff */

/*    while (1) {


		while(uart_read_blocking(0,&charUART))  {
				uart_write_blocking(0, charUART);
		}


        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }
        LED_RED_OFF;
        LED_GREEN_OFF;
        LED_BLUE_OFF;
        uart_write_blocking(0, 'O');

        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }
        LED_RED_ON;
        uart_write_blocking(0, 'R');
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }
        LED_RED_OFF;
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }
        LED_GREEN_ON;
        uart_write_blocking(0, 'G');
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }
        LED_GREEN_OFF;
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }
        LED_BLUE_ON;
        uart_write_blocking(0, 'B');
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }
        LED_RED_OFF;
        LED_GREEN_OFF;
        LED_BLUE_OFF;
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }
        LED_GREEN_ON;
        LED_BLUE_ON;
        LED_RED_ON;
        uart_write_blocking(0, 'A');
        for (int i = 0; i < 1000000; i++) {
            asm("nop");
        }

    } */
}


void delay(uint32_t microseconds){
    /* perform busy-waiting for specified number of microseconds  */
    uint32_t cycles = microseconds * 2; // factor has been found by measure
    for (int i = 0; i < cycles; i++) {
        asm("nop");
    }

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
    NRF_GPIO->DIRSET = (LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN);

    /* turn all LEDs off */
    NRF_GPIO->OUTCLR = (LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN);
}
