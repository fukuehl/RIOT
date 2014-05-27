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
 * @file        uart.c
 * @brief       Low-level UART driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <math.h>

#include "cpu.h"
#include "periph_conf.h"
#include "periph/uart.h"
#include "nrf51.h"


/**
 * @brief Each UART device has to store two callbacks.
 */
typedef struct {
    void (*rx_cb)(char);
    void (*tx_cb)(void);
} uart_conf_t;


/**
 * @brief Allocate memory to store the callback functions.
 */
static uart_conf_t config[UART_NUMOF];


int uart_init(uart_t uart, uint32_t baudrate, void (*rx_cb)(char), void (*tx_cb)(void))
{
    return 0;
}

int uart_init_blocking(uart_t uart, uint32_t baudrate)
{
/*hier anfangen Register beschreiben und clocks etc 115200 standard baudrate Arduino DUO code abgleichen*/
    return 0;
}


void uart_tx_begin(uart_t uart)
{
    /* TODO */
}

void uart_tx_end(uart_t uart)
{
    /* TODO */
}

int uart_write(uart_t uart, char data)
{
	/*copy from cpu/sam3x8e/periph/uart.c*/
	//TODO Register adjust
    switch (uart) {
        case UART_0:
        	  NRF_UART0->TXD = (uint8_t)data;

        	  while (NRF_UART0->EVENTS_TXDRDY!=1)
        	  {
        	    // Wait for TXD data to be sent
        	  }

        	  NRF_UART0->EVENTS_TXDRDY=0;
            break;
        case UART_UNDEFINED:
            return -1;
    }
    return 1;
}

int uart_read_blocking(uart_t uart, char *data)
{	/*copy from cpu/sam3x8e/periph/uart.c*/
	//TODO adjust Registers
    return -1;
    switch (uart) {
        case UART_0:
        	  uint_fast8_t i = 0;
        	  uint8_t ch = data[i++];
        	  while (ch != '\0')
        	  {
        	    uart_write(ch);
        	    ch = data[i++];
        	  }

            //while (!(UART_0_DEV->UART_SR & UART_SR_RXRDY));
            //*data = (char)UART_0_DEV->UART_RHR;
            break;
        case UART_UNDEFINED:
            return -1;
    }
    return 1;
}

int uart_write_blocking(uart_t uart, char data)
{	/*copy from cpu/sam3x8e/periph/uart.c*/
	//TODO adjust Registers
    switch (uart) {
        case UART_0:
            //while (!(UART_0_DEV->UART_SR & UART_SR_TXRDY));
            //UART_0_DEV->UART_THR = data;
            break;
        case UART_UNDEFINED:
            return -1;
    }
    return 1;

}

