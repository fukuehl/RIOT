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
#include "board.h"
#include "pca10000.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"


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
{//Manual: nrf51822.pdf page 144

    // initialize basic functionality
   int res = uart_init_blocking(uart, baudrate);
   if (res != 0) {
       return res;
   }

    return 0;

      /* copy from sam3xbe
       *
       *
     // initialize basic functionality
    int res = uart_init_blocking(uart, baudrate);
    if (res != 0) {
        return res;
    }

    // register callbacks
    config[uart].rx_cb = rx_cb;
    config[uart].tx_cb = tx_cb;

    // configure interrupts and enable RX interrupt
    switch (uart) {
        case UART_0:
            UART_0_DEV->UART_IER = UART_IER_RXRDY;
        break;
        case UART_UNDEFINED:
            return -2;
    }
    return 0;
       */
}

int uart_init_blocking(uart_t uart, uint32_t baudrate)
{
/*hier anfangen Register beschreiben und clocks etc 115200 standard baudrate Arduino DUO code abgleichen*/
//copy from cpu     sam3x8e
    uint16_t clock_divider = F_CPU / (16 * baudrate);
    int baudrate_real;
    switch(baudrate) {
    case 1200 :
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud1200;
    	break;
    case 2400:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud2400;
    	break;
    case 4800:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud4800;
    	break;
    case 9600:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud9600;
    	break;
    case 14400:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud14400;
    	break;
    case 19200:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud19200;
    	break;
    case 28800:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud28800;
    	break;
    case 38400:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud38400;
    	break;
    case 57600:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud57600;
    	break;
    case 76800:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud76800;
    	break;
    case 115200:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud115200;
    	break;
    case 230400:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud230400;
    	break;
    case 250000:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud250000;
    	break;
    case 460800:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud460800;
    	break;
    case 921600:
    	baudrate_real = UART_BAUDRATE_BAUDRATE_Baud921600;
    	break;

    }
    switch (uart) {
	#if UART_0_EN
        case UART_0:
            /* configure PINS */
           // UART_0_PORT->PIO_PDR = UART_0_PINS;
           // UART_0_PORT->PIO_ABSR &= ~UART_0_PINS;

        	/** @snippet [Configure UART RX and TX pin] */
            nrf_gpio_cfg_output(RTS_PIN_NUMBER);
            nrf_gpio_cfg_input(CTS_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

            NRF_UART0->PSELCTS = CTS_PIN_NUMBER;
            NRF_UART0->PSELRTS = RTS_PIN_NUMBER;
            NRF_UART0->CONFIG  = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);


            NRF_UART0->BAUDRATE         = (baudrate_real << UART_BAUDRATE_BAUDRATE_Pos);
            NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
            NRF_UART0->TASKS_STARTTX    = 1;
            NRF_UART0->TASKS_STARTRX    = 1;
            NRF_UART0->EVENTS_RXDRDY    = 0;


            /* set clock divider */
           // UART_0_DEV->UART_BRGR = clock_divider;
            /* set to normal mode without parity */
           // UART_0_DEV->UART_MR = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;
            /* enable receiver and transmitter and reset status bits */
           // UART_0_DEV->UART_CR = UART_CR_RXEN | UART_CR_TXEN | UART_CR_RSTSTA;
            break;
#endif
        case UART_UNDEFINED:
            return -2;
            break;
    }
    return 0;
    //copy from simple_uart.c SDK
    /** @snippet [Configure UART RX and TX pin] */

    /* uint8_t rts_pin_number,
                             uint8_t txd_pin_number,
                             uint8_t cts_pin_number,
                             uint8_t rxd_pin_number,
                             bool    hwfc /*

      nrf_gpio_cfg_output(txd_pin_number);
      nrf_gpio_cfg_input(rxd_pin_number, NRF_GPIO_PIN_NOPULL);

      NRF_UART0->PSELTXD = txd_pin_number;
      NRF_UART0->PSELRXD = rxd_pin_number;
    /** @snippet [Configure UART RX and TX pin] */
    /*
      if (hwfc)
      {
        nrf_gpio_cfg_output(rts_pin_number);
        nrf_gpio_cfg_input(cts_pin_number, NRF_GPIO_PIN_NOPULL);
        NRF_UART0->PSELCTS = cts_pin_number;
        NRF_UART0->PSELRTS = rts_pin_number;
        NRF_UART0->CONFIG  = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
      }

      NRF_UART0->BAUDRATE         = (UART_BAUDRATE_BAUDRATE_Baud38400 << UART_BAUDRATE_BAUDRATE_Pos);
      NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
      NRF_UART0->TASKS_STARTTX    = 1;
      NRF_UART0->TASKS_STARTRX    = 1;
      NRF_UART0->EVENTS_RXDRDY    = 0;
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
    switch (uart) {
        case UART_0:
        	  while (NRF_UART0->EVENTS_RXDRDY != 1)
        	  {
        	    // Wait for RXD data to be received
        	  }

        	  NRF_UART0->EVENTS_RXDRDY = 0;
        	  *data = (char)NRF_UART0->RXD;
        	  return (uint8_t)NRF_UART0->RXD;

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
	//Using:
	 /* while (ch != '\0')
	  {
	    uart_write_blocking(uart, ch);
	    ch = data[i++];
	  }*/
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

