/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_nrf51822
 * @{
 *
 * @file
 * @brief       Low-level NRF51822 RADIO device driver implementation
 *
 * @author      Christian Kuehling <kuehling@zedat.fu-berlin.de>
 * @author      Timo Ziegler <timo.ziegler@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdint.h>
#include <string.h>

#include "cpu.h"
#include "nrf_radio.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"


static char nrf_radio_buf[NRF_RADIO_BUFSIZE];


int nrf_radio_init(void)
{
    /* power on the radio peripheral device */
    nrf_radio_poweron();

    /* reset radio configuration */
    NRF_RADIO->PCNF0 = 0;
    NRF_RADIO->PCNF1 = 0;
    NRF_RADIO->DACNF = 0;

    /* set default mode */
    nrf_radio_set_mode(NRF_RADIO_DEFAULT_MODE);

    /* set the default channel */
    nrf_radio_set_channel(NRF_RADIO_DEFAULT_CHANNEL);

    /* set default transmit power */
    nrf_radio_set_power(NRF_RADIO_DEFAULT_TXPOWER);

    /* TODO: disable CRC's (for now) */
    NRF_RADIO->CRCCNF = 0;

    /* packet length configuration */
    NRF_RADIO->PCNF0 = 8;           /* set length field to 8 bit -> 1 byte, S0 and S1 to 0 bit */
    NRF_RADIO->PCNF1 |= NRF_RADIO_MAX_PACKET_SIZE;

    /* address configuration: base address configuration */
    NRF_RADIO->PCNF1 |= (NRF_RADIO_DEFAULT_BASEADDR_LENGTH << 16);
    NRF_RADIO->BASE0 = NRF_RADIO_DEFAULT_BASEADDR;
    NRF_RADIO->BASE1 = NRF_RADIO_DEFAULT_BASEADDR;
    /* address configuration: prefix configuration */
    // NRF_RADIO->PREFIX0 = NRF_RADIO_DEFAULT_PREFIX;
    // NRF_RADIO->PREFIX1 = NRF_RADIO_DEFAULT_PREFIX;
    /* define TX and RX address */
    NRF_RADIO->TXADDRESS = 0;               /* 1 := BASE0[1] + BASE0[0] + PREFIX0.AP0 */
    NRF_RADIO->RXADDRESSES = (1 << 4);      /* 1 := BASE1[1] + BASE1[0] + PREFIX1.AP4 */

    return 0;
}

int nrf_radio_send(uint8_t addr, char *data, int size)
{
    /* drop data if size is too large */
    if (size > NRF_RADIO_BUFSIZE) {
        DEBUG("radio: data to transmit too large to sent\n");
        return -1;
    }

    /* prepare the data to be send */
    nrf_radio_buf[0] = size;
    memcpy((void*)(&nrf_radio_buf[1]), (void*)data, size);

    /* point radio to the prepared packet */
    NRF_RADIO->PACKETPTR = (uint32_t)nrf_radio_buf;

    /* set the TX address */
    NRF_RADIO->PREFIX0 &= ~(0xff);
    NRF_RADIO->PREFIX0 |= addr;

    /* put radio into transmit mode */
    DEBUG("radio: TXEN\n");
    NRF_RADIO->TASKS_TXEN = 1;
    /* wait until radio is in TXIDLE state */
    while (NRF_RADIO->EVENTS_READY == 0);
    NRF_RADIO->EVENTS_READY = 0;

    /* start packet transmission */
    DEBUG("radio: START\n");
    NRF_RADIO->TASKS_START = 1;
    /* wait until transmission is done */
    while (NRF_RADIO->EVENTS_END == 0);
    NRF_RADIO->EVENTS_END = 0;

    /* return to DISABLED state */
    DEBUG("radio: DISABLE\n");
    NRF_RADIO->TASKS_DISABLE = 1;
    /* wait until radio was disabled */
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;

    DEBUG("radio: TX-DONE\n");
    return size;
}

int nrf_radio_receive(uint8_t addr, char *data, int maxsize)
{
    if (maxsize < NRF_RADIO_BUFSIZE) {
        DEBUG("radio: receive buffer too small\n");
        return -1;
    }

    /* point RADIO device to receiving data buffer */
    NRF_RADIO->PACKETPTR = (uint32_t)data;

    /* set RX address */
    NRF_RADIO->PREFIX1 &= ~(0xff);
    NRF_RADIO->PREFIX1 |= addr;

    /* put radio into receiver mode */
    DEBUG("radio: RXEN\n");
    NRF_RADIO->TASKS_RXEN = 1;
    /* wait until radio is in RXIDLE state */
    while (NRF_RADIO->EVENTS_READY == 0);
    NRF_RADIO->EVENTS_READY = 0;

    /* start actual listening for packets */
    DEBUG("radio: RX\n");
    NRF_RADIO->TASKS_START = 1;
    /* wait until a packet with valid preamble was received */
    while (NRF_RADIO->EVENTS_END == 0);
    NRF_RADIO->EVENTS_END = 0;
    DEBUG("radio: HAPPY HAPPY, a packet was received!\n");

    /* turn of the receiving mode */
    DEBUG("radio: DISABLE\n");
    NRF_RADIO->TASKS_DISABLE = 1;
    /* wait until radio is in DISABLED state again */
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;

    DEBUG("radio: RX-DONE\n");
    return (int)data[0];
}

void nrf_radio_set_mode(nrf_radio_mode_t mode)
{
    NRF_RADIO->MODE = (mode & 0x03);
}

void nrf_radio_set_channel(int channel)
{
    NRF_RADIO->FREQUENCY = (channel & 0x3f);
}

void nrf_radio_set_power(nrf_radio_power_t power)
{
    NRF_RADIO->TXPOWER = (power & 0xff);
}

void nrf_radio_poweron(void)
{
    NRF_RADIO->POWER = 1;
}

void nrf_radio_poweroff(void)
{
    NRF_RADIO->POWER = 0;
}
