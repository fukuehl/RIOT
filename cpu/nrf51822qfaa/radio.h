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
 * @file        radio.c
 * @brief       Low-level timer driver implementation
 *
 * @author      Christian Kuehling <kuehling@zedat.fu-berlin.de>
 *
 * @}
 */

char* receivePacket(void);
void sendPacket(uint8_t addr, char* msg);
void radioConfig(void);
