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
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include "board.h"


int main(void)
{
    while(1){
        // send SOS via morse :):
    	// send S - that is 3 shors:
		LED_BLUE_ON;
		delay(1000000);
		LED_BLUE_OFF;
		delay(1000000);
		LED_BLUE_ON;
		delay(1000000);
		LED_BLUE_OFF;
		delay(2000000); // wait for 2 sec before new lettevr
		// send O - that is 3 long:
		LED_BLUE_ON;
		delay(3000000);
		LED_BLUE_OFF;
		delay(1000000);
		LED_BLUE_ON;
		delay(3000000);
		LED_BLUE_OFF;
		delay(1000000);
		LED_BLUE_ON;
		delay(3000000);
		LED_BLUE_OFF;
		delay(2000000); // wait for 2 sec before new lettevr
		// send S - that is 3 shors:
		LED_BLUE_ON;
		delay(1000000);
		LED_BLUE_OFF;
		delay(1000000);
		LED_BLUE_ON;
		delay(1000000);
		LED_BLUE_OFF;
		delay(1000000);
		LED_BLUE_ON;
		delay(1000000);
		LED_BLUE_OFF;
		delay(2000000); // wait for 2 sec before new lettevr
		// wait 5 more seconds to put new SOS
		delay(5000000);
   }
}
