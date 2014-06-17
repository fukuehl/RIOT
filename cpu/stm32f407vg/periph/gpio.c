/*
 * Copyright (C) 2014 Hauke Petersen <mail@haukepetersen.de>
 *
 * This file is subject to the terms and conditions of the GPLv2 License. 
 * See the file LICENSE in the top level directory for more details.
 */

/**
 * @ingroup     driver_periph
 * @{
 * 
 * @file        gpio.c
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Hauke Petersen <mail@haukepetersen.de>
 * 
 * @}
 */

#include "cpu.h"
#include "periph/gpio.h"
#include "periph_conf.h"

typedef struct {
    void (*cb)(void);
} gpio_state_t;

static inline void irq_handler(gpio_t dev);

static gpio_state_t config[GPIO_NUMOF];


int gpio_init_out(gpio_t dev, gpio_pp_t pushpull)
{
    GPIO_TypeDef *port;
    uint32_t pin;

    switch (dev) {
#if GPIO_0_EN
        case GPIO_0:
            GPIO_0_CLKEN();
            port = GPIO_0_PORT;
            pin = GPIO_0_PIN;
            break;
#endif
#if GPIO_1_EN
        case GPIO_1:
            GPIO_1_CLKEN();
            port = GPIO_1_PORT;
            pin = GPIO_1_PIN;
            break;
#endif
#if GPIO_2_EN
        case GPIO_2:
            GPIO_2_CLKEN();
            port = GPIO_2_PORT;
            pin = GPIO_2_PIN;
            break;
#endif
#if GPIO_3_EN
        case GPIO_3:
            GPIO_3_CLKEN();
            port = GPIO_3_PORT;
            pin = GPIO_3_PIN;
            break;
#endif
#if GPIO_4_EN
        case GPIO_4:
            GPIO_4_CLKEN();
            port = GPIO_4_PORT;
            pin = GPIO_4_PIN;
            break;
#endif
#if GPIO_5_EN
        case GPIO_5:
            GPIO_5_CLKEN();
            port = GPIO_5_PORT;
            pin = GPIO_5_PIN;
            break;
#endif
#if GPIO_6_EN
        case GPIO_6:
            GPIO_6_CLKEN();
            port = GPIO_6_PORT;
            pin = GPIO_6_PIN;
            break;
#endif
#if GPIO_7_EN
        case GPIO_7:
            GPIO_7_CLKEN();
            port = GPIO_7_PORT;
            pin = GPIO_7_PIN;
            break;
#endif
#if GPIO_8_EN
        case GPIO_8:
            GPIO_8_CLKEN();
            port = GPIO_8_PORT;
            pin = GPIO_8_PIN;
            break;
#endif
#if GPIO_9_EN
        case GPIO_9:
            GPIO_9_CLKEN();
            port = GPIO_9_PORT;
            pin = GPIO_9_PIN;
            break;
#endif
#if GPIO_10_EN
        case GPIO_10:
            GPIO_10_CLKEN();
            port = GPIO_10_PORT;
            pin = GPIO_10_PIN;
            break;
#endif
#if GPIO_11_EN
        case GPIO_11:
            GPIO_11_CLKEN();
            port = GPIO_11_PORT;
            pin = GPIO_11_PIN;
            break;
#endif
        case GPIO_UNDEFINED:
        default:
            return -1;
    }

    port->MODER &= ~(2 << (2 * pin));           /* set pin to output mode */
    port->MODER |= (1 << (2 * pin));
    port->OTYPER &= ~(1 << pin);                /* set to push-pull configuration */
    port->OSPEEDR |= (3 << (2 * pin));          /* set to high speed */
    port->PUPDR &= ~(3 << (2 * pin));           /* configure push-pull resistors */
    port->PUPDR |= (pushpull << (2 * pin));
    port->ODR &= ~(1 << pin);                   /* set pin to low signal */

    return 0; /* all OK */
}

int gpio_init_in(gpio_t dev, gpio_pp_t pushpull)
{
    GPIO_TypeDef *port;
    uint32_t pin;

    switch (dev) {
#if GPIO_0_EN
        case GPIO_0:
            GPIO_0_CLKEN();
            port = GPIO_0_PORT;
            pin = GPIO_0_PIN;
            break;
#endif
#if GPIO_1_EN
        case GPIO_1:
            GPIO_1_CLKEN();
            port = GPIO_1_PORT;
            pin = GPIO_1_PIN;
            break;
#endif
#if GPIO_2_EN
        case GPIO_2:
            GPIO_2_CLKEN();
            port = GPIO_2_PORT;
            pin = GPIO_2_PIN;
            break;
#endif
#if GPIO_3_EN
        case GPIO_3:
            GPIO_3_CLKEN();
            port = GPIO_3_PORT;
            pin = GPIO_3_PIN;
            break;
#endif
#if GPIO_4_EN
        case GPIO_4:
            GPIO_4_CLKEN();
            port = GPIO_4_PORT;
            pin = GPIO_4_PIN;
            break;
#endif
#if GPIO_5_EN
        case GPIO_5:
            GPIO_5_CLKEN();
            port = GPIO_5_PORT;
            pin = GPIO_5_PIN;
            break;
#endif
#if GPIO_6_EN
        case GPIO_6:
            GPIO_6_CLKEN();
            port = GPIO_6_PORT;
            pin = GPIO_6_PIN;
            break;
#endif
#if GPIO_7_EN
        case GPIO_7:
            GPIO_7_CLKEN();
            port = GPIO_7_PORT;
            pin = GPIO_7_PIN;
            break;
#endif
#if GPIO_8_EN
        case GPIO_8:
            GPIO_8_CLKEN();
            port = GPIO_8_PORT;
            pin = GPIO_8_PIN;
            break;
#endif
#if GPIO_9_EN
        case GPIO_9:
            GPIO_9_CLKEN();
            port = GPIO_9_PORT;
            pin = GPIO_9_PIN;
            break;
#endif
#if GPIO_10_EN
        case GPIO_10:
            GPIO_10_CLKEN();
            port = GPIO_10_PORT;
            pin = GPIO_10_PIN;
            break;
#endif
#if GPIO_11_EN
        case GPIO_11:
            GPIO_11_CLKEN();
            port = GPIO_11_PORT;
            pin = GPIO_11_PIN;
            break;
#endif
        case GPIO_UNDEFINED:
        default:
            return -1;
    }

    port->MODER &= ~(3 << (2 * pin));           /* configure pin as input */
    port->PUPDR &= ~(3 << (2 * pin));           /* configure push-pull resistors */
    port->PUPDR |= (pushpull << (2 * pin));

    return 0; /* everything alright here */
}

int gpio_init_int(gpio_t dev, gpio_pp_t pushpull, gpio_flank_t flank, void (*cb)(void))
{
    uint32_t pin;

    int res = gpio_init_in(dev, pushpull);
    if (res < 0) {
        return res;
    }

    switch (dev) {
#if GPIO_0_EN
        case GPIO_0:
            pin = GPIO_0_PIN;
            GPIO_0_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_0_PIN);
            break;
#endif
#if GPIO_1_EN
        case GPIO_1:
            pin = GPIO_1_PIN;
            GPIO_1_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_1_PIN);
            break;
#endif
#if GPIO_2_EN
        case GPIO_2:
            pin = GPIO_2_PIN;
            GPIO_2_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_2_PIN);
            break;
#endif
#if GPIO_3_EN
        case GPIO_3:
            pin = GPIO_3_PIN;
            GPIO_3_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_3_PIN);
            break;
#endif
#if GPIO_4_EN
        case GPIO_4:
            pin = GPIO_4_PIN;
            GPIO_4_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_4_PIN);
            break;
#endif
#if GPIO_5_EN
        case GPIO_5:
            pin = GPIO_5_PIN;
            GPIO_5_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_5_PIN);
            break;
#endif
#if GPIO_6_EN
        case GPIO_6:
            pin = GPIO_6_PIN;
            GPIO_6_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_6_PIN);
            break;
#endif
#if GPIO_7_EN
        case GPIO_7:
            pin = GPIO_7_PIN;
            GPIO_7_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_7_PIN);
            break;
#endif
#if GPIO_8_EN
        case GPIO_8:
            pin = GPIO_8_PIN;
            GPIO_8_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_8_PIN);
            break;
#endif
#if GPIO_9_EN
        case GPIO_9:
            pin = GPIO_9_PIN;
            GPIO_9_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_9_PIN);
            break;
#endif
#if GPIO_10_EN
        case GPIO_10:
            pin = GPIO_10_PIN;
            GPIO_10_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_10_PIN);
            break;
#endif
#if GPIO_11_EN
        case GPIO_11:
            pin = GPIO_11_PIN;
            GPIO_11_EXTI_MAP();
            EXTI->IMR |= (1 << GPIO_11_PIN);
            break;
#endif
        case GPIO_UNDEFINED:
        default:
            return -1;
    }

    /* set callback */
    config[dev].cb = cb;

    /* set GPIO interrupt priority */
    NVIC_SetPriority(EXTI0_IRQn, GPIO_IRQ_PRIO);
    NVIC_SetPriority(EXTI1_IRQn, GPIO_IRQ_PRIO);
    NVIC_SetPriority(EXTI2_IRQn, GPIO_IRQ_PRIO);
    NVIC_SetPriority(EXTI3_IRQn, GPIO_IRQ_PRIO);
    NVIC_SetPriority(EXTI4_IRQn, GPIO_IRQ_PRIO);
    NVIC_SetPriority(EXTI9_5_IRQn, GPIO_IRQ_PRIO);
    NVIC_SetPriority(EXTI15_10_IRQn, GPIO_IRQ_PRIO);

    /* configure the active edges */
    switch (flank) {
        case GPIO_RISING:
            EXTI->RTSR |= (1 << pin);
            break;
        case GPIO_FALLING:
            EXTI->FTSR |= (1 << pin);
            break;
        case GPIO_BOTH:
            EXTI->RTSR |= (1 << pin);
            EXTI->FTSR |= (1 << pin);
            break;
    }

    return 0;
}

int gpio_read(gpio_t dev)
{
    GPIO_TypeDef *port;
    uint32_t pin;

    switch (dev) {
#if GPIO_0_EN
        case GPIO_0:
            port = GPIO_0_PORT;
            pin = GPIO_0_PIN;
            break;
#endif
#if GPIO_1_EN
        case GPIO_1:
            port = GPIO_1_PORT;
            pin = GPIO_1_PIN;
            break;
#endif
#if GPIO_2_EN
        case GPIO_2:
            port = GPIO_2_PORT;
            pin = GPIO_2_PIN;
            break;
#endif
#if GPIO_3_EN
        case GPIO_3:
            port = GPIO_3_PORT;
            pin = GPIO_3_PIN;
            break;
#endif
#if GPIO_4_EN
        case GPIO_4:
            port = GPIO_4_PORT;
            pin = GPIO_4_PIN;
            break;
#endif
#if GPIO_5_EN
        case GPIO_5:
            port = GPIO_5_PORT;
            pin = GPIO_5_PIN;
            break;
#endif
#if GPIO_6_EN
        case GPIO_6:
            port = GPIO_6_PORT;
            pin = GPIO_6_PIN;
            break;
#endif
#if GPIO_7_EN
        case GPIO_7:
            port = GPIO_7_PORT;
            pin = GPIO_7_PIN;
            break;
#endif
#if GPIO_8_EN
        case GPIO_8:
            port = GPIO_8_PORT;
            pin = GPIO_8_PIN;
            break;
#endif
#if GPIO_9_EN
        case GPIO_9:
            port = GPIO_9_PORT;
            pin = GPIO_9_PIN;
            break;
#endif
#if GPIO_10_EN
        case GPIO_10:
            port = GPIO_10_PORT;
            pin = GPIO_10_PIN;
            break;
#endif
#if GPIO_11_EN
        case GPIO_11:
            port = GPIO_11_PORT;
            pin = GPIO_11_PIN;
            break;
#endif
        case GPIO_UNDEFINED:
        default:
            return -1;
    }

    if (port->MODER & (3 << (pin * 2))) {       /* if configured as output */
        return port->ODR & (1 << pin);          /* read output data register */
    } else {
        return port->IDR & (1 << pin);          /* else read input data register */
    }
}

int gpio_set(gpio_t dev)
{
    switch (dev) {
#if GPIO_0_EN
        case GPIO_0:
            GPIO_0_PORT->ODR |= (1 << GPIO_0_PIN);
            break;
#endif
#if GPIO_1_EN
        case GPIO_1:
            GPIO_1_PORT->ODR |= (1 << GPIO_1_PIN);
            break;
#endif
#if GPIO_2_EN
        case GPIO_2:
            GPIO_2_PORT->ODR |= (1 << GPIO_2_PIN);
            break;
#endif
#if GPIO_3_EN
        case GPIO_3:
            GPIO_3_PORT->ODR |= (1 << GPIO_3_PIN);
            break;
#endif
#if GPIO_4_EN
        case GPIO_4:
            GPIO_4_PORT->ODR |= (1 << GPIO_4_PIN);
            break;
#endif
#if GPIO_5_EN
        case GPIO_5:
            GPIO_5_PORT->ODR |= (1 << GPIO_5_PIN);
            break;
#endif
#if GPIO_6_EN
        case GPIO_6:
            GPIO_6_PORT->ODR |= (1 << GPIO_6_PIN);
            break;
#endif
#if GPIO_7_EN
        case GPIO_7:
            GPIO_7_PORT->ODR |= (1 << GPIO_7_PIN);
            break;
#endif
#if GPIO_8_EN
        case GPIO_8:
            GPIO_8_PORT->ODR |= (1 << GPIO_8_PIN);
            break;
#endif
#if GPIO_9_EN
        case GPIO_9:
            GPIO_9_PORT->ODR |= (1 << GPIO_9_PIN);
            break;
#endif
#if GPIO_10_EN
        case GPIO_10:
            GPIO_10_PORT->ODR |= (1 << GPIO_10_PIN);
            break;
#endif
#if GPIO_11_EN
        case GPIO_11:
            GPIO_11_PORT->ODR |= (1 << GPIO_11_PIN);
#endif
            break;
        case GPIO_UNDEFINED:
        default:
            return -1;
    }
    return 0;
}

int gpio_clear(gpio_t dev)
{
    switch (dev) {
#if GPIO_0_EN
        case GPIO_0:
            GPIO_0_PORT->ODR &= ~(1 << GPIO_0_PIN);
            break;
#endif
#if GPIO_1_EN
        case GPIO_1:
            GPIO_1_PORT->ODR &= ~(1 << GPIO_1_PIN);
            break;
#endif
#if GPIO_2_EN
        case GPIO_2:
            GPIO_2_PORT->ODR &= ~(1 << GPIO_2_PIN);
            break;
#endif
#if GPIO_3_EN
        case GPIO_3:
            GPIO_3_PORT->ODR &= ~(1 << GPIO_3_PIN);
            break;
#endif
#if GPIO_4_EN
        case GPIO_4:
            GPIO_4_PORT->ODR &= ~(1 << GPIO_4_PIN);
            break;
#endif
#if GPIO_5_EN
        case GPIO_5:
            GPIO_5_PORT->ODR &= ~(1 << GPIO_5_PIN);
            break;
#endif
#if GPIO_6_EN
        case GPIO_6:
            GPIO_6_PORT->ODR &= ~(1 << GPIO_6_PIN);
            break;
#endif
#if GPIO_7_EN
        case GPIO_7:
            GPIO_7_PORT->ODR &= ~(1 << GPIO_7_PIN);
            break;
#endif
#if GPIO_8_EN
        case GPIO_8:
            GPIO_8_PORT->ODR &= ~(1 << GPIO_8_PIN);
            break;
#endif
#if GPIO_9_EN
        case GPIO_9:
            GPIO_9_PORT->ODR &= ~(1 << GPIO_9_PIN);
            break;
#endif
#if GPIO_10_EN
        case GPIO_10:
            GPIO_10_PORT->ODR &= ~(1 << GPIO_10_PIN);
            break;
#endif
#if GPIO_11_EN
        case GPIO_11:
            GPIO_11_PORT->ODR &= ~(1 << GPIO_11_PIN);
            break;
#endif
        case GPIO_UNDEFINED:
        default:
            return -1;
    }
    return 0;
}

int gpio_toggle(gpio_t dev)
{
    if (gpio_read(dev)) {
        return gpio_clear(dev);
    } else {
        return gpio_set(dev);
    }
}

int gpio_write(gpio_t dev, int value)
{
    if (value) {
        return gpio_set(dev);
    } else {
        return gpio_clear(dev);
    }
}


static inline void irq_handler(gpio_t dev) 
{
    config[dev].cb();
}

__attribute__((naked))
void isr_exti0(void)
{
    ISR_ENTER();
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_0);
    }
    ISR_EXIT();
}

__attribute__((naked))
void isr_exti1(void)
{
    ISR_ENTER();
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR |= EXTI_PR_PR1;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_1);
    }
    ISR_EXIT();
}

__attribute__((naked))
void isr_exti2(void)
{
    ISR_ENTER();
    if (EXTI->PR & EXTI_PR_PR2) {
        EXTI->PR |= EXTI_PR_PR2;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_2);
    }
    ISR_EXIT();
}

__attribute__((naked))
void isr_exti3(void)
{
    ISR_ENTER();
    if (EXTI->PR & EXTI_PR_PR3) {
        EXTI->PR |= EXTI_PR_PR3;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_3);
    }
    ISR_EXIT();
}

__attribute__((naked))
void isr_exti4(void)
{
    ISR_ENTER();
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR |= EXTI_PR_PR4;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_4);
    }
    ISR_EXIT();
}

__attribute__((naked))
void isr_exti9_5(void)
{
    ISR_ENTER();
    if (EXTI->PR & EXTI_PR_PR5) {
        EXTI->PR |= EXTI_PR_PR5;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_5);
    }
    else if (EXTI->PR & EXTI_PR_PR6) {
        EXTI->PR |= EXTI_PR_PR6;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_6);
    }
    else if (EXTI->PR & EXTI_PR_PR7) {
        EXTI->PR |= EXTI_PR_PR7;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_7);
    }
    else if (EXTI->PR & EXTI_PR_PR8) {
        EXTI->PR |= EXTI_PR_PR8;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_8);
    }
    else if (EXTI->PR & EXTI_PR_PR9) {
        EXTI->PR |= EXTI_PR_PR9;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_9);
    }
    ISR_EXIT();
}

__attribute__((naked))
void isr_exti15_10(void)
{
    ISR_ENTER();
    if (EXTI->PR & EXTI_PR_PR10) {
        EXTI->PR |= EXTI_PR_PR10;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_10);
    }
    else if (EXTI->PR & EXTI_PR_PR11) {
        EXTI->PR |= EXTI_PR_PR11;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_11);
    }
    else if (EXTI->PR & EXTI_PR_PR12) {
        EXTI->PR |= EXTI_PR_PR12;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_12);
    }
    else if (EXTI->PR & EXTI_PR_PR13) {
        EXTI->PR |= EXTI_PR_PR13;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_13);
    }
    else if (EXTI->PR & EXTI_PR_PR14) {
        EXTI->PR |= EXTI_PR_PR14;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_14);
    }
    else if (EXTI->PR & EXTI_PR_PR15) {
        EXTI->PR |= EXTI_PR_PR15;        /* clear status bit by writing a 1 to it */
        irq_handler(GPIO_IRQ_15);
    }
    ISR_EXIT();
}
