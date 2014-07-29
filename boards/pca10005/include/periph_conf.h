/*
 * Copyright (C) 2013 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_pca10005
 * @{
 *
 * @file        periph_conf.h
 * @brief       Peripheral MCU configuration for the nRF51822 board pca10005
 *
 * @author      Christian Kühling <kuehling@zedat.fu-berlin.de>
 * @author      Timo Ziegler <timo.ziegler@fu-berlin.de>
 */

#ifndef __PERIPH_CONF_H
#define __PERIPH_CONF_H


/**
 * @brief Timer configuration
 */
#define TIMER_NUMOF         (1U)
#define TIMER_0_EN          1
/* timers 1 and 2 are not supported yet */
#define TIMER_1_EN          0
#define TIMER_2_EN          0
#define TIMER_IRQ_PRIO      1

/* Timer 0 configuration */
#define TIMER_0_DEV         NRF_TIMER0
#define TIMER_0_CHANNELS    3
#define TIMER_0_MAX_VALUE   (0xffffffff)
#define TIMER_0_ISR         isr_timer0
#define TIMER_0_IRQ         TIMER0_IRQn

/* Timer 1 configuration */
#define TIMER_1_DEV         NRF_TIMER1
#define TIMER_1_CHANNELS    3
#define TIMER_1_MAX_VALUE   (0xffffffff)
#define TIMER_1_ISR         isr_timer1
#define TIMER_1_IRQ         TIMER1_IRQn

/* Timer 2 configuration */
#define TIMER_2_DEV         NRF_TIMER2
#define TIMER_2_CHANNELS    3
#define TIMER_2_MAX_VALUE   (0xffffffff)
#define TIMER_2_ISR         isr_timer2
#define TIMER_2_IRQ         TIMER2_IRQn

/**
 * @brief UART configuration
 */
#define UART_NUMOF          (1U)
#define UART_0_EN           1
#define UART_1_EN           0
#define UART_IRQ_PRIO       1
#define UART_CLK            14000000

/* UART 0 device configuration */
#define UART_0_DEV          NRF_UART0
 
/**
 * @brief ADC configuration
 */
#define ADC_NUMOF           (0U)
#define ADC_0_EN            0
#define ADC_1_EN            0

/* ADC 0 configuration */
#define ADC_0_DEV           ADC1                                                        /* TODO */
#define ADC_0_SAMPLE_TIMER
/* ADC 0 channel 0 pin config */
#define ADC_0_C0_PORT
#define ADC_0_C0_PIN
#define ADC_0_C0_CLKEN()
#define ADC_0_C0_AFCFG()
/* ADC 0 channel 1 pin config */
#define ADC_0_C1_PORT
#define ADC_0_C1_PIN
#define ADC_0_C1_CLKEN()
#define ADC_0_C1_AFCFG()
/* ADC 0 channel 2 pin config */
#define ADC_0_C2_PORT
#define ADC_0_C2_PIN
#define ADC_0_C2_CLKEN()
#define ADC_0_C2_AFCFG()
/* ADC 0 channel 3 pin config */
#define ADC_0_C3_PORT
#define ADC_0_C3_PIN
#define ADC_0_C3_CLKEN()
#define ADC_0_C3_AFCFG()

/* ADC 0 configuration */
#define ADC_1_DEV           ADC2                                                        /* TODO */
#define ADC_1_SAMPLE_TIMER
/* ADC 0 channel 0 pin config */
#define ADC_1_C0_PORT
#define ADC_1_C0_PIN
#define ADC_1_C0_CLKEN()
#define ADC_1_C0_AFCFG()
/* ADC 0 channel 1 pin config */
#define ADC_1_C1_PORT
#define ADC_1_C1_PIN
#define ADC_1_C1_CLKEN()
#define ADC_1_C1_AFCFG()
/* ADC 0 channel 2 pin config */
#define ADC_1_C2_PORT
#define ADC_1_C2_PIN
#define ADC_1_C2_CLKEN()
#define ADC_1_C2_AFCFG()
/* ADC 0 channel 3 pin config */
#define ADC_1_C3_PORT
#define ADC_1_C3_PIN
#define ADC_1_C3_CLKEN()
#define ADC_1_C3_AFCFG()


/**
 * @brief PWM configuration
 */
#define PWM_NUMOF           (0U)                                                        /* TODO */
#define PWM_0_EN            0
#define PWM_1_EN            0

/* PWM 0 device configuration */
#define PWM_0_DEV           TIM1
#define PWM_0_CHANNELS      4
/* PWM 0 pin configuration */
#define PWM_0_PORT
#define PWM_0_PINS
#define PWM_0_PORT_CLKEN()
#define PWM_0_CH1_AFCFG()
#define PWM_0_CH2_AFCFG()
#define PWM_0_CH3_AFCFG()
#define PWM_0_CH4_AFCFG()

/* PWM 1 device configuration */
#define PWM_1_DEV           TIM3
#define PWM_1_CHANNELS      4
/* PWM 1 pin configuration */
#define PWM_1_PORT
#define PWM_1_PINS
#define PWM_1_PORT_CLKEN()
#define PWM_1_CH1_AFCFG()
#define PWM_1_CH2_AFCFG()
#define PWM_1_CH3_AFCFG()
#define PWM_1_CH4_AFCFG()


/**
 * @brief SPI configuration
 */
#define SPI_NUMOF           (0U)                                                        /* TODO */
#define SPI_0_EN            0
#define SPI_1_EN            0

/* SPI 0 device config */
#define SPI_0_DEV           SPI1
#define SPI_0_CLKEN()
#define SPI_0_IRQ           SPI1_IRQn
#define SPI_0_IRQ_HANDLER
#define SPI_0_IRQ_PRIO      1
/* SPI 1 pin configuration */
#define SPI_0_PORT
#define SPI_0_PINS          ()
#define SPI_1_PORT_CLKEN()
#define SPI_1_SCK_AFCFG()
#define SPI_1_MISO_AFCFG()
#define SPI_1_MOSI_AFCFG()

/* SPI 1 device config */
#define SPI_1_DEV           SPI2
#define SPI_1_CLKEN()
#define SPI_1_IRQ           SPI2_IRQn
#define SPI_1_IRQ_HANDLER
#define SPI_1_IRQ_PRIO      1
/* SPI 1 pin configuration */
#define SPI_1_PORT
#define SPI_1_PINS          ()
#define SPI_1_PORT_CLKEN()
#define SPI_1_SCK_AFCFG()
#define SPI_1_MISO_AFCFG()
#define SPI_1_MOSI_AFCFG()


/**
 * @brief I2C configuration
 */
#define I2C_NUMOF           (0U)                                                        /* TODO */
#define I2C_0_EN            0
#define I2C_0_EN            0

/* SPI 0 device configuration */
#define I2C_0_DEV           I2C1
#define I2C_0_CLKEN()
#define I2C_0_ISR           isr_i2c1
#define I2C_0_IRQ           I2C1_IRQn
#define I2C_0_IRQ_PRIO      1
/* SPI 0 pin configuration */
#define I2C_0_PORT          GPIOB
#define I2C_0_PINS          (GPIO_Pin_6 | GPIO_Pin_7)
#define I2C_0_PORT_CLKEN()
#define I2C_0_SCL_AFCFG()
#define I2C_0_SDA_AFCFG()

/* SPI 1 device configuration */
#define I2C_1_DEV           I2C2
#define I2C_1_CLKEN()
#define I2C_1_ISR           isr_i2c2
#define I2C_1_IRQ           I2C2_IRQn
#define I2C_1_IRQ_PRIO      1
/* SPI 1 pin configuration */
#define I2C_1_PORT          GPIOF
#define I2C_1_PINS          (GPIO_Pin_0 | GPIO_Pin_1)
#define I2C_1_PORT_CLKEN()
#define I2C_1_SCL_AFCFG()
#define I2C_1_SDA_AFCFG()


/**
 * @brief GPIO configuration
 */

#define GPIO_DEV            NRF_GPIO
#define GPIO_NUMOF          16
#define GPIO_0_EN           1
#define GPIO_1_EN           1
#define GPIO_2_EN           1
#define GPIO_3_EN           1
#define GPIO_4_EN           1
#define GPIO_5_EN           1
#define GPIO_6_EN           1
#define GPIO_7_EN           1
#define GPIO_8_EN           1
#define GPIO_9_EN           1
#define GPIO_10_EN          1
#define GPIO_11_EN          1
#define GPIO_12_EN          1
#define GPIO_13_EN          1
#define GPIO_14_EN          1
#define GPIO_15_EN          1
#define GPIO_IRQ_PRIO       1

/* IRQ config */
#define GPIO_IRQ_DEV        NRF_GPIOTE
#define GPIO_IRQ_0          GPIO_0
#define GPIO_IRQ_1          GPIO_1
#define GPIO_IRQ_2          GPIO_0                  /* not configured */
#define GPIO_IRQ_3          GPIO_0                  /* not configured */
#define GPIO_IRQ_4          GPIO_2
#define GPIO_IRQ_5          GPIO_3
#define GPIO_IRQ_6          GPIO_4
#define GPIO_IRQ_7          GPIO_5
#define GPIO_IRQ_8          GPIO_0                  /* not configured */
#define GPIO_IRQ_9          GPIO_0                  /* not configured */
#define GPIO_IRQ_10         GPIO_6
#define GPIO_IRQ_11         GPIO_7
#define GPIO_IRQ_12         GPIO_8
#define GPIO_IRQ_13         GPIO_9
#define GPIO_IRQ_14         GPIO_10
#define GPIO_IRQ_15         GPIO_11

/* GPIO channel 0 config */
#define GPIO_0_PIN          0
#define GPIO_0_IRQ          EXTI0_0_IRQn
/* GPIO channel 1 config */
#define GPIO_1_PIN          1
#define GPIO_1_IRQ          EXTI0_1_IRQn
/* GPIO channel 2 config */
#define GPIO_2_PIN          2
#define GPIO_2_IRQ          EXTI0_2_IRQn
/* GPIO channel 3 config */
#define GPIO_3_PIN          3
#define GPIO_3_IRQ          EXTI0_3_IRQn
/* GPIO channel 4 config */
#define GPIO_4_PIN          4
#define GPIO_4_IRQ          EXTI0_4_IRQn
/* GPIO channel 5 config */
#define GPIO_5_PIN          5
#define GPIO_5_IRQ          EXTI0_5_IRQn
/* GPIO channel 6 config */
#define GPIO_6_PIN          6
#define GPIO_6_IRQ          EXTI0_6_IRQn
/* GPIO channel 7 config */
#define GPIO_7_PIN          7
#define GPIO_7_IRQ          EXTI0_7_IRQn
/* GPIO channel 8 config */
#define GPIO_8_PIN          8
#define GPIO_8_IRQ          EXTI0_8_IRQn
/* GPIO channel 9 config */
#define GPIO_9_PIN          9
#define GPIO_9_IRQ          EXTI0_9_IRQn
/* GPIO channel 10 config */
#define GPIO_10_PIN         10
#define GPIO_10_IRQ         EXTI0_10_IRQn
/* GPIO channel 11 config */
#define GPIO_11_PIN         11
#define GPIO_11_IRQ         EXTI0_11_IRQn
/* GPIO channel 12 config */
#define GPIO_12_PIN         12
#define GPIO_12_IRQ         EXTI0_12_IRQn
/* GPIO channel 13 config */
#define GPIO_13_PIN         13
#define GPIO_13_IRQ         EXTI0_13_IRQn
/* GPIO channel 14 config */
#define GPIO_14_PIN         14
#define GPIO_14_IRQ         EXTI0_14_IRQn
/* GPIO channel 11 config */
#define GPIO_15_PIN         15
#define GPIO_15_IRQ         EXTI0_15_IRQn


#define RADIO_CONF          NRF_RADIO
#endif /* __PERIPH_CONF_H */