/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#pragma once

/*
    ChibiOS - Copyright (C) 2006..2020 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */


/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for MicroCanV3
 */

/*
 * Board identifier.
 */
#define BOARD_ENAC_MicroCanV3
#define BOARD_NAME                  "Enac MicroCanV3" 

/*
 * Board oscillators-related settings.
 */

#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#define STM32_LSE_BYPASS            TRUE
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#define STM32_HSE_BYPASS            TRUE
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32G491xx

/*
 * IO pins assignments.
 */
/*
 * IO pins assignments.
 */
#define	PA00_PS_5V                     0U
#define	PA01_ADSEL                     1U
#define	PA02_DBG_TX                    2U
#define	PA03_DBG_RX                    3U
#define	PA04_F1_a                      4U
#define	PA05_M95P_SCK                  5U
#define	PA06_M95P_MISO                 6U
#define	PA07_M95P_MOSI                 7U
#define	PA08_F1_b                      8U
#define	PA09_F2                        9U
#define	PA10_F3                        10U
#define	PA11_F4                        11U
#define	PA12_F0_a                      12U
#define	PA13_SWDIO                     13U
#define	PA14_SWCLK                     14U
#define	PA15_PULLUP_SCL                15U

#define	PB00_M95P_CS                   0U
#define	PB01                           1U
#define	PB02                           2U
#define	PB03_PULLUP_SDA                3U
#define	PB04_CAN_TERMR_EN              4U
#define	PB05_FDCAN_RX                  5U
#define	PB06_FDCAN_TX                  6U
#define	PB07_F0_b                      7U
#define	PB08_RGBLED                    8U
#define	PB09                           9U
#define	PB10                           10U
#define	PB11                           11U
#define	PB12                           12U
#define	PB13                           13U
#define	PB14                           14U
#define	PB15                           15U

#define	PC00                           0U
#define	PC01                           1U
#define	PC02                           2U
#define	PC03                           3U
#define	PC04                           4U
#define	PC05                           5U
#define	PC06                           6U
#define	PC07                           7U
#define	PC08                           8U
#define	PC09                           9U
#define	PC10                           10U
#define	PC11                           11U
#define	PC12                           12U
#define	PC13                           13U
#define	PC14                           14U
#define	PC15                           15U

#define	PD00                           0U
#define	PD01                           1U
#define	PD02                           2U
#define	PD03                           3U
#define	PD04                           4U
#define	PD05                           5U
#define	PD06                           6U
#define	PD07                           7U
#define	PD08                           8U
#define	PD09                           9U
#define	PD10                           10U
#define	PD11                           11U
#define	PD12                           12U
#define	PD13                           13U
#define	PD14                           14U
#define	PD15                           15U

#define	PE00                           0U
#define	PE01                           1U
#define	PE02                           2U
#define	PE03                           3U
#define	PE04                           4U
#define	PE05                           5U
#define	PE06                           6U
#define	PE07                           7U
#define	PE08                           8U
#define	PE09                           9U
#define	PE10                           10U
#define	PE11                           11U
#define	PE12                           12U
#define	PE13                           13U
#define	PE14                           14U
#define	PE15                           15U

#define	PF00_OSC_IN                    0U
#define	PF01_OSC_OUT                   1U
#define	PF02                           2U
#define	PF03                           3U
#define	PF04                           4U
#define	PF05                           5U
#define	PF06                           6U
#define	PF07                           7U
#define	PF08                           8U
#define	PF09                           9U
#define	PF10                           10U
#define	PF11                           11U
#define	PF12                           12U
#define	PF13                           13U
#define	PF14                           14U
#define	PF15                           15U

#define	PG00                           0U
#define	PG01                           1U
#define	PG02                           2U
#define	PG03                           3U
#define	PG04                           4U
#define	PG05                           5U
#define	PG06                           6U
#define	PG07                           7U
#define	PG08                           8U
#define	PG09                           9U
#define	PG10                           10U
#define	PG11                           11U
#define	PG12                           12U
#define	PG13                           13U
#define	PG14                           14U
#define	PG15                           15U

/*
 * IO lines assignments.
 */
#define	LINE_PS_5V                     PAL_LINE(GPIOA, 0U)
#define	LINE_ADSEL                     PAL_LINE(GPIOA, 1U)
#define	LINE_DBG_TX                    PAL_LINE(GPIOA, 2U)
#define	LINE_DBG_RX                    PAL_LINE(GPIOA, 3U)
#define	LINE_F1_a                      PAL_LINE(GPIOA, 4U)
#define	LINE_M95P_SCK                  PAL_LINE(GPIOA, 5U)
#define	LINE_M95P_MISO                 PAL_LINE(GPIOA, 6U)
#define	LINE_M95P_MOSI                 PAL_LINE(GPIOA, 7U)
#define	LINE_F1_b                      PAL_LINE(GPIOA, 8U)
#define	LINE_F2                        PAL_LINE(GPIOA, 9U)
#define	LINE_F3                        PAL_LINE(GPIOA, 10U)
#define	LINE_F4                        PAL_LINE(GPIOA, 11U)
#define	LINE_F0_a                      PAL_LINE(GPIOA, 12U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)
#define	LINE_PULLUP_SCL                PAL_LINE(GPIOA, 15U)

#define	LINE_M95P_CS                   PAL_LINE(GPIOB, 0U)
#define	LINE_PULLUP_SDA                PAL_LINE(GPIOB, 3U)
#define	LINE_CAN_TERMR_EN              PAL_LINE(GPIOB, 4U)
#define	LINE_FDCAN_RX                  PAL_LINE(GPIOB, 5U)
#define	LINE_FDCAN_TX                  PAL_LINE(GPIOB, 6U)
#define	LINE_F0_b                      PAL_LINE(GPIOB, 7U)
#define	LINE_RGBLED                    PAL_LINE(GPIOB, 8U)

#define	LINE_OSC_IN                    PAL_LINE(GPIOF, 0U)
#define	LINE_OSC_OUT                   PAL_LINE(GPIOF, 1U)


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LEVEL_LOW(n)        (0U << (n))
#define PIN_ODR_LEVEL_HIGH(n)       (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_SPEED_VERYLOW(n) (0U << ((n) * 2U))
#define PIN_OSPEED_SPEED_LOW(n)     (1U << ((n) * 2U))
#define PIN_OSPEED_SPEED_MEDIUM(n)  (2U << ((n) * 2U))
#define PIN_OSPEED_SPEED_HIGH(n)    (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

#define VAL_GPIOA_MODER                 (PIN_MODE_ANALOG(PA00_PS_5V) | \
					 PIN_MODE_ANALOG(PA01_ADSEL) | \
					 PIN_MODE_ALTERNATE(PA02_DBG_TX) | \
					 PIN_MODE_ALTERNATE(PA03_DBG_RX) | \
					 PIN_MODE_INPUT(PA04_F1_a) | \
					 PIN_MODE_ALTERNATE(PA05_M95P_SCK) | \
					 PIN_MODE_ALTERNATE(PA06_M95P_MISO) | \
					 PIN_MODE_ALTERNATE(PA07_M95P_MOSI) | \
					 PIN_MODE_INPUT(PA08_F1_b) | \
					 PIN_MODE_INPUT(PA09_F2) | \
					 PIN_MODE_INPUT(PA10_F3) | \
					 PIN_MODE_INPUT(PA11_F4) | \
					 PIN_MODE_INPUT(PA12_F0_a) | \
					 PIN_MODE_ALTERNATE(PA13_SWDIO) | \
					 PIN_MODE_ALTERNATE(PA14_SWCLK) | \
					 PIN_MODE_OUTPUT(PA15_PULLUP_SCL))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(PA00_PS_5V) | \
					 PIN_OTYPE_PUSHPULL(PA01_ADSEL) | \
					 PIN_OTYPE_PUSHPULL(PA02_DBG_TX) | \
					 PIN_OTYPE_PUSHPULL(PA03_DBG_RX) | \
					 PIN_OTYPE_OPENDRAIN(PA04_F1_a) | \
					 PIN_OTYPE_PUSHPULL(PA05_M95P_SCK) | \
					 PIN_OTYPE_PUSHPULL(PA06_M95P_MISO) | \
					 PIN_OTYPE_PUSHPULL(PA07_M95P_MOSI) | \
					 PIN_OTYPE_OPENDRAIN(PA08_F1_b) | \
					 PIN_OTYPE_OPENDRAIN(PA09_F2) | \
					 PIN_OTYPE_OPENDRAIN(PA10_F3) | \
					 PIN_OTYPE_OPENDRAIN(PA11_F4) | \
					 PIN_OTYPE_OPENDRAIN(PA12_F0_a) | \
					 PIN_OTYPE_PUSHPULL(PA13_SWDIO) | \
					 PIN_OTYPE_PUSHPULL(PA14_SWCLK) | \
					 PIN_OTYPE_PUSHPULL(PA15_PULLUP_SCL))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PA00_PS_5V) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA01_ADSEL) | \
					 PIN_OSPEED_SPEED_HIGH(PA02_DBG_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PA03_DBG_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA04_F1_a) | \
					 PIN_OSPEED_SPEED_HIGH(PA05_M95P_SCK) | \
					 PIN_OSPEED_SPEED_HIGH(PA06_M95P_MISO) | \
					 PIN_OSPEED_SPEED_HIGH(PA07_M95P_MOSI) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA08_F1_b) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA09_F2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA10_F3) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA11_F4) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA12_F0_a) | \
					 PIN_OSPEED_SPEED_HIGH(PA13_SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(PA14_SWCLK) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA15_PULLUP_SCL))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(PA00_PS_5V) | \
					 PIN_PUPDR_FLOATING(PA01_ADSEL) | \
					 PIN_PUPDR_FLOATING(PA02_DBG_TX) | \
					 PIN_PUPDR_FLOATING(PA03_DBG_RX) | \
					 PIN_PUPDR_FLOATING(PA04_F1_a) | \
					 PIN_PUPDR_FLOATING(PA05_M95P_SCK) | \
					 PIN_PUPDR_FLOATING(PA06_M95P_MISO) | \
					 PIN_PUPDR_FLOATING(PA07_M95P_MOSI) | \
					 PIN_PUPDR_FLOATING(PA08_F1_b) | \
					 PIN_PUPDR_FLOATING(PA09_F2) | \
					 PIN_PUPDR_FLOATING(PA10_F3) | \
					 PIN_PUPDR_FLOATING(PA11_F4) | \
					 PIN_PUPDR_FLOATING(PA12_F0_a) | \
					 PIN_PUPDR_FLOATING(PA13_SWDIO) | \
					 PIN_PUPDR_FLOATING(PA14_SWCLK) | \
					 PIN_PUPDR_FLOATING(PA15_PULLUP_SCL))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_LOW(PA00_PS_5V) | \
					 PIN_ODR_LEVEL_LOW(PA01_ADSEL) | \
					 PIN_ODR_LEVEL_HIGH(PA02_DBG_TX) | \
					 PIN_ODR_LEVEL_HIGH(PA03_DBG_RX) | \
					 PIN_ODR_LEVEL_LOW(PA04_F1_a) | \
					 PIN_ODR_LEVEL_HIGH(PA05_M95P_SCK) | \
					 PIN_ODR_LEVEL_HIGH(PA06_M95P_MISO) | \
					 PIN_ODR_LEVEL_HIGH(PA07_M95P_MOSI) | \
					 PIN_ODR_LEVEL_LOW(PA08_F1_b) | \
					 PIN_ODR_LEVEL_LOW(PA09_F2) | \
					 PIN_ODR_LEVEL_LOW(PA10_F3) | \
					 PIN_ODR_LEVEL_LOW(PA11_F4) | \
					 PIN_ODR_LEVEL_LOW(PA12_F0_a) | \
					 PIN_ODR_LEVEL_HIGH(PA13_SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(PA14_SWCLK) | \
					 PIN_ODR_LEVEL_LOW(PA15_PULLUP_SCL))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(PA00_PS_5V, 0) | \
					 PIN_AFIO_AF(PA01_ADSEL, 0) | \
					 PIN_AFIO_AF(PA02_DBG_TX, 12) | \
					 PIN_AFIO_AF(PA03_DBG_RX, 12) | \
					 PIN_AFIO_AF(PA04_F1_a, 0) | \
					 PIN_AFIO_AF(PA05_M95P_SCK, 5) | \
					 PIN_AFIO_AF(PA06_M95P_MISO, 5) | \
					 PIN_AFIO_AF(PA07_M95P_MOSI, 5))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(PA08_F1_b, 0) | \
					 PIN_AFIO_AF(PA09_F2, 0) | \
					 PIN_AFIO_AF(PA10_F3, 0) | \
					 PIN_AFIO_AF(PA11_F4, 0) | \
					 PIN_AFIO_AF(PA12_F0_a, 0) | \
					 PIN_AFIO_AF(PA13_SWDIO, 0) | \
					 PIN_AFIO_AF(PA14_SWCLK, 0) | \
					 PIN_AFIO_AF(PA15_PULLUP_SCL, 0))

#define VAL_GPIOB_MODER                 (PIN_MODE_OUTPUT(PB00_M95P_CS) | \
					 PIN_MODE_INPUT(PB01) | \
					 PIN_MODE_INPUT(PB02) | \
					 PIN_MODE_OUTPUT(PB03_PULLUP_SDA) | \
					 PIN_MODE_OUTPUT(PB04_CAN_TERMR_EN) | \
					 PIN_MODE_ALTERNATE(PB05_FDCAN_RX) | \
					 PIN_MODE_ALTERNATE(PB06_FDCAN_TX) | \
					 PIN_MODE_INPUT(PB07_F0_b) | \
					 PIN_MODE_ALTERNATE(PB08_RGBLED) | \
					 PIN_MODE_INPUT(PB09) | \
					 PIN_MODE_INPUT(PB10) | \
					 PIN_MODE_INPUT(PB11) | \
					 PIN_MODE_INPUT(PB12) | \
					 PIN_MODE_INPUT(PB13) | \
					 PIN_MODE_INPUT(PB14) | \
					 PIN_MODE_INPUT(PB15))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_PUSHPULL(PB00_M95P_CS) | \
					 PIN_OTYPE_OPENDRAIN(PB01) | \
					 PIN_OTYPE_OPENDRAIN(PB02) | \
					 PIN_OTYPE_PUSHPULL(PB03_PULLUP_SDA) | \
					 PIN_OTYPE_PUSHPULL(PB04_CAN_TERMR_EN) | \
					 PIN_OTYPE_PUSHPULL(PB05_FDCAN_RX) | \
					 PIN_OTYPE_PUSHPULL(PB06_FDCAN_TX) | \
					 PIN_OTYPE_OPENDRAIN(PB07_F0_b) | \
					 PIN_OTYPE_PUSHPULL(PB08_RGBLED) | \
					 PIN_OTYPE_OPENDRAIN(PB09) | \
					 PIN_OTYPE_OPENDRAIN(PB10) | \
					 PIN_OTYPE_OPENDRAIN(PB11) | \
					 PIN_OTYPE_OPENDRAIN(PB12) | \
					 PIN_OTYPE_OPENDRAIN(PB13) | \
					 PIN_OTYPE_OPENDRAIN(PB14) | \
					 PIN_OTYPE_OPENDRAIN(PB15))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PB00_M95P_CS) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB03_PULLUP_SDA) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB04_CAN_TERMR_EN) | \
					 PIN_OSPEED_SPEED_HIGH(PB05_FDCAN_RX) | \
					 PIN_OSPEED_SPEED_HIGH(PB06_FDCAN_TX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB07_F0_b) | \
					 PIN_OSPEED_SPEED_HIGH(PB08_RGBLED) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB15))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_FLOATING(PB00_M95P_CS) | \
					 PIN_PUPDR_PULLDOWN(PB01) | \
					 PIN_PUPDR_PULLDOWN(PB02) | \
					 PIN_PUPDR_FLOATING(PB03_PULLUP_SDA) | \
					 PIN_PUPDR_FLOATING(PB04_CAN_TERMR_EN) | \
					 PIN_PUPDR_FLOATING(PB05_FDCAN_RX) | \
					 PIN_PUPDR_FLOATING(PB06_FDCAN_TX) | \
					 PIN_PUPDR_FLOATING(PB07_F0_b) | \
					 PIN_PUPDR_FLOATING(PB08_RGBLED) | \
					 PIN_PUPDR_PULLDOWN(PB09) | \
					 PIN_PUPDR_PULLDOWN(PB10) | \
					 PIN_PUPDR_PULLDOWN(PB11) | \
					 PIN_PUPDR_PULLDOWN(PB12) | \
					 PIN_PUPDR_PULLDOWN(PB13) | \
					 PIN_PUPDR_PULLDOWN(PB14) | \
					 PIN_PUPDR_PULLDOWN(PB15))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_HIGH(PB00_M95P_CS) | \
					 PIN_ODR_LEVEL_HIGH(PB01) | \
					 PIN_ODR_LEVEL_HIGH(PB02) | \
					 PIN_ODR_LEVEL_LOW(PB03_PULLUP_SDA) | \
					 PIN_ODR_LEVEL_LOW(PB04_CAN_TERMR_EN) | \
					 PIN_ODR_LEVEL_HIGH(PB05_FDCAN_RX) | \
					 PIN_ODR_LEVEL_HIGH(PB06_FDCAN_TX) | \
					 PIN_ODR_LEVEL_LOW(PB07_F0_b) | \
					 PIN_ODR_LEVEL_LOW(PB08_RGBLED) | \
					 PIN_ODR_LEVEL_HIGH(PB09) | \
					 PIN_ODR_LEVEL_HIGH(PB10) | \
					 PIN_ODR_LEVEL_HIGH(PB11) | \
					 PIN_ODR_LEVEL_HIGH(PB12) | \
					 PIN_ODR_LEVEL_HIGH(PB13) | \
					 PIN_ODR_LEVEL_HIGH(PB14) | \
					 PIN_ODR_LEVEL_HIGH(PB15))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(PB00_M95P_CS, 0) | \
					 PIN_AFIO_AF(PB01, 0) | \
					 PIN_AFIO_AF(PB02, 0) | \
					 PIN_AFIO_AF(PB03_PULLUP_SDA, 0) | \
					 PIN_AFIO_AF(PB04_CAN_TERMR_EN, 0) | \
					 PIN_AFIO_AF(PB05_FDCAN_RX, 9) | \
					 PIN_AFIO_AF(PB06_FDCAN_TX, 9) | \
					 PIN_AFIO_AF(PB07_F0_b, 0))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(PB08_RGBLED, 10) | \
					 PIN_AFIO_AF(PB09, 0) | \
					 PIN_AFIO_AF(PB10, 0) | \
					 PIN_AFIO_AF(PB11, 0) | \
					 PIN_AFIO_AF(PB12, 0) | \
					 PIN_AFIO_AF(PB13, 0) | \
					 PIN_AFIO_AF(PB14, 0) | \
					 PIN_AFIO_AF(PB15, 0))

#define VAL_GPIOC_MODER                 (PIN_MODE_INPUT(PC00) | \
					 PIN_MODE_INPUT(PC01) | \
					 PIN_MODE_INPUT(PC02) | \
					 PIN_MODE_INPUT(PC03) | \
					 PIN_MODE_INPUT(PC04) | \
					 PIN_MODE_INPUT(PC05) | \
					 PIN_MODE_INPUT(PC06) | \
					 PIN_MODE_INPUT(PC07) | \
					 PIN_MODE_INPUT(PC08) | \
					 PIN_MODE_INPUT(PC09) | \
					 PIN_MODE_INPUT(PC10) | \
					 PIN_MODE_INPUT(PC11) | \
					 PIN_MODE_INPUT(PC12) | \
					 PIN_MODE_INPUT(PC13) | \
					 PIN_MODE_INPUT(PC14) | \
					 PIN_MODE_INPUT(PC15))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_OPENDRAIN(PC00) | \
					 PIN_OTYPE_OPENDRAIN(PC01) | \
					 PIN_OTYPE_OPENDRAIN(PC02) | \
					 PIN_OTYPE_OPENDRAIN(PC03) | \
					 PIN_OTYPE_OPENDRAIN(PC04) | \
					 PIN_OTYPE_OPENDRAIN(PC05) | \
					 PIN_OTYPE_OPENDRAIN(PC06) | \
					 PIN_OTYPE_OPENDRAIN(PC07) | \
					 PIN_OTYPE_OPENDRAIN(PC08) | \
					 PIN_OTYPE_OPENDRAIN(PC09) | \
					 PIN_OTYPE_OPENDRAIN(PC10) | \
					 PIN_OTYPE_OPENDRAIN(PC11) | \
					 PIN_OTYPE_OPENDRAIN(PC12) | \
					 PIN_OTYPE_OPENDRAIN(PC13) | \
					 PIN_OTYPE_OPENDRAIN(PC14) | \
					 PIN_OTYPE_OPENDRAIN(PC15))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PC00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC15))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_PULLDOWN(PC00) | \
					 PIN_PUPDR_PULLDOWN(PC01) | \
					 PIN_PUPDR_PULLDOWN(PC02) | \
					 PIN_PUPDR_PULLDOWN(PC03) | \
					 PIN_PUPDR_PULLDOWN(PC04) | \
					 PIN_PUPDR_PULLDOWN(PC05) | \
					 PIN_PUPDR_PULLDOWN(PC06) | \
					 PIN_PUPDR_PULLDOWN(PC07) | \
					 PIN_PUPDR_PULLDOWN(PC08) | \
					 PIN_PUPDR_PULLDOWN(PC09) | \
					 PIN_PUPDR_PULLDOWN(PC10) | \
					 PIN_PUPDR_PULLDOWN(PC11) | \
					 PIN_PUPDR_PULLDOWN(PC12) | \
					 PIN_PUPDR_PULLDOWN(PC13) | \
					 PIN_PUPDR_PULLDOWN(PC14) | \
					 PIN_PUPDR_PULLDOWN(PC15))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_HIGH(PC00) | \
					 PIN_ODR_LEVEL_HIGH(PC01) | \
					 PIN_ODR_LEVEL_HIGH(PC02) | \
					 PIN_ODR_LEVEL_HIGH(PC03) | \
					 PIN_ODR_LEVEL_HIGH(PC04) | \
					 PIN_ODR_LEVEL_HIGH(PC05) | \
					 PIN_ODR_LEVEL_HIGH(PC06) | \
					 PIN_ODR_LEVEL_HIGH(PC07) | \
					 PIN_ODR_LEVEL_HIGH(PC08) | \
					 PIN_ODR_LEVEL_HIGH(PC09) | \
					 PIN_ODR_LEVEL_HIGH(PC10) | \
					 PIN_ODR_LEVEL_HIGH(PC11) | \
					 PIN_ODR_LEVEL_HIGH(PC12) | \
					 PIN_ODR_LEVEL_HIGH(PC13) | \
					 PIN_ODR_LEVEL_HIGH(PC14) | \
					 PIN_ODR_LEVEL_HIGH(PC15))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(PC00, 0) | \
					 PIN_AFIO_AF(PC01, 0) | \
					 PIN_AFIO_AF(PC02, 0) | \
					 PIN_AFIO_AF(PC03, 0) | \
					 PIN_AFIO_AF(PC04, 0) | \
					 PIN_AFIO_AF(PC05, 0) | \
					 PIN_AFIO_AF(PC06, 0) | \
					 PIN_AFIO_AF(PC07, 0))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(PC08, 0) | \
					 PIN_AFIO_AF(PC09, 0) | \
					 PIN_AFIO_AF(PC10, 0) | \
					 PIN_AFIO_AF(PC11, 0) | \
					 PIN_AFIO_AF(PC12, 0) | \
					 PIN_AFIO_AF(PC13, 0) | \
					 PIN_AFIO_AF(PC14, 0) | \
					 PIN_AFIO_AF(PC15, 0))

#define VAL_GPIOD_MODER                 (PIN_MODE_INPUT(PD00) | \
					 PIN_MODE_INPUT(PD01) | \
					 PIN_MODE_INPUT(PD02) | \
					 PIN_MODE_INPUT(PD03) | \
					 PIN_MODE_INPUT(PD04) | \
					 PIN_MODE_INPUT(PD05) | \
					 PIN_MODE_INPUT(PD06) | \
					 PIN_MODE_INPUT(PD07) | \
					 PIN_MODE_INPUT(PD08) | \
					 PIN_MODE_INPUT(PD09) | \
					 PIN_MODE_INPUT(PD10) | \
					 PIN_MODE_INPUT(PD11) | \
					 PIN_MODE_INPUT(PD12) | \
					 PIN_MODE_INPUT(PD13) | \
					 PIN_MODE_INPUT(PD14) | \
					 PIN_MODE_INPUT(PD15))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_OPENDRAIN(PD00) | \
					 PIN_OTYPE_OPENDRAIN(PD01) | \
					 PIN_OTYPE_OPENDRAIN(PD02) | \
					 PIN_OTYPE_OPENDRAIN(PD03) | \
					 PIN_OTYPE_OPENDRAIN(PD04) | \
					 PIN_OTYPE_OPENDRAIN(PD05) | \
					 PIN_OTYPE_OPENDRAIN(PD06) | \
					 PIN_OTYPE_OPENDRAIN(PD07) | \
					 PIN_OTYPE_OPENDRAIN(PD08) | \
					 PIN_OTYPE_OPENDRAIN(PD09) | \
					 PIN_OTYPE_OPENDRAIN(PD10) | \
					 PIN_OTYPE_OPENDRAIN(PD11) | \
					 PIN_OTYPE_OPENDRAIN(PD12) | \
					 PIN_OTYPE_OPENDRAIN(PD13) | \
					 PIN_OTYPE_OPENDRAIN(PD14) | \
					 PIN_OTYPE_OPENDRAIN(PD15))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PD00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD15))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_PULLDOWN(PD00) | \
					 PIN_PUPDR_PULLDOWN(PD01) | \
					 PIN_PUPDR_PULLDOWN(PD02) | \
					 PIN_PUPDR_PULLDOWN(PD03) | \
					 PIN_PUPDR_PULLDOWN(PD04) | \
					 PIN_PUPDR_PULLDOWN(PD05) | \
					 PIN_PUPDR_PULLDOWN(PD06) | \
					 PIN_PUPDR_PULLDOWN(PD07) | \
					 PIN_PUPDR_PULLDOWN(PD08) | \
					 PIN_PUPDR_PULLDOWN(PD09) | \
					 PIN_PUPDR_PULLDOWN(PD10) | \
					 PIN_PUPDR_PULLDOWN(PD11) | \
					 PIN_PUPDR_PULLDOWN(PD12) | \
					 PIN_PUPDR_PULLDOWN(PD13) | \
					 PIN_PUPDR_PULLDOWN(PD14) | \
					 PIN_PUPDR_PULLDOWN(PD15))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(PD00) | \
					 PIN_ODR_LEVEL_HIGH(PD01) | \
					 PIN_ODR_LEVEL_HIGH(PD02) | \
					 PIN_ODR_LEVEL_HIGH(PD03) | \
					 PIN_ODR_LEVEL_HIGH(PD04) | \
					 PIN_ODR_LEVEL_HIGH(PD05) | \
					 PIN_ODR_LEVEL_HIGH(PD06) | \
					 PIN_ODR_LEVEL_HIGH(PD07) | \
					 PIN_ODR_LEVEL_HIGH(PD08) | \
					 PIN_ODR_LEVEL_HIGH(PD09) | \
					 PIN_ODR_LEVEL_HIGH(PD10) | \
					 PIN_ODR_LEVEL_HIGH(PD11) | \
					 PIN_ODR_LEVEL_HIGH(PD12) | \
					 PIN_ODR_LEVEL_HIGH(PD13) | \
					 PIN_ODR_LEVEL_HIGH(PD14) | \
					 PIN_ODR_LEVEL_HIGH(PD15))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(PD00, 0) | \
					 PIN_AFIO_AF(PD01, 0) | \
					 PIN_AFIO_AF(PD02, 0) | \
					 PIN_AFIO_AF(PD03, 0) | \
					 PIN_AFIO_AF(PD04, 0) | \
					 PIN_AFIO_AF(PD05, 0) | \
					 PIN_AFIO_AF(PD06, 0) | \
					 PIN_AFIO_AF(PD07, 0))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(PD08, 0) | \
					 PIN_AFIO_AF(PD09, 0) | \
					 PIN_AFIO_AF(PD10, 0) | \
					 PIN_AFIO_AF(PD11, 0) | \
					 PIN_AFIO_AF(PD12, 0) | \
					 PIN_AFIO_AF(PD13, 0) | \
					 PIN_AFIO_AF(PD14, 0) | \
					 PIN_AFIO_AF(PD15, 0))

#define VAL_GPIOE_MODER                 (PIN_MODE_INPUT(PE00) | \
					 PIN_MODE_INPUT(PE01) | \
					 PIN_MODE_INPUT(PE02) | \
					 PIN_MODE_INPUT(PE03) | \
					 PIN_MODE_INPUT(PE04) | \
					 PIN_MODE_INPUT(PE05) | \
					 PIN_MODE_INPUT(PE06) | \
					 PIN_MODE_INPUT(PE07) | \
					 PIN_MODE_INPUT(PE08) | \
					 PIN_MODE_INPUT(PE09) | \
					 PIN_MODE_INPUT(PE10) | \
					 PIN_MODE_INPUT(PE11) | \
					 PIN_MODE_INPUT(PE12) | \
					 PIN_MODE_INPUT(PE13) | \
					 PIN_MODE_INPUT(PE14) | \
					 PIN_MODE_INPUT(PE15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_OPENDRAIN(PE00) | \
					 PIN_OTYPE_OPENDRAIN(PE01) | \
					 PIN_OTYPE_OPENDRAIN(PE02) | \
					 PIN_OTYPE_OPENDRAIN(PE03) | \
					 PIN_OTYPE_OPENDRAIN(PE04) | \
					 PIN_OTYPE_OPENDRAIN(PE05) | \
					 PIN_OTYPE_OPENDRAIN(PE06) | \
					 PIN_OTYPE_OPENDRAIN(PE07) | \
					 PIN_OTYPE_OPENDRAIN(PE08) | \
					 PIN_OTYPE_OPENDRAIN(PE09) | \
					 PIN_OTYPE_OPENDRAIN(PE10) | \
					 PIN_OTYPE_OPENDRAIN(PE11) | \
					 PIN_OTYPE_OPENDRAIN(PE12) | \
					 PIN_OTYPE_OPENDRAIN(PE13) | \
					 PIN_OTYPE_OPENDRAIN(PE14) | \
					 PIN_OTYPE_OPENDRAIN(PE15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PE00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_PULLDOWN(PE00) | \
					 PIN_PUPDR_PULLDOWN(PE01) | \
					 PIN_PUPDR_PULLDOWN(PE02) | \
					 PIN_PUPDR_PULLDOWN(PE03) | \
					 PIN_PUPDR_PULLDOWN(PE04) | \
					 PIN_PUPDR_PULLDOWN(PE05) | \
					 PIN_PUPDR_PULLDOWN(PE06) | \
					 PIN_PUPDR_PULLDOWN(PE07) | \
					 PIN_PUPDR_PULLDOWN(PE08) | \
					 PIN_PUPDR_PULLDOWN(PE09) | \
					 PIN_PUPDR_PULLDOWN(PE10) | \
					 PIN_PUPDR_PULLDOWN(PE11) | \
					 PIN_PUPDR_PULLDOWN(PE12) | \
					 PIN_PUPDR_PULLDOWN(PE13) | \
					 PIN_PUPDR_PULLDOWN(PE14) | \
					 PIN_PUPDR_PULLDOWN(PE15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(PE00) | \
					 PIN_ODR_LEVEL_HIGH(PE01) | \
					 PIN_ODR_LEVEL_HIGH(PE02) | \
					 PIN_ODR_LEVEL_HIGH(PE03) | \
					 PIN_ODR_LEVEL_HIGH(PE04) | \
					 PIN_ODR_LEVEL_HIGH(PE05) | \
					 PIN_ODR_LEVEL_HIGH(PE06) | \
					 PIN_ODR_LEVEL_HIGH(PE07) | \
					 PIN_ODR_LEVEL_HIGH(PE08) | \
					 PIN_ODR_LEVEL_HIGH(PE09) | \
					 PIN_ODR_LEVEL_HIGH(PE10) | \
					 PIN_ODR_LEVEL_HIGH(PE11) | \
					 PIN_ODR_LEVEL_HIGH(PE12) | \
					 PIN_ODR_LEVEL_HIGH(PE13) | \
					 PIN_ODR_LEVEL_HIGH(PE14) | \
					 PIN_ODR_LEVEL_HIGH(PE15))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(PE00, 0) | \
					 PIN_AFIO_AF(PE01, 0) | \
					 PIN_AFIO_AF(PE02, 0) | \
					 PIN_AFIO_AF(PE03, 0) | \
					 PIN_AFIO_AF(PE04, 0) | \
					 PIN_AFIO_AF(PE05, 0) | \
					 PIN_AFIO_AF(PE06, 0) | \
					 PIN_AFIO_AF(PE07, 0))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(PE08, 0) | \
					 PIN_AFIO_AF(PE09, 0) | \
					 PIN_AFIO_AF(PE10, 0) | \
					 PIN_AFIO_AF(PE11, 0) | \
					 PIN_AFIO_AF(PE12, 0) | \
					 PIN_AFIO_AF(PE13, 0) | \
					 PIN_AFIO_AF(PE14, 0) | \
					 PIN_AFIO_AF(PE15, 0))

#define VAL_GPIOF_MODER                 (PIN_MODE_ALTERNATE(PF00_OSC_IN) | \
					 PIN_MODE_ALTERNATE(PF01_OSC_OUT) | \
					 PIN_MODE_INPUT(PF02) | \
					 PIN_MODE_INPUT(PF03) | \
					 PIN_MODE_INPUT(PF04) | \
					 PIN_MODE_INPUT(PF05) | \
					 PIN_MODE_INPUT(PF06) | \
					 PIN_MODE_INPUT(PF07) | \
					 PIN_MODE_INPUT(PF08) | \
					 PIN_MODE_INPUT(PF09) | \
					 PIN_MODE_INPUT(PF10) | \
					 PIN_MODE_INPUT(PF11) | \
					 PIN_MODE_INPUT(PF12) | \
					 PIN_MODE_INPUT(PF13) | \
					 PIN_MODE_INPUT(PF14) | \
					 PIN_MODE_INPUT(PF15))

#define VAL_GPIOF_OTYPER                (PIN_OTYPE_PUSHPULL(PF00_OSC_IN) | \
					 PIN_OTYPE_PUSHPULL(PF01_OSC_OUT) | \
					 PIN_OTYPE_OPENDRAIN(PF02) | \
					 PIN_OTYPE_OPENDRAIN(PF03) | \
					 PIN_OTYPE_OPENDRAIN(PF04) | \
					 PIN_OTYPE_OPENDRAIN(PF05) | \
					 PIN_OTYPE_OPENDRAIN(PF06) | \
					 PIN_OTYPE_OPENDRAIN(PF07) | \
					 PIN_OTYPE_OPENDRAIN(PF08) | \
					 PIN_OTYPE_OPENDRAIN(PF09) | \
					 PIN_OTYPE_OPENDRAIN(PF10) | \
					 PIN_OTYPE_OPENDRAIN(PF11) | \
					 PIN_OTYPE_OPENDRAIN(PF12) | \
					 PIN_OTYPE_OPENDRAIN(PF13) | \
					 PIN_OTYPE_OPENDRAIN(PF14) | \
					 PIN_OTYPE_OPENDRAIN(PF15))

#define VAL_GPIOF_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PF00_OSC_IN) | \
					 PIN_OSPEED_SPEED_HIGH(PF01_OSC_OUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PF15))

#define VAL_GPIOF_PUPDR                 (PIN_PUPDR_FLOATING(PF00_OSC_IN) | \
					 PIN_PUPDR_FLOATING(PF01_OSC_OUT) | \
					 PIN_PUPDR_PULLDOWN(PF02) | \
					 PIN_PUPDR_PULLDOWN(PF03) | \
					 PIN_PUPDR_PULLDOWN(PF04) | \
					 PIN_PUPDR_PULLDOWN(PF05) | \
					 PIN_PUPDR_PULLDOWN(PF06) | \
					 PIN_PUPDR_PULLDOWN(PF07) | \
					 PIN_PUPDR_PULLDOWN(PF08) | \
					 PIN_PUPDR_PULLDOWN(PF09) | \
					 PIN_PUPDR_PULLDOWN(PF10) | \
					 PIN_PUPDR_PULLDOWN(PF11) | \
					 PIN_PUPDR_PULLDOWN(PF12) | \
					 PIN_PUPDR_PULLDOWN(PF13) | \
					 PIN_PUPDR_PULLDOWN(PF14) | \
					 PIN_PUPDR_PULLDOWN(PF15))

#define VAL_GPIOF_ODR                   (PIN_ODR_LEVEL_HIGH(PF00_OSC_IN) | \
					 PIN_ODR_LEVEL_HIGH(PF01_OSC_OUT) | \
					 PIN_ODR_LEVEL_HIGH(PF02) | \
					 PIN_ODR_LEVEL_HIGH(PF03) | \
					 PIN_ODR_LEVEL_HIGH(PF04) | \
					 PIN_ODR_LEVEL_HIGH(PF05) | \
					 PIN_ODR_LEVEL_HIGH(PF06) | \
					 PIN_ODR_LEVEL_HIGH(PF07) | \
					 PIN_ODR_LEVEL_HIGH(PF08) | \
					 PIN_ODR_LEVEL_HIGH(PF09) | \
					 PIN_ODR_LEVEL_HIGH(PF10) | \
					 PIN_ODR_LEVEL_HIGH(PF11) | \
					 PIN_ODR_LEVEL_HIGH(PF12) | \
					 PIN_ODR_LEVEL_HIGH(PF13) | \
					 PIN_ODR_LEVEL_HIGH(PF14) | \
					 PIN_ODR_LEVEL_HIGH(PF15))

#define VAL_GPIOF_AFRL			(PIN_AFIO_AF(PF00_OSC_IN, 0) | \
					 PIN_AFIO_AF(PF01_OSC_OUT, 0) | \
					 PIN_AFIO_AF(PF02, 0) | \
					 PIN_AFIO_AF(PF03, 0) | \
					 PIN_AFIO_AF(PF04, 0) | \
					 PIN_AFIO_AF(PF05, 0) | \
					 PIN_AFIO_AF(PF06, 0) | \
					 PIN_AFIO_AF(PF07, 0))

#define VAL_GPIOF_AFRH			(PIN_AFIO_AF(PF08, 0) | \
					 PIN_AFIO_AF(PF09, 0) | \
					 PIN_AFIO_AF(PF10, 0) | \
					 PIN_AFIO_AF(PF11, 0) | \
					 PIN_AFIO_AF(PF12, 0) | \
					 PIN_AFIO_AF(PF13, 0) | \
					 PIN_AFIO_AF(PF14, 0) | \
					 PIN_AFIO_AF(PF15, 0))

#define VAL_GPIOG_MODER                 (PIN_MODE_INPUT(PG00) | \
					 PIN_MODE_INPUT(PG01) | \
					 PIN_MODE_INPUT(PG02) | \
					 PIN_MODE_INPUT(PG03) | \
					 PIN_MODE_INPUT(PG04) | \
					 PIN_MODE_INPUT(PG05) | \
					 PIN_MODE_INPUT(PG06) | \
					 PIN_MODE_INPUT(PG07) | \
					 PIN_MODE_INPUT(PG08) | \
					 PIN_MODE_INPUT(PG09) | \
					 PIN_MODE_INPUT(PG10) | \
					 PIN_MODE_INPUT(PG11) | \
					 PIN_MODE_INPUT(PG12) | \
					 PIN_MODE_INPUT(PG13) | \
					 PIN_MODE_INPUT(PG14) | \
					 PIN_MODE_INPUT(PG15))

#define VAL_GPIOG_OTYPER                (PIN_OTYPE_OPENDRAIN(PG00) | \
					 PIN_OTYPE_OPENDRAIN(PG01) | \
					 PIN_OTYPE_OPENDRAIN(PG02) | \
					 PIN_OTYPE_OPENDRAIN(PG03) | \
					 PIN_OTYPE_OPENDRAIN(PG04) | \
					 PIN_OTYPE_OPENDRAIN(PG05) | \
					 PIN_OTYPE_OPENDRAIN(PG06) | \
					 PIN_OTYPE_OPENDRAIN(PG07) | \
					 PIN_OTYPE_OPENDRAIN(PG08) | \
					 PIN_OTYPE_OPENDRAIN(PG09) | \
					 PIN_OTYPE_OPENDRAIN(PG10) | \
					 PIN_OTYPE_OPENDRAIN(PG11) | \
					 PIN_OTYPE_OPENDRAIN(PG12) | \
					 PIN_OTYPE_OPENDRAIN(PG13) | \
					 PIN_OTYPE_OPENDRAIN(PG14) | \
					 PIN_OTYPE_OPENDRAIN(PG15))

#define VAL_GPIOG_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PG00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PG15))

#define VAL_GPIOG_PUPDR                 (PIN_PUPDR_PULLDOWN(PG00) | \
					 PIN_PUPDR_PULLDOWN(PG01) | \
					 PIN_PUPDR_PULLDOWN(PG02) | \
					 PIN_PUPDR_PULLDOWN(PG03) | \
					 PIN_PUPDR_PULLDOWN(PG04) | \
					 PIN_PUPDR_PULLDOWN(PG05) | \
					 PIN_PUPDR_PULLDOWN(PG06) | \
					 PIN_PUPDR_PULLDOWN(PG07) | \
					 PIN_PUPDR_PULLDOWN(PG08) | \
					 PIN_PUPDR_PULLDOWN(PG09) | \
					 PIN_PUPDR_PULLDOWN(PG10) | \
					 PIN_PUPDR_PULLDOWN(PG11) | \
					 PIN_PUPDR_PULLDOWN(PG12) | \
					 PIN_PUPDR_PULLDOWN(PG13) | \
					 PIN_PUPDR_PULLDOWN(PG14) | \
					 PIN_PUPDR_PULLDOWN(PG15))

#define VAL_GPIOG_ODR                   (PIN_ODR_LEVEL_HIGH(PG00) | \
					 PIN_ODR_LEVEL_HIGH(PG01) | \
					 PIN_ODR_LEVEL_HIGH(PG02) | \
					 PIN_ODR_LEVEL_HIGH(PG03) | \
					 PIN_ODR_LEVEL_HIGH(PG04) | \
					 PIN_ODR_LEVEL_HIGH(PG05) | \
					 PIN_ODR_LEVEL_HIGH(PG06) | \
					 PIN_ODR_LEVEL_HIGH(PG07) | \
					 PIN_ODR_LEVEL_HIGH(PG08) | \
					 PIN_ODR_LEVEL_HIGH(PG09) | \
					 PIN_ODR_LEVEL_HIGH(PG10) | \
					 PIN_ODR_LEVEL_HIGH(PG11) | \
					 PIN_ODR_LEVEL_HIGH(PG12) | \
					 PIN_ODR_LEVEL_HIGH(PG13) | \
					 PIN_ODR_LEVEL_HIGH(PG14) | \
					 PIN_ODR_LEVEL_HIGH(PG15))

#define VAL_GPIOG_AFRL			(PIN_AFIO_AF(PG00, 0) | \
					 PIN_AFIO_AF(PG01, 0) | \
					 PIN_AFIO_AF(PG02, 0) | \
					 PIN_AFIO_AF(PG03, 0) | \
					 PIN_AFIO_AF(PG04, 0) | \
					 PIN_AFIO_AF(PG05, 0) | \
					 PIN_AFIO_AF(PG06, 0) | \
					 PIN_AFIO_AF(PG07, 0))

#define VAL_GPIOG_AFRH			(PIN_AFIO_AF(PG08, 0) | \
					 PIN_AFIO_AF(PG09, 0) | \
					 PIN_AFIO_AF(PG10, 0) | \
					 PIN_AFIO_AF(PG11, 0) | \
					 PIN_AFIO_AF(PG12, 0) | \
					 PIN_AFIO_AF(PG13, 0) | \
					 PIN_AFIO_AF(PG14, 0) | \
					 PIN_AFIO_AF(PG15, 0))

#define AF_PA02_DBG_TX                   12U
#define AF_LINE_DBG_TX                   12U
#define AF_PA03_DBG_RX                   12U
#define AF_LINE_DBG_RX                   12U
#define AF_PA05_M95P_SCK                 5U
#define AF_LINE_M95P_SCK                 5U
#define AF_PA06_M95P_MISO                5U
#define AF_LINE_M95P_MISO                5U
#define AF_PA07_M95P_MOSI                5U
#define AF_LINE_M95P_MOSI                5U
#define AF_PA13_SWDIO                    0U
#define AF_LINE_SWDIO                    0U
#define AF_PA14_SWCLK                    0U
#define AF_LINE_SWCLK                    0U
#define AF_PB05_FDCAN_RX                 9U
#define AF_LINE_FDCAN_RX                 9U
#define AF_PB06_FDCAN_TX                 9U
#define AF_LINE_FDCAN_TX                 9U
#define AF_PB08_RGBLED                   10U
#define AF_LINE_RGBLED                   10U
#define AF_PF00_OSC_IN                   0U
#define AF_LINE_OSC_IN                   0U
#define AF_PF01_OSC_OUT                  0U
#define AF_LINE_OSC_OUT                  0U


#define F1_a_SPI	 1
#define F1_a_SPI_FN	 NSS
#define F1_a_SPI_AF	 5
#define F1_b_TIM	 1
#define F1_b_TIM_FN	 CH
#define F1_b_TIM_CH	 1
#define F1_b_TIM_AF	 6
#define F1_b_I2C	 2
#define F1_b_I2C_FN	 SDA
#define F1_b_I2C_AF	 4
#define F2_TIM	 1
#define F2_TIM_FN	 CH
#define F2_TIM_CH	 2
#define F2_TIM_AF	 6
#define F2_USART	 1
#define F2_USART_FN	 TX
#define F2_USART_AF	 7
#define F2_I2C	 2
#define F2_I2C_FN	 SCL
#define F2_I2C_AF	 4
#define F3_TIM	 1
#define F3_TIM_FN	 CH
#define F3_TIM_CH	 3
#define F3_TIM_AF	 6
#define F3_USART	 1
#define F3_USART_FN	 RX
#define F3_USART_AF	 7
#define F3_SPI	 2
#define F3_SPI_FN	 MISO
#define F3_SPI_AF	 5
#define F4_TIM	 1
#define F4_TIM_FN	 CH
#define F4_TIM_CH	 4
#define F4_TIM_AF	 11
#define F4_USART	 1
#define F4_USART_FN	 CTS
#define F4_USART_AF	 7
#define F4_SPI	 2
#define F4_SPI_FN	 MOSI
#define F4_SPI_AF	 5
#define F0_a_TIM	 4
#define F0_a_TIM_FN	 CH
#define F0_a_TIM_CH	 2
#define F0_a_TIM_AF	 10
#define F0_a_USART	 1
#define F0_a_USART_FN	 RTS
#define F0_a_USART_AF	 7
#define F0_b_TIM	 3
#define F0_b_TIM_FN	 CH
#define F0_b_TIM_CH	 4
#define F0_b_TIM_AF	 10
#define F0_b_USART	 1
#define F0_b_USART_FN	 RX
#define F0_b_USART_AF	 7
#define RGBLED_TIM	 8
#define RGBLED_TIM_FN	 CH
#define RGBLED_TIM_CH	 2
#define RGBLED_TIM_AF	 10

#define BOARD_GROUP_DECLFOREACH(line, group) \
  static const ioline_t group ## _ARRAY[] = {group}; \
  for (ioline_t i=0, line =  group ## _ARRAY[i]; (i < group ## _SIZE) && (line = group ## _ARRAY[i]); i++)

#define BOARD_GROUP_FOREACH(line, group) \
  for (ioline_t i=0, line =  group ## _ARRAY[i]; (i < group ## _SIZE) && (line = group ## _ARRAY[i]); i++)


#define BOARD_GROUP_DECLFOR(array, index, group)  \
  static const ioline_t group ## _ARRAY[] = {group};    \
  for (ioline_t index=0, *array =  (ioline_t *) group ## _ARRAY; index < group ## _SIZE; index++)

#define BOARD_GROUP_FOR(array, index, group)  \
  for (ioline_t index=0, *array =  (ioline_t *) group ## _ARRAY; index < group ## _SIZE; index++)

#define DYNAMIC_FUNCTION_PIN \
	LINE_F1_a, \
	LINE_F1_b, \
	LINE_F0_a, \
	LINE_PULLUP_SCL, \
	LINE_PULLUP_SDA, \
	LINE_F0_b
#define DYNAMIC_FUNCTION_PIN_SIZE 	 6

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

