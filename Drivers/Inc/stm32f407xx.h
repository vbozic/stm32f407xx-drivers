/*
 * stm32f407xx.h
 *
 *  Created on: Sep 17, 2024
 *      Author: Vjeran
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * Base addresses of FLASH and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM2_BASEADDR						0x20001C00U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM								SRAM1_BASEADDR

/*
 * AHPx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE							0x40000000U
#define APB1_PERIPH_BASE					PERIPH_BASE
#define APB2_PERIPH_BASE					0x40010000U
#define AHB1_PERIPH_BASE					0x40020000U
#define AHB2_PERIPH_BASE					0x50000000U

/*
 * Base addresses of peripherals that are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						(AHB1_PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR						(AHB1_PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR						(AHB1_PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR						(AHB1_PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR						(AHB1_PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR						(AHB1_PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR						(AHB1_PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR						(AHB1_PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR						(AHB1_PERIPH_BASE + 0x2000)
#define RCC_BASEADDR						(AHB1_PERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals that are hanging on APB1 bus
 */

#define I2C1_BASEADDR						(APB1_PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR						(APB1_PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR						(APB1_PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR						(APB1_PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR						(APB1_PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR						(APB1_PERIPH_BASE + 0x4400)
#define USART3_BASEADDR						(APB1_PERIPH_BASE + 0x4800)
#define UART4_BASEADDR						(APB1_PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR						(APB1_PERIPH_BASE + 0x5000)

/*
 * Base addresses of peripherals that are hanging on APB2 bus
 */

#define EXTI_BASEADDR						(APB2_PERIPH_BASE + 0x3C00)
#define SPI1_BASEADDR						(APB2_PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADDR						(APB2_PERIPH_BASE + 0x3800)
#define USART1_BASEADDR						(APB2_PERIPH_BASE + 0x1000)
#define USART6_BASEADDR						(APB2_PERIPH_BASE + 0x1400)

#endif /* INC_STM32F407XX_H_ */
