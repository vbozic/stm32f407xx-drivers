/*
 * stm32f407xx.h
 *
 *  Created on: Sep 17, 2024
 *      Author: Vjeran
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

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

/************************************** Peripheral register definition structures **************************************/

typedef struct
{
	__vo uint32_t MODER;						/* 0x00 GPIO port mode register */
	__vo uint32_t OTYPER;						/* 0x04 GPIO port output type register */
	__vo uint32_t OSPEEDR;						/* 0x08 GPIO port output speed register */
	__vo uint32_t PUPDR;						/* 0x0C GPIO port pull-up/pull-down register */
	__vo uint32_t IDR;							/* 0x10 GPIO port input data register */
	__vo uint32_t ODR;							/* 0x14 GPIO port output data register */
	__vo uint32_t BSRR;							/* 0x18 GPIO port bit set/reset register */
	__vo uint32_t LCKR;							/* 0x1C GPIO port configuration lock register */
	__vo uint32_t AFR[2];						/* 0x20 AFR[0] GPIO alternate function low register & 0x24 GPIO alternate function high register */
} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;							/* 0x00 */
	__vo uint32_t PLLCFGR;						/* 0x04 */
	__vo uint32_t CFGR;							/* 0x08 */
	__vo uint32_t CIR;							/* 0x0C */
	__vo uint32_t AHB1RSTR;						/* 0x10 */
	__vo uint32_t AHB2RSTR;						/* 0x14 */
	__vo uint32_t AHB3RSTR;						/* 0x18 */
	uint32_t	  RESERVED0;					/* 0x1C */
	__vo uint32_t APB1RSTR;						/* 0x20 */
	__vo uint32_t APB2RSTR;						/* 0x24 */
	uint32_t	  RESERVED1[2];					/* 0x28 0x2C */
	__vo uint32_t AHB1ENR;						/* 0x30 */
	__vo uint32_t AHB2ENR;						/* 0x34 */
	__vo uint32_t AHB3ENR;						/* 0x38 */
	__vo uint32_t APB1ENR;						/* 0x40 */
	__vo uint32_t APB2ENR;						/* 0x44 */
	uint32_t	  RESERVED3[2];					/* 0x48 0x4C */
	__vo uint32_t AHB1LPENR;					/* 0x50 */
	__vo uint32_t AHB2LPENR;					/* 0x54 */
	__vo uint32_t AHB3LPENR;					/* 0x58 */
	uint32_t 	  RESERVED4;					/* 0x5C */
	__vo uint32_t APB1LPENR;					/* 0x60 */
	__vo uint32_t APB2LPENR;					/* 0x64 */
	uint32_t	  RESERVED5[2];					/* 0x68 0x6C */
	__vo uint32_t BDCR;							/* 0x70 */
	__vo uint32_t CSR;							/* 0x74 */
	uint32_t	  RESERVED6[2];					/* 0x78 0x7C */
	__vo uint32_t SSCGR;						/* 0x80 */
	__vo uint32_t PLLI2SCFGR;					/* 0x84 */
} RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base address typecasted to xxx_RegDef_t)
 */

#define GPIOA								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF								((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG								((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH								((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI								((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))

#endif /* INC_STM32F407XX_H_ */
