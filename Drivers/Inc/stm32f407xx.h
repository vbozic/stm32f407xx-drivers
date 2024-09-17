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

#endif /* INC_STM32F407XX_H_ */
