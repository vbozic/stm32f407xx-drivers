/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 18. ruj 2024.
 *      Author: Vjeran
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * A handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;						/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*******************************************************************************************************************************
 * 											APIs supported by this driver
 * 						For more information about the APIs check the function definition
 ******************************************************************************************************************************/

/*
 * Peripheral Clock Control
 */

void GPIO_PeriClockControl(void);

/*
 * GPIO Init and Deinit
 */

void GPIO_Init(void);
void GPIO_DeInit(void);

/*
 * Data Read and Write
 */

void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
