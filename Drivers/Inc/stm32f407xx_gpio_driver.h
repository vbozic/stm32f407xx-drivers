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
	uint8_t GPIO_PinMode;							/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;							/* possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;					/* possible values from @GPIO_PIN_PUPD */
	uint8_t GPIO_PinOPType;							/* possible values from @GPIO_PIN_OP_TYPE */
	uint8_t GPIO_PinAltFunMode;						/* possible values from @GPIO_PIN_ALT_MODE */
}GPIO_PinConfig_t;

/*
 * Define GPIO pin numbers
 */

#define GPIO_PIN_NO_0					0
#define GPIO_PIN_NO_1					1
#define GPIO_PIN_NO_2					2
#define GPIO_PIN_NO_3					3
#define GPIO_PIN_NO_4					4
#define GPIO_PIN_NO_5					5
#define GPIO_PIN_NO_6					6
#define GPIO_PIN_NO_7					7
#define GPIO_PIN_NO_8					8
#define GPIO_PIN_NO_9					9
#define GPIO_PIN_NO_10					10
#define GPIO_PIN_NO_11					11
#define GPIO_PIN_NO_12					12
#define GPIO_PIN_NO_13					13
#define GPIO_PIN_NO_14					14
#define GPIO_PIN_NO_15					15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN					0
#define GPIO MODE_OUT					1
#define GPIO_MODE_ALTFN					2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_IT_FT					4			// Falling edge
#define GPIO_MODE_IT_RT					5			// Rising edge
#define GPIO_MODE_IT_RFT				6			// Rising and falling edge


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

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO Init and Deinit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t pinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
