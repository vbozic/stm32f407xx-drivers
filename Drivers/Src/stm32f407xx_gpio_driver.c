/* stm32f407xx_gpio_driver.c */

#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Init GPIO
 *
 * @param[in]         - gpio handle
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - DeInit GPIO
 *
 * @param[in]         - gpio handle
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Read data from input pin
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         - pin number which we want to read from
 *
 * @return            - uint8_t data from pin
 *
 * @Note              - none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Read data from input port
 *
 * @param[in]         - gpio handle
 *
 * @return            - uint16_t data
 *
 * @Note              - none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Write data to output pin
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         - pin number
 * @param[in]         - write value
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Write data to output port
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         - write value
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggle output port
 *
 * @param[in]         - base address of gpio peripheral
 * @param[in]         - pin number which we want to toggle
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{

}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - Configure IRQ
 *
 * @param[in]         - IRQ number, must be unique
 * @param[in]         - IRQ Priority
 * @param[in]         - Enable or Disable
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - Handle IRQ
 *
 * @param[in]         - pin number
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_IRQHandling(uint8_t pinNumber)
{

}


