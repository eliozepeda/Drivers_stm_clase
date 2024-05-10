/*
 * stm32l476xx_gpio_driver.c
 *
 *  Created on: Mar 12, 2024
 *      Author: Elio Zepeda
 */

#include "stm32l476xx_gpio_driver.h"


/*Peripheral clock setup*/
/***************************************************************************************
 * @fn						-GPIO_PeriClockControl
 *
 * @brief					-This functions enables or disables peripheral clock for the giving GPIO port
 *
 *
 * @pGPIOx 					-Base address of the GPIO peripheral
 * @EnorDi 					-Enable or disable MACRO
 *
 * @return 					- none
 */
void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLOCK_ENABLE();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLOCK_DISABLE();
		}
	}


}

/*Init and De-init*/
/***************************************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- Initializes the GPIO pin
 *
 * @param pGPIOHandle 		- Pointer to the GPIO handle structure
 *
 * @return 					- None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//1.-Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		//codigo en las siguientes clases (interrupciones)
		/*Check and configure if is rising trigger, falling trigger or falling and rising trigger*/
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Configuring input
			temp = (GPIO_MODE_IN << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER |= temp;

			//1.- Configure the FTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the RTSR bit
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//Configuring input
			temp = (GPIO_MODE_IN << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));;
			pGPIOHandle->pGPIOx->MODER |= temp;
			//1.- Configure the RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the FTSR bit
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT_RT)
		{
			//Configuring input
			temp = (GPIO_MODE_IN << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));;
			pGPIOHandle->pGPIOx->MODER |= temp;
			//1.- Configure both FTSR and RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2.- Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t tmp1, tmp2;
		tmp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		tmp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PERI_CLOCK_ENABLE();
		SYSCFG->EXTICR[tmp1] = portcode << (tmp2*4);

		//3.- Enable the EXTI interrupt delivery using IMR
		EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2.-Configure the speed
	temp=0;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3.-Configure the pupd settings
	temp=0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));;
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//4.-Configure the optype
	temp=0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;
	//5.-Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_RESET();
	}
}

/*Data read and write*/
/***************************************************************************************
 * @fn						- GPIO_ReadFromInputPin
 *
 * @brief					- Reads data from the input pin
 *
 * @param pGPIOx 			- Base address of the GPIO peripheral
 * @param PinNumber 		- Pin number
 *
 * @return 					- Value read from the input pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);

	return value;
}

/***************************************************************************************
 * @fn						- GPIO_ReadFromInputPort
 *
 * @brief					- Reads data from the input port
 *
 * @param pGPIOx 			- Base address of the GPIO peripheral
 *
 * @return 					- Value read from the input port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/***************************************************************************************
 * @fn						- GPIO_WriteToOutputPin
 *
 * @brief					- Writes data to the output pin
 *
 * @param pGPIOx 			- Base address of the GPIO peripheral
 * @param PinNumber 		- Pin number
 * @param Value 			- Value to be written
 *
 * @return 					- None
 */
void GPIO_WriteToOutputPin(GPIO_Reg_Def_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->BSRR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***************************************************************************************
 * @fn						- GPIO_WriteToOutputPort
 *
 * @brief					- Writes data to the output port
 *
 * @param pGPIOx 			- Base address of the GPIO peripheral
 * @param Value 			- Value to be written
 *
 * @return 					- None
 */
void GPIO_WriteToOutputPort(GPIO_Reg_Def_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}


/***************************************************************************************
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					- Toggles the output pin
 *
 * @param pGPIOx 			- Base address of the GPIO peripheral
 * @param PinNumber 		- Pin number
 *
 * @return 					- None
 */
void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*IRQ configuration and handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber %32));
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber %64));
		}
	}
}

void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1 first find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_value = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_value);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR1 & (1 << PinNumber))
	{
		//limpiar la bandera de la interrupción
		EXTI->PR1 |= (1 << PinNumber);
	}
}
