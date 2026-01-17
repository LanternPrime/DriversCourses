/*
 * stm32f411retx_gpio_driver.c
 *
 *  Created on: 8 dic 2025
 *      Author: octav
 */


#include "stm32f411xx_gpio_driver.h"

//PCLK Setup
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PCLKCtrl(GPIOx_Reg_t *pGPIOx, uint8_t ENoDI){
	if(ENoDI == ENABLE){
		if(pGPIOx == GPIOA)
		{
			PA_PCLK_EN();
		}
		if(pGPIOx == GPIOB)
		{
			PB_PCLK_EN();
		}
		if(pGPIOx == GPIOC)
		{
			PC_PCLK_EN();
		}
		if(pGPIOx == GPIOD)
		{
			PD_PCLK_EN();
		}
		if(pGPIOx == GPIOE)
		{
			PE_PCLK_EN();
		}
		if(pGPIOx == GPIOH)
		{
			PH_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			PA_PCLK_DI();
		}
		if(pGPIOx == GPIOB)
		{
			PB_PCLK_DI();
		}
		if(pGPIOx == GPIOC)
		{
			PC_PCLK_DI();
		}
		if(pGPIOx == GPIOD)
		{
			PD_PCLK_DI();
		}
		if(pGPIOx == GPIOE)
		{
			PE_PCLK_DI();
		}
		if(pGPIOx == GPIOH)
		{
			PH_PCLK_DI();
		}
	}
}

//Init & Deinit
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	//Enable Peripheral Clock

	GPIO_PCLKCtrl(pGPIOHandle->pGPIOx, ENABLE);
	//Configure the Mode
	//Non Interrupt Mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		pGPIOHandle->pGPIOx->MODER &= ~(3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	}
	else
	{
		pGPIOHandle->pGPIOx->MODER &= ~(3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		pGPIOHandle->pGPIOx->MODER |= (GPIO_MODE_IN << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
		//Interrupt Mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			//1.Configure The FSTR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			//Clear the corresponding BSRT bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1.Configure The RSTR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			//Clear the corresponding FSRT bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1.Configure Both the FSTR & RSTR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		}

		//2. Configure the GPIO port in SYSCFG_EXTICR
		uint8_t exti_r = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum / 4;
		uint8_t exti_s = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum % 4;
		uint8_t portcode = GPIO_BASEADDR_2CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCKL_EN();
		SYSCFG->EXTICR[exti_r] |= (portcode << (exti_s * 4));

		//3.Enable the EXTI Interrupt Delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
	}

	//Configure the Speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));

	//Configure the PuPd Settings
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtlr << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));

	//Configure the optype
	pGPIOHandle->pGPIOx->OTYPER &= ~(3 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);

	//Configure the Alt Funct
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
	{
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << temp2);
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_DeInit(GPIOx_Reg_t *pGPIOx){
	if(pGPIOx == GPIOA)
	{
		PA_REG_RST();
	}
	if(pGPIOx == GPIOB)
	{
		PB_REG_RST();
	}
	if(pGPIOx == GPIOC)
	{
		PC_REG_RST();
	}
	if(pGPIOx == GPIOD)
	{
		PD_REG_RST();
	}
	if(pGPIOx == GPIOE)
	{
		PE_REG_RST();
	}
	if(pGPIOx == GPIOH)
	{
		PH_REG_RST();
	}
}

//Read & Write
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -

 */
uint8_t GPIO_ReadFromInputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNum){

	uint8_t value;

	value = (uint8_t) ((pGPIOx->IDR >> PinNum) & 0x00000001);

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - 0 or 1
 *
 * @Note              -

 */
uint16_t GPIO_ReadFromInputPort(GPIOx_Reg_t *pGPIOx){
	uint16_t value;

	value = (uint16_t) pGPIOx->IDR;

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WriteToOutputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNum, uint8_t value){
	if(value == SET)
		pGPIOx->ODR |= (1 << PinNum);
	else
		pGPIOx->ODR &= ~(1 << PinNum);
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WriteToOutputPort(GPIOx_Reg_t *pGPIOx, uint8_t value){
	pGPIOx->ODR = value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_ToggleOutputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNum){
	pGPIOx->ODR ^= (1 << PinNum);
}

//IRQ Config & Handling
/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority){
	uint8_t iprx = IRQNum / 4;
	uint8_t ipr_s = IRQNum % 4;

	uint8_t shift = (8 * ipr_s) + (8 - NO_PR_BITS_IMP);

	*(NVIC_IPR_ADDR + iprx) |= (IRQPriority << shift);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t ENoDI){

	if(ENoDI)
	{
		if(IRQNum <= 31)
			*NVIC_ISER0 |= (1 << IRQNum);
		else if( IRQNum > 31 && IRQNum < 64)
			*NVIC_ISER1 |= (1 << IRQNum % 32);
		else if( IRQNum >= 64 && IRQNum < 96)
			*NVIC_ISER2 |= (1 << IRQNum % 64);
	} else {
		if(IRQNum <= 31)
			*NVIC_ICER0 |= (1 << IRQNum);
		else if( IRQNum > 31 && IRQNum < 64)
			*NVIC_ICER1 |= (1 << IRQNum % 32);
		else if( IRQNum >= 64 && IRQNum < 96)
			*NVIC_ICER2 |= (1 << IRQNum % 64);
	}
}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t PinNum){
	if(EXTI->PR & (1 << PinNum))
		EXTI->PR |= (1 << PinNum);
}
