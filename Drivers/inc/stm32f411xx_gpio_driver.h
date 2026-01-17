/*
 * stm32f411retx_gpio_driver.h
 *
 *  Created on: 8 dic 2025
 *      Author: octav
 */

#ifndef INC_STM32F411RETX_GPIO_DRIVER_H_
#define INC_STM32F411RETX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNum;			/*!< Possible Values for @GPIO_PIN_NUM >*/
	uint8_t GPIO_PinMode;			/*!< Possible Values for @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< Possible Values for @GPIO_PIN_SPEEDS >*/
	uint8_t GPIO_PinPuPdCtlr;		/*!< Possible Values for @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOPType;			/*!< Possible Values for @GPIO_PIN_OPT >*/
	uint8_t GPIO_PinAltFunMode;		/*!< Possible Values for @GPIO_PIN_MODES >*/
}GPIO_PinConfig_t;

typedef struct
{
	GPIOx_Reg_t 		*pGPIOx; //Holds the base addr of the GPIO port to which pin belongs
	GPIO_PinConfig_t 	GPIO_PinConfig;
}GPIO_Handle_t;
/*
 * @GPIO_PIN_NUM
 * GPIO Pin Possible Numbers
 * */
#define GPIO_PIN0			0
#define GPIO_PIN1			1
#define GPIO_PIN2			2
#define GPIO_PIN3			3
#define GPIO_PIN4			4
#define GPIO_PIN5			5
#define GPIO_PIN6			6
#define GPIO_PIN7			7
#define GPIO_PIN8			8
#define GPIO_PIN9			9
#define GPIO_PIN10		10
#define GPIO_PIN11		11
#define GPIO_PIN12		12
#define GPIO_PIN13		13
#define GPIO_PIN14		14
#define GPIO_PIN15		15

/*
 * @GPIO_PIN_MODES
 * GPIO Pin Possible Modes
 * */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFUN	2
#define GPIO_MODE_ANALOG	3

#define GPIO_MODE_IT_FT	4
#define GPIO_MODE_IT_RT	5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_OPT
 * GPIO pin possible output types
 * */
#define GPIO_OPT_PP		0
#define GPIO_OPT_OD		1

/*
 * @GPIO_PIN_SPEEDS
 * GPIO Pin Possible Speeds
 * */
#define GPIO_SPEED_LOW	0
#define GPIO_SPEED_MED	1
#define GPIO_SPEED_FST	2
#define GPIO_SPEED_VFST	3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin Pull up & Pull Down configuration macros
 * */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU		1
#define GPIO_PIN_PD		2


// APIs supported by this driver

//PCLK Setup
void GPIO_PCLKCtrl(GPIOx_Reg_t *pGPIOx, uint8_t ENoDI);

//Init & Deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIOx_Reg_t *pGPIOx);

//Read & Write
uint8_t GPIO_ReadFromInputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIOx_Reg_t *pGPIO);
void GPIO_WriteToOutputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNum, uint8_t value);
void GPIO_WriteToOutputPort(GPIOx_Reg_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNum);

//IRQ Config & Handling
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t ENoDI);
void GPIO_IRQHandling(uint8_t PinNum);

#endif /* INC_STM32F411RETX_GPIO_DRIVER_H_ */
