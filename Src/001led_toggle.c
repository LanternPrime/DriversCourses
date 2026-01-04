/*
 * 001led_toggle.c
 *
 *  Created on: Feb 1, 2019
 *      Author: admin
 */

#include "stm32f411xx.h"


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void)
{

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_PIN_PU;

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN5);
		delay();
	}
	return 0;
}
