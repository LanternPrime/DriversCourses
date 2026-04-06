/*
 * 002led_button.c
 *
 *  Created on: Feb 1, 2019
 *      Author: admin
 */

#include "stm32f411xx.h"

#define LOW         0
#define BTN_PRESSED LOW

void delay(void)
{
    for (uint32_t i = 0; i < 1000000 / 2; i++);
}

int main(void)
{

    GPIO_Handle_t GpioLed, GPIOBtn;

    // this is led gpio configuration
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

    GPIO_Init(&GpioLed);

    // this is btn gpio configuration
    GPIOBtn.pGPIOx = GPIOC;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN13;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);

    while (1)
    {
        while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13));
        delay();
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
    }
    return 0;
}
