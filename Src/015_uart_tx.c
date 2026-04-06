/*
 * 010USART2_master_tx_test.c
 *
 *  Created on: 27 mar 2026
 *      Author: octav
 */
#include "stm32f411xx.h"

//  extern void initialise_monitor_handles(void);

uint8_t rxComplete = RESET;

#define MY_ADDR    0x61
#define SLAVE_ADDR 0x68

#define LOW         0
#define BTN_PRESSED LOW

void delay(void)
{
    for (uint32_t i = 0; i < 1000000 / 2; i++);
}

USART_Handle_t USART6_Handler;
uint8_t tx_data[1024] = "Octavio Piña USART data\n";

/*
 * PA2 -> TX
 * PA3 -> RX
 * */

void USART6_GPIOInits(void)
{

    GPIO_Handle_t USART2Pins;
    USART2Pins.pGPIOx = GPIOA;
    USART2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
    USART2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 8;
    USART2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
    USART2Pins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_PIN_PU;
    USART2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;

    // TX
    USART2Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN11;
    GPIO_Init(&USART2Pins);

    // RX
    USART2Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN12;
    GPIO_Init(&USART2Pins);
}

void USART6_Inits(void)
{

    USART6_Handler.pUSARTx = USART6;
    USART6_Handler.USARTConfig.USART_Mode = USART_MODE_ONLY_TX;
    USART6_Handler.USARTConfig.USART_BaudRate = USART_STD_BAUD_115200;
    USART6_Handler.USARTConfig.USART_NoOfStopBits = USART_STOPBITS_1;
    USART6_Handler.USARTConfig.USART_WordLength = USART_WORDLEN_8BITS;
    USART6_Handler.USARTConfig.USART_ParityCtrl = USART_PARITY_DISABLE;
    USART6_Handler.USARTConfig.USART_HW_Flow = USART_HW_FLOW_CTRL_NONE;

    USART_Init(&USART6_Handler);
}

void GPIOBtn_Init(void)
{
    GPIO_Handle_t GPIOBtn;

    // this is btn gpio configuration
    GPIOBtn.pGPIOx = GPIOC;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN13;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);
}

int main(void)
{

    GPIOBtn_Init();
    USART6_GPIOInits();
    USART6_Inits();
    USART_PeripheralControl(USART6, ENABLE);

    while (1)
    {
        while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13));
        delay();
        USART_SendData(&USART6_Handler, tx_data, strlen((char*)tx_data));
    }

    return 0;
}
