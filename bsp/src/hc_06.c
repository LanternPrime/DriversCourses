/*
 * hc_05.h
 *
 *  Created on: 15 abr 2026
 *      Author: octav
 */

#include "hc_06.h"

void USART6_PinConfig(void)
{
    GPIO_Handle_t USART6Handler;
    memset(&USART6Handler, 0, sizeof(USART6Handler));
    USART6Handler.pGPIOx = GPIOC;
    USART6Handler.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
    USART6Handler.GPIO_PinConfig.GPIO_PinAltFunMode = 8;
    USART6Handler.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
    USART6Handler.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_PIN_PU;
    USART6Handler.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;

    // TX
    USART6Handler.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN6;
    GPIO_Init(&USART6Handler);

    // RX
    USART6Handler.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN7;
    GPIO_Init(&USART6Handler);
}

USART_Handle_t USART6_Inits(void)
{
    USART_Handle_t pUSARTx;
    pUSARTx.pUSARTx = USART6;
    pUSARTx.USARTConfig.USART_Mode = USART_MODE_TXRX;
    pUSARTx.USARTConfig.USART_BaudRate = USART_STD_BAUD_9600;
    pUSARTx.USARTConfig.USART_NoOfStopBits = USART_STOPBITS_1;
    pUSARTx.USARTConfig.USART_WordLength = USART_WORDLEN_8BITS;
    pUSARTx.USARTConfig.USART_ParityCtrl = USART_PARITY_DISABLE;
    pUSARTx.USARTConfig.USART_HW_Flow = USART_HW_FLOW_CTRL_NONE;

    USART_Init(&pUSARTx);
    // Enable USART Interrupt
    USART_IRQInterruptConfig(IRQ_NO_USART6, ENABLE);
    // Enable USART PERIPHERAL
    USART_PeripheralControl(USART6, ENABLE);

    return pUSARTx;
}

BT_Handler_t BT_init(void)
{
    BT_Handler_t btHandler;
    // Init the USART6 Pins
    USART6_PinConfig();
    // Initialize the USART6 Peripheral
    btHandler.pUSARTHandle = USART6_Inits();

    return btHandler;
}

uint8_t BT_SendByte(BT_Handler_t btHandler, uint8_t data)
{
    USART_SendData(&btHandler.pUSARTHandle, &data, 1);
    return 0;
}

uint8_t BT_ReadByte(BT_Handler_t *btHandler)
{
    uint8_t rx;
    USART_ReceiveDataIT(&btHandler->pUSARTHandle, &rx, 1);
    return rx;
}

void BT_SendString(BT_Handler_t btHandler, uint8_t *str)
{
    while (*str)
    {
        BT_SendByte(btHandler, *str);
        str++;
    }
}