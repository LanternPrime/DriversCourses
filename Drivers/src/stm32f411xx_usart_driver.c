/*
 * stm32f411retx_usart_driver.c
 *
 *  Created on: 31/03/2026
 *      Author: octav
 */

#include "stm32f411xx_usart_driver.h"

/*********************************************************************
 * @fn      		  - USART_PeriClockControl
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
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_EN();
        }
        else if (pUSARTx == USART6)
        {
            USART6_PCLK_EN();
        }
    }
    else
    {
        // TODO
    }
}

/*********************************************************************
 * @fn      		  - USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi)
{
    if (EnorDi)
    {
        if (IRQNum <= 31)
            *NVIC_ISER0 |= (1 << IRQNum);
        else if (IRQNum > 31 && IRQNum < 64)
            *NVIC_ISER1 |= (1 << IRQNum % 32);
        else if (IRQNum >= 64 && IRQNum < 96)
            *NVIC_ISER2 |= (1 << IRQNum % 64);
    }
    else
    {
        if (IRQNum <= 31)
            *NVIC_ICER0 |= (1 << IRQNum);
        else if (IRQNum > 31 && IRQNum < 64)
            *NVIC_ICER1 |= (1 << IRQNum % 32);
        else if (IRQNum >= 64 && IRQNum < 96)
            *NVIC_ICER2 |= (1 << IRQNum % 64);
    }
}

/*********************************************************************
 * @fn      		  - USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNum / 4;
    uint8_t ipr_s = IRQNum % 4;

    uint8_t shift = (8 * ipr_s) + (8 - NO_PR_BITS_IMP);

    *(NVIC_IPR_ADDR + iprx) |= (IRQPriority << shift);
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
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
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pUSARTx->CR1 |= (1 << USART_CR1_UE);
    }
    else
    {
        pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
    }
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
    if (pUSARTx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}