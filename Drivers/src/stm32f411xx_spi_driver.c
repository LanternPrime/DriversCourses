/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Feb 9, 2019
 *      Author: Octavio Piña
 */

#include "stm32f411xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
        // TODO
    }
}

/*********************************************************************
 * @fn      		  - SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

    // peripheral clock enable

    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // first lets configure the SPI_CR1 register

    uint32_t tempreg = 0;

    // 1. configure the device mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. Configure the bus config
    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // bidi mode should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // bidi mode should be set
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConfig.SPI_BusConfig ==
             SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // BIDI mode should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        // RXONLY bit must be set
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Configure the spi serial clock speed (baud rate)
    tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4.  Configure the DFF
    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. configure the CPOL
    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    // 6 . configure the CPHA
    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    // todo
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if (pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. Wait utill TXE (TX Empty) is set
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // 2. Check the DFF bit in CR1
        if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
        {
            // 16 bits DFF
            //  1.load the data into the DR
            pSPIx->DR = *((uint16_t *)pTxBuffer);
            Len -= 2;
            pTxBuffer += 2;
        }
        else
        {
            // 8bits DFF
            *((__volatile uint8_t *)&pSPIx->DR) = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. Wait utill RXNE (RX Not Empty) is set
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        // 2. Check the DFF bit in CR1
        if ((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
        {
            // 16 bits DFF
            //  1.load the data from DR to RxBuffer address
            *((uint16_t *)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            pRxBuffer += 2;
        }
        else
        {
            // 8bits DFF
            *pRxBuffer = *((__volatile uint8_t *)&pSPIx->DR);
            Len--;
            pRxBuffer++;
        }
    }
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call

 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
                       uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;
    if (state != SPI_BUSY_IN_TX)
    {
        // 1. Save the TX buffer address and Len information in some global
        // variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        // 2. Mark the SPI state as busy in transmission so that
        //  no other code can take over same SPI peripheral untill transmission
        //  is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;
        // 3.Enable the TXEIE control bit to get interrupt whenever TEX flag is
        // set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }
    return state;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
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
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
                          uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;
    if (state != SPI_BUSY_IN_RX)
    {
        // 1. Save the TX buffer address and Len information in some global
        // variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;
        // 2. Mark the SPI state as busy in transmission so that
        //  no other code can take over same SPI peripheral untill transmission
        //  is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;
        // 3.Enable the TXEIE control bit to get interrupt whenever TEX flag is
        // set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }
    return state;
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
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
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi)
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
 * @fn      		  - SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNum / 4;
    uint8_t ipr_s = IRQNum % 4;

    uint8_t shift = (8 * ipr_s) + (8 - NO_PR_BITS_IMP);

    *(NVIC_IPR_ADDR + iprx) |= (IRQPriority << shift);
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
    uint8_t temp1, temp2;
    // check for TXE
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if (temp1 && temp2)
    {
        // handle TXE
        spi_txe_interrupt_handle(pHandle);
    }

    // check for RXNE
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if (temp1 && temp2)
    {
        // handle RXNE
        spi_rxne_interrupt_handle(pHandle);
    }

    // check for OVR
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if (temp1 && temp2)
    {
        // handle RXNE
        spi_ovr_interrupt_handle(pHandle);
    }
}

// some helper function implementations

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
    {
        // 16 bits DFF
        //  1.load the data into the DR
        pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer += 2;
    }
    else
    {
        // 8bits DFF
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if (!pSPIHandle->TxLen)
    {
        SPI_CloseTransmisson(pSPIHandle);

        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        // 16 bits DFF
        *((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer += 2;
    }
    else
    {
        // 8 bits DFF
        *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if (!pSPIHandle->RxLen)
    {
        SPI_CloseReception(pSPIHandle);

        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

    uint8_t tmp;
    // Clear the OVR Flag
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        tmp = pSPIHandle->pSPIx->DR;
        tmp = pSPIHandle->pSPIx->SR;
    }
    (void)tmp;
    // Inform the Application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
    // TXLEN = 0, Close SPI Transmission and inform TX Over

    // This prevents interrupts from setting up of TXE Flag
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    // Reception is complete
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    uint8_t tmp;
    // Clear the OVR Flag
    tmp = pSPIx->DR;
    tmp = pSPIx->SR;
    (void)tmp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,
                                         uint8_t AppEv)
{
    // This is a weak implementation. User Application may OVERRIDE this
    // funtion.
}