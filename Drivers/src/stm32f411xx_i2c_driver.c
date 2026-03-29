/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: 26 mar 2026
 *      Author: octav
 */

#include <stm32f411xx_i2c_driver.h>

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,
                                         uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHanderTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHanderRXNEInterrupt(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,
                                         uint8_t SlaveAddr) {
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr &= ~(1); // SlaveAddr is Slave Address + r/w bit = 0;
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,
                                        uint8_t SlaveAddr) {
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr |= 1; // SlaveAddr is Slave Address + r/w bit = 1;
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
    size_t dummyRead;
    // Check Device Mode
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
        // Master Mode
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
            if (pI2CHandle->RxSize == 1) {
                // Disable Acking
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

                // Clear ADDR Flag (read SR1, read SR2)
                dummyRead = pI2CHandle->pI2Cx->SR1;
                dummyRead = pI2CHandle->pI2Cx->SR2;
                (void)dummyRead;
            }
        } else {
            // Clear ADDR Flag (read SR1, read SR2)
            dummyRead = pI2CHandle->pI2Cx->SR1;
            dummyRead = pI2CHandle->pI2Cx->SR2;
            (void)dummyRead;
        }
    } else {
        // Slave Mode
        // Clear ADDR Flag (read SR1, read SR2)
        dummyRead = pI2CHandle->pI2Cx->SR1;
        dummyRead = pI2CHandle->pI2Cx->SR2;
        (void)dummyRead;
    }
}

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {

    if (EnorDi == ENABLE) {
        if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
    } else {
        // TODO
    }
}

#define FREQ16K 16000000
#define FREQ8K 8000000

uint16_t AHB_PreScalerTable[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScalerTable[4] = {2, 4, 8, 16};

uint32_t RCC_GetPLLOutputClk() {
    /*TODO*/
    return 0;
}

uint32_t RCC_GetPCLK1Value(void) {
    uint32_t pClk1, SysClk;
    uint8_t clksrc, tmp, AHB_Prescaler, APB1_Prescaler;

    clksrc = ((RCC->CFGR >> 2) & 0x3);
    switch (clksrc) {
    case 0:
        SysClk = FREQ16K;
        break;
    case 1:
        SysClk = FREQ8K;
        break;
    case 2:
        SysClk = RCC_GetPLLOutputClk();
        break;
    }

    // AHBP
    tmp = ((RCC->CFGR >> 4) & 0xF);

    if (tmp < 8) {
        AHB_Prescaler = 1;
    } else {
        AHB_Prescaler = AHB_PreScalerTable[tmp % 8];
    }

    // APB1
    tmp = ((RCC->CFGR >> 4) & 0xF);

    if (tmp < 4) {
        APB1_Prescaler = 1;
    } else {
        APB1_Prescaler = AHB_PreScalerTable[tmp % 4];
    }

    pClk1 = (SysClk / (AHB_Prescaler * APB1_Prescaler));

    return pClk1;
}

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {
    uint32_t tmpReg = 0;

    // peripheral clock enable
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    // ACK Control Bit
    tmpReg |= pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;
    pI2CHandle->pI2Cx->CR1 = tmpReg;

    // Configure FREQ field of CR2
    tmpReg = 0;
    tmpReg |= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tmpReg & 0x3F);

    // program the device own address
    tmpReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tmpReg |=
        (1
         << 14); // By Manual Reference: Should always be kept at 1 by software.
    pI2CHandle->pI2Cx->OAR1 = tmpReg;

    // CCR calculations
    uint16_t ccr_value = 0;
    tmpReg = 0;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        // mode is standard mode
        ccr_value =
            (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tmpReg |= (ccr_value & 0xFFF);
    } else {
        // mode is fast mode
        tmpReg |= (1 << 15);
        tmpReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr_value = (RCC_GetPCLK1Value() /
                         (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        } else {
            ccr_value = (RCC_GetPCLK1Value() /
                         (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        tmpReg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tmpReg;

    // TRISE Config
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        // mode is standard mode
        tmpReg = (RCC_GetPCLK1Value() * 1000000U) + 1;
    } else {
        // mode is fast mode
        tmpReg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
    }

    pI2CHandle->pI2Cx->TRISE = (tmpReg & 0x3F);
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    } else {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
    if (pI2Cx->SR1 & FlagName) {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,
                        uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    // 1. Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. Confirm that start generation is completed by checking the SB flag in
    // the SR1
    //  Note: Until SB is cleared SCL will be stretched (pulled to low)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
        ;
    // 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
    // 4. Confirm that address phase is completed by checking the ADDR flag in
    // the SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
        ;
    // 5. Clear the ADDR Flag according to its software sequence.
    //  Note: Until ADDR is cleared SCL will be stretched (pulled to low)
    I2C_ClearADDRFlag(pI2CHandle); // ADDR=1, cleared by reading SR1
                                   // register followed by reading SR2
    // 6. Send the data until len becomes 0
    while (Len > 0) {
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
            ; // Wait till TEX is SET
        pI2CHandle->pI2Cx->DR = *pTxbuffer;
        pTxbuffer++;
        Len--;
    }
    // 7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the
    // STOP condition
    //    Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next
    //    transmission should begin when BTF=1 SCL will be stretched (pulled to
    //    LOW)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
        ;
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
        ;

    // 8. Generate STOP condition and master need not to wait for the completion
    // of stop condition.
    //    Note: generating STOP, automatically clears the BTF
    if (Sr == I2C_DISABLE_SR)
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,
                           uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
    // 1. Generate the Start condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
    // 2. Confirm that start generation is completed by checking the SB flag in
    // the SR1 Note: Until SB is closed SCL will be stretched (pulled to LOW)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
        ;
    // 3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
    // 4. Wait until address phase is completed by checking the ADDR flag in the
    // SR1
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
        ;

    // Procedure to read only 1 byte from slave
    if (Len == 1) {
        // Disable Acking
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
        // Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);
        // Wait until RxNE becomes 1
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
            ;
        // Generate STOP condition
        if (Sr == I2C_DISABLE_SR)
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        // Read data in to buffer
        *(pTxbuffer) = pI2CHandle->pI2Cx->DR;
    }

    // Procedure to read data from slave when Len>1

    if (Len > 1) {
        // Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);
        // Read the data until Len becomes zero
        for (uint32_t i = Len; i > 0; i--) {
            // Wait until RxNE becomes 1
            while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
                ;
            if (i == 2) { // If last 2 bytes are remaining
                // Clear the ACK bit
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
                // Generate the STOP condition
                if (Sr == I2C_DISABLE_SR)
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }

            // Read the data from data register in to buffer
            *(pTxbuffer) = pI2CHandle->pI2Cx->DR;
            // Increment the buffer address
            pTxbuffer++;
        }
    }
    if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_DISABLE)
        // Re-Enable ACKing
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
                             uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        // Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // Implement the code to enable ITEVTEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        // Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                                uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize =
            Len; // Rxsize is used in the ISR code to manage the data reception
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        // Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // Implement the code to enable ITEVTEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        // Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
    // Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    // Implement the code to disable ITEVTEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;
    I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
    // Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    // Implement the code to disable ITEVTEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}

static void I2C_MasterHanderTXEInterrupt(I2C_Handle_t *pI2CHandle) {
    if (pI2CHandle->TxLen > 0) {
        // 1. Load the data in to DR
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
        // 2. Inclement the buffer Address
        pI2CHandle->pTxBuffer++;
        // 3. Decrement the Tx Len
        pI2CHandle->TxLen--;
    }
}

static void I2C_MasterHanderRXNEInterrupt(I2C_Handle_t *pI2CHandle) {
    // We have to do the Data Reception
    if (pI2CHandle->RxSize == 1) {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }
    if (pI2CHandle->RxSize > 1) {
        if (pI2CHandle->RxLen == 2) { // If last 2 bytes are remaining
            // Clear the ACK bit
            I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
        }
        // Read the data from data register in to buffer
        *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
        // Increment the buffer address
        pI2CHandle->pRxBuffer++;
    }

    if (pI2CHandle->RxSize == 0) {
        // Close the I2C data reception
        //  Generate the STOP condition
        if (pI2CHandle->Sr == I2C_DISABLE_SR)
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        // 2. Cloase the I2C Rx
        I2C_CloseReceiveData(pI2CHandle);
        // 3. Notifify  the Application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
    }
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == I2C_ACK_ENABLE)
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    else
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
}

/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi) {
    if (EnorDi) {
        if (IRQNum <= 31)
            *NVIC_ISER0 |= (1 << IRQNum);
        else if (IRQNum > 31 && IRQNum < 64)
            *NVIC_ISER1 |= (1 << IRQNum % 32);
        else if (IRQNum >= 64 && IRQNum < 96)
            *NVIC_ISER2 |= (1 << IRQNum % 64);
    } else {
        if (IRQNum <= 31)
            *NVIC_ICER0 |= (1 << IRQNum);
        else if (IRQNum > 31 && IRQNum < 64)
            *NVIC_ICER1 |= (1 << IRQNum % 32);
        else if (IRQNum >= 64 && IRQNum < 96)
            *NVIC_ICER2 |= (1 << IRQNum % 64);
    }
}

/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority) {
    uint8_t iprx = IRQNum / 4;
    uint8_t ipr_s = IRQNum % 4;

    uint8_t shift = (8 * ipr_s) + (NO_PR_BITS_IMP);

    *(NVIC_IPR_ADDR + iprx) |= (IRQPriority << shift);
}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
    // Interrupt handling for both master and slave mode of a device
    uint32_t tmp1, tmp2, tmp3;
    tmp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    tmp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
    // 1. Handle For interrupt generated by SB event
    //	Note : SB flag is only applicable in Master mode
    if (tmp1 && tmp3) {
        // This block will not be executed in slave mode because for slave SB is
        // always zero
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,
                                         pI2CHandle->DevAddr);
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
    }

    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
    // 2. Handle For interrupt generated by ADDR event
    // Note : When master mode : Address is sent
    //		 When Slave mode   : Address matched with own address
    if (tmp1 && tmp3) {
        // ADDR flag is set
        I2C_ClearADDRFlag(pI2CHandle);
    }
    // 3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    if (tmp1 && tmp3) {
        // BTF flag is set
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
            // make sure that TXE is also set.
            if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
                // BFT = 1, TXE = 1
                if (pI2CHandle->TxLen == 0) {
                    // 1. Generate Stop Condition
                    if (pI2CHandle->Sr == I2C_DISABLE_SR)
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    // 2. Reset All Members Elements of the Handle Structure.
                    I2C_CloseSendData(pI2CHandle);
                    // 3. Notifiy the application about transmission complete.
                    I2C_ApplicationEventCallback(pI2CHandle,
                                                 I2C_EV_TX_COMPLETE);
                }
            } else if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)) {
                ;
            }
        }
    }

    // 4. Handle For interrupt generated by STOPF event
    //  Note : Stop detection flag is applicable only slave mode . For master
    //  this flag will never be set
    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
    if (tmp1 && tmp3) {
        // STOPF flag is set
        // Clear the STOPF (i.e) read SR1 - Write to CR1
        pI2CHandle->pI2Cx->CR1 |= 0x0000;

        // Notify the application that STOP is detected
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    // 5. Handle For interrupt generated by TXE event
    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
    if (tmp1 && tmp2 && tmp3) {
        // Check Device Mode
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
            // TXE flag is set
            // We have to do the data transmission
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
                I2C_MasterHanderTXEInterrupt(pI2CHandle);
            }
        }
    }

    // 6. Handle For interrupt generated by RXNE event
    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
    if (tmp1 && tmp2 && tmp3) {
        // The Device is Master .
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
            // RXNE flag is set
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
                I2C_MasterHanderRXNEInterrupt(pI2CHandle);
            }
        }
    }
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

