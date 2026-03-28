/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: 26 mar 2026
 *      Author: octav
 */

#include <stm32f411xx_i2c_driver.h>

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); // SlaveAddr is Slave Address + r/w bit = 0;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; // SlaveAddr is Slave Address + r/w bit = 1;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	size_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		// TODO
	}
}

#define FREQ16K 16000000
#define FREQ8K 8000000

uint16_t AHB_PreScalerTable[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScalerTable[4] = {2, 4, 8, 16};

uint32_t RCC_GetPLLOutputClk()
{
	/*TODO*/
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pClk1, SysClk;
	uint8_t clksrc, tmp, AHB_Prescaler, APB1_Prescaler;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	switch (clksrc)
	{
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

	if (tmp < 8)
	{
		AHB_Prescaler = 1;
	}
	else
	{
		AHB_Prescaler = AHB_PreScalerTable[tmp % 8];
	}

	// APB1
	tmp = ((RCC->CFGR >> 4) & 0xF);

	if (tmp < 4)
	{
		APB1_Prescaler = 1;
	}
	else
	{
		APB1_Prescaler = AHB_PreScalerTable[tmp % 4];
	}

	pClk1 = (SysClk / (AHB_Prescaler * APB1_Prescaler));

	return pClk1;
}

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
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
	tmpReg |= (1 << 14); // By Manual Reference: Should always be kept at 1 by software.
	pI2CHandle->pI2Cx->OAR1 = tmpReg;

	// CCR calculations
	uint16_t ccr_value = 0;
	tmpReg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tmpReg |= (ccr_value & 0xFFF);
	}
	else
	{
		// mode is fast mode
		tmpReg |= (1 << 15);
		tmpReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tmpReg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tmpReg;

	// TRISE Config
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		tmpReg = (RCC_GetPCLK1Value() * 1000000U) + 1;
	}
	else
	{
		// mode is fast mode
		tmpReg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tmpReg & 0x3F);
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	//  Note: Until SB is cleared SCL will be stretched (pulled to low)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;
	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;
	// 5. Clear the ADDR Flag according to its software sequence.
	//  Note: Until ADDR is cleared SCL will be stretched (pulled to low)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx); // ADDR=1, cleared by reading SR1 register followed by reading SR2
	// 6. Send the data until len becomes 0
	while (Len > 0)
	{
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
			; // Wait till TEX is SET
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	// 7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//    Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//    when BTF=1 SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
		;
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
		;

	// 8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//    Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate the Start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is closed SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;
	// 3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	// 4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;
	// Procedure to read only 1 byte from slave

	if (Len == 1)

	{
		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		// Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Wait until RxNE becomes 1
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
			;
		// Read data in to buffer
		*(pTxbuffer) = pI2CHandle->pI2Cx->DR;
		return;
	}

	// Procedure to read data from slave when Len>1

	if (Len > 1)
	{
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Read the data until Len becomes zero
		for (uint32_t i = Len; i > 0; i--)
		{
			// Wait until RxNE becomes 1
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
				;
			if (i == 2)
			{ // If last 2 bytes are remaining
				// Clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				// Generate the STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// Read the data from data register in to buffer
			*(pTxbuffer) = pI2CHandle->pI2Cx->DR;
			// Increment the buffer address
			pTxbuffer++;
		}
	}
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_DISABLE)
	// Re-Enable ACKing
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	else
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
}
