/*
 * stm32f411retx_RCC_driver.c
 *
 *  Created on: 1/04/2026
 *      Author: octav
 */

#include <stm32f411xx_rcc_driver.h>

#define FREQ16Mhz 16000000
#define FREQ8Mhz  8000000

uint16_t AHB_PreScalerTable[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_PreScalerTable[4] = {2, 4, 8, 16};

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
    case 0: // HSI
        SysClk = FREQ16Mhz;
        break;
    case 1: // HSE
        SysClk = FREQ8Mhz;
        break;
    case 2: // PLL
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
        AHB_Prescaler = AHB_PreScalerTable[tmp - 8];
    }

    // APB1
    tmp = ((RCC->CFGR >> 10) & 0x7);

    if (tmp < 4)
    {
        APB1_Prescaler = 1;
    }
    else
    {
        APB1_Prescaler = APB_PreScalerTable[tmp - 4];
    }

    pClk1 = (SysClk / (AHB_Prescaler * APB1_Prescaler));

    return pClk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t SysClk = 0, tmp, pclk2;
    uint8_t clksrc = (RCC->CFGR >> 2) & 0X3;

    uint8_t AHB_Prescaler, APB2_Prescaler;

    switch (clksrc)
    {
    case 0: // HSI
        SysClk = FREQ16Mhz;
        break;
    case 1: // HSE
        SysClk = FREQ8Mhz;
        break;
    case 2: // PLL
        SysClk = RCC_GetPLLOutputClk();
        break;
    }

    tmp = (RCC->CFGR >> 4) & 0xF;

    if (tmp < 0x08)
    {
        AHB_Prescaler = 1;
    }
    else
    {
        AHB_Prescaler = AHB_PreScalerTable[tmp - 8];
    }

    tmp = (RCC->CFGR >> 13) & 0x7;
    if (tmp < 0x04)
    {
        APB2_Prescaler = 1;
    }
    else
    {
        APB2_Prescaler = APB_PreScalerTable[tmp - 4];
    }

    pclk2 = (SysClk / (AHB_Prescaler * APB2_Prescaler));

    return pclk2;
}