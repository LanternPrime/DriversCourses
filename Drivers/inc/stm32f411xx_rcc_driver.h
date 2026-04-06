/*
 * stm32f411retx_RCC_driver.h
 *
 *  Created on: 1/04/2026
 *      Author: octav
 */

#ifndef INC_STM32F411RETX_RCC_DRIVER_H_
#define INC_STM32F411RETX_RCC_DRIVER_H_

#include "stm32f411xx.h"
// Get APB1 Clock Value
uint32_t RCC_GetPCLK1Value(void);
// Get APB2 Clock Value
uint32_t RCC_GetPCLK2Value(void);

#endif /*INC_STM32F411RETX_RCC_DRIVER_H_*/