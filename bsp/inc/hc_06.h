/*
 * hc_05.c
 *
 *  Created on: 15 abr 2026
 *      Author: octav
 */

#ifndef INC_HC06_H_
#define INC_HC06_H_

#include "stm32f411xx.h"
#define CMD_BUFFER_LEN 32
typedef struct
{
    USART_Handle_t pUSARTHandle;
} BT_Handler_t;

BT_Handler_t BT_init(void);
uint8_t BT_SendByte(BT_Handler_t btHandler, uint8_t data);
uint8_t BT_ReadByte(BT_Handler_t *btHandler);
void BT_SendString(BT_Handler_t btHandler, uint8_t *str);

#endif /*INC_HC06_H_*/
