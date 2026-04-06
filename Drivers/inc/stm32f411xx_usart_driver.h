/*
 * stm32f411retx_usart_driver.h
 *
 *  Created on: 31/03/2026
 *      Author: octav
 */

#ifndef INC_STM32F411RETX_USART_DRIVER_H_
#define INC_STM32F411RETX_USART_DRIVER_H_

#include "stm32f411xx.h"

/*
 *  Configuration structure for USARTx peripheral
 */
typedef struct
{
    uint8_t USART_Mode;
    uint32_t USART_BaudRate;
    uint8_t USART_NoOfStopBits;
    uint8_t USART_WordLength;
    uint8_t USART_ParityCtrl;
    uint8_t USART_HW_Flow;
} USART_Config_t;
typedef struct
{
    USART_RegDef_t *pUSARTx; /*!< This holds the base address of USARTx(x:1,2,6) peripheral >*/
    USART_Config_t USARTConfig;
    uint8_t *pTxBuffer;  // Store the Tx Buffer Address
    uint8_t *pRxBuffer;  // Store the Rx Buffer Address
    uint32_t TxLen;      // Store Tx Len
    uint32_t RxLen;      // Store Rx Len
    uint8_t TxRxState;   // Communication State
    uint8_t TxBusyState; // TX BUSY State
    uint8_t RxBusyState; // RX BUSY State
} USART_Handle_t;

/*
 * USART application states
 */
#define USART_READY      0
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX    2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200   1200
#define USART_STD_BAUD_2400   2400
#define USART_STD_BAUD_9600   9600
#define USART_STD_BAUD_19200  19200
#define USART_STD_BAUD_38400  38400
#define USART_STD_BAUD_57600  57600
#define USART_STD_BAUD_115200 115200
#define USART_STD_BAUD_230400 230400
#define USART_STD_BAUD_460800 460800
#define USART_STD_BAUD_921600 921600
#define USART_STD_BAUD_2M     2000000
#define SUART_STD_BAUD_3M     3000000

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD  2
#define USART_PARITY_EN_EVEN 1
#define USART_PARITY_DISABLE 0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS 0
#define USART_WORDLEN_9BITS 1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1   0
#define USART_STOPBITS_0_5 1
#define USART_STOPBITS_2   2
#define USART_STOPBITS_1_5 3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    0
#define USART_HW_FLOW_CTRL_CTS     1
#define USART_HW_FLOW_CTRL_RTS     2
#define USART_HW_FLOW_CTRL_CTS_RTS 3

/*
 *@USART_OVER8
 *Possible options for USART_Oversampling
 */
#define OVERSAMPLING_8  8
#define OVERSAMPLING_16 16

/*
 * @USART related status flags definitions
 */

#define USART_EVENT_TX_CMPLT 0
#define USART_EVENT_RX_CMPLT 1
#define USART_EVENT_CTS      2
#define USART_EVENT_IDLE     3
#define USART_EVENT_ORE      4
#define USART_ERREVENT_ORE   6
#define USART_ERREVENT_NE    7
#define USART_ERREVENT_FE    8
#define USART_TXE_FLAG       (1 << USART_SR_TXE)
#define USART_TC_FLAG        (1 << USART_SR_TC)
#define USART_RXNE_FLAG      (1 << USART_SR_RXNE)

/******************************************************************************************
 *								APIs supported
 * by this driver For more information about the APIs check the function
 * definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USARTClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);

#endif /*INC_STM32F411RETX_USART_DRIVER_H_*/
