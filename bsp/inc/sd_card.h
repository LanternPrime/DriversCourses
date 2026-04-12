/*
 * sd_card.h
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_
#include "stm32f411xx.h"

typedef enum
{
    SD_OK = 0,
    SD_ERROR,
    SD_ERR_CMD0,
    SD_ERR_CMD8,
    SD_ERR_CMD16,
    SD_TIMEOUT,
    SD_ERR_ACMD41
} SD_Status_t;

typedef struct
{
    uint8_t type;
    uint8_t addressing_mode;
    uint32_t ocr;
} SD_CardInfo_t;

SD_Status_t SDcard_init(SD_CardInfo_t *card);
void SD_SendInitialClockTrain(void);
void SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SD_WaitByte(uint8_t expected, uint32_t timeout);
SD_Status_t SD_GoIdleState(void);
SD_Status_t SD_SendIfCond(uint8_t *R7);
SD_Status_t SD_SendAppOpCond(void);
SD_Status_t SD_ReadOCR(uint8_t *R3, uint32_t *ocr);

SD_Status_t SD_SetBlockLen(uint32_t len);
SD_Status_t SD_ReadSingleBlock(SD_CardInfo_t *sd_Handle, uint32_t addr, uint8_t *buffer);
SD_Status_t SD_WriteSingleBlock(SD_CardInfo_t *sd_Handle, uint32_t addr, uint8_t *buffer);

// TYPE
#define SD_CARD_SDHC    1 // HIGH CAPACITY SD CARD
#define SD_CARD_SDSC_V2 0 // STANDARD CAPACITY SD CARD
// ADDRESSING MODE
#define SD_ADDR_BYTE  0
#define SD_ADDR_BLOCK 1

#define SD_CARD_SPI SPI1

#define SD_ARG_INIT 0x00000000
#define SD_ARG_C8   0x000001AA
#define SD_ARG_A41  0x40000000

#define SD_ACMD41 0x29
#define SD_CMD0   0x00
#define SD_CMD8   0x08
#define SD_CMD16  0x10
#define SD_CMD17  0x11
#define SD_CMD24  0x18
#define SD_CMD55  0x37
#define SD_CMD58  0x3A

#define SD_CRC1    0x95
#define SD_CRC2    0x87
#define SD_CRCDMMY 0x01

#define SD_R1_IDLE  0x01
#define SD_R1_READY 0x00
#define SD_CMD8_OK  0xAA

#define SD_WRITE_TOKEN 0xFE

#endif /* INC_SD_CARD_H_ */
