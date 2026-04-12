/*
 * sd_card.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */

#include "../bsp/inc/sd_card.h"
static void mdelay(uint32_t cnt);

static void mdelay(uint32_t cnt)
{
    for (size_t i = 0; i < (cnt * 5000); i++);
}

SPI_Handle_t SPI1Handler;

void SPI1_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;

    // SCKL
    SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN5;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN7;
    GPIO_Init(&SPIPins);

    // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN6;
    GPIO_Init(&SPIPins);

    // CS
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

    SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN4;
    GPIO_Init(&SPIPins);
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
}

void SPI1_init(void)
{
    SPI1_GPIOInits();
    SPI1Handler.pSPIx = SPI1;
    SPI1Handler.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1Handler.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI1Handler.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256; // Generates SCLK of 62.5kHz
    SPI1Handler.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI1Handler.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI1Handler.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI1Handler.SPIConfig.SPI_SSM = SPI_SSM_EN; // SSM DISABLED

    SPI_Init(&SPI1Handler);

    // This makes NSS signal internally HIGH and avoids MODF error
    SPI_SSIConfig(SPI1, ENABLE);
    // Enable the SPI3 Peripheral
    SPI_PeripheralControl(SPI1, ENABLE);
}

uint8_t SDcard_init(SD_CardInfo_t *card)
{
    uint8_t res, timeout = 10;
    uint8_t response_data[4] = {0}, ccs;
    uint32_t ocr = 0;

    SPI1_init();
    mdelay(10);
    SD_SendInitialClockTrain();
    res = SD_GoIdleState();
    if (res != SD_R1_IDLE)
        return SD_ERR_CMD0;

    res = SD_SendIfCond(response_data);
    if (response_data[3] != SD_CMD8_OK || res != SD_R1_IDLE)
        return SD_ERR_CMD8;

    do
    {
        res = SD_SendAppOpCond();
    } while ((res == SD_R1_IDLE) && (--timeout > 0)); // TODO: SD VER 1. MMC Ver 3

    if (res == SD_R1_IDLE)
        return SD_TIMEOUT;
    else if (res != SD_R1_READY)
        return SD_ERR_ACMD41;

    res = SD_ReadOCR(response_data, &ocr);
    if (res != SD_R1_READY)
        return SD_ERROR;

    ccs = (ocr >> 30) & 1;
    card->ocr = ocr;
    if (ccs)
    {
        card->addressing_mode = SD_ADDR_BLOCK;
        card->type = SD_CARD_SDHC;

        return SD_OK;
    }
    else
    {
        card->addressing_mode = SD_ADDR_BYTE;
        card->type = SD_CARD_SDSC_V2;
        res = SD_SetBlockLen(512);

        if (res != SD_R1_READY)
            return SD_ERR_CMD16;

        return SD_OK;
    }
}

void SD_SendInitialClockTrain(void)
{
    // 1 . Wait 1 ms
    mdelay(10);
    /* 2 .>= 74 dummy clocks */
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET); // Deselect SD card
    for (size_t i = 0; i < 10; i++)
    {
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
    }
}

void SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    SPI_TransferByte(SPI1Handler.pSPIx, 0x40 | cmd);
    SPI_TransferByte(SPI1Handler.pSPIx, (uint8_t)(arg >> 24));
    SPI_TransferByte(SPI1Handler.pSPIx, (uint8_t)(arg >> 16));
    SPI_TransferByte(SPI1Handler.pSPIx, (uint8_t)(arg >> 8));
    SPI_TransferByte(SPI1Handler.pSPIx, (uint8_t)(arg >> 0));
    SPI_TransferByte(SPI1Handler.pSPIx, crc);
}

uint8_t SD_GoIdleState(void) // CMD0
{
    uint8_t res;
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_RESET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    SD_SendCommand(SD_CMD0, SD_ARG_INIT, SD_CRC1);

    res = SD_WaitByte(0x01, 50);

    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    return res;
}

uint8_t SD_SendIfCond(uint8_t *R7) // CMD8
{
    uint8_t r1;
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_RESET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    SD_SendCommand(SD_CMD8, SD_ARG_C8, SD_CRC2);

    r1 = SD_WaitByte(0x01, 50);
    if (r1 == SD_R1_IDLE)
    {
        for (size_t i = 0; i < 4; i++)
        {
            R7[i] = SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
        }
    }

    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    return r1;
}

uint8_t SD_SendAppOpCond(void) // CMD55 && ACMD41
{
    uint8_t res;
    uint8_t timeout = 25;
    do
    {
        // CMD55
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_RESET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

        SD_SendCommand(SD_CMD55, SD_ARG_INIT, SD_CRCDMMY);

        res = SD_WaitByte(0x01, 50);

        if ((res != SD_R1_IDLE) && (res != SD_R1_READY))
        {
            return res; // If CMD55 Fails, return the response
        }

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

        // ACMD41
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_RESET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

        SD_SendCommand(SD_ACMD41, SD_ARG_A41, SD_CRCDMMY);

        res = SD_WaitByte(0x00, 50);

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
    } while (res != 0x00 && timeout-- > 0);

    return res;
}

uint8_t SD_ReadOCR(uint8_t *R3, uint32_t *ocr) // OCR
{
    uint8_t res;
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_RESET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    SD_SendCommand(SD_CMD58, SD_ARG_INIT, SD_CRCDMMY);

    res = SD_WaitByte(SD_R1_READY, 50);
    if (res == SD_R1_READY)
    {
        for (size_t i = 0; i < 4; i++)
        {
            R3[i] = SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
        }
    }

    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    *ocr = ((uint32_t)R3[0] << 24 |
            (uint32_t)R3[1] << 16 |
            (uint32_t)R3[2] << 8 |
            (uint32_t)R3[3] << 0);
    return res;
}

uint8_t SD_SetBlockLen(uint32_t len) // CMD16
{
    uint8_t res;
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_RESET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    SD_SendCommand(SD_CMD16, len, SD_CRC1);

    res = SD_WaitByte(SD_R1_READY, 50);

    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    return res;
}

uint8_t SD_ReadSingleBlock(SD_CardInfo_t *sd_Handle, uint32_t addr, uint8_t *buffer) // CMD17
{
    uint8_t res;
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_RESET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    if (sd_Handle->addressing_mode == SD_ADDR_BYTE)
        SD_SendCommand(SD_CMD17, addr * 512, SD_CRCDMMY);
    else
        SD_SendCommand(SD_CMD17, addr, SD_CRCDMMY);

    res = SD_WaitByte(SD_R1_READY, 50);
    if (res != SD_R1_READY)
    {
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
        return SD_ERROR;
    }

    res = SD_WaitByte(0xFE, 50); // Wait 0xFE Data Token
    if (res != 0xFE)
    {
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
        return SD_ERROR;
    }

    for (size_t i = 0; i < 512; i++)
    {
        buffer[i] = SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
    }

    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF); // CRC
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF); // CRC 2

    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    return SD_OK;
}

uint8_t SD_WriteSingleBlock(SD_CardInfo_t *sd_Handle, uint32_t addr, uint8_t *buffer) // CMD17
{
    uint8_t res, timeout = 25;
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_RESET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);

    if (sd_Handle->addressing_mode == SD_ADDR_BYTE)
        SD_SendCommand(SD_CMD24, addr * 512, SD_CRCDMMY);
    else
        SD_SendCommand(SD_CMD24, addr, SD_CRCDMMY);

    res = SD_WaitByte(SD_R1_READY, 50);
    if (res != SD_R1_READY)
    {
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
        return SD_ERROR;
    }
    // One Dummy byte before data token
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
    // Send start block token
    SPI_TransferByte(SPI1Handler.pSPIx, SD_WRITE_TOKEN); // Write Token
    // Send 512-byte data block
    for (size_t i = 0; i < 512; i++)
    {
        SPI_TransferByte(SPI1Handler.pSPIx, buffer[i]);
    }
    // Send dummy CRC
    SPI_TransferByte(SPI1Handler.pSPIx, SD_CRCDMMY);
    SPI_TransferByte(SPI1Handler.pSPIx, SD_CRCDMMY);

    // Read data response
    uint8_t data_resp = SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
    if ((data_resp & 0x1F) != 0x05) // Data accepted
    {
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
        return SD_ERROR;
    }

    // Wait until SD not BSY
    do
    {
        res = SD_WaitByte(0xFF, 50000);
    } while (res != 0xFF && timeout-- > 0);
    if (res != 0xFF)
    {
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
        SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
        return SD_ERROR;
    }

    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN4, GPIO_PIN_SET);
    SPI_TransferByte(SPI1Handler.pSPIx, 0xFF);
    return SD_OK;
}

uint8_t SD_WaitByte(uint8_t expected, uint32_t timeout)
{
    uint8_t res = 0xFF;

    while (timeout--)
    {
        res = SPI_TransferByte(SPI1, 0xFF);

        if (res != 0xFF || res == expected)
        {
            break;
        }
    }

    return res;
}
