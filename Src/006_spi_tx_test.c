/*
 * 006_spi_tx_test.c
 *
 *  Created on: 21 dic 2025
 *      Author: octav
 */

/*
 * 		SPI3
 *		PA4 NSS
 *		PB3 SCK
 *		PB4 MISO
 *		PB5 MOSI
 *		ALT MODE: 06
 * */
#include <string.h>
#include "stm32f411xx.h"

void SPI3_GPIOInits(void){

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 6;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCKL
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.pGPIOx = GPIOA;
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	//GPIO_Init(&SPIPins);

}

void SPI3_Inits(void){

	SPI_Handle_t SPI3Handler;

	SPI3Handler.pSPIx = SPI3;
	SPI3Handler.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI3Handler.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI3Handler.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // Generates SCLK of 8MHz
	SPI3Handler.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI3Handler.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI3Handler.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI3Handler.SPIConfig.SPI_SSM = SPI_SSM_EN; //SSM Enabled for NSS Pin

	SPI_Init(&SPI3Handler);
}

int main(void){

	char user_data[] = "Hello World";

	SPI3_GPIOInits();

	//This function is used to initialize the SPI3 peripheral parameters
	SPI3_Inits();

	//This makes NSS signal internally HIGH and avoids MODF error
	SPI_SSIConfig(SPI3, ENABLE);

	//Enable the SPI3 Peripheral
	SPI_PeripheralControl(SPI3, ENABLE);

	//SendData
	SPI_SendData(SPI3, (uint8_t*)user_data, strlen(user_data));


	//Disable the SPI3 Peripheral
	SPI_PeripheralControl(SPI3, DISABLE);
	while(1);

	return 0;
}
