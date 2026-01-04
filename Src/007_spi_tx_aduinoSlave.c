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

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void SPI3_GPIOInits(void){

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 6;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;

	//SCKL
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN3;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN5;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN4;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN4;
	GPIO_Init(&SPIPins);

}

void SPI3_Inits(void){

	SPI_Handle_t SPI3Handler;

	SPI3Handler.pSPIx = SPI3;
	SPI3Handler.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI3Handler.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI3Handler.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // Generates SCLK of 2MHz
	SPI3Handler.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI3Handler.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI3Handler.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI3Handler.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware Slave Management Enabled for NSS Pin

	SPI_Init(&SPI3Handler);
}

void GPIOBtn_Init(void)
{
	GPIO_Handle_t GPIOBtn;
	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main(void){

	char user_data[] = "Hello World";

	GPIOBtn_Init();

	SPI3_GPIOInits();

	//This function is used to initialize the SPI3 peripheral parameters
	SPI3_Inits();

	/* Making SSOE = 1 (NSS Output Enable)
	 * Pin Managed by the Hardware
	 * i.e SPE = 1, NSS is pulled to low
	 *  & NSS will be HIGH when SPE = 0
	 * */
	SPI_SSOEConfig(SPI3, ENABLE);

	while(1){
		//Wait till the button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN0));

		delay();

		//Enable the SPI3 Peripheral
		SPI_PeripheralControl(SPI3, ENABLE);

		//First send data length
		uint8_t datalen =  strlen(user_data);
		SPI_SendData(SPI3, &datalen, 1);

		//SendData
		SPI_SendData(SPI3, (uint8_t*)user_data, strlen(user_data));

		//SPI its not BSY
		while( SPI_GetFlagStatus(SPI3, SPI_BUSY_FLAG));

		//Disable the SPI3 Peripheral
		SPI_PeripheralControl(SPI3, DISABLE);
	}

	return 0;
}
