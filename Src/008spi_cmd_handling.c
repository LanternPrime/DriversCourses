/*
 * 006_spi_tx_test.c
 *
 *  Created on: 21 dic 2025
 *      Author: octav
 */

/*
 * 		SPI3
 *		PA4 NSS 	Slave Select
 *		PB3 SCK  	Serial Clock.
 *		PB4 MISO 	Master In Slave Out.
 *		PB5 MOSI 	Master Out Slave In.
 *		ALT MODE: 	06
 * */
#include <string.h>
#include "stm32f411xx.h"

//command codes
#define COMMAND_LED_CTRL         	0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ         	0x52
#define COMMAND_PRINT           	0x53
#define COMMAND_ID_READ         	0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

//arduino led
#define LED_PIN	8

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte == 0xF5){
		//ack
		return 1;
	}
	return 0;
}

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
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN4;
	GPIO_Init(&SPIPins);

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
	SPI3Handler.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16; // Generates SCLK of 500kHz
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
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main(void){

	//uint8_t dummy_write = 0xff;

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
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 1);

		delay();

		// Espera a que se suelte (para no repetir)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 0);

		//Enable the SPI3 Peripheral
		SPI_PeripheralControl(SPI3, ENABLE);

//		1. CMD_LED_CTRL <pin no(1)> <value(1)>
		uint8_t cmdCode = COMMAND_LED_CTRL;
		uint8_t ackByte, dummy_read;
		uint8_t args[2];
		SPI_SendData(SPI3, &cmdCode, 1);
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI3, &ackByte, 1);

////		send some dummy bits (1byte) to fetch the respone from the slave.
//		SPI_SendData(SPI3, &dummy_write, 1);
////		read the ack byte received
//		SPI_ReceiveData(SPI3, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte)){
			//send args
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI3, args, 2);
		    // limpiar RX por los 2 bytes recibidos mientras transmitías
		    SPI_ReceiveData(SPI3, &dummy_read, 1);
		    SPI_ReceiveData(SPI3, &dummy_read, 1);
		}

		//SPI its not BSY
		while(SPI_GetFlagStatus(SPI3, SPI_TXE_FLAG) == FLAG_RESET);
		while(SPI_GetFlagStatus(SPI3, SPI_BUSY_FLAG) == FLAG_SET);

		//Disable the SPI3 Peripheral
		SPI_PeripheralControl(SPI3, DISABLE);
	}

	return 0;
}
