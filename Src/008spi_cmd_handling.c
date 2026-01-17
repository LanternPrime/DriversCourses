/*
 * 006_spi_tx_test.c
 *
 *  Created on: 21 dic 2025
 *      Author: octav
 */

/*
 * 		SPI3
 *		PA4 NSS 	Slave Select			PURPLE
 *		PA5 SCK  	Serial Clock.			BLACK
 *		PA6 MISO 	Master In Slave Out.	WHITE
 *		PA7 MOSI 	Master Out Slave In.	GRAY
 *		ALT MODE: 	06
 * */
#include <string.h>
#include "stm32f411xx.h"

//command codes
#define COMMAND_LED_CTRL         	0x50
#define COMMAND_SENSOR_READ      0x51
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

void SPI1_GPIOInits(void){

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;

	//SCKL
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN7;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN6;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN6;
	GPIO_Init(&SPIPins);

}

void SPI1_Inits(void){

	SPI_Handle_t SPI1Handler;

	SPI1Handler.pSPIx = SPI1;
	SPI1Handler.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handler.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handler.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; // Generates SCLK of 500kHz
	SPI1Handler.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handler.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handler.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handler.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware Slave Management Enabled for NSS Pin

	SPI_Init(&SPI1Handler);
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


	GPIOBtn_Init();

	SPI1_GPIOInits();

	//This function is used to initialize the SPI3 peripheral parameters
	SPI1_Inits();

	/* Making SSOE = 1 (NSS Output Enable)
	 * Pin Managed by the Hardware
	 * i.e SPE = 1, NSS is pulled to low
	 *  & NSS will be HIGH when SPE = 0
	 * */
	SPI_SSOEConfig(SPI1, ENABLE);

	while(1){

		uint8_t dummy_write = 0xFF;
		uint8_t dummy_read;

		uint8_t ackByte;
		uint8_t args[2];

//		1. CMD_LED_CTRL <pin no(1)> <value(1)>
		// Wait until button is pressed (active-low)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 1);
		delay();

		// Wait until button is released (avoid repeated triggers)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 0);
		delay();

		//Enable the SPI3 Peripheral
		SPI_PeripheralControl(SPI1, ENABLE);

		uint8_t cmdCode = COMMAND_LED_CTRL;

		SPI_SendData(SPI1, &cmdCode, 1);
//		read the ack byte received
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		SPI_SendData(SPI1, &dummy_write, 1);
//		read the ack byte received
		SPI_ReceiveData(SPI1, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte)){
			//send args
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI1, args, 2);
		    // limpiar RX por los 2 bytes recibidos mientras transmitías
		    SPI_ReceiveData(SPI1, &dummy_read, 1);
		    SPI_ReceiveData(SPI1, &dummy_read, 1);
		    //printf("COMMAND_LED_CTRL Executed\n");
		}

////		2. COMMAND_SENSOR_READ <analog pin number(1)>
		// Wait until button is pressed (active-low)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 1);
		delay();

		// Wait until button is released (avoid repeated triggers)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 0);
		delay();

		cmdCode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI1, &cmdCode, 1);
//		read the ack byte received
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		SPI_SendData(SPI1, &dummy_write, 1);
//		read the ack byte received
		SPI_ReceiveData(SPI1, &ackByte, 1);

		if(SPI_VerifyResponse(ackByte)){
			//send args
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI1, args, 1);

			SPI_ReceiveData(SPI1, &dummy_read, 1);
			//Insert some delay for the slave can read some data.
			delay();
			uint8_t analogRead;
			//Write a dummy write to fetch (1) the response.
			SPI_SendData(SPI1, &dummy_write, 1);
		    SPI_ReceiveData(SPI1, &analogRead, 1);
		    //printf("COMMAND_SENSOR_READ %d\n",analogRead);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >
		// Wait until button is pressed (active-low)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 1);
		delay();

		// Wait until button is released (avoid repeated triggers)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 0);
		delay();

		cmdCode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI1,&cmdCode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackByte,1);

		if( SPI_VerifyResponse(ackByte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI1,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI1,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI1,&led_status,1);
			//printf("COMMAND_READ_LED %d\n",led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >
		// Wait until button is pressed (active-low)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 1);
		delay();

		// Wait until button is released (avoid repeated triggers)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 0);
		delay();

		cmdCode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI1,&cmdCode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackByte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackByte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI1,args,1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI1,&message[i],1);
				SPI_ReceiveData(SPI1,&dummy_read,1);
			}

			//printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		// Wait until button is pressed (active-low)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 1);
		delay();

		// Wait until button is released (avoid repeated triggers)
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) == 0);
		delay();

		cmdCode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI1,&cmdCode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI1,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI1,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI1,&ackByte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackByte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI1,&dummy_write,1);
				SPI_ReceiveData(SPI1,&id[i],1);
			}

			id[10] = '\0';

			//printf("COMMAND_ID : %s \n",id);

		}

		//SPI its not BSY
		while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG) == FLAG_SET);

		//Disable the SPI3 Peripheral
		SPI_PeripheralControl(SPI1, DISABLE);
	}

	return 0;
}
