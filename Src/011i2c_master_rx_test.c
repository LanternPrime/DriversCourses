/*
 * 010i2c_master_tx_test.c
 *
 *  Created on: 27 mar 2026
 *      Author: octav
 */
#include "stm32f411xx.h"

#define MY_ADDR    0x61
#define SLAVE_ADDR 0x68

#define LOW         0
#define BTN_PRESSED LOW

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

I2C_Handle_t I2C1Handler;
uint8_t rx_data[sizeof(size_t)];

/*
 * PB8 -> SCL
 * PB7 -> SDA
 * */

void I2C1_GPIOInits(void)
{

    GPIO_Handle_t I2CPins;
    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;

    // SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN8;
    GPIO_Init(&I2CPins);

    // SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN7;
    GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{

    I2C1Handler.pI2Cx = I2C1;
    I2C1Handler.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
    I2C1Handler.I2C_Config.I2C_DeviceAddress = MY_ADDR;
    I2C1Handler.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
    I2C1Handler.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&I2C1Handler);
}

void GPIOBtn_Init(void)
{
    GPIO_Handle_t GPIOBtn;

    // this is btn gpio configuration
    GPIOBtn.pGPIOx = GPIOC;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN13;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);
}

int main(void)
{

    uint8_t cmdCode, len;

    GPIOBtn_Init();

    I2C1_GPIOInits();

    // This function is used to initialize the I2C1 peripheral parameters
    I2C1_Inits();

    // Enable the I2C Peripheral
    I2C_PeripheralControl(I2C1, ENABLE);

    // ACK bit is made 1, after PE = 1
    I2C_ManageAcking(I2C1, ENABLE);

    while (1)
    {
        // wait till button is pressed
        while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13));

        // to avoid button de-bouncing related issues 200ms of delay
        delay();

        cmdCode = 0x51;

        // send some data to the slave
        I2C_MasterSendData(&I2C1Handler, &cmdCode, 1, SLAVE_ADDR,
                           I2C_ENABLE_SR);

        I2C_MasterReceiveData(&I2C1Handler, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

        cmdCode = 0x52;

        // send some data to the slave
        I2C_MasterSendData(&I2C1Handler, &cmdCode, 1, SLAVE_ADDR,
                           I2C_ENABLE_SR);

        I2C_MasterReceiveData(&I2C1Handler, rx_data, len, SLAVE_ADDR,
                              I2C_DISABLE_SR);
    }

    return 0;
}
