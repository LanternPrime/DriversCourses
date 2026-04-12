/*
 * 017_gyro_lcd_sd.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */
#include "lcd.h"
#include "mpu_6050.h"
#include "sd_card.h"

MPU6050_Handle_t mpu6050Handler;
I2C_Handle_t hi2c_gyro;
SD_CardInfo_t card;

uint8_t buffer[512] = {0};
uint8_t message[] = "Hola Octavio\0";
char screenText[16];
uint8_t data;

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

void MPU6050_PinConfig(void)
{
    GPIO_Handle_t I2C_Pins;
    I2C_Pins.pGPIOx = GPIOB;
    I2C_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
    I2C_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2C_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_OD;
    I2C_Pins.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_PIN_PU;
    I2C_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    // SCL
    I2C_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN6;
    GPIO_Init(&I2C_Pins);

    // SDA
    I2C_Pins.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN7;
    GPIO_Init(&I2C_Pins);
}

void MPU6050_I2C_Config(I2C_Handle_t *hi2c_gyro)
{
    hi2c_gyro->pI2Cx = MPU6050_I2C;
    hi2c_gyro->I2C_Config.I2C_AckControl = MPU6050_I2C_ACKCTRL;
    hi2c_gyro->I2C_Config.I2C_DeviceAddress = MPU6050_I2C_DEVICE_ADDR;
    hi2c_gyro->I2C_Config.I2C_SCLSpeed = MPU6050_I2C_SCLSPEED;
    hi2c_gyro->I2C_Config.I2C_FMDutyCycle = MPU6050_I2C_DUTYCYCLE;
    I2C_Init(hi2c_gyro);

    I2C_PeripheralControl(hi2c_gyro->pI2Cx, ENABLE);
}

void MPU6050_Setup(MPU6050_Handle_t *mpu6050Handler, I2C_Handle_t *hi2c_gyro)
{
    // Initialize GPIO to I2C Pins
    MPU6050_PinConfig();
    // Initialize I2C Peripheral
    MPU6050_I2C_Config(hi2c_gyro);
    mpu6050Handler->pI2CHandle = hi2c_gyro;
    mpu6050Handler->config.deviceAddr = hi2c_gyro->I2C_Config.I2C_DeviceAddress;
    mpu6050Handler->config.gyro_full_scale = MPU6050_GYRO_SCALE_250;
    mpu6050Handler->config.sample_rate_divider = 9;
    mpu6050Handler->config.dlpf_config = MPU6050_DLPF_CFG_3;
    MPU6050_Init(mpu6050Handler);
}

void GPIO_PinInit(void)
{
    GPIO_Handle_t GPIOBtn, GpioLed;

    // this is btn gpio configuration
    GPIOBtn.pGPIOx = GPIOC;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN13;
    GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);

    // this is led gpio configuration
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

    // GREEN
    GpioLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN8;
    GPIO_Init(&GpioLed);
    // RED
    GpioLed.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN9;
    GPIO_Init(&GpioLed);
}

void led_ok(uint8_t data)
{
    if (data == SD_OK)
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN9, HIGH); // OK
    else
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN8, HIGH); // ERROR
}

int main(void)
{
    // USER BTN & LEDS
    GPIO_PinInit();

    lcd_init();
    lcd_print_string((uint8_t *)"..SDCARD Init..\0");
    data = SDcard_init(&card);
    led_ok(data);

    while (1)
    {
        while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13));
        delay();

        memset(buffer, 0x00, sizeof(buffer));
        SD_WriteSingleBlock(&card, 10, message);
        SD_ReadSingleBlock(&card, 10, buffer);

        lcd_display_clear();
        lcd_set_cursor(1, 3);
        lcd_print_string(buffer);
        lcd_set_cursor(2, 8);
        lcd_print_char(LCD_HEART);
        lcd_print_char(LCD_SMILE);
    }
    return 0;
}
