/*
 * 017_gyro_lcd_sd.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */
#include "ds1307.h"
#include "lcd.h"
#include "sd_card.h"

SD_CardInfo_t card;

uint8_t buffer[512];
uint8_t csd[16];
uint8_t message[512];
uint8_t data;

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
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
    if (data == 0x0)
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN9, HIGH); // OK
    else
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN8, HIGH); // ERROR
}

int main(void)
{
    uint8_t last_block = 9;
    // USER BTN & LEDS
    GPIO_PinInit();
    lcd_init();

    lcd_print_string((uint8_t *)"..SDCARD Init..\0");
    SDcard_init(&card);
    data = DS1307_init();
    led_ok(data);

    while (1)
    {
        while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13));
        delay();
        for (size_t i = 0; i < 11; i++)
        {
            memset(message, 0, sizeof(message));
            sprintf((char *)message, "Hola Octavio %d", i);
            SD_WriteSingleBlock(&card, ++last_block, message);
        }

        lcd_display_clear();
        SD_ReadSingleBlock(&card, last_block, buffer);
        lcd_set_cursor(1, 1);
        lcd_print_string(buffer);
        SD_ReadCSD(&card, csd);
        sprintf((char *)message, "%ld Mb", (uint32_t)card.capacity_mib);
        lcd_set_cursor(2, 1);
        lcd_print_string(message);
        // lcd_print_char(LCD_HEART);
        // lcd_print_char(LCD_SMILE);
    }
    return 0;
}
