/*
 * 017_gyro_lcd_sd.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */
#include "hc_06.h" //BLUETOOTH
#include "lcd.h"   //LCD

uint8_t buffer[CMD_BUFFER_LEN];

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

void GPIO_PinInit(void)
{
    GPIO_Handle_t GPIOBtn, GpioLed, GpioBuzz;

    // this is btn gpio configuration
    GPIOBtn.pGPIOx = GPIOC;
    GPIOBtn.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN13;
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

    // this is BUZZ gpio configuration
    GpioBuzz.pGPIOx = GPIOC;
    GpioBuzz.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioBuzz.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;
    GpioBuzz.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
    GpioBuzz.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;

    // BUZZER
    GpioBuzz.GPIO_PinConfig.GPIO_PinNum = GPIO_PIN8;
    GPIO_Init(&GpioBuzz);
}

void led_ok(uint8_t data)
{
    if (data == 1)
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN9); // GREEN
    else if (data == 0)
        GPIO_ToggleOutputPin(GPIOC, GPIO_PIN8); // BUZZER
    else
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN8); // RED
}

int main(void)
{
    GPIO_PinInit();
    lcd_init();
    BT_init();

    uint8_t i = 0;

    while (1)
    {
        uint8_t rx = BT_ReadByte();

        // ignorar CR
        if (rx == '\r')
            continue;

        // fin de comando
        if (rx == '\n')
        {
            buffer[i] = '\0';

            lcd_display_clear();
            lcd_set_cursor(1, 1);
            lcd_print_string(buffer);

            if (strcmp((char *)buffer, "ON GREEN") == 0)
            {
                led_ok(1);
                BT_SendString((uint8_t *)"LED GREEN\r\n");
            }
            else if (strcmp((char *)buffer, "BUZZER") == 0)
            {
                led_ok(0);
                BT_SendString((uint8_t *)"BUZZER\r\n");
            }
            else if (strcmp((char *)buffer, "ON RED") == 0)
            {
                led_ok(2);
                BT_SendString((uint8_t *)"ON RED\r\n");
            }
            else
            {
                BT_SendString((uint8_t *)"UNKNOWN\r\n");
                lcd_set_cursor(2, 1);
                lcd_print_string((uint8_t *)"UNKNOWN");
            }

            i = 0;
            memset(buffer, 0, sizeof(buffer));
        }
        else
        {
            if (i < sizeof(buffer) - 1)
            {
                buffer[i++] = rx;
            }
            else
            {
                // overflow, resetear
                i = 0;
                memset(buffer, 0, sizeof(buffer));
                BT_SendString((uint8_t *)"ERR\r\n");
                lcd_print_string((uint8_t *)"ERR");
            }
        }
    }
}