/*
 * 017_gyro_lcd_sd.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */
#include "hc_06.h" //BLUETOOTH
#include "lcd.h"   //LCD
#define LED_GREEN  0
#define LED_RED    1
#define LED_BUZZER 2

BT_Handler_t btHandler;
uint8_t buffer[CMD_BUFFER_LEN];
uint8_t rx[CMD_BUFFER_LEN];
__vo uint8_t cmd_complete = 0, overflow = 0, i = 0;

void APP_BT_Taks(void);
void APP_ProcessCommand(uint8_t *cmd);

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

void GPIO_PinInit(void)
{
    GPIO_DigitalPin(GPIO_MODE_IN, GPIOC, GPIO_PIN13); // BUTTTON
    GPIO_DigitalPin(GPIO_MODE_OUT, GPIOA, GPIO_PIN8); // LED GREEN
    GPIO_DigitalPin(GPIO_MODE_OUT, GPIOA, GPIO_PIN9); // LED RED
    GPIO_DigitalPin(GPIO_MODE_OUT, GPIOC, GPIO_PIN8); // BUZZER
}

void led_ok(uint8_t data)
{
    if (data == LED_GREEN)
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN9); // GREEN
    else if (data == LED_BUZZER)
        GPIO_ToggleOutputPin(GPIOC, GPIO_PIN8); // BUZZER
    else if (data == LED_RED)
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN8); // RED
}

int main(void)
{
    lcd_init();
    GPIO_PinInit();
    btHandler = BT_init();

    BT_ReadByte(&btHandler);
    while (1)
    {
        if (cmd_complete)
        {
            APP_BT_Taks();
        }
    }
}

void USART6_IRQHandler(void)
{
    uint32_t flag_set, interrupt_enabled;
    uint8_t tmp;
    flag_set = btHandler.pUSARTHandle.pUSARTx->SR & (1 << USART_SR_RXNE);
    interrupt_enabled = btHandler.pUSARTHandle.pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

    if (flag_set && interrupt_enabled)
    {
        tmp = btHandler.pUSARTHandle.pUSARTx->DR & (uint8_t)0xFF;

        if (!cmd_complete && tmp != '\r')
        {
            if (!overflow)
            {
                if (tmp != '\n')
                {
                    if (i < (CMD_BUFFER_LEN - 1))
                    {
                        rx[i++] = tmp;
                    }
                    else
                    {
                        rx[i] = '\0';
                        overflow = 1;
                    }
                }
                else
                {
                    rx[i] = '\0';

                    cmd_complete = 1;

                    // disable the rxne
                    btHandler.pUSARTHandle.pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
                    btHandler.pUSARTHandle.RxBusyState = USART_READY;
                }
            }
            else
            {
                if (tmp == '\n')
                {
                    i = 0;
                    rx[i] = '\0';
                    overflow = 0;
                }
            }
        }
    }
}

void APP_BT_Taks(void)
{
    APP_ProcessCommand(rx);
    i = 0;
    cmd_complete = 0;
    overflow = 0;
    memset(rx, 0, sizeof(rx));
    BT_ReadByte(&btHandler);
}

void APP_ProcessCommand(uint8_t *cmd)
{

    char *token;
    char *arg;

    token = strtok((char *)cmd, " "); // primer palabra
    arg = strtok(NULL, " ");          // segunda palabra

    lcd_display_clear();
    lcd_set_cursor(1, 1);
    if (strcmp(token, "LED") == 0)
    {
        if (strcmp(arg, "GREEN") == 0)
        {
            led_ok(LED_GREEN); // LED GREEN
            BT_SendString(btHandler, (uint8_t *)"LED GREEN");
            lcd_print_string((uint8_t *)"LED GREEN");
        }
        else if (strcmp(arg, "RED") == 0)
        {
            led_ok(LED_RED); // LED RED
            BT_SendString(btHandler, (uint8_t *)"LED RED");
            lcd_print_string((uint8_t *)"LED RED");
        }
        else if (strcmp(arg, "BUZZER") == 0)
        {
            led_ok(LED_BUZZER); // BUZZER
            BT_SendString(btHandler, (uint8_t *)"BUZZER");
            lcd_print_string((uint8_t *)"BUZZER");
        }
        else
        {
            BT_SendString(btHandler, (uint8_t *)"CMD ERR");
            lcd_print_string((uint8_t *)"CMD ERR");
        }
    }
    else
    {
        led_ok(LED_RED); // LED RED
        BT_SendString(btHandler, (uint8_t *)"CMD ERR");
        lcd_print_string((uint8_t *)"CMD ERR");
    }
}
