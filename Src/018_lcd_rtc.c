/*
 * 017_gyro_lcd_sd.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */
#include "ds1307.h"
#include "hc_06.h"
#include "lcd.h"
#include "sd_card.h"

#define LED_GREEN  0
#define LED_RED    1
#define LED_BUZZER 2

SD_CardInfo_t card;
DS1307_Handle_t rtc;
BT_Handler_t btHandler;

uint8_t message[512];
uint8_t timeBuffer[20];
uint8_t dateBuffer[20];
uint8_t data;
uint8_t buffer[CMD_BUFFER_LEN];
uint8_t indexBuffer = 0;

__vo uint8_t cmd_complete = 0, overflow = 0, i = 0;
__vo uint16_t g_flag_update_display = 0;
__vo uint16_t g_flag_update_log = 0;
__vo uint16_t g_flag_log_sd = 0;
__vo uint16_t g_logs_block = 10;

void APP_BT_Taks(void);
void APP_ProcessCommand(uint8_t *cmd);
void led_ok(uint8_t data);
void App_LogToSD(void);
void App_UpdateDisplay(void);

uint8_t str_to_uint8(const char *str);

void delay(void)
{
    for (uint32_t i = 0; i < 500000 / 2; i++);
}

void systick_init_timer(size_t tick_hz)
{
    __vo uint32_t *pSYST_CSR = SYST_CSR;
    __vo uint32_t *pSYST_RVR = SYST_RVR;

    uint32_t count_value = (SYSTICK_CLOCK / tick_hz) - 1;

    // clean the reload value
    *pSYST_RVR &= ~(0x00FFFFFF);

    // set the count value
    *pSYST_RVR |= count_value;

    // enable the exception and indicate the clock source
    *pSYST_CSR |= (1 << 1); // TICKINT
    *pSYST_CSR |= (1 << 2); // CLKSOURCE = (processor clock)

    // Enable the counter
    *pSYST_CSR |= (1 << 0);
};

void GPIO_PinInit(void)
{
    GPIO_DigitalPin(GPIO_MODE_IN, GPIOC, GPIO_PIN13);
    GPIO_DigitalPin(GPIO_MODE_OUT, GPIOA, GPIO_PIN8);
    GPIO_DigitalPin(GPIO_MODE_OUT, GPIOA, GPIO_PIN9);
    GPIO_DigitalPin(GPIO_MODE_OUT, GPIOC, GPIO_PIN8);
}

void led_ok(uint8_t data)
{
    if (data == LED_GREEN)
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN9);
    else if (data == LED_RED)
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN8);
    else if (data == LED_BUZZER)
        GPIO_ToggleOutputPin(GPIOC, GPIO_PIN8);
}

int main(void)
{
    // Initialization Section
    systick_init_timer(1000);
    GPIO_PinInit();
    btHandler = BT_init();
    lcd_init();
    SDcard_init(&card);
    DS1307_init();

    BT_ReadByte(&btHandler);

    rtc.date.day = WEDNESDAY;
    rtc.date.date = 15;
    rtc.date.month = 04;
    rtc.date.year = 26;

    rtc.time.time_format = TIMEFORMAT_12HRS_PM;
    rtc.time.hours = 3;
    rtc.time.minutes = 31;
    rtc.time.seconds = 00;

    DS1307_setCurrentDate(&rtc);
    DS1307_setCurrentTime(&rtc);

    lcd_display_clear();

    while (1)
    {
        if (g_flag_update_display >= 1000)
        {
            g_flag_update_display = 0;
            App_UpdateDisplay();
        }
        if (cmd_complete)
        {
            APP_BT_Taks();
        }
        if (g_flag_log_sd >= 1000 && g_flag_update_log)
        {
            led_ok(LED_BUZZER);
            g_flag_update_log = 0;
            g_flag_log_sd = 0;
        }
    }
    return 0;
}

void SysTick_Handler(void)
{
    g_flag_update_display++;
    g_flag_log_sd++;
    if (g_flag_update_log == 1)
    {
        g_flag_log_sd++;
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
                        buffer[i++] = tmp;
                    }
                    else
                    {
                        buffer[i] = '\0';
                        overflow = 1;
                    }
                }
                else
                {
                    buffer[i] = '\0';

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
                    buffer[i] = '\0';
                    overflow = 0;
                }
            }
        }
    }
}

void App_UpdateDisplay(void)
{

    DS1307_getCurrentTime(&rtc);
    DS1307_getCurrentDate(&rtc);

    format_timeString(timeBuffer, &rtc.time.time_format, rtc.time.seconds, rtc.time.minutes, rtc.time.hours);
    format_dateString(dateBuffer, rtc.date.day, rtc.date.date, rtc.date.month, rtc.date.year);

    lcd_set_cursor(1, 15);
    if (g_flag_log_sd >= 4000)
        lcd_print_char(LCD_HEART);
    else
        lcd_print_char(LCD_SMILE);

    lcd_set_cursor(1, 1);
    lcd_print_string(timeBuffer);

    lcd_set_cursor(2, 1);
    lcd_print_string(dateBuffer);
}

void App_LogToSD(void)
{
    DS1307_getCurrentTime(&rtc);
    DS1307_getCurrentDate(&rtc);

    codeString(timeBuffer, dateBuffer, message);

    SD_WriteSingleBlock(&card, g_logs_block, message);

    g_logs_block = 10 + ((g_logs_block - 10 + 1) % 10);
}

void APP_BT_Taks(void)
{
    APP_ProcessCommand(buffer);
    i = 0;
    cmd_complete = 0;
    overflow = 0;
    memset(buffer, 0, sizeof(buffer));
    BT_ReadByte(&btHandler);
}

void APP_ProcessCommand(uint8_t *cmd)
{

    char *token, *arg, *values;

    token = strtok((char *)cmd, " "); // FIRST WORD
    arg = strtok(NULL, " ");          // SECOND WORD

    lcd_set_cursor(1, 1);
    if (strcmp(token, "LED") == 0)
    {
        if (strcmp(arg, "GREEN") == 0)
        {
            led_ok(LED_GREEN); // LED GREEN
            BT_SendString(btHandler, (uint8_t *)"LED GREEN");
            // lcd_print_string((uint8_t *)"LED GREEN");
        }
        else if (strcmp(arg, "RED") == 0)
        {
            led_ok(LED_RED); // LED RED
            BT_SendString(btHandler, (uint8_t *)"LED RED");
            // lcd_print_string((uint8_t *)"LED RED");
        }
        else if (strcmp(arg, "BUZZER") == 0)
        {

            BT_SendString(btHandler, (uint8_t *)"BUZZER");
            // lcd_print_string((uint8_t *)"BUZZER");
        }
        else
        {
            BT_SendString(btHandler, (uint8_t *)"CMD ERR");
            lcd_print_string((uint8_t *)"CMD ERR");
        }
    }
    else if (strcmp(token, "SET") == 0)
    {
        if (strcmp(arg, "TIME") == 0)
        {
            values = strtok(NULL, ":");
            rtc.time.hours = str_to_uint8(values);
            values = strtok(NULL, ":");
            rtc.time.minutes = str_to_uint8(values);
            values = strtok(NULL, ":");
            rtc.time.seconds = str_to_uint8(values);

            DS1307_setCurrentTime(&rtc);
        }
        else if (strcmp(arg, "DATE") == 0)
        {
            values = strtok(NULL, "/");
            rtc.date.date = str_to_uint8(values);
            values = strtok(NULL, "/");
            rtc.date.month = str_to_uint8(values);
            values = strtok(NULL, "/");
            rtc.date.year = str_to_uint8(values);

            DS1307_setCurrentDate(&rtc);
        }
        else
        {
            BT_SendString(btHandler, (uint8_t *)"CMD ERR");
            lcd_print_string((uint8_t *)"CMD ERR");
        }
    }
    else if (strcmp(token, "SAVE") == 0)
    {
        led_ok(LED_BUZZER);
        App_LogToSD();
        lcd_display_clear();
        BT_SendString(btHandler, (uint8_t *)"LOG SAVED");
        lcd_print_string((uint8_t *)"LOG SAVED");
        g_flag_update_log = 1;
    }
    else
    {
        led_ok(LED_RED); // LED RED
        BT_SendString(btHandler, (uint8_t *)"CMD ERR");
        lcd_print_string((uint8_t *)"CMD ERR");
    }
}

uint8_t str_to_uint8(const char *str)
{
    char *endptr;
    long val = strtol(str, &endptr, 10);

    if (val < 0 || val > 255 || *endptr != '\0')
    {
        return 0;
    }

    return (uint8_t)val;
}