/*
 * lcd.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */

#include "../bsp/inc/lcd.h"

uint8_t heart[8] = {
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b01110,
    0b01110,
    0b00100,
    0b00000};

uint8_t smile[8] = {
    0b11011,
    0b00000,
    0b01010,
    0b01010,
    0b01010,
    0b00000,
    0b10001,
    0b01110};

static void lcd_enable(void);
static void WriteDLines(uint8_t value);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);

void lcd_send_command(uint8_t cmd)
{
    /*RS = 0, for LCD CMD*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
    /*RW = 0, For Write Cmd*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
    WriteDLines(cmd >> 4);
    WriteDLines(cmd & 0x0F);
}
/*This funtion sends a char to the LCD
 Here we used 4 bit parallel data transmission
 First higher nibble of the data will be sent on the data lines
 Then lower nibble of the data will be set on the data lines*/
void lcd_print_char(uint8_t data)
{
    /*RS = 1, for User Data*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);
    /*RW = 0, For Write Cmd*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
    WriteDLines(data >> 4);   /*Higher Nibble*/
    WriteDLines(data & 0x0F); /*Lower Nibble*/
}

void lcd_print_string(uint8_t *message)
{
    do
    {
        lcd_print_char(*message++);
    } while (*message != '\0');
}

void lcd_init(void)
{
    // 1. Configure GPIO Pins Which Are used for lcd
    GPIO_Handle_t lcd_signal;

    lcd_signal.pGPIOx = LCD_GPIO_PORT;
    lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPT_PP;
    lcd_signal.GPIO_PinConfig.GPIO_PinPuPdCtlr = GPIO_NO_PUPD;
    lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FST;

    lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_RS;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_RW;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_EN;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_D4;
    GPIO_Init(&lcd_signal);
    lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_D5;
    GPIO_Init(&lcd_signal);
    lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_D6;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNum = LCD_GPIO_D7;
    GPIO_Init(&lcd_signal);

    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

    // 2.  Do the LCD Init

    mdelay(40);
    /*RS = 0, For LCD Cmd*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
    /*RW = 0, For LCD Cmd*/
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    WriteDLines(0x3);
    mdelay(5);

    WriteDLines(0x3);
    udelay(150);

    WriteDLines(0x3);
    udelay(150);

    WriteDLines(0x2);
    udelay(150);

    /* Function Set Command*/
    lcd_send_command(LCD_CMD_4DL_2N_5X8F);
    mdelay(2);

    /* Display ON and Cursor ON */
    lcd_send_command(LCD_CMD_DOFC_SCB);
    mdelay(2);

    /* Display Clear*/
    lcd_display_clear();
    mdelay(2);

    /* Entry Mode Set */
    lcd_send_command(LCD_CMD_EMODE_INC);
    mdelay(2);

    lcd_save_char(heart, 0);
    lcd_save_char(smile, 1);
}

void lcd_display_clear(void)
{
    lcd_send_command(LCD_CMD_CLR_DISPLAY);
    mdelay(2);
}

void lcd_return_home(void)
{
    lcd_send_command(LCD_CMD_RTN_HOME);
    mdelay(2);
}

void lcd_set_cursor(uint8_t row, uint8_t column)
{
    column--;
    switch (row)
    {
    case 1:
        lcd_send_command((column |= 0x80));
        mdelay(2);
        break;
    case 2:
        lcd_send_command((column |= 0xC0));
        mdelay(2);
        break;
    default:
        break;
    }
}

void lcd_save_char(uint8_t *pattern, uint8_t slot)
{
    lcd_send_command(0x40 | slot << 3);
    mdelay(2);
    for (int i = 0; i < 8; i++)
    {
        lcd_print_char(pattern[i]);
    }

    lcd_send_command(0x80);
    mdelay(2);
}

static void WriteDLines(uint8_t value)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1));

    lcd_enable();
}
static void lcd_enable(void)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
    udelay(1);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    udelay(100);
}
static void mdelay(uint32_t cnt)
{
    for (size_t i = 0; i < (cnt * 5000); i++);
}
static void udelay(uint32_t cnt)
{
    for (size_t i = 0; i < (cnt * 50); i++);
}
