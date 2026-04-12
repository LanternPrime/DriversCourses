/*
 * lcd.h
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "../Drivers/inc/stm32f411xx.h"

void lcd_display_clear(void);
void lcd_return_home(void);
void lcd_set_cursor(uint8_t row, uint8_t column);

void lcd_save_char(uint8_t *pattern, uint8_t slot);
void lcd_print_char(uint8_t data);
void lcd_print_string(uint8_t *message);
void lcd_send_command(uint8_t cmd);
void lcd_init(void);

/* Aplication Configurable Items*/

#define LCD_GPIO_PORT GPIOB
#define LCD_GPIO_RS   GPIO_PIN12

#define LCD_GPIO_RW GPIO_PIN2
#define LCD_GPIO_EN GPIO_PIN1
#define LCD_GPIO_D4 GPIO_PIN15
#define LCD_GPIO_D5 GPIO_PIN14
#define LCD_GPIO_D6 GPIO_PIN13
#define LCD_GPIO_D7 GPIO_PIN5

/*lcd COMMANDS*/
/* Display Clear*/
/* Return Home*/
/* Entry Mode Set */
/* Display ON and Cursor ON */
/* Function Set Command*/
#define LCD_CMD_CLR_DISPLAY 0x01 /* Clears entire display and sets DDRAM address 0 in address counter.*/
#define LCD_CMD_RTN_HOME    0x02 /* Sets DDRAM address 0 in address counter. Also returns display from being shifted to original position. DDRAM contents remain unchanged.*/
#define LCD_CMD_EMODE_INC   0x06 /* Sets cursor move direction and specifies display shift. These operations are performed during data write and read.*/
#define LCD_CMD_DOFC_SCB    0x0C // Sets entire display (D) on/off, cursor on/off (C), and blinking of cursor position character(B). (0000 1 D C B)
#define LCD_CMD_4DL_2N_5X8F 0x28 /* Sets interface data length (DL), number of display lines (N), and character font (F).*/

/* LCD SYMBOLS*/
#define LCD_HEART 0
#define LCD_SMILE 1

#endif /* INC_lcd_H_ */
