#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h"

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_set_cursor(int row, int col);
void lcd_clear(void);

#endif