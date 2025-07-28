#include "lcd_i2c.h"

// Ganti alamat ini jika PCF8574 Anda memiliki alamat berbeda
#define LCD_I2C_ADDR 0x4E // Alamat umum adalah 0x27 << 1 atau 0x3F << 1. Cek datasheet.

I2C_HandleTypeDef *i2c_handle;

void lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C; // en=1, rs=0
    data_t[1] = data_u | 0x08; // en=0, rs=0
    data_t[2] = data_l | 0x0C; // en=1, rs=0
    data_t[3] = data_l | 0x08; // en=0, rs=0
    HAL_I2C_Master_Transmit(i2c_handle, LCD_I2C_ADDR, (uint8_t *)data_t, 4, 100);
}

void lcd_send_data(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D; // en=1, rs=1
    data_t[1] = data_u | 0x09; // en=0, rs=1
    data_t[2] = data_l | 0x0D; // en=1, rs=1
    data_t[3] = data_l | 0x09; // en=0, rs=1
    HAL_I2C_Master_Transmit(i2c_handle, LCD_I2C_ADDR, (uint8_t *)data_t, 4, 100);
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_set_cursor(int row, int col)
{
    uint8_t cursor_addr;
    switch (row)
    {
    case 0:
        cursor_addr = 0x80 + col;
        break;
    case 1:
        cursor_addr = 0xC0 + col;
        break;
    default:
        cursor_addr = 0x80 + col;
    }
    lcd_send_cmd(cursor_addr);
}

void lcd_init(I2C_HandleTypeDef *hi2c)
{
    i2c_handle = hi2c;
    HAL_Delay(50);
    lcd_send_cmd(0x30);
    HAL_Delay(5);
    lcd_send_cmd(0x30);
    HAL_Delay(1);
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20);
    HAL_Delay(10);

    lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    HAL_Delay(1);
    lcd_send_cmd(0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
    HAL_Delay(1);
    lcd_send_cmd(0x01); // clear display
    HAL_Delay(2);
    lcd_send_cmd(0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
    HAL_Delay(1);
    lcd_send_cmd(0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string(char *str)
{
    while (*str)
        lcd_send_data(*str++);
}