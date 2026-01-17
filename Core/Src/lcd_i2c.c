#include "lcd_i2c.h"

extern I2C_HandleTypeDef hi2c1;
#define LCD_ADDR (0x27 << 1) // Verifica daca e 0x27 sau 0x3F

// BITI PENTRU PCF8574 (Standard Backpack):
// P0 - RS
// P1 - RW
// P2 - EN
// P3 - Backlight
// P4-P7 - Data

// Functie de nivel jos pentru I2C
void LCD_I2C_Write(uint8_t data) {
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data, 1, 10);
}

// Functie care trimite 4 biti + Pulse la Enable
void LCD_Send4Bits(uint8_t data, uint8_t rs) {
    uint8_t val = (data & 0xF0) | 0x08 | rs; // 0x08 este Backlight ON

    // 1. Setam datele cu EN = 0
    LCD_I2C_Write(val);

    // 2. Pulse EN = 1
    LCD_I2C_Write(val | 0x04);
    HAL_Delay(1); // Mica pauza

    // 3. Pulse EN = 0
    LCD_I2C_Write(val & ~0x04);
    HAL_Delay(1);
}

// Functie pentru comenzi (RS=0)
void LCD_SendCommand(uint8_t cmd) {
    uint8_t high_nibble = cmd & 0xF0;
    uint8_t low_nibble = (cmd << 4) & 0xF0;

    LCD_Send4Bits(high_nibble, 0);
    LCD_Send4Bits(low_nibble, 0);
}

// Functie pentru date/litere (RS=1)
void LCD_SendData(uint8_t data) {
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble = (data << 4) & 0xF0;

    LCD_Send4Bits(high_nibble, 1);
    LCD_Send4Bits(low_nibble, 1);
}

// INITIALIZAREA "AGRESIVA"
void LCD_Init(void) {
    HAL_Delay(50); // Asteptam sa se stabilizeze tensiunea

    // Secventa de Resetare magica (Datasheet HD44780)
    // Trimitem 0x30 de 3 ori pentru a fi siguri ca e in 8-bit mode
    LCD_Send4Bits(0x30, 0);
    HAL_Delay(5);
    LCD_Send4Bits(0x30, 0);
    HAL_Delay(1);
    LCD_Send4Bits(0x30, 0);
    HAL_Delay(10);

    // Acum trecem in 4-bit mode
    LCD_Send4Bits(0x20, 0);
    HAL_Delay(10);

    // De aici incolo functioneaza normal pe 4 biti
    LCD_SendCommand(0x28); // 4-bit, 2 lines, 5x8 font
    HAL_Delay(1);
    LCD_SendCommand(0x08); // Display off
    HAL_Delay(1);
    LCD_SendCommand(0x01); // Clear display
    HAL_Delay(5); // Clear dureaza mult!
    LCD_SendCommand(0x06); // Entry mode set (cursor moves right)
    HAL_Delay(1);
    LCD_SendCommand(0x0C); // Display on, Cursor off, Blink off
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x80 : 0xC0;
    address += col;
    LCD_SendCommand(address);
}

void LCD_Print(char *str) {
    while (*str) LCD_SendData(*str++);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    HAL_Delay(5); // Delay critic!
}
