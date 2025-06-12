#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"
#include <string.h>
#include <stdio.h>

// SSD1306 settings
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_I2C_ADDR 0x3C

// Colors
typedef enum {
    Black = 0x00,
    White = 0x01
} SSD1306_COLOR;

// Simple font definition
extern const uint8_t Font8x8[][8];

// Function prototypes
void SSD1306_Init(void);
void SSD1306_Fill(SSD1306_COLOR color);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
void SSD1306_GotoXY(uint8_t x, uint8_t y);
void SSD1306_Puts(char* str, SSD1306_COLOR color);
void SSD1306_Clear(void);
void SSD1306_Printf(uint8_t x, uint8_t y, const char* format, ...);

// Private functions
void SSD1306_WriteCommand(uint8_t byte);

#endif
