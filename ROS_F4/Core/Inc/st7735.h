#ifndef ST7735_H
#define ST7735_H

#include "main.h"
#include "spi.h"
#include "fonts.h"
#include "stdio.h"

#define ST7735_RST_Pin LCD_RES_Pin
#define ST7735_RST_GPIO_Port GPIOC
#define ST7735_DC_Pin LCD_DC_Pin
#define ST7735_DC_GPIO_Port LCD_DC_GPIO_Port
#define ST7735_CS_Pin LCD_CS_Pin
#define ST7735_CS_GPIO_Port GPIOC

#define ST7735_SPI_INSTANCE hspi1

#define ST7735_XSTART 1
#define ST7735_YSTART 26
#define ST7735_WIDTH  160
#define ST7735_HEIGHT 80

// Screen Direction
#define ST7735_ROTATION 1
// Color Mode: RGB or BGR
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MODE ST7735_MADCTL_RGB
// Color Inverse: 0=NO, 1=YES
#define ST7735_INVERSE 0

// Color definitions
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF
#define ST7735_COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))


void ST7735_Init(void);
void ST7735_DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bgColor, const FontDef *font);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image);
void ST7735_DrawFloat(uint16_t x, uint16_t y, float number, uint16_t color, uint16_t bgColor, const FontDef *font, uint8_t decimalPlaces);
void ST7735_DrawInt(uint16_t x, uint16_t y, int number, uint16_t color, uint16_t bgColor, const FontDef *font);
#endif

