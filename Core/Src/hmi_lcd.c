/**
 * @file hmi_lcd.c
 * @brief Low-level LCD driver implementation.
 *
 * This file provides the `LCD_flush` function, which serves as the bridge
 * between the HMI drivers and the specific display driver (ILI9341).
 */

#include "hmi_lcd.h"
#include "ili9341.h"

/**
 * @brief Flushes a rectangular area of pixel data to the LCD.
 * @param area_x1 The starting x-coordinate of the area.
 * @param area_y1 The starting y-coordinate of the area.
 * @param area_x2 The ending x-coordinate of the area.
 * @param area_y2 The ending y-coordinate of the area.
 * @param p_color A pointer to the pixel data buffer.
 */
void LCD_flush(int32_t area_x1, int32_t area_y1, int32_t area_x2, int32_t area_y2, void *p_color) {
    uint16_t width = area_x2 - area_x1 + 1;
    uint16_t height = area_y2 - area_y1 + 1;
    ILI9341_DrawImage(area_x1, area_y1, width, height, (const uint8_t*)p_color);
}


/*******************************************************************************
* Function Name: LCD_flush_dma
* Description: Transmits pixel data to the LCD using DMA (non-blocking).
* Parameter:
* - x1, y1: Top-left coordinates of the flush area.
* - x2, y2: Bottom-right coordinates of the flush area.
* - color_p: Pointer to the pixel data buffer.
* Return: None
*******************************************************************************/
void LCD_flush_dma(int32_t x1, int32_t y1, int32_t x2, int32_t y2, void *color_p)
{
    uint16_t width = x2 - x1 + 1;
    uint16_t height = y2 - y1 + 1;
    ILI9341_DrawImage_DMA(x1, y1, width, height, (const uint8_t*)color_p);
}



