/**
 * @file hmi_lcd.h
 * @brief Header file for the HMI LCD driver.
 */

#ifndef HMI_LCD_H_
#define HMI_LCD_H_

#include <stdint.h>

void LCD_flush(int32_t area_x1, int32_t area_y1, int32_t area_x2, int32_t area_y2, void *p_color);

#endif /* HMI_LCD_H_ */
