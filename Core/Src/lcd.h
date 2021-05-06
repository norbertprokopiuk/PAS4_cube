/*
 * lcd.h
 *
 *  Created on: May 2, 2021
 *      Author: norbe
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_

#include <stdint.h>
#include "stm32f1xx.h"

void lcd_setup(void);

void lcd_clear(void);
void lcd_draw_bitmap(const uint8_t* data);
void lcd_draw_text(int row, int col, const char* text);

void lcd_copy(void);

#endif /* SRC_LCD_H_ */
