/*
 * lcd.c
 *
 *  Created on: May 2, 2021
 *      Author: norbe
 */

#include <string.h>
#include "lcd.h"
#include "font.h"

#define PCD8544_FUNCTION_SET		0x20 //str 14 dokumentacji
#define PCD8544_DISP_CONTROL		0x08
#define PCD8544_DISP_NORMAL			0x0c //ustawienie trybu normalnego 1000 | 0100
#define PCD8544_SET_Y				0x40
#define PCD8544_SET_X				0x80
#define PCD8544_H_TC				0x04
#define PCD8544_H_BIAS				0x10
#define PCD8544_H_VOP				0x80

#define LCD_BUFFER_SIZE			(84 * 48 / 8)

static uint8_t lcd_buffer[LCD_BUFFER_SIZE];

SPI_HandleTypeDef hspi2;

void lcd_cmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_SET);
}

void lcd_setup(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

	lcd_cmd(PCD8544_FUNCTION_SET | 1);//wlaczone funkcje rozszerzone
	lcd_cmd(PCD8544_H_BIAS | 3);//wartosc optymalna
	lcd_cmd(PCD8544_H_VOP | 0x3f); //ustawienie kontrastu ustawione eksperymentalnie
	lcd_cmd(PCD8544_FUNCTION_SET);//zmiana na funkcje standardowe
	lcd_cmd(PCD8544_DISP_NORMAL);//ustawienie wyświetlania w trybie normal
}

void lcd_clear(void)
{
	memset(lcd_buffer, 0, LCD_BUFFER_SIZE);
}

void lcd_draw_text(int row, int col, const char* text)
{
	int i;
	uint8_t* pbuf = &lcd_buffer[row * 84 + col];
	while ((*text) && (pbuf < &lcd_buffer[LCD_BUFFER_SIZE - 6])) {
		int ch = *text++;
		const uint8_t* font = &font_ASCII[ch - ' '][0];
		for (i = 0; i < 5; i++) {
			*pbuf++ = *font++;
		}
		*pbuf++ = 0;
	}
}

void lcd_copy(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*) lcd_buffer, LCD_BUFFER_SIZE, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
}

