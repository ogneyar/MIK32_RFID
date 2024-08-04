#ifndef _MY_UTILS_H_
#define _MY_UTILS_H_


#include "mik32_hal_gpio.h"


#define PIN_LED 7	 // Светодиод управляется выводом PORT_2_7
#define PIN_BUTTON 6 // Кнопка управляет сигналом на выводе PORT_2_6


/* Список функций */
void InitClock(void);
void ledBlink(void);
void ledButton(void);
void giveHexFromByte(uint8_t sourse, char *Hex);


void InitClock(void)
{
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_UART_0_M | PM_CLOCK_APB_P_GPIO_0_M | PM_CLOCK_APB_P_GPIO_1_M | PM_CLOCK_APB_P_GPIO_2_M; // включение тактирования GPIO
	PM->CLK_APB_M_SET |= PM_CLOCK_APB_M_PAD_CONFIG_M | PM_CLOCK_APB_M_WU_M | PM_CLOCK_APB_M_PM_M; // включение тактирования блока для смены режима выводов
	
	PAD_CONFIG->PORT_2_CFG &= ~(0b11 << (2 * PIN_LED));	// Установка вывода 7 порта 2 в режим GPIO
	PAD_CONFIG->PORT_2_CFG &= ~(0b11 << (2 * PIN_BUTTON)); // Установка вывода 6 порта 2 в режим GPIO

	GPIO_2->DIRECTION_OUT = 1 << PIN_LED;	// Установка направления вывода 7 порта 2 на выход
	GPIO_2->DIRECTION_IN = 1 << PIN_BUTTON; // Установка направления вывода 6 порта 2 на вход
}

void ledBlink(void)
{
	GPIO_2->OUTPUT ^= 1 << PIN_LED; // Установка сигнала вывода 7 порта 2 в противоположный уровень
	delay(100);
}

void ledButton(void)
{
	if (GPIO_2->STATE & (1 << PIN_BUTTON))
	{
		GPIO_2->OUTPUT |= 1 << PIN_LED; // Установка сигнала вывода 7 порта 2 в высокий уровень
		while (GPIO_2->STATE & (1 << PIN_BUTTON)) ;
	}
}

void giveHexFromByte(uint8_t sourse, char *Hex)
{
    char const hex_chars[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
 
    *Hex++ = hex_chars[(sourse & 0xF0) >> 4];
    *Hex++ = hex_chars[sourse & 0xF];
    *Hex = '\0';
}



#endif /* _MY_UTILS_H_ */
