#ifndef __MAIN_H_
#define __MAIN_H_

#include "my_spi.h"
#include "my_utils.h"
#include "RFID.h"

#define SS_PORT  GPIO_0  // Порт 0.3
#define SS_PIN  (1 << 3) // d9
#define RST_PORT GPIO_1  // Порт 1.9
#define RST_PIN (1 << 9) // d8

#define PIN_LED 7	 // Светодиод управляется выводом PORT_2_7
#define PIN_BUTTON 6 // Кнопка управляет сигналом на выводе PORT_2_6

void InitClock(void);
void ledBlink(void);
void ledButton(void);

#endif /* __MAIN_H_ */
