
#include "main.h"


MIFARE_Key key;         // Объект ключа
enum StatusCode status; // Объект статуса
const char Hex[2];


int main(void)
{
	InitClock();
    UART_Init(UART_0, (32000000/9600), UART_CONTROL1_TE_M | UART_CONTROL1_M_8BIT_M, 0, 0);
    xprintf("\r\nRFID test run\r\n");

    SPI_Master_Init(); // Инициализация SPI
    
    RFID_init(SS_PORT, SS_PIN, RST_PORT, RST_PIN);

    for (uint8_t i = 0; i < 6; i++) {   // Наполняем ключ
      //key.keyByte[i] = 0xFF;          // Ключ по умолчанию 0xFFFFFFFFFFFF
      key.keyByte[i] = 0xAB;            // Новый ключ 0xABABABABABAB
    }

    int const_timer = 100;
    int timer = const_timer;

    while(1)
    {
	    if (timer == 0)
        {
            digitalWrite(RST_PORT, RST_PIN, HIGH);  // Сбрасываем модуль
            delay(1);
            digitalWrite(RST_PORT, RST_PIN, LOW);   // Отпускаем сброс
            PCD_Init();                             // Инициализируем заного
            // xprintf("Reboot\r\n");
            timer = const_timer;
        }

        if (timer > 0) timer--;

        if ( ! PICC_IsNewCardPresent() ) continue;  // Если новая метка не поднесена - вернуться в начало цикла
        if ( ! PICC_ReadCardSerial() ) continue;    // Если метка не читается - вернуться в начало цикла

        xprintf("UID: ");
        for (uint8_t i = 0; i < 4; i++) {
            ByteToHex(get_uid(i), &Hex);
            xprintf("0x");
            xprintf(Hex);
            xprintf(",");
        }
        xprintf("\r\n");
        ledBlink();
    }
}


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

void ByteToHex(uint8_t sourse, char *Hex)
{
    char const hex_chars[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
 
    *Hex++ = hex_chars[(sourse & 0xF0) >> 4];
    *Hex++ = hex_chars[sourse & 0xF];
    *Hex = '\0';
}
