#ifndef __SPI_H_
#define __SPI_H_

#include "mik32_hal_gpio.h"
#include "mik32_hal_spi.h"
#include "uart_lib.h"
#include "xprintf.h"


SPI_HandleTypeDef hspi1;


/* Список функций */
void SPI_Master_Init(void);
void SPI_SendData(uint8_t Data);
void SPI_SendArray(uint8_t master_output[], uint8_t length);
uint8_t SPI_ReceiveData(void);
uint8_t SPI_transfer(uint8_t data);


//
void SPI_Master_Init(void)
{
    hspi1.Instance = SPI_1;

    /* Режим SPI */
    hspi1.Init.SPI_Mode = HAL_SPI_MODE_MASTER;

    /* Настройки */
    hspi1.Init.CLKPhase = SPI_PHASE_OFF;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.ThresholdTX = SPI_THRESHOLD_DEFAULT;

    /* Настройки для ведущего */
    // hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV64; // 500КГц
    // hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV32; // 1МГц
    // hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV16; // 2МГц
    // hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV8; // 4МГц
    hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV4; // 8МГц
    hspi1.Init.Decoder = SPI_DECODER_NONE;
    hspi1.Init.ManualCS = SPI_MANUALCS_ON;
    hspi1.Init.ChipSelect = SPI_CS_NONE;      

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        xprintf("SPI_Init_Error\n");
    }
    
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; // DD_CS | DD_DC
    GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
    GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
    // GPIO_InitStruct.DS = HAL_GPIO_DS_2MA;
    GPIO_InitStruct.DS = HAL_GPIO_DS_8MA;
    HAL_GPIO_Init(GPIO_1, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3; // DD_RES
    HAL_GPIO_Init(GPIO_0, &GPIO_InitStruct);
}


// передача данных
void SPI_SendData(uint8_t Data)
{
    SPI_transfer(Data);
}


void SPI_SendArray(uint8_t master_output[], uint8_t length)
{
    uint8_t master_input[sizeof(master_output)];
    HAL_StatusTypeDef SPI_Status = HAL_SPI_Exchange(&hspi1, master_output, master_input, length, SPI_TIMEOUT_DEFAULT);
    if (SPI_Status != HAL_OK)
    {
        xprintf("SPI_Error %d, OVR %d, MODF %d\r\n", SPI_Status, hspi1.ErrorCode & HAL_SPI_ERROR_OVR, hspi1.ErrorCode & HAL_SPI_ERROR_MODF);
        HAL_SPI_ClearError(&hspi1);
    }
}

// приём данных
uint8_t SPI_ReceiveData(void)
{
 	return SPI_transfer(0);
}


uint8_t SPI_transfer(uint8_t data)
{
    uint8_t master_output[] = { data };
    uint8_t master_input[sizeof(master_output)];

    HAL_StatusTypeDef SPI_Status = HAL_SPI_Exchange(&hspi1, master_output, master_input, sizeof(master_output), SPI_TIMEOUT_DEFAULT);
    if (SPI_Status != HAL_OK)
    {
        xprintf("SPI_Error %d, OVR %d, MODF %d\r\n", SPI_Status, hspi1.ErrorCode & HAL_SPI_ERROR_OVR, hspi1.ErrorCode & HAL_SPI_ERROR_MODF);
        HAL_SPI_ClearError(&hspi1);
    }

 	return master_input[0];
}


#endif /* __SPI_H_ */
