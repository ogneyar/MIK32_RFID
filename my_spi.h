#ifndef _MY_SPI_H_
#define _MY_SPI_H_

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
HAL_StatusTypeDef SPI_Exchange(SPI_HandleTypeDef *hspi, uint8_t TransmitBytes[], uint8_t ReceiveBytes[], uint32_t DataSize, uint32_t Timeout);


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
    hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV64; // 500КГц
    // hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV32; // 1МГц
    // hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV16; // 2МГц
    // hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV8; // 4МГц
    // hspi1.Init.BaudRateDiv = SPI_BAUDRATE_DIV4; // 8МГц
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
    HAL_StatusTypeDef SPI_Status = SPI_Exchange(&hspi1, master_output, master_input, length, SPI_TIMEOUT_DEFAULT);
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

    HAL_StatusTypeDef SPI_Status = SPI_Exchange(&hspi1, master_output, master_input, sizeof(master_output), SPI_TIMEOUT_DEFAULT);
    if (SPI_Status != HAL_OK)
    {
        xprintf("SPI_Error %d, OVR %d, MODF %d\r\n", SPI_Status, hspi1.ErrorCode & HAL_SPI_ERROR_OVR, hspi1.ErrorCode & HAL_SPI_ERROR_MODF);
        HAL_SPI_ClearError(&hspi1);
    }

 	return master_input[0];
}

// пока у Микрона либа кривая (https://github.com/MikronMIK32/mik32-hal) - сделал такой костыль (:
HAL_StatusTypeDef SPI_Exchange(SPI_HandleTypeDef *hspi, uint8_t TransmitBytes[], uint8_t ReceiveBytes[], uint32_t DataSize, uint32_t Timeout)
{
    uint32_t txallowed = 1;
    HAL_StatusTypeDef error_code = HAL_OK;
    uint32_t timeout_counter = 0;

    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr = (uint8_t *)ReceiveBytes;
    hspi->RxCount = DataSize;
    hspi->pTxBuffPtr = (uint8_t *)TransmitBytes;
    hspi->TxCount = DataSize;

    hspi->Instance->TX_THR = 1;

    /* Включить SPI если выключено */
    if (!(hspi->Instance->ENABLE & SPI_ENABLE_M))
    {
        __HAL_SPI_ENABLE(hspi);
    }

    while ((hspi->TxCount > 0) || (hspi->RxCount > 0))
    {
        /* Проверка флага TX_FIFO_NOT_FULL */
        if ((hspi->Instance->INT_STATUS & SPI_INT_STATUS_TX_FIFO_NOT_FULL_M) && (hspi->TxCount > 0) && (txallowed == 1))
        {
            hspi->Instance->TXDATA = *(hspi->pTxBuffPtr);
            hspi->pTxBuffPtr++;
            hspi->TxCount--;
            /* Следующие данные - прием (Rx). Tx не разрешен */
            txallowed = 0;
        }

        /* Ожидание когда установится флаг RX_FIFO_NOT_EMPTY */
        if ((hspi->Instance->INT_STATUS & SPI_INT_STATUS_RX_FIFO_NOT_EMPTY_M) && (hspi->RxCount > 0))
        {
            *(hspi->pRxBuffPtr) = hspi->Instance->RXDATA;
            hspi->pRxBuffPtr++;
            hspi->RxCount--;
            /* Следующие данные - передача (Tx). Tx разрешается */
            txallowed = 1;
        }

        if (((timeout_counter++) >= Timeout) || (Timeout == 0U))
        {
            error_code = HAL_TIMEOUT;
            goto error;
        }
    }
    
    return error_code; // добавил одну строчку и теперь у меня SPI работает

error:
    __HAL_SPI_DISABLE(hspi);
    hspi->Instance->ENABLE |= SPI_ENABLE_CLEAR_TX_FIFO_M | SPI_ENABLE_CLEAR_RX_FIFO_M; /* Очистка буферов RX и TX */
    volatile uint32_t unused = hspi->Instance->INT_STATUS; /* Очистка флагов ошибок чтением */
    (void) unused;


    return error_code;
}


#endif /* _MY_SPI_H_ */
