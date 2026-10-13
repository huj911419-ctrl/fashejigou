#include "friction_debug.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart10;

#define VOFA_TX_BUF_SIZE 512

static uint8_t tx_buf[VOFA_TX_BUF_SIZE];
static volatile uint8_t tx_busy;

/* Called from USART10_IRQHandler in stm32h7xx_it.c. UART10 is telemetry-only. */
void FrictionVofaUart10RxCallback(void)
{
    /* Do not parse or buffer RX data on the telemetry UART. */
}

static int Uart10Write(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    if (length > VOFA_TX_BUF_SIZE || tx_busy)
    {
        return 0; /* busy or too large */
    }
    memcpy(tx_buf, data, length);
    tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart10, tx_buf, (uint16_t)length) != HAL_OK)
    {
        tx_busy = 0;
        return 0;
    }
    return 1;
}

/* Called from HAL_UART_TxCpltCallback */
void FrictionVofaUart10TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10)
    {
        tx_busy = 0;
    }
}

/* Override HAL weak callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    FrictionVofaUart10TxCpltCallback(huart);
}

Vofa_Port_s FrictionDebugPort(void)
{
    /* UART10 is deliberately telemetry-only. USB is the command interface. */
    return (Vofa_Port_s){
        .read = NULL,
        .write = Uart10Write,
        .context = NULL,
    };
}
