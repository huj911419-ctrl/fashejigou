#include "friction_debug.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart10;

#define VOFA_RX_BUF_SIZE 256
#define VOFA_TX_BUF_SIZE 512

static uint8_t rx_buf[VOFA_RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t tx_buf[VOFA_TX_BUF_SIZE];
static volatile uint8_t tx_busy;

/* Called from USART10_IRQHandler in stm32h7xx_it.c */
void FrictionVofaUart10RxCallback(void)
{
    uint32_t isrflags = READ_REG(huart10.Instance->ISR);
    uint32_t cr1its = READ_REG(huart10.Instance->CR1);

    /* RXNE interrupt */
    if ((isrflags & USART_ISR_RXNE_RXFNE) != RESET && (cr1its & USART_CR1_RXNEIE_RXFNEIE) != RESET)
    {
        uint8_t byte = (uint8_t)(huart10.Instance->RDR & 0xFF);
        uint16_t next = (rx_head + 1) % VOFA_RX_BUF_SIZE;
        if (next != rx_tail)
        {
            rx_buf[rx_head] = byte;
            rx_head = next;
        }
        /* Clear RXNE flag by reading RDR */
        __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_NEF);
    }
}

static int Uart10Read(void *context, uint8_t *data, size_t capacity)
{
    (void)context;
    if (rx_head == rx_tail)
    {
        return 0; /* no data */
    }
    size_t count = 0;
    while (count < capacity && rx_tail != rx_head)
    {
        data[count++] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % VOFA_RX_BUF_SIZE;
    }
    return (int)count;
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
    /* Enable RX interrupt on first call */
    static uint8_t initialized = 0;
    if (!initialized)
    {
        initialized = 1;
        __HAL_UART_ENABLE_IT(&huart10, UART_IT_RXNE);
    }

    return (Vofa_Port_s){
        .read = Uart10Read,
        .write = Uart10Write,
        .context = NULL,
    };
}
