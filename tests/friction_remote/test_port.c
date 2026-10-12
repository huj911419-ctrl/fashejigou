#include "friction_remote_port.h"
#include "test_hal.h"
#include <assert.h>
#include <stdio.h>

UART_HandleTypeDef huart5;
static uint32_t clock_ms;
static uint32_t mask;
static uint8_t *destination;
static HAL_StatusTypeDef rx_result = HAL_OK;
static unsigned receives;
uint32_t HAL_GetTick(void) { return clock_ms; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1U; }
void __set_PRIMASK(uint32_t value) { mask = value; }
void __DMB(void) {}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *bytes, uint16_t count)
{
    assert(handle == &huart5 && count == 1U);
    destination = bytes;
    ++receives;
    return rx_result;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{
    assert(handle == &huart5);
    return HAL_OK;
}
static void Rx(uint8_t byte)
{
    *destination = byte;
    ++clock_ms;
    HAL_UART_RxCpltCallback(&huart5);
}
int main(void)
{
    assert(!FrictionRcPortOnError()); /* Disabled/uninitialized leaves generic USART routing. */
    assert(FrictionRcPortInit());
    UART_HandleTypeDef other;
    HAL_UART_RxCpltCallback(&other);
    assert(receives == 1U);
    Rx(42U);
    FrictionRcByte_s byte;
    assert(FrictionRcPortPop(&byte) && byte.byte == 42U && byte.received_ms == 1U);
    assert(!FrictionRcPortPop(&byte));
    for (unsigned i = 0U; i < 140U; ++i) { Rx((uint8_t)i); }
    assert(FrictionRcPortTakeFaults() == FRICTION_RC_OVERFLOW);
    assert(!FrictionRcPortPop(&byte));
    rx_result = HAL_ERROR;
    Rx(1U);
    assert(FrictionRcPortTakeFaults() & FRICTION_RC_UART_ERROR);
    assert(FrictionRcPortTakeFaults() & FRICTION_RC_UART_ERROR); /* Failed restart stays sticky. */
    rx_result = HAL_OK;
    assert(FrictionRcPortTakeFaults() & FRICTION_RC_UART_ERROR);
    assert(FrictionRcPortTakeFaults() == 0U);
    Rx(99U);
    assert(FrictionRcPortOnError());
    assert(FrictionRcPortTakeFaults() & FRICTION_RC_UART_ERROR);
    assert(!FrictionRcPortPop(&byte));
    FrictionRcPortStats_s stats;
    FrictionRcPortGetStats(&stats);
    assert(stats.started && stats.uart_errors >= 2U && stats.overflows != 0U && stats.bytes == 143U);
    assert(mask == 0U);
    puts("PASS: real UART RX adapter timestamps, handle isolation, queue overflow/discard, sticky restart failure, error recovery");
    return 0;
}
