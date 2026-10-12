#include "friction_remote_port.h"
#include "usart.h"

#define RC_QUEUE_SIZE 128U
static FrictionRcByte_s queue[RC_QUEUE_SIZE];
static volatile uint16_t head;
static volatile uint16_t tail;
static volatile uint32_t faults;
static uint8_t rx_byte;
static bool enabled;
static FrictionRcPortStats_s statistics;

static void RecordFault(uint32_t fault)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    faults |= fault;
    statistics.uart_errors += (fault & FRICTION_RC_UART_ERROR) != 0U;
    statistics.overflows += (fault & FRICTION_RC_OVERFLOW) != 0U;
    __set_PRIMASK(mask);
}

bool FrictionRcPortInit(void)
{
    enabled = true;
    statistics.started = HAL_UART_Receive_IT(&huart5, &rx_byte, 1U) == HAL_OK;
    if (!statistics.started)
    {
        RecordFault(FRICTION_RC_UART_ERROR);
    }
    return statistics.started;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
    if (!enabled || handle != &huart5)
    {
        return;
    }
    uint32_t now_ms = HAL_GetTick();
    ++statistics.bytes;
    statistics.last_byte_ms = now_ms;
    uint16_t next = (head + 1U) % RC_QUEUE_SIZE;
    if (next == tail)
    {
        RecordFault(FRICTION_RC_OVERFLOW);
    }
    else
    {
        queue[head] = (FrictionRcByte_s){.byte = rx_byte, .received_ms = now_ms};
        __DMB();
        head = next;
    }
    if (HAL_UART_Receive_IT(handle, &rx_byte, 1U) != HAL_OK)
    {
        RecordFault(FRICTION_RC_UART_ERROR);
    }
}

bool FrictionRcPortOnError(void)
{
    if (enabled)
    {
        RecordFault(FRICTION_RC_UART_ERROR);
        return true;
    }
    return false;
}

bool FrictionRcPortPop(FrictionRcByte_s *byte)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool present = enabled && head != tail;
    if (present)
    {
        *byte = queue[tail];
        tail = (tail + 1U) % RC_QUEUE_SIZE;
    }
    __set_PRIMASK(mask);
    return present;
}

uint32_t FrictionRcPortTakeFaults(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t result = faults;
    faults = 0U;
    if (result != 0U)
    {
        tail = head;
    }
    if (enabled && (result & FRICTION_RC_UART_ERROR))
    {
        (void)HAL_UART_AbortReceive(&huart5);
        statistics.started = HAL_UART_Receive_IT(&huart5, &rx_byte, 1U) == HAL_OK;
        if (!statistics.started)
        {
            faults |= FRICTION_RC_UART_ERROR;
        }
    }
    __set_PRIMASK(mask);
    return result;
}

void FrictionRcPortGetStats(FrictionRcPortStats_s *stats)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *stats = statistics;
    __set_PRIMASK(mask);
}
