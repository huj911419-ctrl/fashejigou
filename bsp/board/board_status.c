#include "board_status.h"
#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "FreeRTOS.h"
#include <string.h>

static void ReadCan(FDCAN_HandleTypeDef *handle, BoardCanStatus_s *status)
{
    status->state = (uint32_t)HAL_FDCAN_GetState(handle);
    status->hal_error = HAL_FDCAN_GetError(handle);
    status->started = status->state == HAL_FDCAN_STATE_BUSY;
    if (handle->Instance == NULL || status->state == HAL_FDCAN_STATE_RESET)
    {
        return;
    }
    uint32_t divisor = handle->Init.NominalPrescaler *
                       (1U + handle->Init.NominalTimeSeg1 + handle->Init.NominalTimeSeg2);
    if (divisor != 0U)
    {
        status->bitrate = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) / divisor;
    }
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef errors = {0};
    status->readable = HAL_FDCAN_GetProtocolStatus(handle, &protocol) == HAL_OK &&
                       HAL_FDCAN_GetErrorCounters(handle, &errors) == HAL_OK;
    status->protocol = protocol.LastErrorCode;
    status->bus_off = protocol.BusOff != 0U;
    status->error_passive = protocol.ErrorPassive != 0U;
    status->warning = protocol.Warning != 0U;
    status->tx_error_count = errors.TxErrorCnt;
    status->rx_error_count = errors.RxErrorCnt;
    status->tx_free = HAL_FDCAN_GetTxFifoFreeLevel(handle);
    status->tx_pending = handle->Instance->TXBRP;
}

void BoardStatusRead(BoardStatus_s *status)
{
    memset(status, 0, sizeof(*status));
    status->cpu_hz = SystemCoreClock;
    status->uid[0] = HAL_GetUIDw0();
    status->uid[1] = HAL_GetUIDw1();
    status->uid[2] = HAL_GetUIDw2();
    status->reset_flags = RCC->RSR;
    status->heap_free = (uint32_t)xPortGetFreeHeapSize();
    status->heap_minimum = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    status->uart5_state = HAL_UART_GetState(&huart5);
    status->uart5_error = HAL_UART_GetError(&huart5);
    status->spi2_state = (uint32_t)HAL_SPI_GetState(&hspi2);
    status->spi2_error = HAL_SPI_GetError(&hspi2);
    status->power_24v_1_command = (POWER_24V_1_GPIO_Port->ODR & POWER_24V_1_Pin) != 0U;
    status->power_24v_2_command = (POWER_24V_2_GPIO_Port->ODR & POWER_24V_2_Pin) != 0U;
    status->power_5v_command = (POWER_5V_GPIO_Port->ODR & POWER_5V_Pin) != 0U;
    ReadCan(&hfdcan1, &status->can[0]);
    ReadCan(&hfdcan2, &status->can[1]);
    ReadCan(&hfdcan3, &status->can[2]);
}
