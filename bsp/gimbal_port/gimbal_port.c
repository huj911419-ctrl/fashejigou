#include "gimbal_port.h"
#include "fdcan.h"
#include "usart.h"
#include <string.h>

#define GIMBAL_CAN_QUEUE_SIZE 32U
#define GIMBAL_RC_QUEUE_SIZE 128U

static DmCanFrame_s can_queue[GIMBAL_CAN_QUEUE_SIZE];
static GimbalRcByte_s rc_queue[GIMBAL_RC_QUEUE_SIZE];
static volatile uint16_t can_head;
static volatile uint16_t can_tail;
static volatile uint16_t rc_head;
static volatile uint16_t rc_tail;
static volatile uint32_t faults;
static uint8_t rx_byte;
static bool can_enabled;
static bool remote_enabled;
static GimbalPortStats_s statistics;

static void RecordFault(uint32_t fault)
{
    /* UART and CAN IRQ priorities may differ on another board. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    faults |= fault;
    statistics.uart_errors += (fault & GIMBAL_PORT_UART_ERROR) != 0U;
    statistics.can_errors += (fault & GIMBAL_PORT_CAN_ERROR) != 0U;
    statistics.overflows += (fault & GIMBAL_PORT_RX_OVERFLOW) != 0U;
    __set_PRIMASK(mask);
}

bool GimbalPortInit(void)
{
    return GimbalPortInitModules(true, true);
}

static bool StartCan(void)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    /* MASTER_ID is discovered from a parameter response; accept standard IDs.
     * This standalone board port owns CAN1. Protocol validates ID and payload. */
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    bool ok = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) == HAL_OK;
    ok = (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK) && ok;
    ok = (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) && ok;
    ok = (HAL_FDCAN_ActivateNotification(&hfdcan1,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0U) == HAL_OK) && ok;
    return ok;
}

bool GimbalPortInitModules(bool use_can, bool use_remote)
{
    can_enabled = use_can;
    remote_enabled = use_remote;
    bool ok = !use_can || StartCan();
    if (use_remote)
    {
        ok = (HAL_UART_Receive_IT(&huart5, &rx_byte, 1U) == HAL_OK) && ok;
    }
    return ok;
}

bool GimbalPortSend(void *context, const DmCanFrame_s *frame)
{
    (void)context;
    if (!can_enabled || frame == NULL || frame->id > 0x7FFU || (frame->length != 4U && frame->length != 8U))
    {
        ++statistics.tx_rejected;
        return false;
    }
    bool disable = frame->length == 8U && frame->data[0] == 0xFFU && frame->data[7] == 0xFDU;
    uint32_t pending = hfdcan1.Instance->TXBRP;
    if (pending != 0U)
    {
        /* Never accumulate stale speed commands. CAN1 is exclusively owned. */
        (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, pending);
        ++statistics.tx_aborted;
        if (!disable)
        {
            ++statistics.tx_rejected;
            return false;
        }
    }
    FDCAN_ProtocolStatusTypeDef status = {0};
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) != HAL_OK || status.BusOff ||
        HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
    {
        ++statistics.tx_rejected;
        return false;
    }
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = frame->id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = frame->length == 4U ? FDCAN_DLC_BYTES_4 : FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    bool accepted = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, frame->data) == HAL_OK;
    if (accepted)
    {
        ++statistics.tx_accepted;
    }
    else
    {
        ++statistics.tx_rejected;
    }
    return accepted;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t interrupts)
{
    if (!can_enabled || handle != &hfdcan1)
    {
        return;
    }
    if (interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
    {
        RecordFault(GIMBAL_PORT_RX_OVERFLOW);
    }
    /* Bound work per IRQ even with unrelated traffic on the bus. */
    for (unsigned budget = 0U; budget < GIMBAL_CAN_QUEUE_SIZE; ++budget)
    {
        if (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) == 0U)
        {
            break;
        }
        FDCAN_RxHeaderTypeDef header;
        uint8_t bytes[64]; /* HAL can copy a full FD payload before validation. */
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, bytes) != HAL_OK)
        {
            RecordFault(GIMBAL_PORT_CAN_ERROR);
            break;
        }
        ++statistics.can_frames;
        statistics.can_last_ms = HAL_GetTick();
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8)
        {
            ++statistics.can_filtered;
            continue;
        }
        uint16_t next = (can_head + 1U) % GIMBAL_CAN_QUEUE_SIZE;
        if (next == can_tail)
        {
            RecordFault(GIMBAL_PORT_RX_OVERFLOW);
            continue;
        }
        DmCanFrame_s *frame = &can_queue[can_head];
        frame->id = (uint16_t)header.Identifier;
        frame->length = 8U;
        frame->received_ms = HAL_GetTick();
        memcpy(frame->data, bytes, 8U);
        __DMB();
        can_head = next;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
    if (!remote_enabled || handle != &huart5)
    {
        return;
    }
    ++statistics.uart_bytes;
    statistics.uart_last_ms = HAL_GetTick();
    uint16_t next = (rc_head + 1U) % GIMBAL_RC_QUEUE_SIZE;
    if (next == rc_tail)
    {
        RecordFault(GIMBAL_PORT_RX_OVERFLOW);
    }
    else
    {
        rc_queue[rc_head].byte = rx_byte;
        rc_queue[rc_head].received_ms = HAL_GetTick();
        __DMB();
        rc_head = next;
    }
    if (HAL_UART_Receive_IT(handle, &rx_byte, 1U) != HAL_OK)
    {
        RecordFault(GIMBAL_PORT_UART_ERROR);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    if (remote_enabled && handle == &huart5)
    {
        RecordFault(GIMBAL_PORT_UART_ERROR);
    }
}

bool GimbalPortPopCan(DmCanFrame_s *frame)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool present = can_enabled && can_head != can_tail;
    if (present)
    {
        *frame = can_queue[can_tail];
        can_tail = (can_tail + 1U) % GIMBAL_CAN_QUEUE_SIZE;
    }
    __set_PRIMASK(mask);
    return present;
}

bool GimbalPortPopRc(GimbalRcByte_s *byte)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool present = remote_enabled && rc_head != rc_tail;
    if (present)
    {
        *byte = rc_queue[rc_tail];
        rc_tail = (rc_tail + 1U) % GIMBAL_RC_QUEUE_SIZE;
    }
    __set_PRIMASK(mask);
    return present;
}

uint32_t GimbalPortTakeFaults(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t result = faults;
    faults = 0U;
    if (result != 0U)
    {
        can_tail = can_head;
        rc_tail = rc_head;
    }
    if (remote_enabled && (result & GIMBAL_PORT_UART_ERROR))
    {
        (void)HAL_UART_AbortReceive(&huart5);
        if (HAL_UART_Receive_IT(&huart5, &rx_byte, 1U) != HAL_OK)
        {
            faults |= GIMBAL_PORT_UART_ERROR;
        }
    }
    __set_PRIMASK(mask);
    return result;
}

void GimbalPortGetStats(GimbalPortStats_s *stats)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *stats = statistics;
    __set_PRIMASK(mask);
}
