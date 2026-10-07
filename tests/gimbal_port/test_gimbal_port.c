/* Contract tests of production gimbal_port.c, including real IRQ callbacks.
 * HAL is a controllable fake; this does not emulate peripheral registers. */
#include "gimbal_port.h"
#include "test_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static FDCAN_GlobalTypeDef registers;
FDCAN_HandleTypeDef hfdcan1 = {&registers};
UART_HandleTypeDef huart5;
static uint32_t tick, irq_mask, bus_off, fifo_free = 1U;
static HAL_StatusTypeDef setup_result = HAL_OK;
static HAL_StatusTypeDef uart_result = HAL_OK;
static HAL_StatusTypeDef rx_result = HAL_OK;
static HAL_StatusTypeDef tx_result = HAL_OK;
static HAL_StatusTypeDef status_result = HAL_OK;
static uint8_t *uart_destination;
static unsigned setup_calls, uart_arms, uart_aborts, tx_calls, tx_aborts;
static FDCAN_TxHeaderTypeDef last_tx;
static uint8_t last_payload[8];
static FDCAN_RxHeaderTypeDef incoming_header;
static unsigned incoming_count, incoming_index;

uint32_t __get_PRIMASK(void)
{
    return irq_mask;
}

void __disable_irq(void)
{
    irq_mask = 1U;
}

void __set_PRIMASK(uint32_t mask)
{
    irq_mask = mask;
}

void __DMB(void)
{
}

uint32_t HAL_GetTick(void)
{
    return tick;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *handle, const FDCAN_FilterTypeDef *filter)
{
    assert(handle == &hfdcan1 && filter->IdType == FDCAN_STANDARD_ID);
    assert(filter->FilterType == FDCAN_FILTER_MASK && filter->FilterID2 == 0U);
    ++setup_calls;
    return setup_result;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle,
    uint32_t standard, uint32_t extended, uint32_t remote_standard, uint32_t remote_extended)
{
    assert(handle == &hfdcan1 && standard == FDCAN_REJECT && extended == FDCAN_REJECT);
    assert(remote_standard == FDCAN_REJECT_REMOTE && remote_extended == FDCAN_REJECT_REMOTE);
    ++setup_calls;
    return setup_result;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle)
{
    assert(handle == &hfdcan1);
    ++setup_calls;
    return setup_result;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle, uint32_t flags, uint32_t index)
{
    assert(handle == &hfdcan1 && index == 0U);
    assert(flags == (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST));
    ++setup_calls;
    return setup_result;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *buffer, uint16_t size)
{
    assert(handle == &huart5 && size == 1U);
    uart_destination = buffer;
    ++uart_arms;
    return uart_result;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{
    assert(handle == &huart5 && irq_mask == 1U);
    ++uart_aborts;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *handle, uint32_t pending)
{
    assert(handle == &hfdcan1 && pending == registers.TXBRP && pending != 0U);
    /* Real HAL requests cancellation; completion is asynchronous. Do not
     * pretend pending messages have already disappeared from the hardware. */
    ++tx_aborts;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *handle, FDCAN_ProtocolStatusTypeDef *status)
{
    assert(handle == &hfdcan1);
    status->BusOff = bus_off;
    return status_result;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *handle)
{
    assert(handle == &hfdcan1);
    return fifo_free;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *payload)
{
    assert(handle == &hfdcan1);
    last_tx = *header;
    memcpy(last_payload, payload, header->DataLength);
    ++tx_calls;
    return tx_result;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo)
{
    assert(handle == &hfdcan1 && fifo == FDCAN_RX_FIFO0);
    return incoming_count;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *payload)
{
    assert(handle == &hfdcan1 && fifo == FDCAN_RX_FIFO0 && incoming_count > 0U);
    if (rx_result != HAL_OK)
    {
        return rx_result;
    }
    *header = incoming_header;
    memset(payload, (int)(incoming_index++ & 0xFFU),
           header->DataLength == FDCAN_DLC_BYTES_64 ? 64U : 8U);
    --incoming_count;
    return HAL_OK;
}

static void ReceiveCan(unsigned count)
{
    incoming_count = count;
    incoming_index = 0U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

static void ReceiveByte(uint8_t value)
{
    assert(uart_destination != NULL);
    *uart_destination = value;
    HAL_UART_RxCpltCallback(&huart5);
}

static void TestReceive(void)
{
    DmCanFrame_s frame;
    GimbalRcByte_s byte;
    incoming_header = (FDCAN_RxHeaderTypeDef){.Identifier = 0x10U, .DataLength = FDCAN_DLC_BYTES_8};
    tick = UINT32_MAX;
    ReceiveCan(31U);
    assert(GimbalPortTakeFaults() == 0U);
    for (unsigned i = 0U; i < 31U; ++i)
    {
        irq_mask = i & 1U;
        assert(GimbalPortPopCan(&frame));
        assert(irq_mask == (i & 1U));
        assert(frame.id == 0x10U && frame.length == 8U && frame.data[0] == i);
        assert(frame.received_ms == UINT32_MAX);
    }
    assert(!GimbalPortPopCan(&frame));
    ReceiveCan(40U);
    assert(incoming_count == 8U); /* Bounded IRQ work even while overloaded. */
    assert(GimbalPortTakeFaults() == GIMBAL_PORT_RX_OVERFLOW);
    assert(!GimbalPortPopCan(&frame));
    incoming_count = 0U;

    for (unsigned variant = 0U; variant < 4U; ++variant)
    {
        incoming_header = (FDCAN_RxHeaderTypeDef){.DataLength = FDCAN_DLC_BYTES_8};
        if (variant == 0U)
        {
            incoming_header.IdType = FDCAN_EXTENDED_ID;
        }
        if (variant == 1U)
        {
            incoming_header.RxFrameType = FDCAN_REMOTE_FRAME;
        }
        if (variant == 2U)
        {
            incoming_header.DataLength = FDCAN_DLC_BYTES_4;
        }
        if (variant == 3U)
        {
            incoming_header.FDFormat = FDCAN_FD_CAN;
            incoming_header.DataLength = FDCAN_DLC_BYTES_64;
        }
        ReceiveCan(1U);
        assert(!GimbalPortPopCan(&frame) && GimbalPortTakeFaults() == 0U);
    }
    rx_result = HAL_ERROR;
    ReceiveCan(1U);
    assert(GimbalPortTakeFaults() == GIMBAL_PORT_CAN_ERROR);
    rx_result = HAL_OK;
    incoming_count = 0U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
    assert(GimbalPortTakeFaults() == GIMBAL_PORT_RX_OVERFLOW);

    for (unsigned i = 0U; i < 127U; ++i)
    {
        tick = i;
        ReceiveByte((uint8_t)i);
    }
    assert(GimbalPortTakeFaults() == 0U);
    for (unsigned i = 0U; i < 127U; ++i)
    {
        irq_mask = i & 1U;
        assert(GimbalPortPopRc(&byte) && irq_mask == (i & 1U));
        assert(byte.byte == i && byte.received_ms == i);
    }
    assert(!GimbalPortPopRc(&byte));
    for (unsigned i = 0U; i < 128U; ++i)
    {
        ReceiveByte((uint8_t)i);
    }
    assert(GimbalPortTakeFaults() == GIMBAL_PORT_RX_OVERFLOW);
    assert(!GimbalPortPopRc(&byte));
}

static void TestUartRecovery(void)
{
    GimbalRcByte_s byte;
    unsigned before = uart_aborts;
    irq_mask = 1U;
    uart_result = HAL_ERROR;
    ReceiveByte(42U);
    assert(GimbalPortTakeFaults() == GIMBAL_PORT_UART_ERROR && irq_mask == 1U);
    assert(uart_aborts == before + 1U && !GimbalPortPopRc(&byte));
    uart_result = HAL_OK;
    assert(GimbalPortTakeFaults() == GIMBAL_PORT_UART_ERROR);
    assert(uart_aborts == before + 2U);
    assert(GimbalPortTakeFaults() == 0U);
    ReceiveByte(43U);
    assert(GimbalPortPopRc(&byte) && byte.byte == 43U);
    HAL_UART_ErrorCallback(&huart5);
    assert(uart_aborts == before + 2U); /* Recovery belongs to the task. */
    assert(GimbalPortTakeFaults() == GIMBAL_PORT_UART_ERROR);
    UART_HandleTypeDef unrelated_uart;
    FDCAN_HandleTypeDef unrelated_can;
    unsigned arms = uart_arms;
    HAL_UART_ErrorCallback(&unrelated_uart);
    HAL_UART_RxCpltCallback(&unrelated_uart);
    HAL_FDCAN_RxFifo0Callback(&unrelated_can, FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
    assert(GimbalPortTakeFaults() == 0U && uart_arms == arms);
}

static void TestTransmit(void)
{
    DmCanFrame_s frame = {.id = 0x200U, .length = 4U, .data = {1U, 2U, 3U, 4U}};
    assert(GimbalPortSend(NULL, &frame));
    assert(last_tx.Identifier == 0x200U && last_tx.DataLength == FDCAN_DLC_BYTES_4);
    assert(last_tx.FDFormat == FDCAN_CLASSIC_CAN && last_tx.IdType == FDCAN_STANDARD_ID);
    assert(memcmp(last_payload, frame.data, 4U) == 0);
    unsigned before = tx_calls;
    assert(!GimbalPortSend(NULL, NULL));
    frame.id = 0x800U;
    assert(!GimbalPortSend(NULL, &frame));
    frame.id = 0x200U;
    frame.length = 3U;
    assert(!GimbalPortSend(NULL, &frame));
    frame.length = 4U;
    bus_off = 1U;
    assert(!GimbalPortSend(NULL, &frame));
    bus_off = 0U;
    fifo_free = 0U;
    assert(!GimbalPortSend(NULL, &frame));
    fifo_free = 1U;
    status_result = HAL_ERROR;
    assert(!GimbalPortSend(NULL, &frame));
    status_result = HAL_OK;
    assert(tx_calls == before);
    tx_result = HAL_ERROR;
    assert(!GimbalPortSend(NULL, &frame));
    tx_result = HAL_OK;
    registers.TXBRP = 4U;
    before = tx_calls;
    assert(!GimbalPortSend(NULL, &frame));
    assert(tx_aborts == 1U && tx_calls == before);
    registers.TXBRP = 2U;
    frame.length = 8U;
    memset(frame.data, 0xFF, 8U);
    frame.data[7] = 0xFDU;
    assert(GimbalPortSend(NULL, &frame));
    assert(tx_aborts == 2U && last_tx.DataLength == FDCAN_DLC_BYTES_8);
    assert(memcmp(last_payload, frame.data, 8U) == 0);
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "init-failure") != 0)
    {
        bool use_can = strcmp(argv[1], "can-only") == 0;
        bool use_remote = strcmp(argv[1], "remote-only") == 0;
        assert(use_can || use_remote || strcmp(argv[1], "off") == 0);
        assert(GimbalPortInitModules(use_can, use_remote));
        assert(setup_calls == (use_can ? 4U : 0U));
        assert(uart_arms == (use_remote ? 1U : 0U));
        if (use_can)
        {
            TestTransmit();
        }
        else
        {
            DmCanFrame_s frame = {.id = 0x200U, .length = 4U};
            assert(!GimbalPortSend(NULL, &frame) && tx_calls == 0U);
            ReceiveCan(1U);
            assert(!GimbalPortPopCan(&frame));
        }
        if (use_remote)
        {
            GimbalRcByte_s byte;
            ReceiveByte(42U);
            assert(GimbalPortPopRc(&byte) && byte.byte == 42U);
        }
        else
        {
            HAL_UART_RxCpltCallback(&huart5);
            HAL_UART_ErrorCallback(&huart5);
            assert(uart_arms == 0U && GimbalPortTakeFaults() == 0U);
        }
        puts("PASS: selective BSP setup and disabled IRQ/TX guards");
        return 0;
    }
    if (argc > 1)
    {
        assert(strcmp(argv[1], "init-failure") == 0);
        setup_result = HAL_ERROR;
        assert(!GimbalPortInit());
        puts("PASS: board initialization failure is reported");
        return 0;
    }
    assert(GimbalPortInit() && setup_calls == 4U && uart_arms == 1U);
    TestReceive();
    TestUartRecovery();
    TestTransmit();
    GimbalPortStats_s stats;
    GimbalPortGetStats(&stats);
    assert(stats.uart_bytes != 0U && stats.can_frames != 0U);
    assert(stats.uart_errors != 0U && stats.overflows != 0U);
    assert(stats.tx_accepted != 0U && stats.tx_rejected != 0U && stats.tx_aborted == 2U);
    puts("PASS: real port queues, loss, UART recovery, CAN filtering and TX failure contracts");
    return 0;
}
