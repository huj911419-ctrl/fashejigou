#include "usb_console_port.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <string.h>

#define USB_CONSOLE_RX_SIZE 1024U
#define USB_CONSOLE_ENDPOINT_SIZE 512U

extern USBD_HandleTypeDef hUsbDeviceHS;
static uint8_t endpoint_rx[USB_CONSOLE_ENDPOINT_SIZE];
static uint8_t endpoint_tx[TEXT_CONSOLE_PACKET_SIZE];
static uint8_t rx_ring[USB_CONSOLE_RX_SIZE];
static uint8_t line_coding[7] = {0x00U, 0xC2U, 0x01U, 0x00U, 0U, 0U, 8U};
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_lost;
static volatile bool active;
static volatile uint32_t generation;
static UsbConsoleStats_s counters;

static uint32_t Lock(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

static bool Configured(void)
{
    return active && hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED &&
           hUsbDeviceHS.pClassData != NULL;
}

static void NewSession(void)
{
    ++generation;
    if (generation == 0U)
    {
        ++generation;
    }
    ++counters.sessions;
    rx_head = 0U;
    rx_tail = 0U;
    rx_lost = false;
}

static int8_t Init(void)
{
    NewSession();
    counters.dtr = false;
    counters.rts = false;
    active = true;
    USBD_CDC_SetTxBuffer(&hUsbDeviceHS, endpoint_tx, 0U);
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, endpoint_rx);
    return USBD_OK;
}

static int8_t DeInit(void)
{
    active = false;
    rx_head = 0U;
    rx_tail = 0U;
    counters.dtr = false;
    counters.rts = false;
    return USBD_OK;
}

static int8_t Control(uint8_t command, uint8_t *bytes, uint16_t length)
{
    if (command == CDC_SET_LINE_CODING && length == sizeof(line_coding))
    {
        memcpy(line_coding, bytes, sizeof(line_coding));
    }
    else if (command == CDC_GET_LINE_CODING && length == sizeof(line_coding))
    {
        memcpy(bytes, line_coding, sizeof(line_coding));
    }
    else if (command == CDC_SET_CONTROL_LINE_STATE)
    {
        /* ST passes the setup request itself for zero-data control requests. */
        const USBD_SetupReqTypedef *request = (const USBD_SetupReqTypedef *)(const void *)bytes;
        bool dtr = (request->wValue & 1U) != 0U;
        if (dtr != counters.dtr)
        {
            NewSession();
        }
        counters.dtr = dtr;
        counters.rts = (request->wValue & 2U) != 0U;
    }
    return USBD_OK;
}

static int8_t Receive(uint8_t *bytes, uint32_t *length)
{
    uint32_t count = *length;
    counters.rx_bytes += count;
    size_t pending = (rx_head + USB_CONSOLE_RX_SIZE - rx_tail) % USB_CONSOLE_RX_SIZE;
    if (count > sizeof(endpoint_rx) || count > USB_CONSOLE_RX_SIZE - 1U - pending || rx_lost)
    {
        counters.rx_dropped += count;
        ++counters.rx_overflows;
        rx_lost = true;
    }
    else
    {
        for (uint32_t i = 0U; i < count; ++i)
        {
            rx_ring[rx_head] = bytes[i];
            rx_head = (rx_head + 1U) % USB_CONSOLE_RX_SIZE;
        }
    }
    /* Copy first: the next packet may overwrite endpoint_rx after rearming. */
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, endpoint_rx);
    if (USBD_CDC_ReceivePacket(&hUsbDeviceHS) != USBD_OK)
    {
        ++counters.io_errors;
    }
    return USBD_OK;
}

static int8_t TransmitComplete(uint8_t *bytes, uint32_t *length, uint8_t endpoint)
{
    (void)bytes;
    (void)endpoint;
    counters.tx_completed_bytes += *length;
    return USBD_OK;
}

static uint32_t Session(void *context)
{
    (void)context;
    uint32_t previous = Lock();
    uint32_t result = Configured() ? generation : 0U;
    __set_PRIMASK(previous);
    return result;
}

static size_t Read(void *context, uint8_t *bytes, size_t capacity, bool *lost)
{
    (void)context;
    uint32_t previous = Lock();
    *lost = rx_lost;
    if (rx_lost)
    {
        counters.rx_dropped += (rx_head + USB_CONSOLE_RX_SIZE - rx_tail) % USB_CONSOLE_RX_SIZE;
        rx_tail = rx_head;
        rx_lost = false;
    }
    size_t count = 0U;
    /* Bound interrupt masking even if a future caller requests a large read. */
    while (!*lost && count < capacity && count < 64U && rx_tail != rx_head)
    {
        bytes[count++] = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1U) % USB_CONSOLE_RX_SIZE;
    }
    __set_PRIMASK(previous);
    return count;
}

static bool Write(void *context, const uint8_t *bytes, size_t size)
{
    (void)context;
    if (bytes == NULL || size == 0U || size > sizeof(endpoint_tx))
    {
        return false;
    }
    /* Prevent USB reset/DeInit between checking the class and submitting TX.
     * This is one bounded copy + nonblocking USB enqueue, never a busy wait. */
    uint32_t previous = Lock();
    bool accepted = false;
    if (Configured())
    {
        USBD_CDC_HandleTypeDef *cdc = hUsbDeviceHS.pClassData;
        if (cdc->TxState == 0U)
        {
            memcpy(endpoint_tx, bytes, size);
            USBD_CDC_SetTxBuffer(&hUsbDeviceHS, endpoint_tx, (uint32_t)size);
            accepted = USBD_CDC_TransmitPacket(&hUsbDeviceHS) == USBD_OK;
            if (accepted)
            {
                counters.tx_accepted_bytes += (uint32_t)size;
            }
            else
            {
                ++counters.io_errors;
            }
        }
        else
        {
            ++counters.tx_busy;
        }
    }
    __set_PRIMASK(previous);
    return accepted;
}

TextConsolePort_s UsbConsolePortInit(void)
{
    USBD_Interface_fops_HS = (USBD_CDC_ItfTypeDef){Init, DeInit, Control, Receive, TransmitComplete};
    /* A Chinese transfer fits one full-speed packet without a ZLP boundary.
     * ASCII CSV can still use the existing 128-byte application transfer. */
    return (TextConsolePort_s){.session = Session, .read = Read, .write = Write,
                              .utf8_packet_size = CDC_DATA_FS_MAX_PACKET_SIZE - 1U};
}

void UsbConsolePortGetStats(UsbConsoleStats_s *stats)
{
    uint32_t previous = Lock();
    *stats = counters;
    stats->configured = Configured();
    stats->device_state = hUsbDeviceHS.dev_state;
    stats->full_speed = hUsbDeviceHS.dev_speed == USBD_SPEED_FULL;
    stats->rx_pending = (rx_head + USB_CONSOLE_RX_SIZE - rx_tail) % USB_CONSOLE_RX_SIZE;
    __set_PRIMASK(previous);
}
