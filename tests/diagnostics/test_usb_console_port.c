#include "usb_console_port.h"
#include "usbd_cdc_if.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

USBD_HandleTypeDef hUsbDeviceHS;
USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
static USBD_CDC_HandleTypeDef cdc;
static uint32_t primask;
static uint8_t tx_result;
static uint8_t rx_result;
static unsigned submissions;

uint32_t __get_PRIMASK(void)
{
    return primask;
}

void __disable_irq(void)
{
    primask = 1U;
}

void __set_PRIMASK(uint32_t previous)
{
    primask = previous;
}

uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device, uint8_t *bytes, uint32_t length)
{
    assert(device->pClassData != NULL);
    cdc.TxBuffer = bytes;
    cdc.TxLength = length;
    return USBD_OK;
}

uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *device, uint8_t *bytes)
{
    assert(device->pClassData != NULL);
    cdc.RxBuffer = bytes;
    return USBD_OK;
}

uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *device)
{
    assert(primask == 1U && device->pClassData == &cdc && cdc.TxState == 0U);
    ++submissions;
    if (tx_result == USBD_OK)
    {
        cdc.TxState = 1U;
    }
    return tx_result;
}

uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *device)
{
    assert(device->pClassData == &cdc);
    /* Poison immediately on rearm: proves callback copied BEFORE rearming. */
    memset(cdc.RxBuffer, 0xCC, 512U);
    return rx_result;
}

static void Complete(void)
{
    cdc.TxState = 0U;
    USBD_Interface_fops_HS.TransmitCplt(cdc.TxBuffer, &cdc.TxLength, 1U);
}

int main(void)
{
    TextConsolePort_s port = UsbConsolePortInit();
    uint8_t text[] = "hello";
    assert(port.session(NULL) == 0U);
    assert(!port.write(NULL, text, 5U) && submissions == 0U);
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    assert(!port.write(NULL, text, 5U)); /* Configured but NULL class. */
    hUsbDeviceHS.pClassData = &cdc;
    hUsbDeviceHS.dev_speed = USBD_SPEED_FULL;
    USBD_Interface_fops_HS.Init();
    assert(port.session(NULL) != 0U);
    assert(port.write(NULL, text, 5U)); /* No DTR required. */
    memset(text, 'x', 5U);
    assert(memcmp(cdc.TxBuffer, "hello", 5U) == 0);
    assert(!port.write(NULL, text, 5U) && submissions == 1U);
    assert(memcmp(cdc.TxBuffer, "hello", 5U) == 0);
    Complete();
    primask = 1U;
    assert(port.write(NULL, text, 5U) && primask == 1U);
    primask = 0U;
    Complete();
    uint8_t coding[7] = {0};
    USBD_Interface_fops_HS.Control(CDC_GET_LINE_CODING, coding, 7U);
    assert(coding[1] == 0xC2U && coding[6] == 8U);
    coding[0] = 7U;
    USBD_Interface_fops_HS.Control(CDC_SET_LINE_CODING, coding, 7U);
    coding[0] = 0U;
    USBD_Interface_fops_HS.Control(CDC_GET_LINE_CODING, coding, 7U);
    assert(coding[0] == 7U);
    USBD_SetupReqTypedef request = {.wValue = 3U};
    uint32_t session = port.session(NULL);
    USBD_Interface_fops_HS.Control(CDC_SET_CONTROL_LINE_STATE, (uint8_t *)&request, 0U);
    assert(port.session(NULL) != session);
    memcpy(cdc.RxBuffer, "ping\n", 5U);
    uint32_t length = 5U;
    USBD_Interface_fops_HS.Receive(cdc.RxBuffer, &length);
    bool lost = false;
    uint8_t bytes[128] = {0};
    assert(port.read(NULL, bytes, sizeof(bytes), &lost) == 5U && !lost);
    assert(memcmp(bytes, "ping\n", 5U) == 0);
    length = 512U;
    USBD_Interface_fops_HS.Receive(cdc.RxBuffer, &length);
    USBD_Interface_fops_HS.Receive(cdc.RxBuffer, &length);
    assert(port.read(NULL, bytes, sizeof(bytes), &lost) == 0U && lost);
    assert(port.read(NULL, bytes, sizeof(bytes), &lost) == 0U && !lost);
    UsbConsoleStats_s stats;
    UsbConsolePortGetStats(&stats);
    assert(stats.dtr && stats.rts && stats.configured && stats.full_speed);
    assert(stats.rx_dropped == 1024U && stats.rx_overflows == 1U);
    assert(stats.tx_completed_bytes == 10U && stats.tx_busy == 1U);
    tx_result = USBD_FAIL;
    assert(!port.write(NULL, bytes, 1U));
    rx_result = USBD_FAIL;
    length = 1U;
    USBD_Interface_fops_HS.Receive(cdc.RxBuffer, &length);
    UsbConsolePortGetStats(&stats);
    assert(stats.io_errors == 2U);
    USBD_Interface_fops_HS.DeInit();
    hUsbDeviceHS.pClassData = NULL;
    assert(port.session(NULL) == 0U && !port.write(NULL, bytes, 1U));
    hUsbDeviceHS.pClassData = &cdc;
    cdc.TxState = 0U;
    USBD_Interface_fops_HS.Init();
    assert(port.session(NULL) != session);
    UsbConsolePortGetStats(&stats);
    assert(!stats.dtr && stats.tx_completed_bytes == 10U && stats.rx_pending == 0U);
    assert(primask == 0U);
    puts("PASS: real CDC adapter callbacks, no enumeration, async buffer lifetime, busy, RX loss and reconnect");
    return 0;
}
