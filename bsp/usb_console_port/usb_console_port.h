#ifndef USB_CONSOLE_PORT_H
#define USB_CONSOLE_PORT_H

#include "text_console.h"

typedef struct
{
    uint32_t sessions;
    uint32_t rx_bytes;
    uint32_t rx_dropped;
    uint32_t rx_overflows;
    uint32_t tx_accepted_bytes;
    uint32_t tx_completed_bytes;
    uint32_t tx_busy;
    uint32_t io_errors;
    uint16_t rx_pending;
    uint8_t device_state;
    bool configured;
    bool dtr;
    bool rts;
    bool full_speed;
} UsbConsoleStats_s;

/** Install once BEFORE MX_USB_DEVICE_Init / scheduler. Exclusive CDC owner:
 * do not use USBInit, CDCInitRxbufferNcallback or CDC_Transmit_HS concurrently.
 * Replaces the generated interface callback table without editing vendor files.
 * Current H723 port requires USB DMA disabled; no cache coherency assumed.
 * Returned callbacks are nonblocking, task-only. RX IRQ only copies bytes.
 * DTR is observable, not required (some serial terminals never assert it). */
TextConsolePort_s UsbConsolePortInit(void);
/** Task context, coherent copy. Counters wrap at 2^32 and persist across sessions.
 * Completed USB transfer does not prove that a PC application consumed it. */
void UsbConsolePortGetStats(UsbConsoleStats_s *stats);

#endif
