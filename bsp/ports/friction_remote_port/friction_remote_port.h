#ifndef FRICTION_REMOTE_PORT_H
#define FRICTION_REMOTE_PORT_H

#include <stdbool.h>
#include <stdint.h>

enum
{
    FRICTION_RC_UART_ERROR = 1U,
    FRICTION_RC_OVERFLOW = 2U
};
typedef struct
{
    uint8_t byte;
    uint32_t received_ms;
} FrictionRcByte_s;
typedef struct
{
    uint32_t bytes;
    uint32_t last_byte_ms;
    uint32_t uart_errors;
    uint32_t overflows;
    bool started;
} FrictionRcPortStats_s;

/** Friction bench only: exclusively owns UART5 byte-IT RX, never CAN/VOFA.
 * Init once before scheduler. USART init already supplies 100k, 8E1, PD2 RX.
 * IRQ enqueues receive timestamps; one task pops. No allocation/DMA/cache IO. */
bool FrictionRcPortInit(void);
bool FrictionRcPortPop(FrictionRcByte_s *byte);
/** Task context: discard corrupt queue and retry UART reception on errors.
 * Caller MUST invalidate decoder and stop. Failed restart remains sticky. */
uint32_t FrictionRcPortTakeFaults(void);
void FrictionRcPortGetStats(FrictionRcPortStats_s *stats);
/** HAL error callback routing: called only for UART5 by bsp_usart.
 * True if this initialized port owns RX; false leaves generic USART routing. */
bool FrictionRcPortOnError(void);

#endif
