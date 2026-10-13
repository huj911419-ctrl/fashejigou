#ifndef GIMBAL_PORT_H
#define GIMBAL_PORT_H

#include "dm_velocity.h"

enum
{
    GIMBAL_PORT_UART_ERROR = 1U,
    GIMBAL_PORT_CAN_ERROR = 2U,
    GIMBAL_PORT_RX_OVERFLOW = 4U
};

typedef struct
{
    uint8_t byte;
    uint32_t received_ms;
} GimbalRcByte_s;

typedef struct
{
    uint32_t uart_bytes;
    uint32_t uart_last_ms;
    uint32_t can_frames;
    uint32_t can_last_ms;
    uint32_t can_filtered;
    uint32_t uart_errors;
    uint32_t can_errors;
    uint32_t overflows;
    uint32_t tx_accepted;
    uint32_t tx_rejected;
    uint32_t tx_aborted;
} GimbalPortStats_s;

/** Task-only coherent history, counters wrap at 2^32. Accepted TX only means
 * queued locally; raw RX counts do not mean a valid remote/motor response. */
void GimbalPortGetStats(GimbalPortStats_s *stats);

/** H723 standalone binding: CAN1 + UART5 (DBUS). Call once before task start.
 * This build owns their HAL callbacks; exclude legacy bsp_can/bsp_usart.
 * IRQs only enqueue; the control task pops and parses. No DMA cache concerns.
 */
bool GimbalPortInit(void);
/** Select resources once before tasks start. False means no start/receive/send
 * on that resource, not an offline fault. Never use this as a runtime toggle.
 * CAN1 remains exclusively owned; UART5 can run without the motor. */
bool GimbalPortInitModules(bool use_can, bool use_remote);
bool GimbalPortSend(void *context, const DmCanFrame_s *frame);
bool GimbalPortPopCan(DmCanFrame_s *frame);
bool GimbalPortPopRc(GimbalRcByte_s *byte);
/** Task context. Returns/clears sticky errors, discards queued data on error
 * and retries UART reception. The caller must invalidate input and disarm.
 */
uint32_t GimbalPortTakeFaults(void);

#endif
