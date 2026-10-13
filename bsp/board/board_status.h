#ifndef BOARD_STATUS_H
#define BOARD_STATUS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t state;
    uint32_t hal_error;
    uint32_t protocol;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t tx_free;
    uint32_t tx_pending;
    uint32_t bitrate;
    bool readable;
    bool started;
    bool bus_off;
    bool error_passive;
    bool warning;
} BoardCanStatus_s;

typedef struct
{
    uint32_t cpu_hz;
    uint32_t uid[3];
    uint32_t reset_flags; /* Sticky RCC flags since last clear, NOT necessarily one reset. */
    uint32_t heap_free;
    uint32_t heap_minimum;
    uint32_t uart5_state;
    uint32_t uart5_error;
    uint32_t spi2_state;
    uint32_t spi2_error;
    bool power_24v_1_command;
    bool power_24v_2_command;
    bool power_5v_command;
    BoardCanStatus_s can[3];
} BoardStatus_s;

/** Task-only observation; no allocation, bus transmissions or reconfiguration.
 * CAN PSR/ECR reads clear hardware LEC/CEL history per STM32 semantics; use
 * current bus-off/passive/TEC/REC and driver faults as evidence, not LEC alone.
 * ODR power bits are output commands, never measured voltage or power-good. */
void BoardStatusRead(BoardStatus_s *status);

#endif
