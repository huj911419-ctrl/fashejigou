#ifndef DBUS_INPUT_H
#define DBUS_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#define DBUS_FRAME_SIZE 18U
#define DBUS_CHANNEL_COUNT 4U
#define DBUS_CHANNEL_CENTER 1024U
#define DBUS_CHANNEL_SPAN 660.0f

typedef struct
{
    float channel[DBUS_CHANNEL_COUNT]; /* Normalized [-1, 1]. */
    uint8_t s1;
    uint8_t s2;
    uint16_t keys;
    uint32_t received_ms;
    uint32_t sequence;
    bool valid;
} DbusInput_s;

typedef struct
{
    uint8_t frame[DBUS_FRAME_SIZE];
    uint8_t count;
    uint32_t last_byte_ms;
    bool seen_byte;
    bool synchronized;
    DbusInput_s input;
} DbusDecoder_s;

/** Pure protocol, no HAL/RTOS. One caller owns each instance.
 * Push uses the byte's RECEIVE time, not the consumer's processing time.
 * Startup/errors wait for an inter-frame gap >2 ms before accepting a frame.
 * Returns true only for a complete valid frame. DBUS has no CRC.
 */
void DbusDecoderInit(DbusDecoder_s *decoder);
bool DbusDecoderPush(DbusDecoder_s *decoder, uint8_t byte, uint32_t received_ms);
/** Invalidate immediately after UART error or lost bytes; wait for a new gap. */
void DbusDecoderInvalidate(DbusDecoder_s *decoder);

#endif
