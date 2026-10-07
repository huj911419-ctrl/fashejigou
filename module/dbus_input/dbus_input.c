#include "dbus_input.h"
#include <string.h>

#define DBUS_GAP_MS 2U
#define DBUS_CHANNEL_MIN 364U
#define DBUS_CHANNEL_MAX 1684U

void DbusDecoderInit(DbusDecoder_s *decoder)
{
    memset(decoder, 0, sizeof(*decoder));
}

void DbusDecoderInvalidate(DbusDecoder_s *decoder)
{
    decoder->count = 0U;
    decoder->synchronized = false;
    decoder->input.valid = false;
}

static bool Decode(DbusDecoder_s *decoder, uint32_t now)
{
    const uint8_t *b = decoder->frame;
    uint16_t raw[DBUS_CHANNEL_COUNT];
    raw[0] = (b[0] | ((uint16_t)b[1] << 8)) & 0x7FFU;
    raw[1] = ((b[1] >> 3) | ((uint16_t)b[2] << 5)) & 0x7FFU;
    raw[2] = ((b[2] >> 6) | ((uint16_t)b[3] << 2) | ((uint16_t)b[4] << 10)) & 0x7FFU;
    raw[3] = ((b[4] >> 1) | ((uint16_t)b[5] << 7)) & 0x7FFU;
    uint8_t s1 = (b[5] >> 6) & 3U;
    uint8_t s2 = (b[5] >> 4) & 3U;
    for (unsigned i = 0U; i < DBUS_CHANNEL_COUNT; ++i)
    {
        if (raw[i] < DBUS_CHANNEL_MIN || raw[i] > DBUS_CHANNEL_MAX)
        {
            return false;
        }
    }
    if (s1 == 0U || s2 == 0U || b[12] > 1U || b[13] > 1U)
    {
        return false;
    }
    for (unsigned i = 0U; i < DBUS_CHANNEL_COUNT; ++i)
    {
        decoder->input.channel[i] = ((float)raw[i] - DBUS_CHANNEL_CENTER) / DBUS_CHANNEL_SPAN;
    }
    decoder->input.s1 = s1;
    decoder->input.s2 = s2;
    decoder->input.keys = b[14] | ((uint16_t)b[15] << 8);
    decoder->input.received_ms = now;
    ++decoder->input.sequence;
    decoder->input.valid = true;
    return true;
}

bool DbusDecoderPush(DbusDecoder_s *decoder, uint8_t byte, uint32_t received_ms)
{
    if (decoder->seen_byte && received_ms - decoder->last_byte_ms > DBUS_GAP_MS)
    {
        decoder->count = 0U;
        decoder->synchronized = true;
    }
    decoder->seen_byte = true;
    decoder->last_byte_ms = received_ms;
    if (!decoder->synchronized)
    {
        return false;
    }
    decoder->frame[decoder->count++] = byte;
    if (decoder->count != DBUS_FRAME_SIZE)
    {
        return false;
    }
    decoder->count = 0U;
    decoder->synchronized = false; /* Never slide through arbitrary bytes. */
    bool valid = Decode(decoder, received_ms);
    if (!valid)
    {
        decoder->input.valid = false;
    }
    return valid;
}
