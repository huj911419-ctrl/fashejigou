#ifndef DBUS_FIXTURE_H
#define DBUS_FIXTURE_H

#include <stdint.h>
#include <string.h>
static void DbusFixture(uint8_t frame[18], uint8_t s1, uint8_t s2, uint16_t vertical)
{
    memset(frame, 0, 18U);
    uint16_t raw[4] = {1024U, vertical, 1024U, 1024U};
    for (unsigned channel = 0U; channel < 4U; ++channel)
    {
        for (unsigned bit = 0U; bit < 11U; ++bit)
        {
            unsigned position = channel * 11U + bit;
            frame[position / 8U] |= (uint8_t)(((raw[channel] >> bit) & 1U) << (position % 8U));
        }
    }
    frame[5] |= (uint8_t)(s1 << 6U | s2 << 4U);
}

#endif
