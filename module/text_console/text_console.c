#include "text_console.h"
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

bool TextConsoleInit(TextConsole_s *console, const TextConsolePort_s *port,
                     TextConsoleCommand_f command, void *context)
{
    if (console == NULL || port == NULL || port->session == NULL ||
        port->read == NULL || port->write == NULL || command == NULL)
    {
        return false;
    }
    memset(console, 0, sizeof(*console));
    console->port = *port;
    console->command = command;
    console->command_context = context;
    return true;
}

static void Receive(TextConsole_s *console, uint32_t now_ms)
{
    if (console->line_length != 0U && now_ms - console->last_byte_ms > 2000U)
    {
        console->discard_line = true;
        console->line_length = 0U;
        ++console->rejected_commands;
    }
    if (console->rx_position == console->rx_count)
    {
        bool lost = false;
        console->rx_count = console->port.read(console->port.context, console->rx,
                                              sizeof(console->rx), &lost);
        console->rx_position = 0U;
        if (lost || console->rx_count > sizeof(console->rx))
        {
            console->rx_count = 0U;
            console->line_length = 0U;
            console->discard_line = true;
            ++console->rejected_commands;
        }
    }
    unsigned commands = 0U;
    while (console->rx_position < console->rx_count && commands < 2U)
    {
        uint8_t byte = console->rx[console->rx_position++];
        console->last_byte_ms = now_ms;
        if (byte == '\r' || byte == '\n')
        {
            if (!console->discard_line && console->line_length != 0U)
            {
                console->line[console->line_length] = '\0';
                console->command(console->command_context, console->line, now_ms);
                ++commands;
            }
            console->line_length = 0U;
            console->discard_line = false;
        }
        else if (!console->discard_line)
        {
            if (byte < 32U || byte > 126U || console->line_length + 1U >= sizeof(console->line))
            {
                console->discard_line = true;
                console->line_length = 0U;
                ++console->rejected_commands;
            }
            else
            {
                console->line[console->line_length++] = (char)byte;
            }
        }
    }
}

void TextConsolePoll(TextConsole_s *console, uint32_t now_ms)
{
    uint32_t session = console->port.session(console->port.context);
    if (session != console->session)
    {
        console->session = session;
        console->tx_head = 0U;
        console->tx_count = 0U;
        console->rx_count = 0U;
        console->rx_position = 0U;
        console->line_length = 0U;
        console->discard_line = false;
        console->connected_event = session != 0U;
        /* Let the owner publish its session banner before processing commands.
         * Otherwise an immediate status request can enqueue two full reports. */
        return;
    }
    if (session == 0U)
    {
        return;
    }
    if (console->tx_count != 0U)
    {
        size_t count = TEXT_CONSOLE_TX_SIZE - console->tx_head;
        if (count > console->tx_count)
        {
            count = console->tx_count;
        }
        if (count > TEXT_CONSOLE_PACKET_SIZE)
        {
            count = TEXT_CONSOLE_PACKET_SIZE;
        }
        if (console->port.write(console->port.context, console->tx + console->tx_head, count))
        {
            console->tx_head = (console->tx_head + count) % TEXT_CONSOLE_TX_SIZE;
            console->tx_count -= count;
        }
        else
        {
            ++console->busy_polls;
        }
    }
    Receive(console, now_ms);
}

bool TextConsoleTakeConnected(TextConsole_s *console)
{
    bool result = console->connected_event;
    console->connected_event = false;
    return result;
}

size_t TextConsolePending(const TextConsole_s *console)
{
    return console->tx_count;
}

bool TextConsolePrint(TextConsole_s *console, const char *format, ...)
{
    char line[TEXT_CONSOLE_LINE_SIZE];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(line, sizeof(line) - 2U, format, args);
    va_end(args);
    if (console->session == 0U || length < 0 || (size_t)length >= sizeof(line) - 2U ||
        (size_t)length + 2U > TEXT_CONSOLE_TX_SIZE - console->tx_count)
    {
        ++console->dropped_lines;
        return false;
    }
    line[length++] = '\r';
    line[length++] = '\n';
    size_t tail = (console->tx_head + console->tx_count) % TEXT_CONSOLE_TX_SIZE;
    for (int i = 0; i < length; ++i)
    {
        console->tx[tail] = (uint8_t)line[i];
        tail = (tail + 1U) % TEXT_CONSOLE_TX_SIZE;
    }
    console->tx_count += (size_t)length;
    return true;
}

int32_t TextConsoleScaled(double value, double scale)
{
    double result = value * scale;
    if (!isfinite(result))
    {
        return 0;
    }
    if (result >= INT32_MAX)
    {
        return INT32_MAX;
    }
    if (result <= INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)result;
}
