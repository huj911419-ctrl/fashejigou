#include "text_console.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static TextConsole_s console;
static uint32_t session;
static bool busy;
static bool lost;
static uint8_t input[512];
static size_t input_count;
static char output[32768];
static size_t output_count;
static unsigned commands;
static char last_command[80];

static uint32_t Session(void *context)
{
    (void)context;
    return session;
}

static size_t Read(void *context, uint8_t *bytes, size_t capacity, bool *loss)
{
    (void)context;
    *loss = lost;
    lost = false;
    size_t count = capacity < input_count ? capacity : input_count;
    memcpy(bytes, input, count);
    memmove(input, input + count, input_count - count);
    input_count -= count;
    return count;
}

static bool Write(void *context, const uint8_t *bytes, size_t count)
{
    (void)context;
    assert(count <= TEXT_CONSOLE_PACKET_SIZE);
    if (busy)
    {
        return false;
    }
    assert(output_count + count < sizeof(output));
    memcpy(output + output_count, bytes, count);
    output_count += count;
    output[output_count] = '\0';
    return true;
}

static void Command(void *context, const char *line, uint32_t now_ms)
{
    (void)context;
    (void)now_ms;
    ++commands;
    strcpy(last_command, line);
}

static void Feed(const char *text, uint32_t now_ms)
{
    input_count = strlen(text);
    assert(input_count <= sizeof(input));
    memcpy(input, text, input_count);
    TextConsolePoll(&console, now_ms);
}

int main(void)
{
    TextConsolePort_s port = {.session = Session, .read = Read, .write = Write};
    assert(!TextConsoleInit(NULL, &port, Command, NULL));
    assert(TextConsoleInit(&console, &port, Command, NULL));
    assert(!TextConsolePrint(&console, "disconnected"));
    session = 1U;
    TextConsolePoll(&console, 0U);
    assert(TextConsoleTakeConnected(&console));
    assert(!TextConsoleTakeConnected(&console));
    assert(TextConsolePrint(&console, "hello %u", 17U));
    busy = true;
    TextConsolePoll(&console, 1U);
    assert(TextConsolePending(&console) == 10U && console.busy_polls == 1U);
    busy = false;
    TextConsolePoll(&console, 2U);
    assert(strcmp(output, "hello 17\r\n") == 0);
    assert(TextConsolePending(&console) == 0U);

    Feed("sta", 3U);
    assert(commands == 0U);
    Feed("tus\r\nping\nversion\n", 4U);
    assert(commands == 2U && strcmp(last_command, "ping") == 0);
    TextConsolePoll(&console, 5U);
    assert(commands == 3U && strcmp(last_command, "version") == 0);
    Feed("partial", UINT32_MAX - 10U);
    Feed("\n", 10U);
    assert(commands == 4U); /* Clock wrap is not a timeout. */
    Feed("slow", 20U);
    Feed("tail\n", 2021U);
    assert(commands == 4U && console.rejected_commands == 1U);
    char long_line[120];
    memset(long_line, 'x', sizeof(long_line) - 1U);
    long_line[sizeof(long_line) - 1U] = '\0';
    Feed(long_line, 2022U);
    TextConsolePoll(&console, 2023U);
    Feed("ping\n", 2024U);
    assert(commands == 4U);
    Feed("ping\n", 2025U);
    assert(commands == 5U);
    Feed("a\001ping\n", 2026U);
    assert(commands == 5U);
    lost = true;
    Feed("broken", 2027U);
    Feed("ping\n", 2028U);
    assert(commands == 5U);
    Feed("ping\n", 2029U);
    assert(commands == 6U);

    Feed("old", 2030U);
    assert(TextConsolePrint(&console, "old output"));
    session = 0U;
    TextConsolePoll(&console, 2031U);
    assert(TextConsolePending(&console) == 0U);
    session = 2U;
    Feed("new\n", 2032U);
    assert(commands == 6U); /* Owner gets the connection event before commands. */
    TextConsolePoll(&console, 2033U);
    assert(commands == 7U && strcmp(last_command, "new") == 0);
    assert(strstr(output, "old output") == NULL);

    output_count = 0U;
    for (unsigned i = 0U; i < 1200U; ++i)
    {
        assert(TextConsolePrint(&console, "line%04u", i));
        TextConsolePoll(&console, 2040U + i);
    }
    assert(output_count == 12000U && strstr(output, "line1199\r\n") != NULL);
    busy = true;
    while (TextConsolePrint(&console, "whole line"))
    {
        TextConsolePoll(&console, 4000U);
    }
    assert(TextConsolePending(&console) <= TEXT_CONSOLE_TX_SIZE);
    size_t before = TextConsolePending(&console);
    assert(!TextConsolePrint(&console, "%0300u", 0U));
    assert(TextConsolePending(&console) == before);
    assert(TextConsoleScaled(NAN, 1.0) == 0);
    assert(TextConsoleScaled(INFINITY, 1.0) == 0);
    assert(TextConsoleScaled(1e20, 1.0) == INT32_MAX);
    assert(TextConsoleScaled(-1e20, 1.0) == INT32_MIN);
    assert(TextConsoleScaled(-1.25, 1000.0) == -1250);
    puts("PASS: bounded console, backpressure, wraparound, reconnect, malformed/lost input and conversion");
    return 0;
}
