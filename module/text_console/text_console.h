#ifndef TEXT_CONSOLE_H
#define TEXT_CONSOLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TEXT_CONSOLE_TX_SIZE 8192U
#define TEXT_CONSOLE_LINE_SIZE 256U
#define TEXT_CONSOLE_COMMAND_SIZE 80U
#define TEXT_CONSOLE_PACKET_SIZE 128U

typedef struct
{
    void *context;
    /* 0 means unavailable; a new nonzero generation marks a new session. */
    uint32_t (*session)(void *context);
    /* Nonblocking, at most capacity bytes. On loss discard queued RX and set lost. */
    size_t (*read)(void *context, uint8_t *bytes, size_t capacity, bool *lost);
    /* All-or-none acceptance; MUST copy bytes before returning true. */
    bool (*write)(void *context, const uint8_t *bytes, size_t size);
} TextConsolePort_s;

typedef void (*TextConsoleCommand_f)(void *context, const char *line, uint32_t now_ms);

typedef struct
{
    TextConsolePort_s port;
    TextConsoleCommand_f command;
    void *command_context;
    uint8_t tx[TEXT_CONSOLE_TX_SIZE];
    uint8_t rx[64];
    char line[TEXT_CONSOLE_COMMAND_SIZE];
    size_t tx_head;
    size_t tx_count;
    size_t rx_position;
    size_t rx_count;
    size_t line_length;
    uint32_t session;
    uint32_t last_byte_ms;
    uint32_t dropped_lines;
    uint32_t rejected_commands;
    uint32_t busy_polls;
    bool discard_line;
    bool connected_event;
} TextConsole_s;

/** Pure C instance, single task owner, no allocation or IO in Init.
 * Callbacks/storage outlive the instance. Caller supplies one wrapping ms clock.
 * Poll processes <=64 RX bytes, <=2 commands and <=128 TX bytes per call.
 * Partial/invalid/overlong commands are discarded through the next newline.
 * Session changes discard old output/partial commands and defer RX until the
 * next Poll so the owner can consume the connected event and queue a banner. */
bool TextConsoleInit(TextConsole_s *console, const TextConsolePort_s *port,
                     TextConsoleCommand_f command, void *context);
void TextConsolePoll(TextConsole_s *console, uint32_t now_ms);
bool TextConsoleTakeConnected(TextConsole_s *console);
size_t TextConsolePending(const TextConsole_s *console);
/** Adds one complete CRLF line or drops the entire line and counts it.
 * Max formatted text 253 bytes; use integer units (no target printf float dependency).
 * Task owner only; never call from IRQ/control tasks. */
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 2, 3)))
#endif
bool TextConsolePrint(TextConsole_s *console, const char *format, ...);
/** Saturating diagnostic conversion; nonfinite values return 0 (check validity flags). */
int32_t TextConsoleScaled(double value, double scale);

#endif
