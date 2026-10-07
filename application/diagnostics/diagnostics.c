#include "../bench_features.h"
#include "diagnostics.h"
#include "board_status.h"
#include "usb_console_port.h"
#include "runtime_events.h"
#include "friction.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>

static TextConsole_s console;
static const char *profile;
static DiagnosticsReport_f report_profile;
static uint32_t period_ms = 1000U;
static uint32_t last_report_ms;
static uint32_t report_number;
static uint32_t skipped_reports;
static uint32_t last_poll_ms;
static uint32_t max_poll_gap_ms;
static bool have_poll;
static bool streaming = true;
static EventHistorySnapshot_s event_snapshot; /* Diagnostic task owns this copy, not its stack. */

static void Version(void)
{
    TextConsolePrint(&console, "[firmware] profile=%s console=1 build=%s %s", profile, __DATE__, __TIME__);
    TextConsolePrint(&console, "[USB] friction commands: off; set <mask> <rpm1> <rpm2> <rpm3>; pid <wheel> <kp> <ki> <kd> <limit>; ramp <rpm_s>");
    TextConsolePrint(&console, "[units] mrad=0.001rad; mm_s2=0.001m/s^2; milli=0.001; cC=0.01C; all commands read-only for actuators");
}

static void BoardReport(void)
{
    if (!BENCH_ENABLE_BOARD_DIAGNOSTICS)
    {
        TextConsolePrint(&console, "[board_diagnostics] DISABLED");
        return;
    }
    BoardStatus_s board;
    UsbConsoleStats_s usb;
    BoardStatusRead(&board);
    UsbConsolePortGetStats(&usb);
    TextConsolePrint(&console, "[board] cpu_hz=%lu uid=%08lx-%08lx-%08lx reset_sticky=0x%08lx",
                     (unsigned long)board.cpu_hz, (unsigned long)board.uid[0],
                     (unsigned long)board.uid[1], (unsigned long)board.uid[2], (unsigned long)board.reset_flags);
    TextConsolePrint(&console, "[rtos] heap_free_B=%lu heap_min_B=%lu usb_task_max_gap_ms=%lu stack_watermark=NOT_ENABLED",
                     (unsigned long)board.heap_free, (unsigned long)board.heap_minimum, (unsigned long)max_poll_gap_ms);
    TextConsolePrint(&console, "[power] GPIO_command_only: 24V_1=%u 24V_2=%u 5V=%u voltage=NOT_MEASURED",
                     board.power_24v_1_command, board.power_24v_2_command, board.power_5v_command);
    TextConsolePrint(&console, "[usb] configured=%u state=%u speed=%s DTR=%u RTS=%u sessions=%lu rx_B=%lu rx_pending=%u",
                     usb.configured, usb.device_state, usb.full_speed ? "FULL" : "OTHER", usb.dtr, usb.rts,
                     (unsigned long)usb.sessions, (unsigned long)usb.rx_bytes, usb.rx_pending);
    TextConsolePrint(&console, "[usb] tx_accepted_B=%lu tx_complete_B=%lu busy=%lu rx_drop_B=%lu overflow=%lu io_error=%lu",
                     (unsigned long)usb.tx_accepted_bytes, (unsigned long)usb.tx_completed_bytes,
                     (unsigned long)usb.tx_busy, (unsigned long)usb.rx_dropped,
                     (unsigned long)usb.rx_overflows, (unsigned long)usb.io_errors);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const BoardCanStatus_s *can = &board.can[i];
        const char *state = !can->readable ? "UNAVAILABLE" : !can->started ? "NOT_STARTED" :
                            can->bus_off ? "BUS_OFF" : can->error_passive ? "ERROR_PASSIVE" :
                            can->warning ? "WARNING" : "STARTED_NO_LINK_PROOF";
        TextConsolePrint(&console, "[CAN%u] %s bitrate=%lu HALstate=%lu HALerror=0x%lx TEC=%lu REC=%lu LEC=%lu free=%lu pending=0x%lx",
                         i + 1U, state, (unsigned long)can->bitrate, (unsigned long)can->state,
                         (unsigned long)can->hal_error, (unsigned long)can->tx_error_count,
                         (unsigned long)can->rx_error_count, (unsigned long)can->protocol,
                         (unsigned long)can->tx_free, (unsigned long)can->tx_pending);
        if (can->bus_off || can->error_passive || can->tx_error_count != 0U)
        {
            TextConsolePrint(&console, "[hint CAN%u] check external motor power, common ground, CAN-H/L, termination, bitrate and target ID", i + 1U);
        }
    }
    TextConsolePrint(&console, "[peripheral] UART5_state=0x%lx error=0x%lx SPI2_state=%lu error=0x%lx; HAL ready is not device online",
                     (unsigned long)board.uart5_state, (unsigned long)board.uart5_error,
                     (unsigned long)board.spi2_state, (unsigned long)board.spi2_error);
    TextConsolePrint(&console, "[unmonitored] voltage ADC, RS485, LCD, flash, camera, referee, vision, feeder, pitch motor");
}

static void Report(uint32_t now_ms)
{
    ++report_number;
    uint32_t dropped_before = console.dropped_lines;
    TextConsolePrint(&console, "=== status #%lu profile=%s uptime_ms=%lu ===",
                     (unsigned long)report_number, profile, (unsigned long)now_ms);
    TextConsolePrint(&console, "[modules] usb=%u board_diagnostics=%u rtt_log=%u events=%u",
                     BENCH_ENABLE_USB, BENCH_ENABLE_BOARD_DIAGNOSTICS,
                     BENCH_ENABLE_RTT_LOG, BENCH_ENABLE_EVENT_HISTORY);
    BoardReport();
    report_profile(&console, now_ms);
    TextConsolePrint(&console, "[console] period_ms=%lu stream=%u skipped=%lu dropped_lines=%lu rejected_commands=%lu queue_B=%lu",
                     (unsigned long)period_ms, streaming, (unsigned long)skipped_reports,
                     (unsigned long)console.dropped_lines, (unsigned long)console.rejected_commands,
                     (unsigned long)TextConsolePending(&console));
    TextConsolePrint(&console, "=== end #%lu complete=%u ===", (unsigned long)report_number,
                     console.dropped_lines == dropped_before);
}

static void EventsReport(void)
{
    if (!BENCH_ENABLE_EVENT_HISTORY)
    {
        TextConsolePrint(&console, "[events] DISABLED");
        return;
    }
    RuntimeEventsGet(&event_snapshot);
    TextConsolePrint(&console, "=== events retained=%lu overwritten=%lu storage=RAM_RESET_LOSES_HISTORY ===",
                     (unsigned long)event_snapshot.count, (unsigned long)event_snapshot.overwritten);
    uint32_t dropped = console.dropped_lines;
    for (size_t i = 0U; i < event_snapshot.count; ++i)
    {
        const EventHistoryEntry_s *event = &event_snapshot.entries[i];
        TextConsolePrint(&console, "[event] seq=%lu t_ms=%lu key=%s kind=%s previous=0x%lx value=0x%lx",
                         (unsigned long)event->sequence, (unsigned long)event->timestamp_ms,
                         RuntimeEventsName(event->key), event->initial ? "INITIAL" : "CHANGE",
                         (unsigned long)event->previous, (unsigned long)event->value);
    }
    TextConsolePrint(&console, "=== end events complete=%u ===", dropped == console.dropped_lines);
}

void DiagnosticsPrintTiming(TextConsole_s *output, const char *name, const LoopStats_s *stats)
{
    TextConsolePrint(output, "[%s_timing] calls=%lu interval_ms=%lu max_interval_ms=%lu over_limit=%lu same_ms=%lu (not execution time)",
                     name, (unsigned long)stats->samples, (unsigned long)stats->last_interval_ms,
                     (unsigned long)stats->max_interval_ms, (unsigned long)stats->late_count,
                     (unsigned long)stats->zero_interval_count);
}

static void Command(void *context, const char *line, uint32_t now_ms)
{
    (void)context;
    if (strcmp(line, "events") == 0)
    {
        if (TextConsolePending(&console) == 0U)
        {
            EventsReport();
            last_report_ms = now_ms;
        }
        else
        {
            TextConsolePrint(&console, "[BUSY] previous output draining; retry events");
        }
    }
    else if (strcmp(line, "status") == 0)
    {
        if (TextConsolePending(&console) == 0U)
        {
            Report(now_ms);
            last_report_ms = now_ms;
        }
        else
        {
            TextConsolePrint(&console, "[BUSY] previous report draining; retry status");
        }
    }
    else if (strcmp(line, "ping") == 0)
    {
        TextConsolePrint(&console, "[pong] uptime_ms=%lu", (unsigned long)now_ms);
    }
    else if (strcmp(line, "help") == 0 || strcmp(line, "version") == 0)
    {
        Version();
    }
    else if (strcmp(line, "stream on") == 0 || strcmp(line, "stream off") == 0)
    {
        streaming = strcmp(line, "stream on") == 0;
        TextConsolePrint(&console, "[OK] stream=%u", streaming);
    }
    else if (strcmp(line, "off") == 0)
    {
        FrictionDisable();
        TextConsolePrint(&console, "[OK] friction off");
    }
    else if (strncmp(line, "set ", 4) == 0 || strncmp(line, "pid ", 4) == 0 ||
             strncmp(line, "ramp ", 5) == 0)
    {
        if (FrictionSubmitFromUsbCommand(line, now_ms))
        {
            TextConsolePrint(&console, "[OK] friction command accepted; wait for status enabled_mask before judging output");
        }
        else
        {
            TextConsolePrint(&console, "[ERR] invalid/busy friction command; use set/pid/ramp syntax from help");
        }
    }
    else if (strncmp(line, "period ", 7U) == 0)
    {
        uint32_t value = 0U;
        const char *cursor = line + 7U;
        bool valid = *cursor != '\0';
        while (*cursor != '\0' && valid)
        {
            valid = *cursor >= '0' && *cursor <= '9' && value <= 10000U;
            if (valid)
            {
                value = value * 10U + (uint32_t)(*cursor - '0');
                ++cursor;
            }
        }
        if (valid && value >= 200U && value <= 10000U)
        {
            period_ms = value;
            TextConsolePrint(&console, "[OK] period_ms=%lu", (unsigned long)value);
        }
        else
        {
            TextConsolePrint(&console, "[ERR] period requires integer 200..10000 ms");
        }
    }
    else
    {
        TextConsolePrint(&console, "[ERR] unknown command; send help/status/ping/off/set/pid/ramp");
    }
}

bool DiagnosticsInit(const char *profile_name, DiagnosticsReport_f report)
{
    if (!BENCH_ENABLE_USB)
    {
        return true;
    }
    if (profile_name == NULL || report == NULL)
    {
        return false;
    }
    profile = profile_name;
    report_profile = report;
    TextConsolePort_s port = UsbConsolePortInit();
    return TextConsoleInit(&console, &port, Command, NULL);
}

void DiagnosticsPoll(uint32_t now_ms)
{
    if (!BENCH_ENABLE_USB)
    {
        return;
    }
    if (have_poll && now_ms - last_poll_ms > max_poll_gap_ms)
    {
        max_poll_gap_ms = now_ms - last_poll_ms;
    }
    have_poll = true;
    last_poll_ms = now_ms;
    TextConsolePoll(&console, now_ms);
    if (TextConsoleTakeConnected(&console))
    {
        Version();
        Report(now_ms);
        last_report_ms = now_ms;
    }
    else if (console.session != 0U && streaming && now_ms - last_report_ms >= period_ms)
    {
        last_report_ms = now_ms;
        if (TextConsolePending(&console) == 0U)
        {
            Report(now_ms);
        }
        else
        {
            ++skipped_reports;
        }
    }
}
