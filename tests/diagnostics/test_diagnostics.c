#include "../../application/bench_features.h"
#include "diagnostics.h"
#include "board_status.h"
#include "usb_console_port.h"
#include "gimbal_diagnostics.h"
#include "friction_diagnostics.h"
#include "gimbal.h"
#include "gimbal_imu.h"
#include "gimbal_port.h"
#include "friction.h"
#include "control_platform.h"
#include "runtime_events.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static char output[32768];
static size_t used;
static char input[128];
static size_t input_size;
static uint32_t session = 1U;
static uint32_t now_ms = 5000U;
static bool busy;
static bool gimbal_profile;
static GimbalStatus_s yaw;
static GimbalImuStatus_s imu;
static GimbalPortStats_s io;
static bool locked;

uint32_t ControlPlatformEnter(void)
{
    assert(!locked);
    locked = true;
    return 0U;
}

void ControlPlatformExit(uint32_t previous)
{
    assert(locked && previous == 0U);
    locked = false;
}

uint32_t ControlPlatformNowMs(void)
{
    return now_ms;
}

void GimbalGetStatus(GimbalStatus_s *status)
{
    *status = yaw;
}

void GimbalImuGetStatus(GimbalImuStatus_s *status)
{
    *status = imu;
}

void GimbalPortGetStats(GimbalPortStats_s *stats)
{
    *stats = io;
}

void FrictionGetTelemetry(Friction_Telemetry_s *telemetry)
{
    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->sample_ms = now_ms;
    telemetry->sample_id = 1U;
    telemetry->status.fault = FRICTION_FAULT_CONFIG;
}

void BoardStatusRead(BoardStatus_s *board)
{
    assert(BENCH_ENABLE_BOARD_DIAGNOSTICS);
    memset(board, 0, sizeof(*board));
    board->cpu_hz = 550000000U;
    board->reset_flags = UINT32_MAX;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        board->uid[i] = UINT32_MAX;
        board->can[i] = (BoardCanStatus_s){.readable = true, .started = true, .bus_off = true,
            .state = UINT32_MAX, .hal_error = UINT32_MAX, .protocol = 7U, .tx_error_count = 255U,
            .rx_error_count = 127U, .tx_free = 32U, .tx_pending = UINT32_MAX, .bitrate = 1000000U};
    }
}

void UsbConsolePortGetStats(UsbConsoleStats_s *usb)
{
    *usb = (UsbConsoleStats_s){.sessions = UINT32_MAX, .rx_bytes = UINT32_MAX,
        .rx_dropped = UINT32_MAX, .rx_overflows = UINT32_MAX, .tx_accepted_bytes = UINT32_MAX,
        .tx_completed_bytes = UINT32_MAX, .tx_busy = UINT32_MAX, .io_errors = UINT32_MAX,
        .rx_pending = 1023U, .device_state = 3U, .configured = true, .full_speed = true};
}

static uint32_t Session(void *context)
{
    (void)context;
    return session;
}

static size_t Read(void *context, uint8_t *bytes, size_t capacity, bool *lost)
{
    (void)context;
    *lost = false;
    size_t count = input_size < capacity ? input_size : capacity;
    memcpy(bytes, input, count);
    memmove(input, input + count, input_size - count);
    input_size -= count;
    return count;
}

static bool Write(void *context, const uint8_t *bytes, size_t count)
{
    (void)context;
    if (busy)
    {
        return false;
    }
    assert(used + count < sizeof(output));
    memcpy(output + used, bytes, count);
    used += count;
    output[used] = '\0';
    return true;
}

TextConsolePort_s UsbConsolePortInit(void)
{
    assert(BENCH_ENABLE_USB);
    return (TextConsolePort_s){.session = Session, .read = Read, .write = Write};
}

static void Profile(TextConsole_s *console, uint32_t stamp)
{
    if (gimbal_profile)
    {
        GimbalDiagnosticsReport(console, stamp);
    }
    else
    {
        FrictionDiagnosticsReport(console, stamp);
    }
}

static void Drain(void)
{
    for (unsigned i = 0U; i < 100U; ++i)
    {
        DiagnosticsPoll(now_ms);
    }
}

static void Send(const char *line)
{
    used = 0U;
    output[0] = '\0';
    input_size = strlen(line);
    assert(input_size < sizeof(input));
    memcpy(input, line, input_size);
    Drain();
}

int main(void)
{
    RuntimeEventsInit();
    assert(DiagnosticsInit("host-fixture", Profile));
#ifdef TEST_MODULE_SWITCHES
    RuntimeEventsObserve(RUNTIME_YAW_FAULT, 1U);
    EventHistorySnapshot_s events;
    RuntimeEventsGet(&events);
    assert(events.count == 0U);
    if (!BENCH_ENABLE_USB)
    {
        Drain();
        assert(used == 0U);
        puts("PASS: USB disabled never binds or polls a port");
        return 0;
    }
    Drain();
    Send("stream off\n");
    Send("status\n");
    assert(strstr(output, "[friction] DISABLED") != NULL);
    assert(strstr(output, "[board_diagnostics] DISABLED") != NULL);
    gimbal_profile = true;
    Send("status\n");
    assert(strstr(output, "[DBUS] DISABLED") != NULL);
    assert(strstr(output, "[DM-J4310] DISABLED") != NULL);
    assert(strstr(output, "[BMI088] DISABLED") != NULL);
    assert(strstr(output, "NO_FEEDBACK") == NULL && strstr(output, "STALE") == NULL);
    Send("events\n");
    assert(strstr(output, "[events] DISABLED") != NULL);
    puts("PASS: disabled modules are explicit, do not read board hardware or record history");
    return 0;
#endif
    strcpy(input, "status\n");
    input_size = strlen(input); /* Host queries immediately as the port opens. */
    Drain();
    assert(strstr(output, "[F1] UNCONFIGURED") != NULL);
    assert(strstr(output, "[CAN1] BUS_OFF") != NULL);
    assert(strstr(output, "voltage=NOT_MEASURED") != NULL);
    assert(strstr(output, "dropped_lines=0") != NULL);
    assert(strstr(output, "complete=1") != NULL);
    assert(strstr(output, "complete=0") == NULL);
    assert(strstr(output, "[BUSY]") != NULL);
    assert(used < 6500U); /* Report + banner fit before the first TX drains. */
    Send("stream off\n");
    assert(strstr(output, "[OK] stream=0") != NULL);
    for (unsigned i = 0U; i < 40U; ++i)
    {
        RuntimeEventsObserve(RUNTIME_FRICTION_FAULT, i);
    }
    Send("events\n");
    assert(strstr(output, "retained=32 overwritten=8") != NULL);
    assert(strstr(output, "key=friction.fault") != NULL);
    assert(strstr(output, "end events complete=1") != NULL);
    assert(used < 6500U);
    Send("period 199\nperiod 10001\n");
    assert(strstr(output, "[ERR]") != NULL && strstr(output, "[OK]") == NULL);
    Send("period 999999999999999999999999999\n");
    assert(strstr(output, "[ERR]") != NULL);
    Send("period 200\n");
    assert(strstr(output, "[OK] period_ms=200") != NULL);
    Send("ping\n");
    assert(strstr(output, "[pong] uptime_ms=5000") != NULL);
    Send("set 7 100 100 100\nreboot\n");
    assert(strstr(output, "unknown read-only command") != NULL);
    Send("status\n");
    assert(strstr(output, "complete=1") != NULL);

    gimbal_profile = true;
    yaw.axis.sample_ms = now_ms;
    yaw.axis.motor_state = DM_VELOCITY_DISCOVER;
    yaw.axis.inhibit = YAW_INHIBIT_COMMAND | YAW_INHIBIT_ACTUATOR;
    imu.sensor_state = BMI088_SENSOR_PROBE;
    imu.attitude.state = ATTITUDE_CALIBRATING;
    Send("status\n");
    assert(strstr(output, "[DBUS] NO_BYTES") != NULL);
    assert(strstr(output, "link=NO_FEEDBACK stage=DISCOVER") != NULL);
    assert(strstr(output, "[BMI088] STARTING") != NULL);
    assert(strstr(output, "for_control=0 mount_confirmed=0") != NULL);
    assert(strstr(output, "[yaw_inhibit] snapshot=FRESH mask=0x6 config=0 command=1 actuator=1") != NULL);
    assert(strstr(output, "complete=1") != NULL);

    yaw.remote.input.valid = true;
    yaw.axis.inhibit = YAW_INHIBIT_REARM_REQUIRED | YAW_INHIBIT_START_NOT_ZERO;
    yaw.remote.input.received_ms = now_ms;
    yaw.axis.feedback.valid = true;
    yaw.axis.feedback.received_ms = now_ms;
    yaw.axis.feedback.count = UINT32_MAX;
    yaw.remote.input.sequence = UINT32_MAX;
    io.uart_bytes = UINT32_MAX;
    io.can_frames = UINT32_MAX;
    io.can_filtered = UINT32_MAX;
    io.tx_accepted = UINT32_MAX;
    io.tx_rejected = UINT32_MAX;
    io.tx_aborted = UINT32_MAX;
    io.uart_errors = UINT32_MAX;
    io.can_errors = UINT32_MAX;
    io.overflows = UINT32_MAX;
    Send("status\n");
    assert(strstr(output, "[DBUS] ONLINE") != NULL);
    assert(strstr(output, "switch_off=0 rearm=1 start_nonzero=1") != NULL);
    assert(strstr(output, "link=ONLINE") != NULL);
    assert(strstr(output, "dropped_lines=0") != NULL);
    assert(strstr(output, "complete=1") != NULL);
    assert(used < 6500U);
    now_ms += 101U;
    Send("status\n");
    assert(strstr(output, "[yaw_task] STALE") != NULL);
    assert(strstr(output, "[yaw_inhibit] snapshot=STALE") != NULL);
    assert(strstr(output, "[DBUS] NO_FRESH_VALID_FRAME") != NULL);
    assert(strstr(output, "link=STALE") != NULL);

    Send("stream on\n");
    busy = true;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        now_ms += 200U;
        DiagnosticsPoll(now_ms);
    }
    busy = false;
    used = 0U;
    Drain();
    assert(strstr(output, "complete=1") != NULL);
    Send("status\n");
    assert(strstr(output, "skipped=19") != NULL);
    assert(strstr(output, "dropped_lines=0") != NULL);
    ++session;
    used = 0U;
    Drain();
    assert(strstr(output, "[firmware]") != NULL && strstr(output, "complete=1") != NULL);
    puts("PASS: real diagnostic commands/profiles, full report size, explicit unknown hardware, stale feedback and bounded backpressure");
    return 0;
}
