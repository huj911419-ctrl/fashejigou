/* REAL remote producer, bench composition, friction task and USB parser.
 * Fake byte transport and motors: no real peripheral timing/performance proof. */
#include "../../application/bench_features.h"
#include "friction_remote_bench.h"
#include "friction_remote_config.h"
#include "friction_usb_command.h"
#include "control_platform.h"
#include "runtime_events.h"
#include "dbus_fixture.h"
#include <assert.h>
#include <stdio.h>

static uint32_t clock_ms;
static FrictionRcByte_s queue[512];
static unsigned head, tail;
static uint32_t faults;
static unsigned port_starts;
static bool driver_ok = true;
static unsigned tuning_applies;
static DjiSpeedFeedback_s feedback[3];
static bool enabled[3];
static unsigned physical_slot[3];
uint32_t ControlPlatformNowMs(void) { return clock_ms; }
uint32_t ControlPlatformEnter(void) { return 0U; }
void ControlPlatformExit(uint32_t value) { (void)value; }
bool FrictionRcPortInit(void) { ++port_starts; return true; }
bool FrictionRcPortPop(FrictionRcByte_s *byte)
{
    if (head == tail) { return false; }
    *byte = queue[tail++ % 512U];
    return true;
}
uint32_t FrictionRcPortTakeFaults(void)
{
    uint32_t result = faults;
    faults = 0U;
    if (result) { tail = head; }
    return result;
}
void FrictionRcPortGetStats(FrictionRcPortStats_s *value)
{
    *value = (FrictionRcPortStats_s){.started = true, .bytes = head, .last_byte_ms = clock_ms};
}
bool DjiSpeedGroupInit(DjiSpeedGroup_s *group, const DjiSpeedMotorConfig_s *config, unsigned count)
{
    memset(group, 0, sizeof(*group));
    unsigned slot = 0U;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (BENCH_FRICTION_MASK & (1U << i)) { physical_slot[slot++] = i; }
    }
    assert(count == slot);
    memcpy(group->config, config, count * sizeof(*config));
    group->count = count;
    return group->initialized = true;
}
void DjiSpeedGroupRead(const DjiSpeedGroup_s *group, unsigned i, DjiSpeedFeedback_s *value)
{
    (void)group;
    *value = feedback[physical_slot[i]];
}
void DjiSpeedGroupGetOutput(const DjiSpeedGroup_s *group, unsigned i, DjiSpeedOutput_s *value)
{
    (void)group;
    *value = (DjiSpeedOutput_s){.output = enabled[physical_slot[i]] ? 100.0f : 0.0f};
}
void DjiSpeedGroupTune(DjiSpeedGroup_s *group, const DjiSpeedTuning_s *value)
{
    (void)group;
    (void)value;
    ++tuning_applies;
}
void DjiSpeedGroupSet(DjiSpeedGroup_s *group, unsigned i, float rpm, bool enable)
{
    (void)group;
    (void)rpm;
    enabled[physical_slot[i]] = enable;
}
void DjiSpeedGroupStop(DjiSpeedGroup_s *group) { (void)group; memset(enabled, 0, sizeof(enabled)); }
bool DjiSpeedGroupControl(void) { return driver_ok; }
static Friction_Telemetry_s Snapshot(void)
{
    Friction_Telemetry_s sample;
    FrictionGetTelemetry(&sample);
    return sample;
}
static void Tick(void)
{
    ++clock_ms;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        feedback[i] = (DjiSpeedFeedback_s){.valid = true, .last_rx_ms = clock_ms, .temperature_c = 25U};
    }
    FrictionRemoteBenchPoll(clock_ms);
    FrictionTask();
}
static void Advance(unsigned count)
{
    for (unsigned i = 0U; i < count; ++i) { Tick(); }
}
static void Enqueue(uint8_t s1, uint8_t s2, uint16_t vertical, uint32_t time)
{
    uint8_t frame[18];
    DbusFixture(frame, s1, s2, vertical);
    for (unsigned i = 0U; i < 18U; ++i)
    {
        queue[head++ % 512U] = (FrictionRcByte_s){.byte = frame[i], .received_ms = time};
    }
}
static void Frame(uint8_t s1, uint8_t s2, uint16_t vertical)
{
    Advance(7U);
    Enqueue(s1, s2, vertical, clock_ms);
    Tick();
}
static void ArmRun(void)
{
    Frame(1U, 2U, 1024U);
    Frame(1U, 2U, 1024U); /* Also grants consent after NONE reclaim. */
    Frame(3U, 3U, 1024U);
    assert(Snapshot().status.enabled_mask == BENCH_FRICTION_MASK);
}
static void TestGearChanges(void)
{
    const uint8_t switches[3] = {2U, 3U, 1U};
    const float targets[2][3] = {{1000, 2000, 3000}, {5000, 6000, 7000}};
    unsigned applies_before = tuning_applies;
    uint32_t epoch = Snapshot().source_epoch;
    for (unsigned group = 0U; group < 2U; ++group)
    {
        for (unsigned gear = 0U; gear < 3U; ++gear)
        {
            Frame(group == 0U ? 3U : 2U, switches[gear], 1024U);
            Friction_Telemetry_s sample = Snapshot();
            FrictionRemoteBenchStatus_s rc;
            FrictionRemoteBenchGetStatus(&rc);
            assert(sample.status.enabled_mask == BENCH_FRICTION_MASK && sample.source_epoch == epoch);
            assert(rc.remote.gear == group * 3U + gear + 1U);
            assert(rc.gear_changed_ms == rc.remote.input.received_ms && rc.gear_changes != 0U);
            for (unsigned wheel = 0U; wheel < 3U; ++wheel)
            {
                assert(sample.status.requested_rpm[wheel] ==
                       ((BENCH_FRICTION_MASK & (1U << wheel)) ? targets[group][gear] : 0.0f));
            }
        }
    }
    /* Settle the reference at 7000 using fresh input, then test a downshift.
     * Fake feedback stays zero: this checks command/ramp behavior, not ready. */
    for (unsigned i = 0U; i < 450U; ++i)
    {
        Frame(2U, 1U, i % 2U ? 364U : 1684U);
    }
    unsigned slot = physical_slot[0];
    float reference = Snapshot().status.reference_rpm[slot];
    assert(reference == 7000.0f);
    FrictionRemoteBenchStatus_s before, after;
    FrictionRemoteBenchGetStatus(&before);
    Frame(2U, 2U, 1024U);
    FrictionRemoteBenchGetStatus(&after);
    assert(Snapshot().status.enabled_mask == BENCH_FRICTION_MASK);
    assert(Snapshot().status.reference_rpm[slot] == reference - 2.0f); /* First 1ms after new target. */
    assert(after.previous_gear == 6U && after.remote.gear == 4U);
    assert(after.gear_changes == before.gear_changes + 1U);
    assert(tuning_applies == applies_before); /* No PID reinitialization on gears. */
    Frame(1U, 1U, 1684U);
    assert(Snapshot().status.enabled_mask == 0U && Snapshot().status.reference_rpm[slot] == 0.0f);
    ArmRun();
}
int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--wrap") == 0)
    {
        clock_ms = UINT32_MAX - 40U;
    }
    RuntimeEventsInit();
    assert(FrictionInit());
    FrictionRemoteBenchInit();
    Tick();
    if (!BENCH_FRICTION_REMOTE_ACTIVE)
    {
        assert(port_starts == 0U);
        assert(FrictionSelectSource(FRICTION_SOURCE_REMOTE) == FRICTION_SUBMIT_UNAVAILABLE);
        puts("PASS: disabled remote/group does not start UART or select remote authority");
        return 0;
    }
    assert(port_starts == 1U);
    if (BENCH_ENABLE_USB)
    {
        assert(Snapshot().command_source == FRICTION_SOURCE_REMOTE);
        Frame(2U, 1U, 1684U);
        assert(Snapshot().status.enabled_mask == 0U);
        assert(FrictionSelectSource(FRICTION_SOURCE_USB) == FRICTION_SUBMIT_ACCEPTED);
        assert(FrictionUsbExecute("off", clock_ms) == FRICTION_USB_QUEUED);
        Tick();
        Advance(400U); /* Explicit idle USB owner is independent of remote. */
        assert(!Snapshot().status.fault && !Snapshot().status.rearm_required);
        Frame(1U, 2U, 1024U);
        Frame(2U, 1U, 1684U);
        assert(Snapshot().command_source == FRICTION_SOURCE_USB && Snapshot().status.enabled_mask == 0U);
        char first_command[80];
        snprintf(first_command, sizeof(first_command), "set %u 500 500 500", BENCH_FRICTION_MASK);
        assert(FrictionUsbExecute(first_command, clock_ms) == FRICTION_USB_QUEUED);
        Tick();
        assert(Snapshot().status.enabled_mask == BENCH_FRICTION_MASK);
        faults = FRICTION_RC_UART_ERROR;
        Tick();
        assert(Snapshot().status.enabled_mask == BENCH_FRICTION_MASK && !Snapshot().status.fault);
        Frame(1U, 2U, 1024U);
        assert(Snapshot().command_source == FRICTION_SOURCE_USB);
        assert(Snapshot().status.enabled_mask == BENCH_FRICTION_MASK);
        assert(FrictionSelectSource(FRICTION_SOURCE_REMOTE) == FRICTION_SUBMIT_ACCEPTED);
        assert(Snapshot().source_requires_off);
        Tick();
        assert(Snapshot().status.enabled_mask == 0U);
    }
    assert(Snapshot().command_source == FRICTION_SOURCE_REMOTE);
    Frame(2U, 1U, 1684U);
    Frame(2U, 1U, 1684U);
    assert(Snapshot().status.enabled_mask == 0U);
    ArmRun();
    for (unsigned i = 0U; i < 3U; ++i)
    {
        assert(Snapshot().status.requested_rpm[i] == ((BENCH_FRICTION_MASK & (1U << i)) ? 2000.0f : 0.0f));
    }
    TestGearChanges();
    assert(FrictionUsbExecute("set 7 500 500 500", clock_ms) ==
           (BENCH_ENABLE_USB ? FRICTION_USB_WRONG_SOURCE : FRICTION_USB_UNAVAILABLE));
    uint32_t receive_ms = Snapshot().sample_ms - Snapshot().status.command_age_ms;
    while (clock_ms - receive_ms < 100U) { Tick(); }
    assert(Snapshot().status.enabled_mask == BENCH_FRICTION_MASK);
    Tick();
    assert(Snapshot().status.enabled_mask == 0U); /* Stops at 101ms without invented keepalive. */
    Frame(2U, 1U, 1684U);
    assert(Snapshot().status.enabled_mask == 0U);
    ArmRun();
    if (!BENCH_ENABLE_USB)
    {
        faults = FRICTION_RC_UART_ERROR;
        Tick();
        Frame(2U, 1U, 1684U);
        assert(Snapshot().status.enabled_mask == 0U);
        ArmRun();
        puts("PASS: USB-disabled build defaults to remote with OFF rearm, receive-time lease and UART error stop");
        return 0;
    }
    assert(FrictionUsbExecute("off", clock_ms) == FRICTION_USB_QUEUED);
    const Friction_Command_s race_run = {.enable_mask = BENCH_FRICTION_MASK, .target_rpm = {500, 500, 500}};
    assert(FrictionSubmitFrom(FRICTION_SOURCE_REMOTE, &race_run, clock_ms) == FRICTION_SUBMIT_WRONG_SOURCE);
    Tick();
    Frame(2U, 1U, 1684U);
    assert(Snapshot().status.enabled_mask == 0U && Snapshot().command_source == FRICTION_SOURCE_NONE);
    ArmRun();
    assert(FrictionSelectSource(FRICTION_SOURCE_USB) == FRICTION_SUBMIT_ACCEPTED);
    Tick();
    Frame(1U, 2U, 1024U);
    assert(FrictionUsbExecute("off", clock_ms) == FRICTION_USB_QUEUED);
    Tick();
    Frame(1U, 2U, 1024U);
    assert(Snapshot().command_source == FRICTION_SOURCE_USB); /* Remote cannot steal USB off -> set. */
    char command[80];
    snprintf(command, sizeof(command), "set %u 500 500 500", BENCH_FRICTION_MASK);
    assert(FrictionUsbExecute(command, clock_ms) == FRICTION_USB_QUEUED);
    Tick();
    assert(Snapshot().status.enabled_mask == BENCH_FRICTION_MASK);
    Frame(2U, 1U, 1684U);
    assert(Snapshot().status.requested_rpm[physical_slot[0]] == 500.0f);
    Enqueue(1U, 2U, 1024U, clock_ms - 12U);
    Enqueue(2U, 1U, 1684U, clock_ms - 4U); /* Old queued OFF/ON cannot authorize a handoff. */
    assert(FrictionSelectSource(FRICTION_SOURCE_REMOTE) == FRICTION_SUBMIT_ACCEPTED);
    Tick();
    Frame(2U, 1U, 1684U);
    assert(Snapshot().status.enabled_mask == 0U);
    ArmRun();
    faults = FRICTION_RC_UART_ERROR;
    Tick();
    assert(Snapshot().status.enabled_mask == 0U);
    Frame(2U, 1U, 1684U);
    assert(Snapshot().status.enabled_mask == 0U);
    ArmRun();
    /* Both full frames in ONE poll: late RUN cannot erase STOP. */
    Advance(7U);
    Enqueue(1U, 2U, 1024U, clock_ms - 4U);
    Enqueue(2U, 1U, 1684U, clock_ms);
    Tick();
    assert(Snapshot().status.enabled_mask == 0U);
    Frame(3U, 3U, 1024U);
    assert(Snapshot().status.enabled_mask == BENCH_FRICTION_MASK);
    driver_ok = false;
    Tick();
    driver_ok = true;
    Frame(2U, 1U, 1684U);
    assert(Snapshot().status.enabled_mask == 0U);
    ArmRun();
    /* Fresh processing does not make an old queued OFF/ON fresh input. */
    Advance(150U);
    Enqueue(1U, 2U, 1024U, clock_ms - 120U);
    Enqueue(2U, 1U, 1684U, clock_ms - 110U);
    Tick();
    assert(Snapshot().status.enabled_mask == 0U);
    Frame(2U, 1U, 1684U);
    assert(Snapshot().status.enabled_mask == 0U);
    clock_ms += 0x80000000U; /* Long inactive interval, beyond signed half-range. */
    Tick();
    ArmRun();
    puts("PASS: real wheel task + remote producer, receive-time timeout, USB OFF/authority races, source switching, driver fault rearm, old/batched inputs, fixed wheel mask");
    return 0;
}
