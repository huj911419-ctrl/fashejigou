#include "friction.h"
#include "../../application/bench_features.h"
#include "friction_usb_command.h"
#include "control_platform.h"
#include "runtime_events.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t clock_ms;
static bool fresh_feedback = true;
static DjiSpeedFeedback_s feedback[3];
static bool enabled[3];
static DjiSpeedTuning_s applied[3];
uint32_t ControlPlatformNowMs(void) { return clock_ms; }
uint32_t ControlPlatformEnter(void) { return 0U; }
void ControlPlatformExit(uint32_t value) { (void)value; }
bool DjiSpeedGroupInit(DjiSpeedGroup_s *group, const DjiSpeedMotorConfig_s *config, unsigned count)
{
    memset(group, 0, sizeof(*group));
    assert(count == 3U);
    memcpy(group->config, config, count * sizeof(*config));
    group->count = count;
    return group->initialized = true;
}
void DjiSpeedGroupRead(const DjiSpeedGroup_s *group, unsigned i, DjiSpeedFeedback_s *value)
{
    (void)group;
    *value = feedback[i];
}
void DjiSpeedGroupGetOutput(const DjiSpeedGroup_s *group, unsigned i, DjiSpeedOutput_s *value)
{
    (void)group;
    *value = (DjiSpeedOutput_s){.output = enabled[i] ? 123.0f : 0.0f};
}
void DjiSpeedGroupTune(DjiSpeedGroup_s *group, const DjiSpeedTuning_s *value)
{
    memcpy(applied, value, group->count * sizeof(*value));
}
void DjiSpeedGroupSet(DjiSpeedGroup_s *group, unsigned i, float rpm, bool enable)
{
    (void)group;
    (void)rpm;
    enabled[i] = enable;
}
void DjiSpeedGroupStop(DjiSpeedGroup_s *group) { (void)group; memset(enabled, 0, sizeof(enabled)); }
bool DjiSpeedGroupControl(void) { return true; }
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
        if (fresh_feedback)
        {
            feedback[i] = (DjiSpeedFeedback_s){.valid = true, .last_rx_ms = clock_ms, .temperature_c = 25U};
        }
    }
    FrictionTask();
}
int main(int argc, char **argv)
{
    bool idle_start = argc == 2 && strcmp(argv[1], "--idle") == 0;
    bool offline_start = argc == 2 && strcmp(argv[1], "--offline") == 0;
    if (argc == 2 && strcmp(argv[1], "--wrap") == 0)
    {
        clock_ms = UINT32_MAX - 5U;
    }
    RuntimeEventsInit();
    assert(FrictionInit());
    fresh_feedback = !offline_start;
    if (BENCH_FRICTION_REMOTE_ACTIVE)
    {
        assert(Snapshot().command_source == FRICTION_SOURCE_REMOTE);
        assert(FrictionUsbExecute("set 7 500 500 500", clock_ms) == FRICTION_USB_WRONG_SOURCE);
        assert(FrictionSelectSource(FRICTION_SOURCE_USB) == FRICTION_SUBMIT_ACCEPTED);
        assert(FrictionUsbExecute("off", clock_ms) == FRICTION_USB_QUEUED);
        Tick();
    }
    assert(Snapshot().command_source == FRICTION_SOURCE_USB);
    assert(Snapshot().status.enabled_mask == 0U && !enabled[0] && !enabled[1] && !enabled[2]);
    assert(!Snapshot().status.fault && !Snapshot().status.rearm_required);
    if (idle_start)
    {
        for (unsigned i = 0U; i < 500U; ++i) { Tick(); }
        assert(!Snapshot().status.fault && !Snapshot().status.rearm_required);
    }
    fresh_feedback = !offline_start;
    /* First USB RUN after explicit source selection and OFF. */
    assert(FrictionUsbExecute("set 7 500 500 500", clock_ms) == FRICTION_USB_QUEUED);
    Tick();
    if (offline_start)
    {
        assert(Snapshot().status.active_fault & FRICTION_FAULT_OFFLINE);
        assert(Snapshot().status.enabled_mask == 0U && Snapshot().status.rearm_required);
        fresh_feedback = true;
        assert(FrictionInit()); /* Repeated init is not fault recovery. */
        assert(FrictionUsbExecute("set 7 500 500 500", clock_ms) == FRICTION_USB_QUEUED);
        Tick();
        assert(Snapshot().status.enabled_mask == 0U && Snapshot().status.rearm_required);
        assert(FrictionUsbExecute("off", clock_ms) == FRICTION_USB_QUEUED);
        Tick();
        assert(FrictionUsbExecute("set 7 500 500 500", clock_ms) == FRICTION_USB_QUEUED);
        Tick();
        assert(Snapshot().status.enabled_mask == 7U && !Snapshot().status.fault);
        puts("PASS: direct startup RUN still requires feedback; repeated init cannot clear a fault; explicit OFF recovers");
        return 0;
    }
    assert(Snapshot().status.enabled_mask == 7U && Snapshot().status.requested_mask == 7U);
    assert(Snapshot().command_generation != 0U && Snapshot().command_received_now);
    assert(FrictionInit());
    assert(Snapshot().status.enabled_mask == 7U); /* Idempotent init cannot stop or reset a run. */
    for (unsigned i = 0U; i < 249U; ++i) { Tick(); }
    assert(Snapshot().status.command_age_ms == 250U && Snapshot().status.enabled_mask == 7U);
    Tick();
    assert(Snapshot().status.enabled_mask == 0U && Snapshot().status.rearm_required);
    assert(Snapshot().status.active_fault == FRICTION_FAULT_COMMAND);
    assert(FrictionInit());
    assert(Snapshot().status.rearm_required); /* Boot-only OFF cannot be replayed at runtime. */
    assert(FrictionUsbExecute("set 7 500 500 500", clock_ms) == FRICTION_USB_QUEUED);
    Tick();
    assert(Snapshot().status.active_fault == 0U && Snapshot().status.fault == FRICTION_FAULT_COMMAND);
    assert(Snapshot().status.enabled_mask == 0U); /* Accepted RUN cannot clear latched recovery. */
    assert(FrictionUsbExecute("off", clock_ms) == FRICTION_USB_QUEUED);
    Tick();
    assert(FrictionUsbExecute("pid 1 3 0.1 0 12000", clock_ms) == FRICTION_USB_QUEUED);
    assert(Snapshot().tuning_revision == 0U && Snapshot().tuning_pending_now);
    Tick();
    assert(Snapshot().tuning_revision == 1U && !Snapshot().tuning_pending_now);
    assert(applied[0].kp == 3.0f && Snapshot().tuning.motor[0].current_limit == 12000.0f);
    assert(FrictionUsbExecute("set 1 8000 0 0", clock_ms) == FRICTION_USB_QUEUED);
    Tick();
    assert(Snapshot().status.active_fault & FRICTION_FAULT_COMMAND);
    assert(Snapshot().status.requested_rpm[0] == 8000.0f && Snapshot().status.enabled_mask == 0U);
    assert(FrictionUsbExecute("pid 1 3 0.1 0 15000", clock_ms) == FRICTION_USB_RANGE);
    assert(FrictionUsbExecute("set 1 nan 0 0", clock_ms) == FRICTION_USB_SYNTAX);
    puts("PASS: explicit USB selection before first RUN, idle/wrapped startup, 250ms lease boundary, fault recovery OFF, atomic applied revision and unchanged range interlock");
    return 0;
}
