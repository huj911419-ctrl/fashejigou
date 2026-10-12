#include "../bench_features.h"
#include "friction_remote_bench.h"
#include "friction_remote_config.h"
#include "friction_config.h"
#include "control_platform.h"

#define RC_POLL_BYTE_BUDGET 64U
static FrictionRemote_s remote;
static FrictionRemoteBenchStatus_s published;
static uint32_t observed_epoch;
static uint32_t handoff_ms;
static bool handoff_filter;
static bool initialized;
static uint32_t accepted_frames;
static uint32_t gear_changed_ms;
static uint32_t gear_changes;
static uint8_t observed_gear;
static uint8_t previous_gear;
static Friction_Submit_Result_e last_submit = FRICTION_SUBMIT_UNAVAILABLE;

void FrictionRemoteBenchInit(void)
{
    if (!BENCH_FRICTION_REMOTE_ACTIVE)
    {
        return;
    }
    const FrictionRemoteConfig_s config = FRICTION_REMOTE_CONFIG;
    const Friction_Control_Config_s limits = FRICTION_CONTROL_CONFIG;
    initialized = config.max_rpm <= limits.max_rpm && config.timeout_ms < limits.command_timeout_ms &&
                  FrictionRemoteInit(&remote, &config);
    if (initialized)
    {
        (void)FrictionRcPortInit(); /* Errors are visible; retry occurs in Poll. */
        /* Starting RX only observes input; FrictionInit owns startup policy. */
    }
    Friction_Telemetry_s sample;
    FrictionGetTelemetry(&sample);
    observed_epoch = sample.source_epoch;
    handoff_ms = ControlPlatformNowMs();
    handoff_filter = true;
    published = (FrictionRemoteBenchStatus_s){
        .remote = remote.status, .sample_ms = handoff_ms,
        .initialized = initialized, .last_submit = last_submit,
    };
    FrictionRcPortGetStats(&published.port);
}

void FrictionRemoteBenchPoll(uint32_t now_ms)
{
    if (!BENCH_FRICTION_REMOTE_ACTIVE || !initialized)
    {
        return;
    }
    Friction_Telemetry_s sample;
    FrictionGetTelemetry(&sample);
    if (sample.source_epoch != observed_epoch)
    {
        observed_epoch = sample.source_epoch;
        handoff_ms = now_ms;
        handoff_filter = true;
        FrictionRemoteInvalidate(&remote);
    }
    if (FrictionRcPortTakeFaults() != 0U)
    {
        FrictionRemoteInvalidate(&remote);
    }
    if (sample.status.fault && sample.status.rearm_required)
    {
        FrictionRemoteDisarm(&remote);
    }
    FrictionRcByte_s byte;
    /* Old pre-handoff bytes are stale after one remote timeout anyway.
     * Retire the absolute-time filter; keeping it forever would reject all
     * reception after half of the uint32 clock range (~24.8 days). */
    if (now_ms - handoff_ms > FRICTION_REMOTE_TIMEOUT_MS)
    {
        handoff_filter = false;
    }
    for (unsigned count = 0U; count < RC_POLL_BYTE_BUDGET && FrictionRcPortPop(&byte); ++count)
    {
        /* Queued bytes from before a handoff cannot grant fresh consent. */
        if (!handoff_filter || (uint32_t)(byte.received_ms - handoff_ms) < 0x80000000U)
        {
            FrictionRemotePush(&remote, byte.byte, byte.received_ms);
        }
    }
    if (FrictionRcPortTakeFaults() != 0U)
    {
        FrictionRemoteInvalidate(&remote);
    }
    if (sample.command_source != FRICTION_SOURCE_REMOTE)
    {
        FrictionRemoteDisarm(&remote);
    }
    FrictionRemoteRequest_s request;
    FrictionRemoteTake(&remote, now_ms, &request);
    if (request.new_frame && remote.status.online && remote.status.gear != observed_gear)
    {
        previous_gear = observed_gear;
        observed_gear = remote.status.gear;
        gear_changed_ms = request.received_ms;
        ++gear_changes;
    }
    if (request.new_frame && remote.status.online)
    {
        handoff_filter = false;
    }
    if (sample.command_source == FRICTION_SOURCE_NONE && request.qualified_off)
    {
        last_submit = FrictionSelectSource(FRICTION_SOURCE_REMOTE);
        if (last_submit == FRICTION_SUBMIT_ACCEPTED)
        {
            /* Next fresh OFF frame still grants the producer's arming consent. */
            FrictionGetTelemetry(&sample);
            observed_epoch = sample.source_epoch;
        }
    }
    if (sample.command_source == FRICTION_SOURCE_REMOTE)
    {
        Friction_Command_s command = {0};
        if (request.run && request.new_frame)
        {
            command.enable_mask = BENCH_FRICTION_MASK;
            for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
            {
                command.target_rpm[i] = (BENCH_FRICTION_MASK & (1U << i)) ? request.target_rpm : 0.0f;
            }
        }
        if (request.stop_requested || (request.run && request.new_frame))
        {
            last_submit = FrictionSubmitFrom(FRICTION_SOURCE_REMOTE, &command, request.received_ms);
            accepted_frames += last_submit == FRICTION_SUBMIT_ACCEPTED && request.new_frame;
        }
    }
    FrictionRemoteBenchStatus_s status = {
        .remote = remote.status, .sample_ms = now_ms, .initialized = initialized,
        .accepted_frames = accepted_frames, .last_submit = last_submit,
        .gear_changed_ms = gear_changed_ms, .gear_changes = gear_changes, .previous_gear = previous_gear,
    };
    FrictionRcPortGetStats(&status.port);
    uint32_t lock = ControlPlatformEnter();
    published = status;
    ControlPlatformExit(lock);
}

void FrictionRemoteBenchGetStatus(FrictionRemoteBenchStatus_s *status)
{
    uint32_t lock = ControlPlatformEnter();
    *status = published;
    ControlPlatformExit(lock);
}
