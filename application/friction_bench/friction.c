#include "../bench_features.h"
#include "friction.h"
#include "friction_config.h"
#include "dji_speed_group.h"
#include "control_platform.h"
#include "runtime_events.h"
#include <math.h>
#include <string.h>

static const DjiSpeedMotorConfig_s motor_config[FRICTION_MOTOR_COUNT] = FRICTION_MOTOR_CONFIGS;
static Friction_Control_Config_s control_config = FRICTION_CONTROL_CONFIG;
static const SpeedReady_Config_s ready_config = FRICTION_READY_CONFIG;
static Friction_Monitor_s monitor;
static DjiSpeedGroup_s motors;
static unsigned motor_index[FRICTION_MOTOR_COUNT]; /* F slot -> compact registered group slot. */
static Friction_Control_s control;
static Friction_Status_s published_status;
static Friction_Command_s pending_command;
static uint32_t command_ms;
static uint32_t generation;
static uint32_t stop_generation;
static uint32_t stop_ms;
static uint8_t stop_pending;
static uint8_t initialized;
static uint8_t init_attempted;
static uint8_t command_received;
static Friction_Telemetry_s telemetry;
static Friction_Tuning_s pending_tuning;
static uint16_t pending_tune_sequence;
static uint8_t tuning_pending;
static uint8_t profile_pending;
static Friction_Command_Source_e command_source =
    BENCH_FRICTION_REMOTE_ACTIVE ? FRICTION_SOURCE_REMOTE :
    (BENCH_FRICTION_ACTIVE && BENCH_ENABLE_USB ? FRICTION_SOURCE_USB :
     (BENCH_FRICTION_ACTIVE && BENCH_ENABLE_RTT_COMMAND ? FRICTION_SOURCE_RTT : FRICTION_SOURCE_NONE));
static uint32_t source_epoch;
static uint8_t source_requires_off;

uint8_t FrictionInit(void)
{
    if (init_attempted)
    {
        return !BENCH_FRICTION_ACTIVE || initialized;
    }
    init_attempted = 1;
    if (!BENCH_FRICTION_ACTIVE)
    {
        return 1; /* Disabled is not a configuration fault; no driver is registered. */
    }
    FrictionControlInit(&control, ControlPlatformNowMs());
    published_status = control.status;
    uint8_t monitor_ready = !BENCH_SPEED_READY_ACTIVE || FrictionMonitorInit(&monitor, &ready_config);
    telemetry.readiness.phase = FRICTION_PHASE_FAULT;
    telemetry.tuning.ramp_rpm_s = control_config.ramp_rpm_s;
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        telemetry.tuning.motor[i] = (Friction_PID_Tuning_s){
            motor_config[i].kp, motor_config[i].ki, 0, motor_config[i].current_limit};
    }
    DjiSpeedMotorConfig_s selected[FRICTION_MOTOR_COUNT];
    unsigned count = 0U;
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        motor_index[i] = DJI_SPEED_GROUP_CAPACITY;
        if (BENCH_FRICTION_MASK & (1U << i))
        {
            motor_index[i] = count;
            selected[count++] = motor_config[i];
        }
    }
    if (!monitor_ready || !DjiSpeedGroupInit(&motors, selected, count))
    {
        published_status.fault = FRICTION_FAULT_CONFIG;
        published_status.active_fault = FRICTION_FAULT_CONFIG;
        return 0;
    }

    /* Before any task runs, explicitly initialize the controller with OFF.
     * This sends no motor output and leaves no pending stop to discard the
     * first received RUN. Runtime stops and fault rearm keep their normal gate. */
    uint32_t startup_ms = ControlPlatformNowMs();
    const Friction_Command_s stopped = {0};
    const Friction_Feedback_s feedback[FRICTION_MOTOR_COUNT] = {0};
    FrictionControlStep(&control, &control_config, &stopped, startup_ms,
                        feedback, startup_ms, 0U);
    pending_command = stopped;
    command_ms = startup_ms;
    command_received = 1;
    ++generation;
    published_status = control.status;
    if (BENCH_SPEED_READY_ACTIVE)
    {
        FrictionMonitorUpdate(&monitor, &control.status, startup_ms);
    }
    initialized = 1;
    telemetry.readiness = monitor.status;
    return 1;
}

/* Caller holds the mailbox lock. A global stop preserves a pending handoff's
 * explicit-owner OFF requirement; it must not manufacture recovery consent. */
static void StopLocked(uint32_t received_ms)
{
    memset(&pending_command, 0, sizeof(pending_command));
    command_received = !source_requires_off;
    command_ms = received_ms;
    ++generation;
    stop_pending = 1;
    stop_generation = generation;
    stop_ms = received_ms;
}

static bool SourceAvailable(Friction_Command_Source_e source)
{
    return source == FRICTION_SOURCE_NONE ||
        (BENCH_FRICTION_ACTIVE &&
         ((source == FRICTION_SOURCE_RTT && BENCH_ENABLE_RTT_COMMAND) ||
          (source == FRICTION_SOURCE_VOFA && BENCH_ENABLE_VOFA) ||
          (source == FRICTION_SOURCE_USB && BENCH_ENABLE_USB) ||
          (source == FRICTION_SOURCE_REMOTE && BENCH_FRICTION_REMOTE_ACTIVE)));
}

Friction_Submit_Result_e FrictionSelectSource(Friction_Command_Source_e source)
{
    if (!SourceAvailable(source))
    {
        return FRICTION_SUBMIT_UNAVAILABLE;
    }
    uint32_t lock = ControlPlatformEnter();
    Friction_Submit_Result_e result = FRICTION_SUBMIT_ACCEPTED;
    if (source != command_source)
    {
        if (profile_pending)
        {
            result = FRICTION_SUBMIT_BUSY;
        }
        else
        {
            command_source = source;
            ++source_epoch;
            source_requires_off = 1;
            StopLocked(ControlPlatformNowMs());
        }
    }
    ControlPlatformExit(lock);
    return result;
}

Friction_Command_Source_e FrictionGetSource(void)
{
    uint32_t lock = ControlPlatformEnter();
    Friction_Command_Source_e source = command_source;
    ControlPlatformExit(lock);
    return source;
}

Friction_Submit_Result_e FrictionSubmitFrom(Friction_Command_Source_e source,
                                           const Friction_Command_s *command,
                                           uint32_t received_ms)
{
    if (!BENCH_FRICTION_ACTIVE || !initialized)
    {
        return FRICTION_SUBMIT_UNAVAILABLE;
    }
    if (command == NULL)
    {
        FrictionDisable();
        return FRICTION_SUBMIT_ACCEPTED;
    }
    uint32_t lock = ControlPlatformEnter();
    bool fresh = ControlPlatformNowMs() - received_ms <= control_config.command_timeout_ms;
    bool owner = source != FRICTION_SOURCE_NONE &&
        (source == command_source || (command_source == FRICTION_SOURCE_NONE && source != FRICTION_SOURCE_REMOTE));
    Friction_Submit_Result_e result = FRICTION_SUBMIT_ACCEPTED;
    if (!command->enable_mask)
    {
        if (owner && fresh)
        {
            source_requires_off = 0;
        }
        StopLocked(received_ms);
        result = fresh ? FRICTION_SUBMIT_ACCEPTED : FRICTION_SUBMIT_STALE;
    }
    else if (!SourceAvailable(source))
    {
        result = FRICTION_SUBMIT_UNAVAILABLE;
    }
    else if (!owner)
    {
        result = FRICTION_SUBMIT_WRONG_SOURCE;
    }
    else if (!fresh)
    {
        result = FRICTION_SUBMIT_STALE;
    }
    else if (profile_pending)
    {
        result = FRICTION_SUBMIT_BUSY;
    }
    else if (source_requires_off)
    {
        result = FRICTION_SUBMIT_REARM_REQUIRED;
    }
    else
    {
        if (command_source == FRICTION_SOURCE_NONE)
        {
            command_source = source;
        }
        pending_command = *command;
        command_received = 1;
        command_ms = received_ms;
        ++generation;
    }
    ControlPlatformExit(lock);
    return result;
}

void FrictionSubmit(const Friction_Command_s *command)
{
    (void)FrictionSubmitFrom(FRICTION_SOURCE_RTT, command, ControlPlatformNowMs());
}

void FrictionDisable(void)
{
    if (BENCH_FRICTION_ACTIVE)
    {
        uint32_t lock = ControlPlatformEnter();
        source_requires_off = 0;
        if (command_source != FRICTION_SOURCE_NONE)
        {
            ++source_epoch;
        }
        command_source = FRICTION_SOURCE_NONE;
        StopLocked(ControlPlatformNowMs());
        ControlPlatformExit(lock);
    }
}

void FrictionStopFromUsb(void)
{
    if (!BENCH_FRICTION_ACTIVE || !BENCH_ENABLE_USB)
    {
        return;
    }
    uint32_t lock = ControlPlatformEnter();
    if (command_source != FRICTION_SOURCE_USB && command_source != FRICTION_SOURCE_NONE)
    {
        command_source = FRICTION_SOURCE_NONE;
        ++source_epoch;
    }
    source_requires_off = 0;
    StopLocked(ControlPlatformNowMs());
    ControlPlatformExit(lock);
}

void FrictionGetStatus(Friction_Status_s *status)
{
    if (status == NULL)
    {
        return;
    }
    uint32_t lock = ControlPlatformEnter();
    *status = published_status;
    ControlPlatformExit(lock);
}

void FrictionGetTelemetry(Friction_Telemetry_s *result)
{
    if (result == NULL)
    {
        return;
    }
    uint32_t lock = ControlPlatformEnter();
    *result = telemetry;
    result->status = published_status;
    result->command_source = command_source;
    result->source_epoch = source_epoch;
    result->source_requires_off = source_requires_off;
    result->initialized = initialized;
    result->command_received_now = command_received;
    result->pending_mask_now = pending_command.enable_mask;
    result->tuning_pending_now = tuning_pending;
    result->profile_pending_now = profile_pending;
    ControlPlatformExit(lock);
}

static Friction_Tune_Result_e ValidateTuning(const Friction_Tuning_s *tuning, uint16_t sequence)
{
    if (!initialized)
    {
        return FRICTION_TUNE_UNAVAILABLE;
    }
    if (!tuning || !sequence || !isfinite(tuning->ramp_rpm_s) ||
        tuning->ramp_rpm_s < 1 || tuning->ramp_rpm_s > FRICTION_TUNE_RAMP_MAX)
    {
        return FRICTION_TUNE_RANGE;
    }
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        const Friction_PID_Tuning_s *p = &tuning->motor[i];
        if (!isfinite(p->kp) || p->kp < 0 || p->kp > FRICTION_TUNE_KP_MAX ||
            !isfinite(p->ki) || p->ki < 0 || p->ki > FRICTION_TUNE_KI_MAX ||
            !isfinite(p->kd) || p->kd < 0 || p->kd > FRICTION_TUNE_KD_MAX ||
            !isfinite(p->current_limit) || p->current_limit < 1 ||
            p->current_limit > motor_config[i].current_limit)
        {
            return FRICTION_TUNE_RANGE;
        }
    }
    return FRICTION_TUNE_QUEUED;
}

uint8_t FrictionTuningAvailable(void)
{
    return initialized;
}

void FrictionGetCurrentLimits(float limits[FRICTION_MOTOR_COUNT])
{
    if (limits != NULL)
    {
        for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
        {
            limits[i] = motor_config[i].current_limit;
        }
    }
}

/* Caller holds the mailbox lock. A previously sampled RUN remains in the
 * mailbox until an explicit stop, so published status alone is never used. */
static Friction_Tune_Result_e ProfileAdmissionLocked(void)
{
    if (!initialized)
    {
        return FRICTION_TUNE_UNAVAILABLE;
    }
    return (published_status.enabled_mask || pending_command.enable_mask || tuning_pending)
        ? FRICTION_TUNE_BUSY : FRICTION_TUNE_QUEUED;
}

Friction_Tune_Result_e FrictionProfileAdmission(void)
{
    uint32_t lock = ControlPlatformEnter();
    Friction_Tune_Result_e result = ProfileAdmissionLocked();
    ControlPlatformExit(lock);
    return result;
}

Friction_Tune_Result_e FrictionQueueProfile(const Friction_Tuning_s *tuning,
                                          uint16_t sequence, uint32_t base_revision)
{
    Friction_Tune_Result_e result = ValidateTuning(tuning, sequence);
    if (result != FRICTION_TUNE_QUEUED)
    {
        return result;
    }
    uint32_t lock = ControlPlatformEnter();
    result = ProfileAdmissionLocked();
    if (result == FRICTION_TUNE_QUEUED && base_revision != telemetry.tuning_revision)
    {
        result = FRICTION_TUNE_BUSY;
    }
    if (result == FRICTION_TUNE_QUEUED)
    {
        pending_tuning = *tuning;
        pending_tune_sequence = sequence;
        tuning_pending = 1;
        profile_pending = 1;
    }
    ControlPlatformExit(lock);
    return result;
}

Friction_Tune_Result_e FrictionQueueTuning(const Friction_Tuning_s *tuning,
                                         uint16_t sequence, uint16_t base_sequence)
{
    Friction_Tune_Result_e result = ValidateTuning(tuning, sequence);
    if (result != FRICTION_TUNE_QUEUED)
    {
        return result;
    }
    uint32_t lock = ControlPlatformEnter();
    if (tuning_pending || base_sequence != telemetry.applied_sequence)
    {
        ControlPlatformExit(lock);
        return FRICTION_TUNE_BUSY;
    }
    pending_tuning = *tuning;
    pending_tune_sequence = sequence;
    tuning_pending = 1;
    ControlPlatformExit(lock);
    return FRICTION_TUNE_QUEUED;
}

static void ApplyTuning(const Friction_Tuning_s *tuning)
{
    if (BENCH_SPEED_READY_ACTIVE)
    {
        FrictionMonitorReset(&monitor);
    }
    control_config.ramp_rpm_s = tuning->ramp_rpm_s;
    DjiSpeedTuning_s selected[FRICTION_MOTOR_COUNT] = {0};
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        if (BENCH_FRICTION_MASK & (1U << i))
        {
            selected[motor_index[i]] = tuning->motor[i];
        }
    }
    DjiSpeedGroupTune(&motors, selected);
}

static void PublishTelemetry(uint8_t applied_tuning, const Friction_Tuning_s *tuning,
                              uint16_t tune_sequence, uint32_t sampled_generation)
{
    /* Only this task writes telemetry. Build the sample before taking the short
     * publication lock; readers can only access it through the snapshot APIs. */
    Friction_Telemetry_s sample = telemetry;
    sample.status = control.status;
    sample.readiness = monitor.status;
    sample.sample_ms = ControlPlatformNowMs();
    ++sample.sample_id;
    sample.command_generation = sampled_generation;
    if (applied_tuning)
    {
        sample.tuning = *tuning;
        sample.applied_sequence = tune_sequence;
        ++sample.tuning_revision;
    }
    for (unsigned i = 0; initialized && i < FRICTION_MOTOR_COUNT; ++i)
    {
        DjiSpeedOutput_s output;
        DjiSpeedGroupGetOutput(&motors, motor_index[i], &output);
        sample.current[i] = control.status.feedback[i].current_raw * motor_config[i].direction;
        sample.output[i] = (control.status.enabled_mask & (1U << i)) ? output.output : 0.0f;
        sample.p[i] = output.p;
        sample.i[i] = output.i;
        sample.d[i] = output.d;
    }

    uint32_t lock = ControlPlatformEnter();
    published_status = sample.status;
    telemetry = sample;
    if (applied_tuning)
    {
        tuning_pending = 0;
        profile_pending = 0;
    }
    ControlPlatformExit(lock);
}

void FrictionTask(void)
{
    if (!BENCH_FRICTION_ACTIVE)
    {
        return;
    }
    Friction_Command_s command;
    Friction_Feedback_s feedback[FRICTION_MOTOR_COUNT] = {0};
    uint32_t submitted_ms;
    uint32_t fault = 0;
    uint32_t sampled_generation;
    uint8_t stopped;
    Friction_Tuning_s tuning;
    uint16_t tune_sequence;
    uint8_t apply_tuning;

    uint32_t lock = ControlPlatformEnter();
    if (!command_received)
    {
        fault |= FRICTION_FAULT_COMMAND;
    }
    apply_tuning = tuning_pending;
    tuning = pending_tuning;
    tune_sequence = pending_tune_sequence;
    command = pending_command;
    submitted_ms = command_ms;
    sampled_generation = generation;
    stopped = stop_pending;
    stop_pending = 0;
    if (stopped)
    {
        /* Discard any enable posted before this stop was consumed. */
        memset(&pending_command, 0, sizeof(pending_command));
        memset(&command, 0, sizeof(command));
        command_ms = submitted_ms = stop_ms;
    }
    if (profile_pending && (command.enable_mask || control.status.enabled_mask))
    {
        apply_tuning = 0; /* Owner rechecks stopped state before changing its PIDs. */
    }
    ControlPlatformExit(lock);

    if (apply_tuning && initialized)
    {
        ApplyTuning(&tuning);
    }

    if (!initialized)
    {
        fault |= FRICTION_FAULT_CONFIG;
    }
    for (unsigned i = 0; initialized && i < FRICTION_MOTOR_COUNT; ++i)
    {
        DjiSpeedFeedback_s sample;
        DjiSpeedGroupRead(&motors, motor_index[i], &sample);
        feedback[i].rpm = sample.rpm;
        feedback[i].current_raw = sample.current_raw;
        feedback[i].temperature_c = sample.temperature_c;
        feedback[i].valid = sample.valid;
        feedback[i].last_rx_ms = sample.last_rx_ms;
    }
    if (command.enable_mask & ~BENCH_FRICTION_MASK)
    {
        fault |= FRICTION_FAULT_COMMAND; /* Never silently run a subset of a rejected request. */
    }
    FrictionControlStep(&control, &control_config, &command, submitted_ms,
                        feedback, ControlPlatformNowMs(), fault);

    /* A disable arriving during calculation takes precedence before output.
     * No potentially blocking operations occur with interrupts disabled. */
    lock = ControlPlatformEnter();
    uint8_t interrupted_stop = stop_pending ||
        (generation != sampled_generation && stop_generation == generation);
    ControlPlatformExit(lock);
    if (interrupted_stop)
    {
        control.status.enabled_mask = 0;
        memset(control.status.reference_rpm, 0, sizeof(control.status.reference_rpm));
    }
    for (unsigned i = 0; initialized && i < FRICTION_MOTOR_COUNT; ++i)
    {
        DjiSpeedGroupSet(&motors, motor_index[i], control.status.reference_rpm[i],
                        (control.status.enabled_mask & (1U << i)) != 0U);
    }

    if (!DjiSpeedGroupControl())
    {
        control.status.fault |= FRICTION_FAULT_DRIVER;
        control.status.active_fault |= FRICTION_FAULT_DRIVER;
        control.require_disable = 1;
        control.status.rearm_required = 1;
        control.status.enabled_mask = 0;
        memset(control.status.reference_rpm, 0, sizeof(control.status.reference_rpm));
        DjiSpeedGroupStop(&motors);
    }
    if (BENCH_SPEED_READY_ACTIVE)
    {
        FrictionMonitorUpdate(&monitor, &control.status, ControlPlatformNowMs());
    }

    PublishTelemetry(apply_tuning, &tuning, tune_sequence, sampled_generation);
    RuntimeEventsObserve(RUNTIME_FRICTION_FAULT, control.status.fault);
    RuntimeEventsObserve(RUNTIME_FRICTION_ENABLE, control.status.enabled_mask);
    RuntimeEventsObserve(RUNTIME_FRICTION_ONLINE, control.status.online_mask);
    if (BENCH_SPEED_READY_ACTIVE)
    {
        RuntimeEventsObserve(RUNTIME_FRICTION_PHASE, monitor.status.phase);
    }
}
