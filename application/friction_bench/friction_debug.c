#include "../bench_features.h"
#include "friction_debug.h"
#include "friction_debug_protocol.h"
#include "friction.h"
#include "friction_config.h"
#include "friction_profile_service.h"

#include <math.h>

_Static_assert(FRICTION_VOFA_PERIOD_MS >= FRICTION_STREAM_PERIOD_MIN_MS &&
               FRICTION_VOFA_PERIOD_MS <= FRICTION_STREAM_PERIOD_MAX_MS,
               "VOFA period must be 2..1000 ms");
_Static_assert(FRICTION_CHANNEL_COUNT <= VOFA_MAX_CHANNELS,
               "Friction telemetry exceeds VOFA channel capacity");

static Vofa_s debug;
static FrictionProfileService_s profiles;
static uint32_t period_ms;
static uint32_t last_frame_ms;
static uint32_t input_ms;
static const Friction_Control_Config_s limits = FRICTION_CONTROL_CONFIG;

static float *Parameter(Friction_Tuning_s *tuning, uint16_t id)
{
    if (id == FRICTION_PARAM_RAMP)
    {
        return &tuning->ramp_rpm_s;
    }

    unsigned wheel = id / FRICTION_PARAM_WHEEL_STRIDE;
    if (wheel < 1U || wheel > FRICTION_MOTOR_COUNT)
    {
        return NULL;
    }

    Friction_PID_Tuning_s *pid = &tuning->motor[wheel - 1U];
    switch (id % FRICTION_PARAM_WHEEL_STRIDE)
    {
    case FRICTION_PID_FIELD_KP:
        return &pid->kp;
    case FRICTION_PID_FIELD_KI:
        return &pid->ki;
    case FRICTION_PID_FIELD_KD:
        return &pid->kd;
    case FRICTION_PID_FIELD_CURRENT_LIMIT:
        return &pid->current_limit;
    default:
        return NULL;
    }
}

static Vofa_Result_e Queue(Friction_Telemetry_s *snapshot, uint16_t sequence)
{
    switch (FrictionQueueTuning(&snapshot->tuning, sequence, snapshot->applied_sequence))
    {
    case FRICTION_TUNE_QUEUED:
        return VOFA_QUEUED;
    case FRICTION_TUNE_RANGE:
        return VOFA_RANGE;
    case FRICTION_TUNE_BUSY:
        return VOFA_BUSY;
    default:
        return VOFA_UNAVAILABLE;
    }
}

static Vofa_Result_e Submit(const Friction_Command_s *command)
{
    switch (FrictionSubmitFrom(FRICTION_SOURCE_VOFA, command, input_ms))
    {
    case FRICTION_SUBMIT_ACCEPTED:
        return VOFA_QUEUED;
    case FRICTION_SUBMIT_BUSY:
    case FRICTION_SUBMIT_REARM_REQUIRED:
        return VOFA_BUSY;
    default:
        return VOFA_UNAVAILABLE;
    }
}

static Vofa_Result_e Get(void *context, uint16_t id, float *value)
{
    (void)context;
    Vofa_Result_e profile_result = FrictionProfileServiceGet(&profiles, id, value);
    if (profile_result != VOFA_UNKNOWN)
    {
        return profile_result;
    }
    if (id == FRICTION_PARAM_STREAM_PERIOD)
    {
        *value = (float)period_ms;
        return VOFA_OK;
    }
    if (id == FRICTION_PARAM_MAX_RPM)
    {
        *value = limits.max_rpm;
        return VOFA_OK;
    }

    Friction_Telemetry_s snapshot;
    FrictionGetTelemetry(&snapshot);
    float *parameter = Parameter(&snapshot.tuning, id);
    if (parameter == NULL)
    {
        return VOFA_UNKNOWN;
    }
    *value = *parameter;
    return VOFA_OK;
}

static Vofa_Result_e Set(void *context, uint16_t sequence, uint16_t id, float value)
{
    (void)context;
    if (FrictionProfileServiceBusy(&profiles))
    {
        return VOFA_BUSY;
    }
    if (id == FRICTION_PARAM_STREAM_PERIOD)
    {
        if (value < FRICTION_STREAM_PERIOD_MIN_MS || value > FRICTION_STREAM_PERIOD_MAX_MS ||
            value != floorf(value))
        {
            return VOFA_RANGE;
        }
        period_ms = (uint32_t)value;
        return VOFA_OK;
    }

    Friction_Telemetry_s snapshot;
    FrictionGetTelemetry(&snapshot);
    float *parameter = Parameter(&snapshot.tuning, id);
    if (parameter == NULL)
    {
        return VOFA_UNKNOWN;
    }
    *parameter = value;
    return Queue(&snapshot, sequence);
}

static Vofa_Result_e RunCommand(const float *arguments, size_t count)
{
    if (count != FRICTION_RUN_ARGUMENT_COUNT || arguments[0] < 0.0f ||
        arguments[0] > FRICTION_ALL_MOTORS || arguments[0] != floorf(arguments[0]))
    {
        FrictionDisable();
        return VOFA_RANGE;
    }
    for (unsigned i = 1; i < FRICTION_RUN_ARGUMENT_COUNT; ++i)
    {
        if (fabsf(arguments[i]) > limits.max_rpm)
        {
            FrictionDisable();
            return VOFA_RANGE;
        }
    }

    Friction_Status_s status;
    FrictionGetStatus(&status);
    if (!BENCH_FRICTION_ACTIVE || (status.fault & FRICTION_FAULT_CONFIG))
    {
        return VOFA_UNAVAILABLE;
    }

    if ((uint8_t)arguments[0] & ~BENCH_FRICTION_MASK)
    {
        FrictionDisable();
        return VOFA_UNAVAILABLE;
    }
    Friction_Command_s command = {
        .enable_mask = (uint8_t)arguments[0],
        .target_rpm = {arguments[1], arguments[2], arguments[3]},
    };
    if (command.enable_mask && FrictionProfileServiceBusy(&profiles))
    {
        return VOFA_BUSY;
    }
    return Submit(&command);
}

static Vofa_Result_e PIDCommand(uint16_t sequence, const float *arguments, size_t count)
{
    if (FrictionProfileServiceBusy(&profiles))
    {
        return VOFA_BUSY;
    }
    if (count != FRICTION_PID_ARGUMENT_COUNT || arguments[0] < 1.0f ||
        arguments[0] > FRICTION_MOTOR_COUNT || arguments[0] != floorf(arguments[0]))
    {
        return VOFA_RANGE;
    }

    Friction_Telemetry_s snapshot;
    FrictionGetTelemetry(&snapshot);
    unsigned wheel = (unsigned)arguments[0] - 1U;
    snapshot.tuning.motor[wheel] = (Friction_PID_Tuning_s){
        .kp = arguments[1],
        .ki = arguments[2],
        .kd = arguments[3],
        .current_limit = arguments[4],
    };
    return Queue(&snapshot, sequence);
}

static Vofa_Result_e Call(void *context, uint16_t sequence, uint16_t operation,
                          const float *arguments, size_t count)
{
    (void)context;
    switch (operation)
    {
    case FRICTION_OPERATION_STOP:
        if (count != 0U)
        {
            return VOFA_RANGE;
        }
        {
            const Friction_Command_s stopped = {0};
            return Submit(&stopped);
        }
    case FRICTION_OPERATION_RUN:
        return RunCommand(arguments, count);
    case FRICTION_OPERATION_PID:
        return PIDCommand(sequence, arguments, count);
    default:
        return FrictionProfileServiceCall(&profiles, sequence, operation, arguments, count);
    }
}

static void RxFault(void *context)
{
    (void)context;
    FrictionProfileServiceInvalidate(&profiles);
    FrictionDisable();
}

static void FillWheelChannels(float *channels, const Friction_Telemetry_s *snapshot,
                               unsigned wheel)
{
    channels[FRICTION_WAVE_TARGET] = snapshot->status.requested_rpm[wheel];
    channels[FRICTION_WAVE_REFERENCE] = snapshot->status.reference_rpm[wheel];
    channels[FRICTION_WAVE_MEASURED] = snapshot->status.feedback[wheel].rpm;
    channels[FRICTION_WAVE_ERROR] = channels[FRICTION_WAVE_REFERENCE] - channels[FRICTION_WAVE_MEASURED];
    channels[FRICTION_WAVE_OUTPUT] = snapshot->output[wheel];
    channels[FRICTION_WAVE_CURRENT] = snapshot->current[wheel];
    channels[FRICTION_WAVE_P] = snapshot->p[wheel];
    channels[FRICTION_WAVE_I] = snapshot->i[wheel];
    channels[FRICTION_WAVE_D] = snapshot->d[wheel];
    channels[FRICTION_WAVE_TEMPERATURE] = (float)snapshot->status.feedback[wheel].temperature_c;
}

void FrictionDebugInit(const Vofa_Port_s *port)
{
    /* UART10 is telemetry-only; starting waves must not select a motor owner. */
    FrictionProfileServiceInit(&profiles);
    Vofa_Handler_s handler = {
        .get = Get,
        .set = Set,
        .call = Call,
        .rx_fault = RxFault,
    };
    Vofa_Port_s output = {0};
    if (BENCH_ENABLE_VOFA && port != NULL)
    {
        output = *port;
        output.read = NULL; /* Never accept motor commands from UART10. */
    }
    VofaInit(&debug, BENCH_ENABLE_VOFA ? &output : NULL, &handler);
    period_ms = FRICTION_VOFA_PERIOD_MS;
    last_frame_ms = 0;
    input_ms = 0;
}

uint8_t FrictionDebugOwnsCommands(void)
{
    Friction_Command_Source_e source = FrictionGetSource();
    return source == FRICTION_SOURCE_VOFA;
}

void FrictionDebugPoll(uint32_t now_ms)
{
    if (!BENCH_ENABLE_VOFA)
    {
        return;
    }
    input_ms = now_ms; /* Current protocol callbacks share this validation timestamp. */
    FrictionProfileServicePoll(&profiles, now_ms);
    VofaPoll(&debug, now_ms);
    if (debug.port.write == NULL || now_ms - last_frame_ms < period_ms)
    {
        return;
    }
    last_frame_ms = now_ms;

    Friction_Telemetry_s snapshot;
    FrictionGetTelemetry(&snapshot);
    float channels[FRICTION_CHANNEL_COUNT - FRICTION_CHANNEL_ACK_COUNT] = {
        [FRICTION_CHANNEL_TIME - FRICTION_CHANNEL_ACK_COUNT] =
            (float)(snapshot.sample_ms & FRICTION_TELEMETRY_COUNTER_MASK),
        [FRICTION_CHANNEL_SAMPLE - FRICTION_CHANNEL_ACK_COUNT] =
            (float)(snapshot.sample_id & FRICTION_TELEMETRY_COUNTER_MASK),
        [FRICTION_CHANNEL_FAULT - FRICTION_CHANNEL_ACK_COUNT] = (float)snapshot.status.fault,
        [FRICTION_CHANNEL_ONLINE - FRICTION_CHANNEL_ACK_COUNT] = (float)snapshot.status.online_mask,
        [FRICTION_CHANNEL_ENABLED - FRICTION_CHANNEL_ACK_COUNT] = (float)snapshot.status.enabled_mask,
        [FRICTION_CHANNEL_APPLIED_SEQUENCE - FRICTION_CHANNEL_ACK_COUNT] = (float)snapshot.applied_sequence,
        [FRICTION_CHANNEL_TX_DROPPED - FRICTION_CHANNEL_ACK_COUNT] = (float)debug.tx_dropped,
        [FRICTION_CHANNEL_STEP_MS - FRICTION_CHANNEL_ACK_COUNT] = (float)snapshot.status.step_ms,
        [FRICTION_CHANNEL_PHASE - FRICTION_CHANNEL_ACK_COUNT] = (float)snapshot.readiness.phase,
        [FRICTION_CHANNEL_READY_MASK - FRICTION_CHANNEL_ACK_COUNT] = (float)snapshot.readiness.ready_mask,
    };
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        unsigned base = FRICTION_CHANNEL_WHEEL_BASE - FRICTION_CHANNEL_ACK_COUNT +
                        FRICTION_CHANNEL_WHEEL_STRIDE * i;
        FillWheelChannels(&channels[base], &snapshot, i);
    }

    /* Invalid samples reject the whole frame; do not disguise them as zeros. */
    (void)VofaSend(&debug, channels, sizeof(channels) / sizeof(channels[0]));
}
