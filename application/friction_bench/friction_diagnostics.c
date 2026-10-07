#include "../bench_features.h"
#include "friction_diagnostics.h"
#include "friction.h"
#include "friction_config.h"
#include "control_platform.h"
#include "diagnostics.h"

void FrictionDiagnosticsReport(TextConsole_s *console, uint32_t now_ms)
{
    static const DjiSpeedMotorConfig_s motors[FRICTION_MOTOR_COUNT] = FRICTION_MOTOR_CONFIGS;
    static const Friction_Control_Config_s config = FRICTION_CONTROL_CONFIG;
    static const char *const phase_names[] = {"STOPPED", "SPINUP", "READY", "RECOVERING", "FAULT", "BENCH"};
    TextConsolePrint(console, "[modules_friction] motors_mask=%u vofa=%u rtt_command=%u speed_ready=%u parameters=%u",
                     BENCH_FRICTION_MASK, BENCH_ENABLE_VOFA, BENCH_ENABLE_RTT_COMMAND,
                     BENCH_SPEED_READY_ACTIVE, BENCH_ENABLE_PARAMETERS);
    if (!BENCH_FRICTION_ACTIVE)
    {
        TextConsolePrint(console, "[friction] DISABLED; [F1] DISABLED; [F2] DISABLED; [F3] DISABLED; [readiness] DISABLED");
        return;
    }
    Friction_Telemetry_s telemetry;
    FrictionGetTelemetry(&telemetry);
    static const char *const sources[] = {"NONE", "RTT", "VOFA", "USB"};
    unsigned source = (unsigned)telemetry.command_source;
    TextConsolePrint(console, "[friction_authority] selected_now=%s epoch=%lu owner_off_required=%u; selected source is not motor enable",
                     source < 4U ? sources[source] : "UNKNOWN", (unsigned long)telemetry.source_epoch,
                     telemetry.source_requires_off);
    TextConsolePrint(console, "[parameters] %s revision=%lu storage=UNBOUND; applied values in RAM only",
                     BENCH_ENABLE_PARAMETERS ? "ENABLED" : "DISABLED", (unsigned long)telemetry.tuning_revision);
    /* Sample time after copying: the producer may have preempted this task. */
    now_ms = ControlPlatformNowMs();
    const Friction_Status_s *status = &telemetry.status;
    DiagnosticsPrintTiming(console, "friction", &status->timing);
    bool fresh = telemetry.sample_id != 0U && now_ms - telemetry.sample_ms <= config.max_step_ms;
    TextConsolePrint(console, "[friction_task] %s sample=%lu age_ms=%lu step_ms=%lu command_age_ms=%lu fault=0x%lx enabled_mask=%u",
                     fresh ? "UPDATING" : "STALE", (unsigned long)telemetry.sample_id,
                     (unsigned long)(now_ms - telemetry.sample_ms), (unsigned long)status->step_ms,
                     (unsigned long)status->command_age_ms, (unsigned long)status->fault, status->enabled_mask);
    if (BENCH_SPEED_READY_ACTIVE)
    {
        TextConsolePrint(console, "[readiness] phase=%s mask=%u all=%u snapshot_fresh=%u; speed qualification only, not permission to feed",
                         (unsigned)telemetry.readiness.phase < 6U ? phase_names[telemetry.readiness.phase] : "UNKNOWN",
                         telemetry.readiness.ready_mask, telemetry.readiness.all_ready, fresh);
    }
    else
    {
        TextConsolePrint(console, "[readiness] DISABLED");
    }
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        if (!(BENCH_FRICTION_MASK & (1U << i)))
        {
            TextConsolePrint(console, "[F%u] DISABLED", i + 1U);
            continue;
        }
        const Friction_Feedback_s *feedback = &status->feedback[i];
        const Friction_PID_Tuning_s *pid = &telemetry.tuning.motor[i];
        bool online = fresh && feedback->valid && now_ms - feedback->last_rx_ms <= config.feedback_timeout_ms;
        const char *link = motors[i].id == 0U ? "UNCONFIGURED" : online ? "ONLINE" :
                           feedback->valid ? "STALE" : "NO_FEEDBACK";
        TextConsolePrint(console, "[F%u] %s CAN=%u ID=%u direction=%d valid=%u age_ms=%lu temp_C=%u iq_raw=%d",
                         i + 1U, link, motors[i].can_bus, motors[i].id, motors[i].direction,
                         feedback->valid, (unsigned long)(now_ms - feedback->last_rx_ms),
                         feedback->temperature_c, feedback->current_raw);
        TextConsolePrint(console, "[F%u_speed] target_rpm=%ld ref_rpm=%ld actual_rpm=%ld error_rpm=%ld output_raw=%ld P=%ld I=%ld D=%ld",
                         i + 1U, (long)TextConsoleScaled(status->requested_rpm[i], 1.0),
                         (long)TextConsoleScaled(status->reference_rpm[i], 1.0),
                         (long)TextConsoleScaled(feedback->rpm, 1.0),
                         (long)TextConsoleScaled(status->reference_rpm[i] - feedback->rpm, 1.0),
                         (long)TextConsoleScaled(telemetry.output[i], 1.0), (long)TextConsoleScaled(telemetry.p[i], 1.0),
                         (long)TextConsoleScaled(telemetry.i[i], 1.0), (long)TextConsoleScaled(telemetry.d[i], 1.0));
        TextConsolePrint(console, "[F%u_PID] kp_milli=%ld ki_milli=%ld kd_micro=%ld current_limit_raw=%ld; PID error unit=deg/s",
                         i + 1U, (long)TextConsoleScaled(pid->kp, 1000.0), (long)TextConsoleScaled(pid->ki, 1000.0),
                         (long)TextConsoleScaled(pid->kd, 1000000.0), (long)TextConsoleScaled(pid->current_limit, 1.0));
    }
    TextConsolePrint(console, "[friction_config] max_rpm=%ld ramp_rpm_s=%ld feedback_timeout_ms=%lu command_timeout_ms=%lu temperature_limit_C=%u tune_seq=%u",
                     (long)TextConsoleScaled(config.max_rpm, 1.0), (long)TextConsoleScaled(telemetry.tuning.ramp_rpm_s, 1.0),
                     (unsigned long)config.feedback_timeout_ms, (unsigned long)config.command_timeout_ms,
                     config.max_temperature_c, telemetry.applied_sequence);
    if (status->fault != 0U)
    {
        TextConsolePrint(console, "[hint friction] fault bits: 1=config 2=command 4=offline 8=temperature 16=timing 32=driver; ID=0 must be configured first");
        if ((status->fault & FRICTION_FAULT_COMMAND) != 0U && status->enabled_mask == 0U)
        {
            TextConsolePrint(console, "[hint friction] command interlock is active; send USB 'off', wait for enabled_mask=0, then send a fresh 'set' keepalive");
        }
    }
    TextConsolePrint(console, "[profile] DBUS/DM-J4310/BMI088=NOT_MONITORED; VOFA uses separate port binding; USB console is text only");
}
