#include "../../bench_features.h"
#include "friction_diagnostics.h"
#include "../friction.h"
#include "../config/friction_config.h"
#include "control_platform.h"
#include "diagnostics.h"
#include "../input/friction_remote_bench.h"
#include "../config/friction_remote_config.h"
#include <math.h>

void FrictionDiagnosticsExplain(TextConsole_s *console, const Friction_Telemetry_s *telemetry,
                                uint32_t now_ms)
{
    static const Friction_Control_Config_s config = FRICTION_CONTROL_CONFIG;
    const Friction_Status_s *status = &telemetry->status;
    if (!BENCH_FRICTION_ACTIVE)
    {
        TextConsolePrint(console, "[friction_why] DISABLED: 摩擦轮模块关闭，检查 bench_features.h 的开关。");
        return;
    }
    if (!telemetry->initialized)
    {
        TextConsolePrint(console, "[friction_why] INIT_FAILED: 整组初始化失败；查启用轮的 CAN/ID/方向、重复 ID、PID 配置和驱动初始化。");
        return;
    }
    if (telemetry->sample_id == 0U || now_ms - telemetry->sample_ms > config.max_step_ms)
    {
        TextConsolePrint(console, "[friction_why] TASK_STALE: 电机任务没有新鲜快照；先查任务运行和调度，旧转速不能证明当前状态。");
        return;
    }
    TextConsolePrint(console, "[friction_gate] requested=%u enabled=%u online=%u rearm=%u active_fault=0x%lx latched_fault=0x%lx",
                     status->requested_mask, status->enabled_mask, status->online_mask,
                     status->rearm_required, (unsigned long)status->active_fault, (unsigned long)status->fault);
    if (telemetry->source_requires_off)
    {
        TextConsolePrint(console, "[friction_why] SOURCE_REARM: USB 来源先 off -> set；REMOTE 来源先 S1 拨停止，再选运行组。");
    }
    if (status->active_fault & FRICTION_FAULT_CONFIG)
    {
        TextConsolePrint(console, "[friction_why] CONFIG: 控制配置不合法；核对配置范围与初始化结果。");
    }
    if (status->active_fault & FRICTION_FAULT_COMMAND)
    {
        if (!telemetry->command_received_now)
        {
            TextConsolePrint(console, "[friction_why] NO_COMMAND: 尚无可用命令。USB：off -> 周期 set；遥控：S1 停止 -> 运行组。查询/调参不续期。");
        }
        else if (status->requested_mask && status->command_age_ms > config.command_timeout_ms)
        {
            TextConsolePrint(console, "[friction_why] COMMAND_TIMEOUT: 运行命令超过租约，未持续到达。先 off，再每 100ms 重发 set。");
            /* Values are emitted separately to keep every UTF-8 line bounded. */
            TextConsolePrint(console, "[friction_command_age] actual_ms=%lu limit_ms=%lu",
                             (unsigned long)status->command_age_ms, (unsigned long)config.command_timeout_ms);
        }
        if (status->requested_mask & ~BENCH_FRICTION_MASK)
        {
            TextConsolePrint(console, "[friction_why] DISABLED_WHEEL: 请求包含被关闭的轮；整条运行请求停用，检查 set 的 mask。");
        }
        for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
        {
            if ((status->requested_mask & (1U << i)) &&
                (!isfinite(status->requested_rpm[i]) || fabsf(status->requested_rpm[i]) > config.max_rpm))
            {
                TextConsolePrint(console, "[friction_why] F%u TARGET_RANGE: 请求转速无效或越界；max_rpm=%ld。先修正目标，再 off -> set。",
                                 i + 1U, (long)TextConsoleScaled(config.max_rpm, 1.0));
            }
        }
    }
    if (status->active_fault & FRICTION_FAULT_OFFLINE)
    {
        for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
        {
            if ((status->requested_mask & (1U << i)) && !(status->online_mask & (1U << i)))
            {
                TextConsolePrint(console, "[friction_why] F%u %s: 运行所需反馈缺失；检查电机供电、CAN、ID、反馈周期。整组已停用。",
                                 i + 1U, status->feedback[i].valid ? "FEEDBACK_STALE" : "NO_FEEDBACK");
            }
        }
    }
    if (status->active_fault & FRICTION_FAULT_TEMPERATURE)
    {
        TextConsolePrint(console, "[friction_why] TEMPERATURE: 启用轮温度达到保护门槛；看各轮 temp_C，降温并查负载，再 off -> set。");
    }
    if (status->active_fault & FRICTION_FAULT_TIMING)
    {
        TextConsolePrint(console, "[friction_why] TIMING: 控制调用间隔为 0 或超过上限；查 friction_timing 和任务阻塞，先不要改 PID。");
    }
    if (status->active_fault & FRICTION_FAULT_DRIVER)
    {
        TextConsolePrint(console, "[friction_why] DRIVER: CAN 输出提交失败；查 CAN 的 Bus-off/TX 队列/错误计数，再 off -> set。");
    }
    if (status->fault && !status->active_fault)
    {
        TextConsolePrint(console, "[friction_why] FAULT_LATCHED: 当前原因已消失，历史故障仍锁定。USB：off -> 新 set；遥控：S1 停止 -> 运行组。");
    }
    if (status->rearm_required && !status->fault)
    {
        TextConsolePrint(console, "[friction_why] NEED_OFF: USB 来源先 off -> 新 set；遥控来源先S1 停止 -> 运行组；等待实际停用后重新启动。");
    }
    if (!status->fault && !status->rearm_required && !status->enabled_mask)
    {
        TextConsolePrint(console, "[friction_why] STOPPED: 当前未使能；USB 等新 set，遥控等解锁后的运行档位。off 为零电流滑行，惯性转动不等于仍被驱动。");
    }
    if (status->enabled_mask)
    {
        TextConsolePrint(console, "[friction_why] ENABLED: 软件已允许输出；是否实际转动看新鲜 actual_rpm，不能仅看命令的 OK。");
        for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
        {
            if (!(status->enabled_mask & (1U << i)))
            {
                continue;
            }
            if (status->requested_rpm[i] == 0.0f)
            {
                TextConsolePrint(console, "[friction_why] F%u ZERO_TARGET: 已使能但目标为零；这是零速闭环，可能有制动电流，不等于 off。", i + 1U);
            }
            else if (fabsf(status->reference_rpm[i] - status->requested_rpm[i]) > 1.0f)
            {
                TextConsolePrint(console, "[friction_why] F%u RAMPING: 参考转速仍在爬升；先比较 actual 与 ref，不要用最终 target 误判 PID。", i + 1U);
            }
            if (fabsf(telemetry->output[i]) >= telemetry->tuning.motor[i].current_limit - 1.0f)
            {
                TextConsolePrint(console, "[friction_why] F%u OUTPUT_LIMIT: 当前采样输出触限；需连续日志判断持续时间，再查供电/阻力/参数。", i + 1U);
            }
        }
    }
}

void FrictionDiagnosticsReport(TextConsole_s *console, uint32_t now_ms)
{
    static const DjiSpeedMotorConfig_s motors[FRICTION_MOTOR_COUNT] = FRICTION_MOTOR_CONFIGS;
    static const Friction_Control_Config_s config = FRICTION_CONTROL_CONFIG;
    static const char *const phase_names[] = {"STOPPED", "SPINUP", "READY", "RECOVERING", "FAULT", "BENCH"};
    TextConsolePrint(console, "[modules_friction] motors_mask=%u vofa=%u rtt_command=%u speed_ready=%u parameters=%u remote=%u",
                     BENCH_FRICTION_MASK, BENCH_ENABLE_VOFA, BENCH_ENABLE_RTT_COMMAND,
                     BENCH_SPEED_READY_ACTIVE, BENCH_ENABLE_PARAMETERS, BENCH_FRICTION_REMOTE_ACTIVE);
    if (BENCH_FRICTION_REMOTE_ACTIVE)
    {
        FrictionRemoteBenchStatus_s rc;
        FrictionRemoteBenchGetStatus(&rc);
        bool fresh = rc.initialized && now_ms - rc.sample_ms <= config.max_step_ms;
        bool online = fresh && rc.remote.online && rc.remote.input.valid &&
                      now_ms - rc.remote.input.received_ms <= FRICTION_REMOTE_TIMEOUT_MS;
        TextConsolePrint(console, "[remote] %s init=%u rx_started=%u valid=%u task_fresh=%u age_ms=%lu frames=%lu bytes=%lu errors=%lu overflow=%lu",
                         !rc.initialized ? "INIT_FAILED" : !rc.port.started ? "UART_ERROR" : online ? "ONLINE" : "OFFLINE",
                         rc.initialized, rc.port.started, rc.remote.input.valid, fresh,
                         (unsigned long)(now_ms - rc.remote.input.received_ms),
                         (unsigned long)rc.remote.input.sequence, (unsigned long)rc.port.bytes,
                         (unsigned long)rc.port.uart_errors, (unsigned long)rc.port.overflows);
        TextConsolePrint(console, "[remote_input] S1=%u S2=%u CH0_milli=%ld CH1_milli=%ld CH2_milli=%ld CH3_milli=%ld",
                         rc.remote.input.s1, rc.remote.input.s2,
                         (long)TextConsoleScaled(rc.remote.input.channel[0], 1000.0),
                         (long)TextConsoleScaled(rc.remote.input.channel[1], 1000.0),
                         (long)TextConsoleScaled(rc.remote.input.channel[2], 1000.0),
                         (long)TextConsoleScaled(rc.remote.input.channel[3], 1000.0));
        TextConsolePrint(console, "[remote_control] switches_on=%u needs_off=%u target_rpm=%ld queued_frames=%lu last_submit=%u max_rpm=%ld channel=%u",
                         rc.remote.switches_on, rc.remote.needs_off,
                         (long)TextConsoleScaled(rc.remote.target_rpm, 1.0), (unsigned long)rc.accepted_frames,
                         (unsigned)rc.last_submit, (long)FRICTION_REMOTE_MAX_RPM, FRICTION_REMOTE_CHANNEL);
        TextConsolePrint(console, "[remote_gear] group=%u gear=%u selected_rpm=%ld previous_gear=%u changed_rx_ms=%lu changes=%lu; sticks do not control speed",
                         rc.remote.group, rc.remote.gear, (long)TextConsoleScaled(rc.remote.selected_rpm, 1.0),
                         rc.previous_gear, (unsigned long)rc.gear_changed_ms, (unsigned long)rc.gear_changes);
        if (rc.remote.needs_off)
        {
            TextConsolePrint(console, "[remote_why] NEED_OFF: S1 拨到停止位置（默认值 1），保持约 0.2 秒，再选低速或高速组；摇杆不参与控制。");
        }
    }
    else
    {
        TextConsolePrint(console, "[remote] DISABLED");
    }
    if (!BENCH_FRICTION_ACTIVE)
    {
        TextConsolePrint(console, "[friction] DISABLED; [F1] DISABLED; [F2] DISABLED; [F3] DISABLED; [readiness] DISABLED");
        return;
    }
    Friction_Telemetry_s telemetry;
    FrictionGetTelemetry(&telemetry);
    static const char *const sources[] = {"NONE", "RTT", "VOFA", "USB", "REMOTE"};
    unsigned source = (unsigned)telemetry.command_source;
    TextConsolePrint(console, "[friction_authority] selected_now=%s epoch=%lu owner_off_required=%u; selected source is not motor enable",
                     source < sizeof(sources) / sizeof(sources[0]) ? sources[source] : "UNKNOWN", (unsigned long)telemetry.source_epoch,
                     telemetry.source_requires_off);
    TextConsolePrint(console, "[parameters] %s revision=%lu storage=UNBOUND; applied values in RAM only",
                     BENCH_ENABLE_PARAMETERS ? "ENABLED" : "DISABLED", (unsigned long)telemetry.tuning_revision);
    /* Sample time after copying: the producer may have preempted this task. */
    now_ms = ControlPlatformNowMs();
    FrictionDiagnosticsExplain(console, &telemetry, now_ms);
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
    TextConsolePrint(console, "[profile] DM-J4310/BMI088=NOT_MONITORED; DBUS UART5 is friction-only; VOFA UART10 remains waves; USB is text");
}
