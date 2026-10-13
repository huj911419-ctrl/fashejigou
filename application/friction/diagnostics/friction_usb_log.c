#include "../../bench_features.h"
#include "friction_usb_log.h"
#include "../input/friction_usb_command.h"
#include "friction_log_stats.h"
#include "friction_diagnostics.h"
#include "friction_status_text.h"
#include "../config/friction_config.h"
#include "../friction.h"
#include "diagnostics.h"
#include "control_platform.h"
#include "board_status.h"
#include "../input/friction_remote_bench.h"
#include "../config/friction_remote_config.h"
#include <limits.h>
#include <math.h>
#include <string.h>

#define USB_RUN_LOG_DEFAULT_PERIOD_MS 20U
#define USB_RUN_LOG_MIN_PERIOD_MS 20U
#define USB_RUN_LOG_MAX_PERIOD_MS 1000U
#define USB_RUN_LOG_METADATA_PERIOD_MS 5000U
#define USB_RUN_LOG_SCHEMA 1U
#define USB_RUN_LOG_SUMMARY_PERIOD_MS 1000U

static unsigned log_mode;
static uint32_t run_period_ms;
static uint32_t sequence;
static uint32_t command_sequence;
static uint32_t header_session;
static uint32_t header_revision;
static uint32_t last_header_ms;
static uint32_t last_emit_ms;
static bool header_needed;
static bool have_emit;
static FrictionLogStats_s statistics;
static uint32_t summary_sequence;

static int32_t LogScaled(double value, double scale)
{
    /* Round fixed-point text to avoid a float 0.02 becoming 19999 ppm.
     * Saturation/nonfinite handling remains in the shared console helper. */
    return TextConsoleScaled(round(value * scale), 1.0);
}

static const char *SourceName(Friction_Command_Source_e source)
{
    static const char *const names[] = {"NONE", "RTT", "VOFA", "USB", "REMOTE"};
    return (unsigned)source < sizeof(names) / sizeof(names[0]) ? names[source] : "UNKNOWN";
}

static bool NumericValid(const Friction_Telemetry_s *sample, unsigned wheel)
{
    const Friction_Status_s *status = &sample->status;
    return isfinite(status->requested_rpm[wheel]) && isfinite(status->reference_rpm[wheel]) &&
           isfinite(status->feedback[wheel].rpm) && isfinite(sample->output[wheel]) &&
           isfinite(sample->current[wheel]) && isfinite(sample->p[wheel]) &&
           isfinite(sample->i[wheel]) && isfinite(sample->d[wheel]);
}

static bool NumericClipped(const Friction_Telemetry_s *sample, unsigned wheel)
{
    const Friction_Status_s *status = &sample->status;
    double values[] = {
        status->requested_rpm[wheel] * 1000.0, status->reference_rpm[wheel] * 1000.0,
        status->feedback[wheel].rpm * 1000.0,
        (status->reference_rpm[wheel] - status->feedback[wheel].rpm) * 1000.0,
        (status->requested_rpm[wheel] - status->feedback[wheel].rpm) * 1000.0,
        sample->output[wheel], sample->current[wheel], sample->p[wheel], sample->i[wheel], sample->d[wheel]
    };
    for (unsigned i = 0U; i < sizeof(values) / sizeof(values[0]); ++i)
    {
        if (isfinite(values[i]) && (values[i] > INT32_MAX || values[i] < INT32_MIN))
        {
            return true;
        }
    }
    return false;
}

static void Metadata(TextConsole_s *console, const Friction_Telemetry_s *sample, uint32_t now_ms)
{
    static const DjiSpeedMotorConfig_s motors[FRICTION_MOTOR_COUNT] = FRICTION_MOTOR_CONFIGS;
    static const Friction_Control_Config_s config = FRICTION_CONTROL_CONFIG;
    static const SpeedReady_Config_s ready = FRICTION_READY_CONFIG;
    uint32_t dropped_before = console->dropped_lines;
    TextConsolePrint(console, "FRMETA,schema=%u,session=%lu,emit_ms=%lu,profile=friction_bench,build=%s %s,period_ms=%lu,revision=%lu",
                     USB_RUN_LOG_SCHEMA, (unsigned long)console->session, (unsigned long)now_ms,
                     __DATE__, __TIME__, (unsigned long)run_period_ms, (unsigned long)sample->tuning_revision);
    TextConsolePrint(console, "FRUNITS,speed_mrpm/1000=motor_rotor_rpm,current_and_PID=raw_C620_units,kp_u/1e6=kp,ki_u/1e6=ki,kd_n/1e9=kd,PID_error_deg_s=6*track_rpm,PID_dt=seconds");
    TextConsolePrint(console, "FRCTRL,policy=%s,continuous=1,anti_windup=conditional,output=current_raw",
                     DJI_SPEED_GROUP_CONTROL_POLICY);
    TextConsolePrint(console, "FRREMOTE,enabled=%u,mode=six_gear,UART=5,timeout_ms=%u,max_rotor_rpm=%ld,stop_s1=%u,low_s1=%u,high_s1=%u,stick=observation_only",
                     BENCH_FRICTION_REMOTE_ACTIVE, FRICTION_REMOTE_TIMEOUT_MS, (long)FRICTION_REMOTE_MAX_RPM,
                     FRICTION_REMOTE_STOP_S1, FRICTION_REMOTE_LOW_S1, FRICTION_REMOTE_HIGH_S1);
    static const FrictionRemoteConfig_s remote_config = FRICTION_REMOTE_CONFIG;
    for (unsigned group = 0U; group < FRICTION_REMOTE_GROUP_COUNT; ++group)
    {
        for (unsigned gear = 0U; gear < FRICTION_REMOTE_GEARS_PER_GROUP; ++gear)
        {
            TextConsolePrint(console, "FRGEARCFG,gear=%u,s1=%u,s2=%u,rpm=%ld",
                             group * FRICTION_REMOTE_GEARS_PER_GROUP + gear + 1U,
                             group == 0U ? remote_config.low_s1 : remote_config.high_s1,
                             remote_config.gear_s2[gear], (long)remote_config.gear_rpm[group][gear]);
        }
    }
    TextConsolePrint(console, "FRREMOTEGUIDE,last_submit:0=QUEUED,1=WRONG_SOURCE,2=UNAVAILABLE,3=BUSY,4=STALE,5=REARM; S1 STOP before run; CH_milli/1000=observed_stick_only");
    TextConsolePrint(console, "FRGEARGUIDE,FRGEAR:seq,rc_sample_ms,online,group,gear,selected_rpm,target_rpm,previous_gear,last_change_rx_ms,changes; selection is not motor enable");
    TextConsolePrint(console, "FRCONTEXT,gear_ratio=UNKNOWN,wheel_diameter=UNKNOWN,projectile_speed=NOT_MEASURED,shot_times=NOT_MEASURED,voltage=NOT_MEASURED,parameters=RAM_ONLY,READY=speed_only_not_feed_permission");
    TextConsolePrint(console, "FRLIMIT,max_rpm=%ld,ramp_rpm_s=%ld,cmd_timeout_ms=%lu,fb_timeout_ms=%lu,max_step_ms=%lu,temp_limit_C=%u,control_period_ms=%u,compiled_mask=%u,vofa=%u",
                     (long)LogScaled(config.max_rpm, 1.0),
                     (long)LogScaled(sample->tuning.ramp_rpm_s, 1.0),
                     (unsigned long)config.command_timeout_ms, (unsigned long)config.feedback_timeout_ms,
                     (unsigned long)config.max_step_ms, config.max_temperature_c, FRICTION_PERIOD_MS,
                     BENCH_FRICTION_MASK, BENCH_ENABLE_VOFA);
    TextConsolePrint(console, "FRREADY,min_rpm=%ld,ref_tol_mrpm=%ld,enter_abs_mrpm=%ld,enter_rel_ppm=%ld,exit_abs_mrpm=%ld,exit_rel_ppm=%ld,dwell_ms=%lu,max_gap_ms=%lu,enabled=%u",
                     (long)LogScaled(ready.minimum_target, 1.0),
                     (long)LogScaled(ready.reference_tolerance, 1000.0),
                     (long)LogScaled(ready.enter_absolute_error, 1000.0),
                     (long)LogScaled(ready.enter_relative_error, 1000000.0),
                     (long)LogScaled(ready.exit_absolute_error, 1000.0),
                     (long)LogScaled(ready.exit_relative_error, 1000000.0),
                     (unsigned long)ready.dwell_ms, (unsigned long)ready.maximum_sample_gap_ms,
                     BENCH_SPEED_READY_ACTIVE);
    TextConsolePrint(console, "FRFAULT,bit1=CONFIG,bit2=COMMAND,bit4=OFFLINE,bit8=TEMPERATURE,bit16=TIMING,bit32=DRIVER,active_fault=this_step,fault=latched_history");
    TextConsolePrint(console, "FRTUNE,kp_max_u=%ld,ki_max_u=%ld,kd_max_n=%ld,ramp_min_rpm_s=1,ramp_max_rpm_s=%ld",
                     (long)LogScaled(FRICTION_TUNE_KP_MAX, 1000000.0),
                     (long)LogScaled(FRICTION_TUNE_KI_MAX, 1000000.0),
                     (long)LogScaled(FRICTION_TUNE_KD_MAX, 1000000000.0),
                     (long)LogScaled(FRICTION_TUNE_RAMP_MAX, 1.0));
    TextConsolePrint(console, "FRPHASE,0=STOPPED,1=SPINUP,2=READY,3=RECOVERING,4=FAULT,5=BENCH; masks: F1=1 F2=2 F3=4 ALL=7");
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        const Friction_PID_Tuning_s *pid = &sample->tuning.motor[i];
        TextConsolePrint(console, "FRPID,wheel=%u,can=%u,id=%u,dir=%d,kp_u=%ld,ki_u=%ld,kd_n=%ld,limit_raw=%ld,ceiling_raw=%ld,configured=%u",
                         i + 1U, motors[i].can_bus, motors[i].id, motors[i].direction,
                         (long)LogScaled(pid->kp, 1000000.0),
                         (long)LogScaled(pid->ki, 1000000.0),
                         (long)LogScaled(pid->kd, 1000000000.0),
                         (long)LogScaled(pid->current_limit, 1.0),
                         (long)LogScaled(motors[i].current_limit, 1.0),
                         (BENCH_FRICTION_MASK & (1U << i)) != 0U);
    }
    if (BENCH_ENABLE_BOARD_DIAGNOSTICS)
    {
        BoardStatus_s board;
        BoardStatusRead(&board);
        TextConsolePrint(console, "FRBOARD,cpu_hz=%lu,uid=%08lx-%08lx-%08lx,reset_flags=%lu,heap_free_B=%lu,heap_min_B=%lu",
                         (unsigned long)board.cpu_hz, (unsigned long)board.uid[0],
                         (unsigned long)board.uid[1], (unsigned long)board.uid[2],
                         (unsigned long)board.reset_flags, (unsigned long)board.heap_free,
                         (unsigned long)board.heap_minimum);
        for (unsigned i = 0U; i < 3U; ++i)
        {
            const BoardCanStatus_s *can = &board.can[i];
            TextConsolePrint(console, "FRCAN,bus=%u,readable=%u,started=%u,bus_off=%u,passive=%u,TEC=%lu,REC=%lu,HALerror=%lu,free=%lu,pending=%lu,observed_ms=%lu",
                             i + 1U, can->readable, can->started, can->bus_off, can->error_passive,
                             (unsigned long)can->tx_error_count, (unsigned long)can->rx_error_count,
                             (unsigned long)can->hal_error, (unsigned long)can->tx_free,
                             (unsigned long)can->tx_pending, (unsigned long)now_ms);
        }
    }
    TextConsolePrint(console, "FRGUIDE,Analyze fresh enabled feedback only; compare actual to ref during ramp and to target after ramp; never tune PID to fix timeout/offline/rearm/timing failures.");
    TextConsolePrint(console, "FRGUIDE,Frames are decimated observations, not every 1ms control step; use board sample_ms; gaps/incomplete/clipped/invalid data limit acceptance; do not infer projectile performance.");
    TextConsolePrint(console, "FRSCHEMA,FS,seq,emit_ms,sample_ms,sample_id,age_ms,fresh,cmd_age_ms,step_ms,fault,active_fault,request_mask,enabled_mask,online_mask,ready_mask,all_ready,phase,revision,tune_seq,cmd_gen,rearm");
    TextConsolePrint(console, "FRSCHEMA,FM,seq,owner_now,owner_epoch,owner_off,init_ok,cmd_received_now,pending_mask_now,tune_pending_now,profile_pending_now");
    TextConsolePrint(console, "FRSCHEMA,FRRC,seq,sample_ms,rx_ms,frames,valid,online,needs_off,s1,s2,stick_milli,target_mrpm,uart_bytes,uart_errors,overflow,last_submit; separate observer snapshot");
    TextConsolePrint(console, "FRSCHEMA,FT,seq,calls,interval_ms,max_interval_ms,over_limit,same_ms; invocation spacing, not execution time");
    TextConsolePrint(console, "FRSCHEMA,FW,seq,wheel,en,valid,fb_age_ms,temp_C,target_mrpm,ref_mrpm,actual_mrpm,track_mrpm,final_mrpm,out_raw,iq_raw,iq_logical_raw,P_raw,I_raw,D_raw,sat,finite,clip");
    TextConsolePrint(console, "FRSCHEMA,FE,seq,complete,skipped_reports,dropped_lines,busy_polls,emit_gap_ms; FM mailbox fields may be newer than FS motor sample; FW valid alone does not mean fresh");
    TextConsolePrint(console, "FRSCHEMA,FAS,window,start_ms,end_ms,frames,max_gap_ms,revision,request_mask,enabled_mask,fault");
    TextConsolePrint(console, "FRSCHEMA,FA,window,wheel,target_mrpm,valid_n,steady_n,in_band_n,sat_n,actual_min_mrpm,actual_max_mrpm,track_abs_max_mrpm,final_mean_mrpm,final_rms_mrpm,final_min_mrpm,final_max_mrpm,out_abs_max_raw,temp_max_C");
    TextConsolePrint(console, "FRGUIDE,FA is a ~1s summary of observed samples only; steady_n excludes ramp; zero valid/steady counts mean unavailable; target/mask/fault/revision changes split windows; FAS_END marks completeness.");
    TextConsolePrint(console, "FRMETA_END,revision=%lu,complete=%u", (unsigned long)sample->tuning_revision,
                     console->dropped_lines == dropped_before);
    header_needed = console->dropped_lines != dropped_before;
    header_session = console->session;
    header_revision = sample->tuning_revision;
    last_header_ms = now_ms;
}

static void Summary(TextConsole_s *console)
{
    if (statistics.frames == 0U)
    {
        return;
    }
    ++summary_sequence;
    uint32_t dropped_before = console->dropped_lines;
    TextConsolePrint(console, "FAS,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%lu",
                     (unsigned long)summary_sequence, (unsigned long)statistics.start_ms,
                     (unsigned long)statistics.end_ms, (unsigned long)statistics.frames,
                     (unsigned long)statistics.maximum_gap_ms, (unsigned long)statistics.revision,
                     statistics.requested_mask, statistics.enabled_mask, (unsigned long)statistics.fault);
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        const FrictionLogWheelStats_s *wheel = &statistics.wheel[i];
        double mean = wheel->steady_samples ? wheel->final_sum / wheel->steady_samples : 0.0;
        double rms = wheel->steady_samples ? sqrt(wheel->final_square_sum / wheel->steady_samples) : 0.0;
        TextConsolePrint(console, "FA,%lu,%u,%ld,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u",
                         (unsigned long)summary_sequence, i + 1U,
                         (long)LogScaled(statistics.target[i], 1000.0),
                         (unsigned long)wheel->valid_samples, (unsigned long)wheel->steady_samples,
                         (unsigned long)wheel->in_band_samples, (unsigned long)wheel->saturated_samples,
                         (long)LogScaled(wheel->actual_min, 1000.0),
                         (long)LogScaled(wheel->actual_max, 1000.0),
                         (long)LogScaled(wheel->tracking_abs_max, 1000.0),
                         (long)LogScaled(mean, 1000.0), (long)LogScaled(rms, 1000.0),
                         (long)LogScaled(wheel->final_min, 1000.0),
                         (long)LogScaled(wheel->final_max, 1000.0),
                         (long)LogScaled(wheel->output_abs_max, 1.0), wheel->temperature_max);
    }
    TextConsolePrint(console, "FAS_END,%lu,complete=%u", (unsigned long)summary_sequence,
                     console->dropped_lines == dropped_before);
    FrictionLogStatsReset(&statistics);
}

static void RunReport(TextConsole_s *console, uint32_t now_ms)
{
    static const Friction_Control_Config_s config = FRICTION_CONTROL_CONFIG;
    Friction_Telemetry_s sample;
    FrictionGetTelemetry(&sample);
    now_ms = ControlPlatformNowMs();
    uint32_t dropped_before = console->dropped_lines;
    if (header_needed || header_session != console->session || header_revision != sample.tuning_revision ||
        now_ms - last_header_ms >= USB_RUN_LOG_METADATA_PERIOD_MS)
    {
        Metadata(console, &sample, now_ms);
    }
    if (!FrictionLogStatsMatches(&statistics, &sample) ||
        (statistics.frames != 0U && now_ms - statistics.start_ms >= USB_RUN_LOG_SUMMARY_PERIOD_MS))
    {
        Summary(console);
    }
    uint32_t emit_gap = have_emit ? now_ms - last_emit_ms : 0U;
    have_emit = true;
    last_emit_ms = now_ms;
    ++sequence;
    const Friction_Status_s *status = &sample.status;
    uint32_t age_ms = now_ms - sample.sample_ms;
    bool fresh = sample.sample_id != 0U && age_ms <= config.max_step_ms;
    uint8_t usable_mask = 0U;
    TextConsolePrint(console, "FS,%lu,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%lu,%u,%lu,%u",
                     (unsigned long)sequence, (unsigned long)now_ms, (unsigned long)sample.sample_ms,
                     (unsigned long)sample.sample_id, (unsigned long)age_ms, fresh,
                     (unsigned long)status->command_age_ms, (unsigned long)status->step_ms,
                     (unsigned long)status->fault, (unsigned long)status->active_fault,
                     status->requested_mask, status->enabled_mask, status->online_mask,
                     sample.readiness.ready_mask, sample.readiness.all_ready, (unsigned)sample.readiness.phase,
                     (unsigned long)sample.tuning_revision, sample.applied_sequence,
                     (unsigned long)sample.command_generation, status->rearm_required);
    TextConsolePrint(console, "FM,%lu,%s,%lu,%u,%u,%u,%u,%u,%u", (unsigned long)sequence,
                     SourceName(sample.command_source), (unsigned long)sample.source_epoch,
                     sample.source_requires_off, sample.initialized, sample.command_received_now,
                     sample.pending_mask_now, sample.tuning_pending_now, sample.profile_pending_now);
    if (BENCH_FRICTION_REMOTE_ACTIVE)
    {
        FrictionRemoteBenchStatus_s rc;
        FrictionRemoteBenchGetStatus(&rc);
        bool online = rc.initialized && now_ms - rc.sample_ms <= config.max_step_ms && rc.remote.online &&
                      rc.remote.input.valid && now_ms - rc.remote.input.received_ms <= FRICTION_REMOTE_TIMEOUT_MS;
        TextConsolePrint(console, "FRRC,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%ld,%ld,%lu,%lu,%lu,%u",
                         (unsigned long)sequence, (unsigned long)rc.sample_ms,
                         (unsigned long)rc.remote.input.received_ms, (unsigned long)rc.remote.input.sequence,
                         rc.remote.input.valid, online, rc.remote.needs_off, rc.remote.input.s1, rc.remote.input.s2,
                         (long)LogScaled(rc.remote.stick, 1000.0), (long)LogScaled(rc.remote.target_rpm, 1000.0),
                         (unsigned long)rc.port.bytes, (unsigned long)rc.port.uart_errors,
                         (unsigned long)rc.port.overflows, (unsigned)rc.last_submit);
        TextConsolePrint(console, "FRGEAR,%lu,%lu,%u,%u,%u,%ld,%ld,%u,%lu,%lu",
                         (unsigned long)sequence, (unsigned long)rc.sample_ms, online,
                         rc.remote.group, rc.remote.gear, (long)LogScaled(rc.remote.selected_rpm, 1.0),
                         (long)LogScaled(rc.remote.target_rpm, 1.0), rc.previous_gear,
                         (unsigned long)rc.gear_changed_ms, (unsigned long)rc.gear_changes);
    }
    const LoopStats_s *timing = &status->timing;
    TextConsolePrint(console, "FT,%lu,%lu,%lu,%lu,%lu,%lu", (unsigned long)sequence,
                     (unsigned long)timing->samples, (unsigned long)timing->last_interval_ms,
                     (unsigned long)timing->max_interval_ms, (unsigned long)timing->late_count,
                     (unsigned long)timing->zero_interval_count);
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        const Friction_Feedback_s *feedback = &status->feedback[i];
        bool saturated = (status->enabled_mask & (1U << i)) &&
                         sample.tuning.motor[i].current_limit > 0.0f &&
                         fabsf(sample.output[i]) >= sample.tuning.motor[i].current_limit - 1.0f;
        if (fresh && feedback->valid && now_ms - feedback->last_rx_ms <= config.feedback_timeout_ms &&
            NumericValid(&sample, i) && !NumericClipped(&sample, i))
        {
            usable_mask |= (uint8_t)(1U << i);
        }
        TextConsolePrint(console, "FW,%lu,%u,%u,%u,%lu,%u,%ld,%ld,%ld,%ld,%ld,%ld,%d,%ld,%ld,%ld,%ld,%u,%u,%u",
                         (unsigned long)sequence, i + 1U, (status->enabled_mask & (1U << i)) != 0U,
                         feedback->valid, (unsigned long)(now_ms - feedback->last_rx_ms), feedback->temperature_c,
                         (long)LogScaled(status->requested_rpm[i], 1000.0),
                         (long)LogScaled(status->reference_rpm[i], 1000.0),
                         (long)LogScaled(feedback->rpm, 1000.0),
                         (long)LogScaled(status->reference_rpm[i] - feedback->rpm, 1000.0),
                         (long)LogScaled(status->requested_rpm[i] - feedback->rpm, 1000.0),
                         (long)LogScaled(sample.output[i], 1.0), feedback->current_raw,
                         (long)LogScaled(sample.current[i], 1.0),
                         (long)LogScaled(sample.p[i], 1.0), (long)LogScaled(sample.i[i], 1.0),
                         (long)LogScaled(sample.d[i], 1.0), saturated, NumericValid(&sample, i),
                         NumericClipped(&sample, i));
    }
    TextConsolePrint(console, "FE,%lu,%u,%lu,%lu,%lu,%lu", (unsigned long)sequence,
                     console->dropped_lines == dropped_before, (unsigned long)DiagnosticsSkippedReports(),
                     (unsigned long)console->dropped_lines, (unsigned long)console->busy_polls,
                     (unsigned long)emit_gap);
    static const SpeedReady_Config_s ready = FRICTION_READY_CONFIG;
    FrictionLogStatsObserve(&statistics, &sample, now_ms, usable_mask, &ready);
}

void FrictionUsbLogInit(void)
{
    log_mode = BENCH_FRICTION_USB_LOG_MODE;
    run_period_ms = USB_RUN_LOG_DEFAULT_PERIOD_MS;
    sequence = 0U;
    command_sequence = 0U;
    header_session = 0U;
    header_revision = 0U;
    last_header_ms = 0U;
    last_emit_ms = 0U;
    have_emit = false;
    header_needed = true;
    summary_sequence = 0U;
    FrictionLogStatsReset(&statistics);
    (void)DiagnosticsSetStatusReport(FrictionStatusTextReport, FRICTION_DIAGNOSTICS_PERIOD_MS);
    DiagnosticsSetPeriodicReport(log_mode == 2U ? RunReport : NULL, run_period_ms);
}

static bool LogCommand(TextConsole_s *console, const char *line)
{
    if (strcmp(line, "log 1") == 0 || strcmp(line, "log 2") == 0)
    {
        log_mode = (unsigned)(line[4] - '0');
        header_needed = true;
        have_emit = false;
        FrictionLogStatsReset(&statistics);
        DiagnosticsSetPeriodicReport(log_mode == 2U ? RunReport : NULL, run_period_ms);
        TextConsolePrint(console, "[OK] log_mode=%u (%s); stream on/off still controls periodic output",
                         log_mode, log_mode == 1U ? "fault diagnostics" : "friction run CSV");
        return true;
    }
    if (strncmp(line, "log period ", 11U) == 0)
    {
        uint32_t value = 0U;
        const char *cursor = line + 11U;
        bool valid = *cursor != '\0';
        while (*cursor != '\0' && valid)
        {
            valid = *cursor >= '0' && *cursor <= '9' && value <= USB_RUN_LOG_MAX_PERIOD_MS;
            if (valid)
            {
                value = value * 10U + (uint32_t)(*cursor++ - '0');
            }
        }
        if (valid && value >= USB_RUN_LOG_MIN_PERIOD_MS && value <= USB_RUN_LOG_MAX_PERIOD_MS)
        {
            run_period_ms = value;
            header_needed = true;
            if (log_mode == 2U)
            {
                DiagnosticsSetPeriodicReport(RunReport, run_period_ms);
            }
            TextConsolePrint(console, "[OK] log_period_ms=%lu", (unsigned long)run_period_ms);
        }
        else
        {
            TextConsolePrint(console, "[ERR] log period requires integer 20..1000 ms");
        }
        return true;
    }
    TextConsolePrint(console, "[ERR] log syntax: log 1 / log 2 / log period 20..1000");
    return true;
}

static const char *ResultHint(FrictionUsbResult_e result)
{
    switch (result)
    {
        case FRICTION_USB_SYNTAX: return "格式/参数数量错误或包含非有限数；用 help 核对命令。";
        case FRICTION_USB_RANGE: return "PID/限流/斜坡参数越界；查当前配置与编译上限。";
        case FRICTION_USB_WRONG_SOURCE: return "USB 不是当前控制来源；先 source usb，再 off -> set。";
        case FRICTION_USB_REARM_REQUIRED: return "来源切换尚未解锁；先 off，等停用，再 set。";
        case FRICTION_USB_BUSY: return "参数事务忙或版本已变化；等新快照后再试。";
        case FRICTION_USB_STALE: return "接收时间已过期；查 USB 任务调度，重新发命令。";
        case FRICTION_USB_UNAVAILABLE: return "摩擦轮未初始化或模块关闭；查配置和模块开关。";
        default: return "输入已排队；以电机快照或实际应用参数确认结果。";
    }
}

bool FrictionUsbConsoleCommand(TextConsole_s *console, const char *line, uint32_t now_ms)
{
    if (strcmp(line, "help") == 0)
    {
        TextConsolePrint(console, "[USB] off; set <mask> <rpm1> <rpm2> <rpm3>; pid <wheel> <kp> <ki> <kd> <limit>; ramp <rpm_s>");
        TextConsolePrint(console, "[USB] log 1=fault diagnostics; log 2=AI run CSV; log period 20..1000 ms; current mode=%u", log_mode);
        TextConsolePrint(console, "[阅读] log 1每5秒简报；status查一次；status detail查详细中文，status raw查原始字段。");
        TextConsolePrint(console, "[USB] source usb / source remote: stop and select owner; USB requires off -> set; remote requires S1 STOP -> run group");
        TextConsolePrint(console, "[USB] set keepalive: repeat every 100ms; OK/QUEUED is not motor enable or PID applied; queries/logging do not renew commands");
        return true;
    }
    if (strcmp(line, "status detail") == 0)
    {
        if (TextConsolePending(console) != 0U)
        {
            TextConsolePrint(console, "上一份报告还在发送，请稍后再查详情。");
            return true;
        }
        uint32_t dropped_before = console->dropped_lines;
        TextConsolePrint(console, "%s", "");
        TextConsolePrint(console, "-------- 摩擦轮详情 --------");
        FrictionStatusTextDetailReport(console, now_ms);
        if (console->dropped_lines != dropped_before)
        {
            TextConsolePrint(console, "报告不完整，请稍后重新查询。");
        }
        TextConsolePrint(console, "--------------------------");
        return true;
    }
    if (strncmp(line, "log", 3U) == 0 && (line[3] == '\0' || line[3] == ' '))
    {
        return LogCommand(console, line);
    }
    if (strncmp(line, "source", 6U) == 0 && (line[6] == '\0' || line[6] == ' '))
    {
        Friction_Command_Source_e source;
        if (strcmp(line, "source usb") == 0)
        {
            source = FRICTION_SOURCE_USB;
        }
        else if (strcmp(line, "source remote") == 0)
        {
            source = FRICTION_SOURCE_REMOTE;
        }
        else
        {
            TextConsolePrint(console, "[ERR] syntax: source usb / source remote");
            return true;
        }
        Friction_Submit_Result_e result = FrictionSelectSource(source);
        TextConsolePrint(console, "[%s] source=%s result=%u; source change queues STOP, then requires fresh owner OFF",
                         result == FRICTION_SUBMIT_ACCEPTED ? "OK" : "ERR", SourceName(source), (unsigned)result);
        TextConsolePrint(console, "FRSOURCE,%lu,%s,%u", (unsigned long)now_ms, SourceName(source), (unsigned)result);
        return true;
    }
    bool stop = strcmp(line, "off") == 0;
    bool run = strncmp(line, "set ", 4U) == 0;
    bool tune = strncmp(line, "pid ", 4U) == 0 || strncmp(line, "ramp ", 5U) == 0;
    if (!stop && !run && !tune)
    {
        return false;
    }
    FrictionUsbResult_e result;
    if (stop)
    {
        result = FrictionUsbExecute(line, now_ms);
        TextConsolePrint(console, "[OK] friction off");
    }
    else
    {
        result = FrictionUsbExecute(line, now_ms);
        TextConsolePrint(console, "[%s] friction %s: %s", result == FRICTION_USB_QUEUED ? "OK" : "ERR",
                         FrictionUsbResultName(result), ResultHint(result));
    }
    ++command_sequence;
    TextConsolePrint(console, "FRCMD,%lu,%lu,%s,%s", (unsigned long)command_sequence,
                     (unsigned long)now_ms, FrictionUsbResultName(result), line);
    if (result != FRICTION_USB_QUEUED)
    {
        Friction_Telemetry_s sample;
        FrictionGetTelemetry(&sample);
        FrictionDiagnosticsExplain(console, &sample, ControlPlatformNowMs());
    }
    return true;
}
