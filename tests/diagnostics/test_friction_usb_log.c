#include "../../application/bench_features.h"
#include "diagnostics.h"
#include "friction_usb_log.h"
#include "friction_usb_command.h"
#include "friction_diagnostics.h"
#include "friction.h"
#include "board_status.h"
#include "usb_console_port.h"
#include "control_platform.h"
#include "runtime_events.h"
#include "friction_remote_bench.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static char output[65536];
static size_t used;
static char input[128];
static size_t input_size;
static uint32_t clock_ms = 1000U;
static uint32_t session = 1U;
static bool busy;
static Friction_Telemetry_s snapshot;
extern FrictionRemoteBenchStatus_s remote_observer_fixture;
static unsigned submits, stops, tunes;
static Friction_Command_s submitted;
static Friction_Tuning_s queued;
static Friction_Submit_Result_e submit_result = FRICTION_SUBMIT_ACCEPTED;
static Friction_Tune_Result_e tune_result = FRICTION_TUNE_QUEUED;

uint32_t ControlPlatformNowMs(void) { return clock_ms; }
uint32_t ControlPlatformEnter(void) { return 0U; }
void ControlPlatformExit(uint32_t value) { (void)value; }
void FrictionGetTelemetry(Friction_Telemetry_s *value) { *value = snapshot; }
void FrictionDisable(void) { ++stops; }
void FrictionStopFromUsb(void) { ++stops; }
Friction_Submit_Result_e FrictionSelectSource(Friction_Command_Source_e source)
{
    snapshot.command_source = source;
    return FRICTION_SUBMIT_ACCEPTED;
}
Friction_Submit_Result_e FrictionSubmitFrom(Friction_Command_Source_e source,
                                           const Friction_Command_s *command, uint32_t timestamp)
{
    assert(source == FRICTION_SOURCE_USB && timestamp == clock_ms);
    ++submits;
    submitted = *command;
    return submit_result;
}
Friction_Tune_Result_e FrictionQueueTuning(const Friction_Tuning_s *value, uint16_t seq, uint16_t base)
{
    assert(seq != 0U && base == snapshot.applied_sequence);
    ++tunes;
    queued = *value;
    return tune_result;
}
void BoardStatusRead(BoardStatus_s *value)
{
    memset(value, 0, sizeof(*value));
    value->cpu_hz = 550000000U;
    value->uid[0] = UINT32_MAX;
    value->can[1].readable = true;
    value->can[1].started = true;
}
void UsbConsolePortGetStats(UsbConsoleStats_s *value) { memset(value, 0, sizeof(*value)); }
static uint32_t Session(void *context) { (void)context; return session; }
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
    if (busy) { return false; }
    assert(used + count < sizeof(output));
    memcpy(output + used, bytes, count);
    used += count;
    output[used] = '\0';
    return true;
}
TextConsolePort_s UsbConsolePortInit(void)
{
    return (TextConsolePort_s){.session = Session, .read = Read, .write = Write, .utf8_packet_size = 63U};
}
static void ClearOutput(void) { used = 0U; output[0] = '\0'; }
static void Drain(void) { for (unsigned i = 0U; i < 100U; ++i) { DiagnosticsPoll(clock_ms); } }
static void Send(const char *line)
{
    ClearOutput();
    input_size = strlen(line);
    assert(input_size < sizeof(input));
    memcpy(input, line, input_size);
    Drain();
}
static void Advance(uint32_t interval)
{
    clock_ms += interval;
    remote_observer_fixture.sample_ms = clock_ms;
    remote_observer_fixture.remote.input.received_ms = clock_ms;
    snapshot.sample_ms = clock_ms;
    ++snapshot.sample_id;
    for (unsigned i = 0U; i < 3U; ++i) { snapshot.status.feedback[i].last_rx_ms = clock_ms; }
    ClearOutput();
    Drain();
}

int main(int argc, char **argv)
{
    RuntimeEventsInit();
    remote_observer_fixture = (FrictionRemoteBenchStatus_s){
        .initialized = true, .port = {.started = true}, .gear_changed_ms = 990U, .gear_changes = 3U, .previous_gear = 4U,
        .remote = {.input = {.valid = true, .s1 = 2U, .s2 = 3U}, .online = true,
                   .needs_off = true, .group = 2U, .gear = 5U, .selected_rpm = 6000.0f},
    };
    snapshot.initialized = 1U;
    snapshot.sample_id = 1U;
    snapshot.sample_ms = clock_ms;
    snapshot.command_source = FRICTION_SOURCE_USB;
    snapshot.command_received_now = 1U;
    snapshot.status.step_ms = 1U;
    snapshot.status.enabled_mask = 7U;
    snapshot.status.requested_mask = 7U;
    snapshot.status.online_mask = 7U;
    snapshot.readiness.phase = FRICTION_PHASE_READY;
    snapshot.readiness.ready_mask = 7U;
    snapshot.readiness.all_ready = 1U;
    snapshot.tuning.ramp_rpm_s = 1300.0f;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        snapshot.status.requested_rpm[i] = 1000.0f;
        snapshot.status.reference_rpm[i] = 1000.0f;
        snapshot.status.feedback[i] = (Friction_Feedback_s){
            .valid = 1U, .rpm = 999.125f, .last_rx_ms = clock_ms, .temperature_c = 25U, .current_raw = -44};
        snapshot.tuning.motor[i] = (Friction_PID_Tuning_s){2.1f, 0.02f, 0.0f, 14000.0f};
        snapshot.current[i] = 44.0f;
        snapshot.output[i] = 1234.0f;
        snapshot.p[i] = 1200.0f;
        snapshot.i[i] = 34.0f;
    }
    assert(DiagnosticsInitWithCommands("friction_bench", FrictionDiagnosticsReport, FrictionUsbConsoleCommand));
    FrictionUsbLogInit();
    Drain();
    assert(strstr(output, "console=2") != NULL);
    Send("log 1\n");
    assert(!DiagnosticsSetStatusReport(NULL, 199U));
    assert(!DiagnosticsSetStatusReport(NULL, 10001U));
    Advance(4999U);
    assert(used == 0U); /* Human output no longer refreshes each second. */
    Advance(1U);
    assert(strstr(output, "控制方式：USB电脑") != NULL);
    assert(strstr(output, "F1：在线 | 目标 1000 | 实际 999 RPM") != NULL);
    assert(strstr(output, "PID") == NULL && strstr(output, "CAN2") == NULL);
    assert(used < 1000U && strstr(output, "报告不完整") == NULL);
    Send("status detail\n");
    assert(strstr(output, "当前结论与处理方法") != NULL);
    assert(strstr(output, "Kp=2.100，Ki=0.020，Kd=0.000000") != NULL);
    assert(strstr(output, "[board] cpu_hz") == NULL && strstr(output, "[F1_PID] kp_milli") == NULL);
    assert(strstr(output, "报告不完整") == NULL && used < 6500U);
    Send("period 10000\n");
    Advance(9999U);
    assert(used == 0U);
    Advance(1U);
    assert(strstr(output, "控制方式：USB电脑") != NULL && strstr(output, "PID") == NULL);
    Send("period 5000\n");
    Friction_Telemetry_s saved = snapshot;
    FrictionRemoteBenchStatus_s saved_remote = remote_observer_fixture;
    snapshot.command_source = FRICTION_SOURCE_REMOTE;
    snapshot.status.requested_mask = snapshot.status.enabled_mask = 0U;
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        snapshot.status.requested_rpm[i] = snapshot.status.reference_rpm[i] = 0.0f;
        snapshot.output[i] = 0.0f;
    }
    snapshot.status.feedback[0].rpm = 1234.0f;
    snapshot.readiness.phase = FRICTION_PHASE_STOPPED;
    snapshot.readiness.ready_mask = snapshot.readiness.all_ready = 0U;
    remote_observer_fixture.remote.input.s1 = 1U;
    remote_observer_fixture.remote.group = remote_observer_fixture.remote.gear = 0U;
    remote_observer_fixture.remote.selected_rpm = remote_observer_fixture.remote.target_rpm = 0.0f;
    remote_observer_fixture.remote.needs_off = false;
    Send("status\n");
    assert(strstr(output, "S1在停止位置，这是正常停机") != NULL);
    assert(strstr(output, "可能在滑行") != NULL);
    assert(used < 1000U && strstr(output, "PID") == NULL);
    if (argc == 3 && strcmp(argv[1], "--human-sample") == 0)
    {
        FILE *example = fopen(argv[2], "wb");
        assert(example != NULL);
        fputs("# 主机模拟示例：并非实物运行记录。\n", example);
        assert(fwrite(output, 1U, used, example) == used);
        assert(fclose(example) == 0);
    }
    snapshot.status.requested_mask = 7U;
    snapshot.status.online_mask = 0U;
    snapshot.status.fault = snapshot.status.active_fault = 63U;
    snapshot.status.command_age_ms = 251U;
    snapshot.status.step_ms = 8U;
    for (unsigned i = 0U; i < 3U; ++i) { snapshot.status.feedback[i].valid = 0U; }
    Send("status\n");
    assert(strstr(output, "当前存在故障") != NULL && strstr(output, "运行命令无效或过期") != NULL);
    assert(strstr(output, "F1反馈未收到") != NULL && strstr(output, "CAN输出提交失败") != NULL);
    assert(strstr(output, "S1先拨停止") != NULL);
    assert(strstr(output, "报告不完整") == NULL && used < 2500U);
    snapshot = saved;
    snapshot.sample_id = 0U;
    Send("status\n");
    assert(strstr(output, "控制快照过期") != NULL);
    assert(strstr(output, "当前转速未知") != NULL);
    Send("status detail\n");
    assert(strstr(output, "旧记录：请求运行") != NULL);
    snapshot = saved;
    snapshot.output[0] = NAN;
    Send("status detail\n");
    assert(strstr(output, "指令或上限数值无效") != NULL);
    snapshot = saved;
    remote_observer_fixture = saved_remote;
    snapshot.status.feedback[0].rpm = NAN;
    Send("status\n");
    assert(strstr(output, "F1：数据过期或无效；当前转速未知") != NULL);
    Send("status detail\n");
    assert(strstr(output, "[F1速度] 无有效速度记录") != NULL);
    snapshot = saved;
    snapshot.command_source = FRICTION_SOURCE_REMOTE;
    remote_observer_fixture.remote.online = false;
    Send("status\n");
    assert(strstr(output, "没有有效遥控信号") != NULL && strstr(output, "source usb，再off") != NULL);
    assert(used < 1000U);
    snapshot = saved;
    remote_observer_fixture = saved_remote;
    assert(submits == 0U && stops == 0U && tunes == 0U); /* Presentation never authorizes control. */
    Send("log 2\n");
    Advance(20U);
    assert(strstr(output, "FRMETA,schema=1") != NULL);
    assert(strstr(output, "FRCTRL,policy=speed_pid_conditional_v1") != NULL);
    assert(strstr(output, "FRUNITS,") != NULL && strstr(output, "FRPID,wheel=1") != NULL);
    assert(strstr(output, "kp_u=2100000,ki_u=20000") != NULL);
    assert(strstr(output, "enter_rel_ppm=20000") != NULL);
    assert(strstr(output, "FRSCHEMA,FS,") != NULL && strstr(output, "FRSCHEMA,FW,") != NULL);
    assert(strstr(output, ",1000000,1000000,999125,875,875,1234,-44,44,1200,34,0,0,1,0") != NULL);
    assert(strstr(output, "FRMETA_END,revision=0,complete=1") != NULL);
    assert(strstr(output, "mode=six_gear") != NULL);
    assert(strstr(output, "FRGEARCFG,gear=5,s1=2,s2=3,rpm=6000") != NULL);
    assert(strstr(output, "FRGEAR,") != NULL);
    assert(strstr(output, ",1,2,5,6000,0,4,990,3") != NULL); /* Selected != authorized target. */
    assert(submits == 0U && stops == 0U && tunes == 0U);
    assert(strstr(output, "FE,") != NULL && used < 6000U);
    if (argc == 3 && strcmp(argv[1], "--sample") == 0)
    {
        FILE *example = fopen(argv[2], "wb");
        assert(example != NULL);
        fputs("# SIMULATED HOST FIXTURE: not hardware evidence or tuned parameters.\n", example);
        assert(fwrite(output, 1U, used, example) == used);
        assert(fclose(example) == 0);
    }

    Advance(20U);
    assert(strstr(output, "FS,") != NULL && strstr(output, "FRMETA,") == NULL);
    ++snapshot.tuning_revision;
    snapshot.tuning.motor[0].kp = 3.0f;
    Advance(20U);
    assert(strstr(output, "FRMETA,") != NULL && strstr(output, "kp_u=3000000") != NULL);
    snapshot.output[0] = 14000.0f;
    snapshot.status.feedback[1].rpm = NAN;
    snapshot.status.requested_rpm[2] = 1e20f;
    Advance(20U);
    assert(strstr(output, "14000,-44,44,1200,34,0,1,1,0") != NULL);
    assert(strstr(output, "1200,34,0,0,0,0") != NULL); /* Nonfinite zero is explicitly invalid. */
    assert(strstr(output, "1200,34,0,0,1,1") != NULL); /* Saturating conversion is explicit. */
    snapshot.status.feedback[1].rpm = 999.125f;
    snapshot.status.requested_rpm[2] = 1000.0f;

    bool summary_seen = false;
    for (unsigned i = 0U; i < 50U; ++i)
    {
        Advance(20U);
        if (strstr(output, "FAS,") != NULL)
        {
            assert(strstr(output, "FAS_END,") != NULL && strstr(output, "FA,") != NULL);
            assert(strstr(output, "complete=1") != NULL);
            summary_seen = true;
        }
    }
    assert(summary_seen);

    snapshot.sample_ms = clock_ms - 6U;
    Send("status raw\n");
    assert(strstr(output, "TASK_STALE") != NULL);
    snapshot.sample_ms = clock_ms;
    snapshot.initialized = 0U;
    Send("status raw\n");
    assert(strstr(output, "INIT_FAILED") != NULL);
    snapshot.initialized = 1U;

    snapshot.status.fault = FRICTION_FAULT_COMMAND;
    snapshot.status.active_fault = FRICTION_FAULT_COMMAND;
    snapshot.status.command_age_ms = 251U;
    snapshot.status.enabled_mask = 0U;
    snapshot.status.rearm_required = 1U;
    Send("status raw\n");
    assert(strstr(output, "COMMAND_TIMEOUT") != NULL && strstr(output, "actual_ms=251 limit_ms=250") != NULL);
    snapshot.status.active_fault = 0U;
    Send("status raw\n");
    assert(strstr(output, "FAULT_LATCHED") != NULL);
    snapshot.status.fault = FRICTION_FAULT_OFFLINE;
    snapshot.status.active_fault = FRICTION_FAULT_OFFLINE;
    snapshot.status.online_mask = 6U;
    snapshot.status.feedback[0].valid = 0U;
    Send("status raw\n");
    assert(strstr(output, "F1 NO_FEEDBACK") != NULL);
    snapshot.status.feedback[0].valid = 1U;
    snapshot.status.feedback[0].last_rx_ms = clock_ms - 100U;
    Send("status raw\n");
    assert(strstr(output, "F1 FEEDBACK_STALE") != NULL);

    submit_result = FRICTION_SUBMIT_WRONG_SOURCE;
    Send("set 7 500 500 500\n");
    assert(submits == 1U && submitted.enable_mask == 7U && submitted.target_rpm[0] == 500.0f);
    assert(strstr(output, "[ERR] friction WRONG_SOURCE") != NULL && strstr(output, "FRCMD,") != NULL);
    submit_result = FRICTION_SUBMIT_ACCEPTED;
    Send("set 1 200 0 0\n");
    assert(strstr(output, "[OK] friction QUEUED") != NULL);
    Send("pid 2 2.2 0.03 0 12000\n");
    assert(tunes == 1U && fabsf(queued.motor[1].kp - 2.2f) < 1e-6f);
    assert(queued.motor[0].kp == 3.0f);
    tune_result = FRICTION_TUNE_RANGE;
    Send("ramp 99999\n");
    assert(strstr(output, "[ERR] friction RANGE") != NULL);
    Send("pid 1 2 x 0 1000\n");
    assert(strstr(output, "SYNTAX") != NULL && tunes == 2U);
    Send("off\n");
    assert(stops == 1U && strstr(output, "[OK] friction off") != NULL);
    unsigned prior_submits = submits, prior_stops = stops, prior_tunes = tunes;
    Send("log period 19\nlog period 1001\n");
    assert(strstr(output, "[ERR]") != NULL && strstr(output, "[OK]") == NULL);
    Send("log period 999999999999999999999\n");
    assert(strstr(output, "[ERR]") != NULL);
    Send("log period 50\n");
    assert(strstr(output, "log_period_ms=50") != NULL);
    Send("stream off\n");
    Advance(100U);
    assert(used == 0U);
    Send("stream on\n");
    Drain();
    busy = true;
    for (unsigned i = 0U; i < 20U; ++i) { clock_ms += 50U; DiagnosticsPoll(clock_ms); }
    assert(DiagnosticsSkippedReports() > 0U);
    busy = false;
    ClearOutput();
    Drain();
    Advance(50U);
    assert(strstr(output, "FE,") != NULL);
    ++session;
    ClearOutput();
    Drain();
    Advance(50U);
    assert(strstr(output, "FRMETA,schema=1,session=2") != NULL);
    assert(submits == prior_submits && stops == prior_stops && tunes == prior_tunes);
    Send("log 1\n");
    Advance(4999U);
    assert(used == 0U);
    Advance(1U);
    assert(strstr(output, "-------- 状态 #") != NULL && strstr(output, "FS,") == NULL);
    Send("source remote\n");
    assert(snapshot.command_source == FRICTION_SOURCE_REMOTE && strstr(output, "FRSOURCE,") != NULL);
    Send("source usb\n");
    assert(snapshot.command_source == FRICTION_SOURCE_USB);
    Send("source invalid\n");
    assert(snapshot.command_source == FRICTION_SOURCE_USB && strstr(output, "[ERR] syntax") != NULL);
    puts("PASS: real USB parser, modes, CSV units/precision, applied revision, causes, invalid data, bounded backlog, reconnect and observation-only commands");
    return 0;
}
