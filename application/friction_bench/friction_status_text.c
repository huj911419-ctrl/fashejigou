#include "../bench_features.h"
#include "friction_status_text.h"
#include "friction.h"
#include "friction_config.h"
#include "friction_remote_bench.h"
#include "friction_remote_config.h"
#include "control_platform.h"
#include "board_status.h"
#include "usb_console_port.h"
#include <math.h>

static const Friction_Control_Config_s limits = FRICTION_CONTROL_CONFIG;
static const DjiSpeedMotorConfig_s motor_config[FRICTION_MOTOR_COUNT] = FRICTION_MOTOR_CONFIGS;

static const char *WheelNames(uint8_t mask)
{
    static const char *const names[] = {
        "无", "F1", "F2", "F1+F2", "F3", "F1+F3", "F2+F3", "F1+F2+F3",
    };
    return mask < 8U ? names[mask] : "非法轮编号";
}

static const char *SourceName(Friction_Command_Source_e source)
{
    static const char *const names[] = {"未选择", "RTT调试", "VOFA调试", "USB电脑", "遥控器"};
    return (unsigned)source < sizeof(names) / sizeof(names[0]) ? names[source] : "未知来源";
}

static long Rounded(float value)
{
    return (long)TextConsoleScaled(round((double)value), 1.0);
}

static void RecoveryGuide(TextConsole_s *console, Friction_Command_Source_e source)
{
    if (source == FRICTION_SOURCE_REMOTE)
    {
        TextConsolePrint(console, "[恢复方法] 排除原因后，S1先拨停止，再选择运行组；保持运行档不会自动恢复。");
    }
    else
    {
        TextConsolePrint(console, "[恢复方法] USB控制先 source usb、off，确认停用，再每100ms循环发set。");
    }
}

static void PrintReason(TextConsole_s *console, const Friction_Telemetry_s *sample,
                        const FrictionRemoteBenchStatus_s *rc, bool fresh, bool remote_online)
{
    const Friction_Status_s *status = &sample->status;
    if (!sample->initialized)
    {
        TextConsolePrint(console, "[停止原因] 电机初始化失败；检查启用轮的CAN、ID、方向及参数配置。");
        return;
    }
    if (!fresh)
    {
        TextConsolePrint(console, "[停止原因] 控制快照过期，当前运行状态无法确认；先查控制任务与调度。");
        return;
    }
    uint32_t fault = status->active_fault;
    if (fault != 0U)
    {
        TextConsolePrint(console, "[故障] 当前存在故障，下面逐项给出原因；不要先猜PID。");
        if (fault & FRICTION_FAULT_CONFIG)
        {
            TextConsolePrint(console, "[原因] 控制配置无效：核对上限、超时、方向和电机ID。");
        }
        if (fault & FRICTION_FAULT_COMMAND)
        {
            TextConsolePrint(console, "[原因] 运行命令无效或过期：命令年龄%lu ms，上限%lu ms。",
                             (unsigned long)status->command_age_ms, (unsigned long)limits.command_timeout_ms);
        }
        if (fault & FRICTION_FAULT_OFFLINE)
        {
            for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
            {
                if ((status->requested_mask & (1U << i)) && !(status->online_mask & (1U << i)))
                {
                    TextConsolePrint(console, "[原因] F%u反馈%s；检查电机供电、CAN-H/L、ID和反馈更新。",
                                     i + 1U, status->feedback[i].valid ? "已过期" : "未收到");
                }
            }
        }
        if (fault & FRICTION_FAULT_TEMPERATURE)
        {
            TextConsolePrint(console, "[原因] 电机温度达到%u℃保护门槛；先降温并排查负载。", limits.max_temperature_c);
        }
        if (fault & FRICTION_FAULT_TIMING)
        {
            TextConsolePrint(console, "[原因] 控制调用间隔异常：本次%lu ms，允许1～%lu ms。",
                             (unsigned long)status->step_ms, (unsigned long)limits.max_step_ms);
        }
        if (fault & FRICTION_FAULT_DRIVER)
        {
            TextConsolePrint(console, "[原因] CAN输出提交失败；查看下方CAN错误，检查总线与发送队列。");
        }
        RecoveryGuide(console, sample->command_source);
        return;
    }
    if (status->fault != 0U || status->rearm_required || sample->source_requires_off)
    {
        TextConsolePrint(console, "[停止原因] 等待重新解锁%s。",
                         status->fault != 0U ? "；当前原因已消失，但历史故障仍锁定" : "");
        RecoveryGuide(console, sample->command_source);
        return;
    }
    if (sample->command_source == FRICTION_SOURCE_REMOTE)
    {
        if (!remote_online)
        {
            TextConsolePrint(console, "[停止原因] 当前选遥控控制，但没有有效遥控信号。");
            TextConsolePrint(console, "[操作] 用遥控：接好接收器，S1先拨停止；用USB：先发source usb，再off。");
            return;
        }
        if (rc->remote.needs_off)
        {
            TextConsolePrint(console, "[停止原因] 遥控尚未解锁：S1先拨停止，再选择运行组。");
            return;
        }
        if (rc->remote.group == 0U)
        {
            TextConsolePrint(console, "[停止原因] S1在停止位置，这是正常停机；S2不会单独启动电机。");
            return;
        }
    }
    if (!status->enabled_mask)
    {
        TextConsolePrint(console, "[停止原因] 没有生效的运行请求；当前控制来源：%s。", SourceName(sample->command_source));
        if (sample->command_source == FRICTION_SOURCE_USB)
        {
            TextConsolePrint(console, "[操作] 已选USB，确认停机后每100ms循环发送set；查询和日志不续期。");
        }
    }
    else
    {
        TextConsolePrint(console, "[运行] 已允许驱动：%s；实际是否转动，请看三轮反馈。", WheelNames(status->enabled_mask));
    }
}

static void PrintRemote(TextConsole_s *console, const FrictionRemoteBenchStatus_s *rc, bool online)
{
    TextConsolePrint(console, "--- 遥控档位 ---");
    if (!BENCH_FRICTION_REMOTE_ACTIVE)
    {
        TextConsolePrint(console, "[遥控] 模块已关闭；不会启动遥控接收串口。");
        return;
    }
    TextConsolePrint(console, "[遥控] %s；S1=%u，S2=%u；摇杆不控制摩擦轮。",
                     online ? "在线，数据有效" : "数据不可用，请查连接或原始报告", rc->remote.input.s1, rc->remote.input.s2);
    static const char *const groups[] = {"停止", "低速组", "高速组"};
    unsigned group = rc->remote.group;
    TextConsolePrint(console, "[档位] %s / %u档；开关选择%ld RPM，生成目标%ld RPM。",
                     group < 3U ? groups[group] : "无效组", rc->remote.gear,
                     Rounded(rc->remote.selected_rpm), Rounded(rc->remote.target_rpm));
    TextConsolePrint(console, "[提示] 档位选择不等于已运行；只有当前来源为遥控时才控制电机。");
    TextConsolePrint(console, "[解锁] %s；串口错误%lu次，接收溢出%lu次（均为累计）。",
                     rc->remote.needs_off ? "需要S1先拨停止" : "不再等待停止位置",
                     (unsigned long)rc->port.uart_errors, (unsigned long)rc->port.overflows);
}

static void PrintWheels(TextConsole_s *console, const Friction_Telemetry_s *sample, bool fresh, uint32_t now_ms)
{
    TextConsolePrint(console, "--- 三轮反馈与参数 ---");
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        if (!(BENCH_FRICTION_MASK & (1U << i)))
        {
            TextConsolePrint(console, "[F%u] 模块关闭，不要求反馈。", i + 1U);
            continue;
        }
        const Friction_Status_s *status = &sample->status;
        const Friction_Feedback_s *feedback = &status->feedback[i];
        const Friction_PID_Tuning_s *pid = &sample->tuning.motor[i];
        bool online = fresh && feedback->valid && isfinite(feedback->rpm) &&
                      now_ms - feedback->last_rx_ms <= limits.feedback_timeout_ms;
        const char *link = !feedback->valid ? "未收到反馈" : online ? "反馈在线" : "反馈或控制快照过期/无效";
        TextConsolePrint(console, "[F%u] %s；CAN%u / ID%u；%s软件驱动%s。", i + 1U, link,
                         motor_config[i].can_bus, motor_config[i].id,
                         fresh ? "" : "旧记录：",
                         (status->enabled_mask & (1U << i)) ? "已使能" : "已停用");
        if (feedback->valid && isfinite(feedback->rpm) && isfinite(status->requested_rpm[i]) &&
            isfinite(status->reference_rpm[i]))
        {
            TextConsolePrint(console, "[F%u速度] 目标%ld / 斜坡给定%ld / %s实际%ld RPM。", i + 1U,
                             Rounded(status->requested_rpm[i]), Rounded(status->reference_rpm[i]),
                             online ? "" : "旧记录", Rounded(feedback->rpm));
            TextConsolePrint(console, "[F%u反馈] %s温度%u℃；反馈年龄%lu ms，允许%lu ms。", i + 1U,
                             online ? "" : "最近记录", feedback->temperature_c,
                             (unsigned long)(now_ms - feedback->last_rx_ms), (unsigned long)limits.feedback_timeout_ms);
            if (online && !(status->enabled_mask & (1U << i)) && fabsf(feedback->rpm) > 100.0f)
            {
                TextConsolePrint(console, "[F%u提示] 已停用但轮仍转动，可能在滑行或被外力带动。", i + 1U);
            }
        }
        else
        {
            TextConsolePrint(console, "[F%u速度] 无有效速度记录；不能把缺失数据当作0 RPM。", i + 1U);
        }
        if (isfinite(sample->output[i]) && isfinite(pid->current_limit))
        {
            TextConsolePrint(console, "[F%u输出] %s指令%ld / 电流反馈原始值%d；输出上限%ld（均不是安培）。",
                             i + 1U, fresh ? "" : "旧记录", Rounded(sample->output[i]),
                             feedback->current_raw, Rounded(pid->current_limit));
        }
        else
        {
            TextConsolePrint(console, "[F%u输出] 指令或上限数值无效；不能当作0输出。", i + 1U);
        }
        if (isfinite(pid->kp) && isfinite(pid->ki) && isfinite(pid->kd) && pid->kp >= 0 && pid->ki >= 0 && pid->kd >= 0)
        {
            long kp = (long)TextConsoleScaled(round((double)pid->kp * 1000.0), 1.0);
            long ki = (long)TextConsoleScaled(round((double)pid->ki * 1000.0), 1.0);
            long kd = (long)TextConsoleScaled(round((double)pid->kd * 1000000.0), 1.0);
            TextConsolePrint(console, "[F%u PID] Kp=%ld.%03ld，Ki=%ld.%03ld，Kd=%ld.%06ld。", i + 1U,
                             kp / 1000L, kp % 1000L, ki / 1000L, ki % 1000L, kd / 1000000L, kd % 1000000L);
        }
        else
        {
            TextConsolePrint(console, "[F%u PID] 参数数值无效，请检查配置与原始报告。", i + 1U);
        }
        if (fresh && (status->enabled_mask & (1U << i)) && pid->current_limit > 0.0f &&
            fabsf(sample->output[i]) >= pid->current_limit - 1.0f)
        {
            TextConsolePrint(console, "[F%u提示] 本次输出触限；需log 2判断是否持续，再查供电、阻力和参数。", i + 1U);
        }
    }
}

static void PrintBoard(TextConsole_s *console)
{
    TextConsolePrint(console, "--- 硬件检查范围 ---");
    if (!BENCH_ENABLE_BOARD_DIAGNOSTICS)
    {
        TextConsolePrint(console, "[硬件] 板级诊断已关闭，未读取硬件状态。");
        return;
    }
    BoardStatus_s board;
    UsbConsoleStats_s usb;
    BoardStatusRead(&board);
    UsbConsolePortGetStats(&usb);
    for (unsigned bus = 0U; bus < 3U; ++bus)
    {
        bool used = false;
        for (unsigned wheel = 0U; wheel < FRICTION_MOTOR_COUNT; ++wheel)
        {
            used = used || ((BENCH_FRICTION_MASK & (1U << wheel)) && motor_config[wheel].can_bus == bus + 1U);
        }
        if (!used)
        {
            continue;
        }
        const BoardCanStatus_s *can = &board.can[bus];
        const char *state = !can->readable ? "状态不可读取" : !can->started ? "未启动" :
                            can->bus_off ? "总线关闭故障" : can->error_passive ? "错误被动" :
                            can->warning ? "错误预警" : "已启动";
        TextConsolePrint(console, "[CAN%u] %s；发送错误计数%lu，接收错误计数%lu。", bus + 1U, state,
                         (unsigned long)can->tx_error_count, (unsigned long)can->rx_error_count);
    }
    TextConsolePrint(console, "[通信说明] CAN启动不等于电机已连接；连接是否有效看上面的各轮反馈。");
    TextConsolePrint(console, "[USB] %s；累计接收丢失%lu字节，溢出%lu次，接口错误%lu次。",
                     usb.configured ? "已连接" : "尚未完成连接", (unsigned long)usb.rx_dropped,
                     (unsigned long)usb.rx_overflows, (unsigned long)usb.io_errors);
    TextConsolePrint(console, "[电源] 仅知道控制引脚状态；没有实测电压，不能据此判断电源是否正常。");
    TextConsolePrint(console, "[范围] 本固件检查摩擦轮与遥控；云台、IMU、裁判及供弹未监测。");
}

static bool ReadStatus(Friction_Telemetry_s *sample, FrictionRemoteBenchStatus_s *rc,
                       uint32_t *now_ms, bool *remote_online)
{
    *rc = (FrictionRemoteBenchStatus_s){0};
    FrictionGetTelemetry(sample);
    if (BENCH_FRICTION_REMOTE_ACTIVE)
    {
        FrictionRemoteBenchGetStatus(rc);
    }
    *now_ms = ControlPlatformNowMs(); /* Re-sample after coherent snapshot copies. */
    *remote_online = rc->initialized && rc->port.started && *now_ms - rc->sample_ms <= limits.max_step_ms &&
                     rc->remote.online && rc->remote.input.valid &&
                     *now_ms - rc->remote.input.received_ms <= FRICTION_REMOTE_TIMEOUT_MS;
    return sample->sample_id != 0U && *now_ms - sample->sample_ms <= limits.max_step_ms;
}

void FrictionStatusTextReport(TextConsole_s *console, uint32_t now_ms)
{
    Friction_Telemetry_s sample;
    FrictionRemoteBenchStatus_s rc;
    bool remote_online;
    bool fresh = ReadStatus(&sample, &rc, &now_ms, &remote_online);
    if (!BENCH_FRICTION_ACTIVE)
    {
        TextConsolePrint(console, "摩擦轮模块已关闭。");
        return;
    }
    TextConsolePrint(console, "控制方式：%s", SourceName(sample.command_source));
    PrintReason(console, &sample, &rc, fresh, remote_online);
    bool coasting = false;
    for (unsigned i = 0U; i < FRICTION_MOTOR_COUNT; ++i)
    {
        const Friction_Feedback_s *feedback = &sample.status.feedback[i];
        bool online = fresh && feedback->valid && isfinite(feedback->rpm) &&
                      now_ms - feedback->last_rx_ms <= limits.feedback_timeout_ms;
        if (!(BENCH_FRICTION_MASK & (1U << i)))
        {
            TextConsolePrint(console, "F%u：模块关闭", i + 1U);
        }
        else if (!online || !isfinite(sample.status.requested_rpm[i]))
        {
            TextConsolePrint(console, "F%u：%s；当前转速未知", i + 1U,
                             !feedback->valid ? "未收到反馈" : "数据过期或无效");
        }
        else
        {
            TextConsolePrint(console, "F%u：在线 | 目标 %ld | 实际 %ld RPM", i + 1U,
                             Rounded(sample.status.requested_rpm[i]), Rounded(feedback->rpm));
            coasting = coasting || (!(sample.status.enabled_mask & (1U << i)) && fabsf(feedback->rpm) > 100.0f);
        }
    }
    if (coasting)
    {
        TextConsolePrint(console, "提示：已停用的轮仍有转速，可能在滑行或被外力带动。");
    }
}

void FrictionStatusTextDetailReport(TextConsole_s *console, uint32_t now_ms)
{
    Friction_Telemetry_s sample;
    FrictionRemoteBenchStatus_s rc;
    bool remote_online;
    bool fresh = ReadStatus(&sample, &rc, &now_ms, &remote_online);
    TextConsolePrint(console, "--- 当前结论与处理方法 ---");
    TextConsolePrint(console, "[控制来源] %s；%s请求运行：%s；软件使能：%s。", SourceName(sample.command_source),
                     fresh ? "" : "旧记录：",
                     WheelNames(sample.status.requested_mask), WheelNames(sample.status.enabled_mask));
    if (!BENCH_FRICTION_ACTIVE)
    {
        TextConsolePrint(console, "[结论] 摩擦轮模块已关闭，不会驱动电机。");
        return;
    }
    TextConsolePrint(console, "[控制数据] %s，年龄%lu ms；当前故障%s，历史锁定%s。",
                     fresh ? "新鲜" : "过期或尚未更新", (unsigned long)(now_ms - sample.sample_ms),
                     sample.status.active_fault ? "存在" : "未记录到", sample.status.fault ? "存在" : "无");
    PrintReason(console, &sample, &rc, fresh, remote_online);
    static const char *const phases[] = {"停止", "升速中", "转速稳定", "掉速恢复中", "故障", "台架测试"};
    unsigned phase = sample.readiness.phase;
    TextConsolePrint(console, "[就绪] %s；稳定轮：%s；%s。", phase < 6U ? phases[phase] : "未知",
                     WheelNames(sample.readiness.ready_mask),
                     !BENCH_SPEED_READY_ACTIVE ? "判定模块关闭" : !fresh ? "旧记录，不能确认当前稳定" :
                     sample.readiness.all_ready ? "启用轮均稳定，仅表示转速条件" : "尚未满足全部启用轮稳定条件");
    PrintRemote(console, &rc, remote_online);
    TextConsolePrint(console, "[速度含义] 目标=收到的转速要求；斜坡给定=逐步接近目标；实际=电机反馈，单位RPM。");
    PrintWheels(console, &sample, fresh, now_ms);
    TextConsolePrint(console, "[斜坡] %ld RPM/s；控制上限%ld RPM；温度保护%u℃。",
                     Rounded(sample.tuning.ramp_rpm_s), Rounded(limits.max_rpm), limits.max_temperature_c);
    TextConsolePrint(console, "[参数保存] 上电使用固件默认值；在线调参仅存RAM，尚未接入Flash保存。");
    PrintBoard(console);
    TextConsolePrint(console, "[更多] status raw看原始字段；log 2给AI分析；VOFA实际转速通道14/24/34。");
}
