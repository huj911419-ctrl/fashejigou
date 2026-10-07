#include "gimbal.h"
#include "gimbal_config.h"
#include "gimbal_port.h"
#include "control_platform.h"
#include "runtime_events.h"
#include "gimbal_arbiter.h"
#include "../bench_features.h"
#include <stddef.h>
#include <string.h>

enum
{
    GIMBAL_CAN_RX_BUDGET = 32U,
    GIMBAL_RC_RX_BUDGET = 128U
};

static const GimbalControlConfig_s config =
{
    .motor = GIMBAL_DM_CONFIG,
    .yaw = GIMBAL_YAW_RATE_CONFIG,
    .command_timeout_ms = GIMBAL_REMOTE_TIMEOUT_MS
};
static const GimbalRemoteConfig_s remote_config = GIMBAL_REMOTE_CONFIG;
static GimbalControl_s control;
static GimbalRemote_s remote;
static GimbalArbiter_s arbiter;
static GimbalStatus_s published;
static bool port_ready;

bool GimbalInit(uint32_t now_ms)
{
    memset(&published, 0, sizeof(published));
    bool valid = !BENCH_ENABLE_YAW || GimbalControlInit(&control, &config, GimbalPortSend, NULL, now_ms);
    bool input_valid = !BENCH_ENABLE_REMOTE || GimbalRemoteInit(&remote, &remote_config);
    const ControlAuthorityConfig_s authority_config =
    {
        .allowed_sources = BENCH_ENABLE_REMOTE ? CONTROL_SOURCE_MASK(CONTROL_SOURCE_REMOTE) : 0U,
        .timeout_ms = {GIMBAL_REMOTE_TIMEOUT_MS, GIMBAL_REMOTE_TIMEOUT_MS, GIMBAL_REMOTE_TIMEOUT_MS}
    };
    bool authority_valid = true;
    if (BENCH_ENABLE_COMMAND_AUTHORITY)
    {
        authority_valid = GimbalArbiterInit(&arbiter, &authority_config, GIMBAL_MAX_RATE_RAD_S) &&
            GimbalArbiterSelect(&arbiter, BENCH_ENABLE_REMOTE ? CONTROL_SOURCE_REMOTE : CONTROL_SOURCE_NONE);
    }
    port_ready = valid && input_valid && authority_valid &&
                 GimbalPortInitModules(BENCH_ENABLE_YAW, BENCH_ENABLE_REMOTE);
    return port_ready;
}

static void HandleTransportFaults(void)
{
    uint32_t faults = GimbalPortTakeFaults();
    if (faults != 0U)
    {
        if (BENCH_ENABLE_COMMAND_AUTHORITY)
        {
            GimbalArbiterStop(&arbiter);
        }
        if (BENCH_ENABLE_REMOTE)
        {
            GimbalRemoteInvalidate(&remote);
        }
        if (BENCH_ENABLE_YAW)
        {
            GimbalControlTransportFault(&control, faults);
        }
        else
        {
            control.status.transport_faults |= faults;
        }
    }
}

void GimbalTask(void)
{
    /* 每次调用只处理一轮；外层 YawTask 负责每 2 ms 调用本函数。 */
    HandleTransportFaults();
    /* 1. 取出 CAN 中断收到的数据，更新电机反馈和握手状态。 */
    DmCanFrame_s frame;
    for (unsigned budget = 0U; BENCH_ENABLE_YAW && budget < GIMBAL_CAN_RX_BUDGET && GimbalPortPopCan(&frame); ++budget)
    {
        GimbalControlReceiveCan(&control, &frame, ControlPlatformNowMs());
    }
    /* 2. 取出遥控器字节，拼成可用的通道值和开关状态。 */
    GimbalRcByte_s byte;
    for (unsigned budget = 0U; BENCH_ENABLE_REMOTE && budget < GIMBAL_RC_RX_BUDGET && GimbalPortPopRc(&byte); ++budget)
    {
        GimbalRemoteReceiveByte(&remote, byte.byte, byte.received_ms);
    }
    /* Catch loss occurring during the drain before making an output decision. */
    HandleTransportFaults();
    /* 3. 输入层生成有单位、有时效的请求，云台层检查并执行。 */
    uint32_t now_ms = ControlPlatformNowMs();
    GimbalCommand_s command = {0};
    if (BENCH_ENABLE_REMOTE)
    {
        command = GimbalRemoteTakeCommand(&remote, now_ms);
        if (BENCH_ENABLE_COMMAND_AUTHORITY)
        {
            /* Remote decoding and arbitration share this task. Future queued
             * sources must attach their activation epoch at the producer. */
            GimbalArbiterSubmit(&arbiter, CONTROL_SOURCE_REMOTE, arbiter.authority.status.epoch, &command);
        }
    }
    if (BENCH_ENABLE_COMMAND_AUTHORITY)
    {
        command = GimbalArbiterStep(&arbiter, now_ms);
    }
    if (BENCH_ENABLE_YAW)
    {
        GimbalControlStep(&control, &command, now_ms, port_ready);
    }
    else
    {
        /* Preserve an updating snapshot for a remote-only bench. */
        control.status.sample_ms = now_ms;
        control.status.port_ready = port_ready;
        LoopStatsObserve(&control.status.timing, now_ms, config.yaw.max_step_ms);
    }
    /* 4. 发布本轮状态副本，供 USB 等诊断读取。 */
    uint32_t lock = ControlPlatformEnter();
    published.axis = control.status;
    published.remote = remote.status;
    if (BENCH_ENABLE_COMMAND_AUTHORITY)
    {
        published.authority = arbiter.authority.status;
    }
    ControlPlatformExit(lock);
    /* 记录状态变化，持续相同的状态不会刷满历史。 */
    if (BENCH_ENABLE_YAW)
    {
        RuntimeEventsObserve(RUNTIME_YAW_PORT, control.status.port_ready);
    }
    if (BENCH_ENABLE_REMOTE)
    {
        RuntimeEventsObserve(RUNTIME_RC_ONLINE, remote.status.online);
    }
    if (BENCH_ENABLE_YAW)
    {
        RuntimeEventsObserve(RUNTIME_YAW_STAGE, control.status.motor_state);
        RuntimeEventsObserve(RUNTIME_YAW_FAULT, control.status.motor_fault);
        RuntimeEventsObserve(RUNTIME_YAW_IO_HISTORY, control.status.transport_faults);
        RuntimeEventsObserve(RUNTIME_YAW_ENABLE, control.status.enabled);
        RuntimeEventsObserve(RUNTIME_DM_ON, control.status.motor_enabled);
        RuntimeEventsObserve(RUNTIME_YAW_INHIBIT, control.status.inhibit);
    }
}

void GimbalGetStatus(GimbalStatus_s *status)
{
    if (status != NULL)
    {
        uint32_t lock = ControlPlatformEnter();
        *status = published;
        ControlPlatformExit(lock);
    }
}
