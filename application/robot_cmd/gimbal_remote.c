#include "gimbal_remote.h"
#include <math.h>
#include <string.h>

bool GimbalRemoteInit(GimbalRemote_s *remote, const GimbalRemoteConfig_s *config)
{
    if (remote == NULL)
    {
        return false;
    }
    memset(remote, 0, sizeof(*remote));
    if (config == NULL)
    {
        return false;
    }
    remote->config = *config;
    DbusDecoderInit(&remote->decoder);
    remote->configured = config->timeout_ms > 0U && config->timeout_ms < 0x80000000U &&
                         isfinite(config->max_rate_rad_s) && config->max_rate_rad_s > 0.0f &&
                         isfinite(config->deadband) && config->deadband >= 0.0f &&
                         config->deadband < 1.0f && config->yaw_channel < DBUS_CHANNEL_COUNT &&
                         config->arm_s1_value >= 1U && config->arm_s1_value <= 3U &&
                         (config->yaw_direction == 1 || config->yaw_direction == -1);
    return remote->configured;
}

void GimbalRemoteReceiveByte(GimbalRemote_s *remote, uint8_t byte, uint32_t received_ms)
{
    if (remote == NULL || !remote->configured)
    {
        return;
    }
    bool complete = DbusDecoderPush(&remote->decoder, byte, received_ms);
    if (!remote->decoder.input.valid ||
        (complete && remote->decoder.input.s1 != remote->config.arm_s1_value))
    {
        /* Preserve transient invalid/OFF even if another frame follows it. */
        remote->stop_pending = true;
    }
}

void GimbalRemoteInvalidate(GimbalRemote_s *remote)
{
    if (remote != NULL)
    {
        DbusDecoderInvalidate(&remote->decoder);
        remote->stop_pending = true;
    }
}

GimbalCommand_s GimbalRemoteTakeCommand(GimbalRemote_s *remote, uint32_t now_ms)
{
    GimbalCommand_s command = {0};
    if (remote == NULL || !remote->configured)
    {
        return command;
    }
    const DbusInput_s *input = &remote->decoder.input;
    float stick = input->channel[remote->config.yaw_channel];
    float magnitude = fabsf(stick);
    float normalized = magnitude <= remote->config.deadband ? 0.0f :
                       copysignf((magnitude - remote->config.deadband) /
                                 (1.0f - remote->config.deadband), stick);
    bool online = input->valid && now_ms - input->received_ms <= remote->config.timeout_ms;
    bool arm = input->s1 == remote->config.arm_s1_value;
    command = (GimbalCommand_s)
    {
        .received_ms = input->received_ms,
        .sequence = input->sequence,
        .rate_rad_s = normalized * remote->config.yaw_direction * remote->config.max_rate_rad_s,
        .valid = online,
        .arm = arm,
        .stop_requested = remote->stop_pending
    };
    remote->stop_pending = false;
    remote->status = (GimbalRemoteStatus_s)
    {
        .input = *input,
        .stick = stick,
        .online = online,
        .arm_switch = arm
    };
    return command;
}
