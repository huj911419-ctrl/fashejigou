#include "friction_remote.h"
#include <math.h>
#include <string.h>

bool FrictionRemoteInit(FrictionRemote_s *remote, const FrictionRemoteConfig_s *config)
{
    if (remote == NULL || config == NULL || config->timeout_ms == 0U ||
        config->timeout_ms >= 0x80000000U || !isfinite(config->max_rpm) || config->max_rpm <= 0.0f ||
        config->channel >= DBUS_CHANNEL_COUNT ||
        config->stop_s1 < 1U || config->stop_s1 > 3U ||
        config->low_s1 < 1U || config->low_s1 > 3U ||
        config->high_s1 < 1U || config->high_s1 > 3U ||
        config->stop_s1 == config->low_s1 || config->stop_s1 == config->high_s1 ||
        config->low_s1 == config->high_s1 || (config->direction != 1 && config->direction != -1))
    {
        return false;
    }
    for (unsigned gear = 0U; gear < FRICTION_REMOTE_GEARS_PER_GROUP; ++gear)
    {
        if (config->gear_s2[gear] < 1U || config->gear_s2[gear] > 3U)
        {
            return false;
        }
        for (unsigned other = 0U; other < gear; ++other)
        {
            if (config->gear_s2[gear] == config->gear_s2[other])
            {
                return false;
            }
        }
        for (unsigned group = 0U; group < FRICTION_REMOTE_GROUP_COUNT; ++group)
        {
            float rpm = config->gear_rpm[group][gear];
            if (!isfinite(rpm) || rpm <= 0.0f || rpm > config->max_rpm)
            {
                return false;
            }
        }
    }
    memset(remote, 0, sizeof(*remote));
    remote->config = *config;
    remote->needs_off = true;
    remote->status.needs_off = true;
    return true;
}

void FrictionRemoteDisarm(FrictionRemote_s *remote)
{
    remote->needs_off = true;
    remote->stop_pending = true;
}

void FrictionRemoteInvalidate(FrictionRemote_s *remote)
{
    DbusDecoderInvalidate(&remote->decoder);
    FrictionRemoteDisarm(remote);
}

static uint8_t SelectedGroup(const FrictionRemote_s *remote)
{
    if (!remote->decoder.input.valid)
    {
        return 0U;
    }
    return remote->decoder.input.s1 == remote->config.low_s1 ? 1U :
           remote->decoder.input.s1 == remote->config.high_s1 ? 2U : 0U;
}

static uint8_t SelectedGear(const FrictionRemote_s *remote, uint8_t group)
{
    if (group == 0U)
    {
        return 0U;
    }
    for (unsigned gear = 0U; gear < FRICTION_REMOTE_GEARS_PER_GROUP; ++gear)
    {
        if (remote->decoder.input.s2 == remote->config.gear_s2[gear])
        {
            return (uint8_t)((group - 1U) * FRICTION_REMOTE_GEARS_PER_GROUP + gear + 1U);
        }
    }
    return 0U;
}

void FrictionRemotePush(FrictionRemote_s *remote, uint8_t byte, uint32_t received_ms)
{
    uint32_t previous_ms = remote->decoder.input.received_ms;
    bool previous_valid = remote->decoder.input.valid;
    bool accepted = DbusDecoderPush(&remote->decoder, byte, received_ms);
    if (!remote->decoder.input.valid)
    {
        FrictionRemoteDisarm(remote);
        return;
    }
    if (!accepted)
    {
        return;
    }
    if (!previous_valid || received_ms - previous_ms > remote->config.timeout_ms)
    {
        FrictionRemoteDisarm(remote);
    }
    if (remote->decoder.input.s1 == remote->config.stop_s1)
    {
        remote->stop_pending = true;
        remote->needs_off = false;
    }
}

void FrictionRemoteTake(FrictionRemote_s *remote, uint32_t now_ms, FrictionRemoteRequest_s *request)
{
    const DbusInput_s *input = &remote->decoder.input;
    bool online = input->valid && now_ms - input->received_ms <= remote->config.timeout_ms;
    if (!online)
    {
        remote->needs_off = true;
        remote->stop_pending = remote->stop_pending || remote->was_online;
    }
    bool new_frame = input->sequence != remote->taken_sequence;
    float stick = input->channel[remote->config.channel] * remote->config.direction;
    uint8_t group = SelectedGroup(remote);
    uint8_t gear = SelectedGear(remote, group);
    float rpm = gear != 0U ?
        remote->config.gear_rpm[group - 1U][(gear - 1U) % FRICTION_REMOTE_GEARS_PER_GROUP] : 0.0f;
    bool switches_on = gear != 0U;
    bool run = online && !remote->needs_off && switches_on && rpm > 0.0f && !remote->stop_pending;
    *request = (FrictionRemoteRequest_s){
        .received_ms = input->received_ms,
        .sequence = input->sequence,
        .target_rpm = run ? rpm : 0.0f,
        .new_frame = new_frame,
        .run = run,
        .stop_requested = remote->stop_pending || (online && new_frame && !run),
        .qualified_off = online && new_frame && input->s1 == remote->config.stop_s1,
    };
    remote->status = (FrictionRemoteStatus_s){
        .input = *input, .stick = stick, .target_rpm = run ? rpm : 0.0f,
        .selected_rpm = rpm, .group = group, .gear = gear,
        .online = online, .switches_on = switches_on, .needs_off = remote->needs_off,
    };
    remote->taken_sequence = input->sequence;
    remote->stop_pending = false;
    remote->was_online = online;
}
