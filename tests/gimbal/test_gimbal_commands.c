#include "gimbal_remote.h"
#include "gimbal_control.h"
#include "gimbal_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void RemoteFrame(GimbalRemote_s *remote, uint32_t now, uint8_t s1, uint16_t channel)
{
    uint64_t bits = (uint64_t)channel | ((uint64_t)1024U << 11) |
                    ((uint64_t)1024U << 22) | ((uint64_t)1024U << 33) |
                    ((uint64_t)1U << 44) | ((uint64_t)s1 << 46);
    for (unsigned i = 0U; i < DBUS_FRAME_SIZE; ++i)
    {
        GimbalRemoteReceiveByte(remote, i < 6U ? (uint8_t)(bits >> (8U * i)) : 0U, now);
    }
}

static void TestRemoteMapping(void)
{
    GimbalRemoteConfig_s config = GIMBAL_REMOTE_CONFIG;
    config.max_rate_rad_s = 2.0f;
    config.yaw_direction = -1;
    GimbalRemote_s remote;
    GimbalRemote_s other;
    assert(GimbalRemoteInit(&remote, &config));
    config.yaw_direction = 1;
    assert(GimbalRemoteInit(&other, &config));
    assert(!GimbalRemoteTakeCommand(&remote, 0U).valid);
    RemoteFrame(&remote, 0U, 2U, 1024U); /* Initial unframed bytes ignored. */
    RemoteFrame(&remote, 10U, 2U, 1024U);
    GimbalCommand_s command = GimbalRemoteTakeCommand(&remote, 10U);
    assert(command.valid && !command.arm && command.stop_requested);
    assert(command.rate_rad_s == 0.0f && command.received_ms == 10U);
    RemoteFrame(&remote, 20U, 1U, 1040U); /* Inside the input deadband. */
    command = GimbalRemoteTakeCommand(&remote, 20U);
    assert(command.valid && command.arm && !command.stop_requested && command.rate_rad_s == 0.0f);
    RemoteFrame(&remote, 30U, 1U, 1684U);
    command = GimbalRemoteTakeCommand(&remote, 30U);
    assert(command.rate_rad_s == -2.0f && remote.status.stick == 1.0f);
    assert(remote.config.yaw_direction == -1); /* Copied config, independent instances. */
    assert(!GimbalRemoteTakeCommand(&other, 30U).valid);
    uint32_t sequence = command.sequence;
    command = GimbalRemoteTakeCommand(&remote, 130U);
    assert(command.valid && command.received_ms == 30U && command.sequence == sequence);
    command = GimbalRemoteTakeCommand(&remote, 131U);
    assert(!command.valid && command.received_ms == 30U);
    RemoteFrame(&remote, 140U, 2U, 1024U);
    RemoteFrame(&remote, 150U, 1U, 1024U);
    command = GimbalRemoteTakeCommand(&remote, 150U);
    assert(command.arm && command.stop_requested); /* OFF cannot disappear in a batch. */
    assert(!GimbalRemoteTakeCommand(&remote, 150U).stop_requested);
    RemoteFrame(&remote, 160U, 0U, 1024U); /* Invalid switch followed by a valid frame. */
    RemoteFrame(&remote, 170U, 1U, 1024U);
    command = GimbalRemoteTakeCommand(&remote, 170U);
    assert(command.valid && command.stop_requested);
    GimbalRemoteInvalidate(&remote);
    command = GimbalRemoteTakeCommand(&remote, 172U);
    assert(!command.valid && command.stop_requested);
    RemoteFrame(&remote, 180U, 1U, 364U);
    assert(GimbalRemoteTakeCommand(&remote, 180U).rate_rad_s == 2.0f);
    assert(!GimbalRemoteTakeCommand(&remote, 179U).valid); /* Future RX timestamp. */

    assert(GimbalRemoteInit(&remote, &config));
    RemoteFrame(&remote, UINT32_MAX - 20U, 2U, 1024U);
    RemoteFrame(&remote, UINT32_MAX - 10U, 1U, 1024U);
    assert(GimbalRemoteTakeCommand(&remote, 0U).valid);
    assert(!GimbalRemoteTakeCommand(&remote, 100U).valid);
    config.deadband = 1.0f;
    assert(!GimbalRemoteInit(&remote, &config));
    assert(!GimbalRemoteTakeCommand(&remote, 0U).valid);
    config.deadband = 0.04f;
    config.yaw_channel = DBUS_CHANNEL_COUNT;
    assert(!GimbalRemoteInit(&remote, &config));
    config.yaw_channel = 0U;
    config.yaw_direction = 0;
    assert(!GimbalRemoteInit(&remote, &config));
    assert(!GimbalRemoteInit(&remote, NULL));
    assert(!GimbalRemoteInit(NULL, &config));
    assert(!GimbalRemoteTakeCommand(NULL, 0U).valid);
}

/* Only the external CAN device is faked. All command, limit, authorization and
 * DM handshake logic is the production implementation, with no state seeding. */
typedef struct
{
    uint32_t now;
    DmCanFrame_s reply;
    bool pending;
    bool on;
} Drive_s;

static bool Send(void *context, const DmCanFrame_s *frame)
{
    Drive_s *drive = context;
    drive->reply = (DmCanFrame_s){.id = 0x10U, .length = 8U, .received_ms = drive->now};
    drive->pending = true;
    if (frame->id == 0x7FFU)
    {
        memcpy(drive->reply.data, frame->data, frame->length);
        drive->reply.data[4] = frame->data[3] == 7U ? 0x10U : 3U;
    }
    else
    {
        if (frame->length == 8U)
        {
            drive->on = frame->data[7] == 0xFCU;
        }
        drive->reply.data[0] = drive->on ? 0x10U : 0U;
        drive->reply.data[1] = 0x80U;
        drive->reply.data[3] = 0x80U;
        drive->reply.data[6] = 25U;
        drive->reply.data[7] = 25U;
    }
    return true;
}

static void Cycle(GimbalControl_s *control, Drive_s *drive, const GimbalCommand_s *command)
{
    if (drive->pending)
    {
        GimbalControlReceiveCan(control, &drive->reply, drive->now);
        drive->pending = false;
    }
    GimbalControlStep(control, command, drive->now, true);
    drive->now += 2U;
}

static void FreshCycles(GimbalControl_s *control, Drive_s *drive, GimbalCommand_s *command,
                        unsigned count)
{
    for (unsigned i = 0U; i < count; ++i)
    {
        command->received_ms = drive->now;
        ++command->sequence;
        Cycle(control, drive, command);
    }
}

static void Rearm(GimbalControl_s *control, Drive_s *drive, GimbalCommand_s *command)
{
    *command = (GimbalCommand_s){.valid = true};
    FreshCycles(control, drive, command, 20U);
    command->arm = true;
    FreshCycles(control, drive, command, 20U);
    assert(control->status.enabled && drive->on);
}

static void TestSourceIndependentControl(uint32_t start)
{
    GimbalControlConfig_s config =
    {
        .motor = GIMBAL_DM_CONFIG,
        .yaw = GIMBAL_YAW_RATE_CONFIG,
        .command_timeout_ms = 100U
    };
    config.motor.power_up_ms = 0U;
    GimbalControl_s control;
    Drive_s drive = {.now = start};
    assert(GimbalControlInit(&control, &config, Send, &drive, start));
    GimbalCommand_s command = {.valid = true, .arm = true};
    FreshCycles(&control, &drive, &command, 150U);
    assert(control.status.motor_state == DM_VELOCITY_READY && !control.status.enabled);
    Rearm(&control, &drive, &command);
    command.rate_rad_s = 0.4f;
    FreshCycles(&control, &drive, &command, 20U);
    assert(control.status.target_rad_s == 0.4f && control.status.reference_rad_s > 0.0f);
    for (unsigned i = 0U; i < 51U; ++i)
    {
        Cycle(&control, &drive, &command); /* Re-reading the same request is not a renewal. */
    }
    assert(!control.status.command_online && !control.status.enabled);
    assert(control.status.inhibit & YAW_INHIBIT_COMMAND);
    FreshCycles(&control, &drive, &command, 20U);
    assert(!control.status.enabled); /* Fresh RUN alone cannot restore authorization. */
    Rearm(&control, &drive, &command);
    command.stop_requested = true;
    FreshCycles(&control, &drive, &command, 1U);
    assert(!control.status.enabled);
    command.stop_requested = false;
    FreshCycles(&control, &drive, &command, 20U);
    assert(!control.status.enabled);
    Rearm(&control, &drive, &command);
    command.received_ms = drive.now + 1U;
    Cycle(&control, &drive, &command);
    assert(!control.status.command_online && !control.status.enabled);
    const float bad_rates[] = {NAN, INFINITY, -INFINITY, 1.01f, -1.01f};
    for (unsigned i = 0U; i < sizeof(bad_rates) / sizeof(bad_rates[0]); ++i)
    {
        Rearm(&control, &drive, &command);
        command.rate_rad_s = bad_rates[i];
        FreshCycles(&control, &drive, &command, 1U);
        assert(!control.status.enabled && control.status.reference_rad_s == 0.0f);
        assert(control.status.inhibit & YAW_INHIBIT_RATE);
    }
    Rearm(&control, &drive, &command);
    Cycle(&control, &drive, NULL);
    assert(!control.status.command_online && !control.status.enabled);
}

int main(void)
{
    TestRemoteMapping();
    TestSourceIndependentControl(0U);
    TestSourceIndependentControl(UINT32_MAX - 100U);
    puts("gimbal commands: source mapping, retained stops, original lease and protocol-independent control passed");
    return 0;
}
