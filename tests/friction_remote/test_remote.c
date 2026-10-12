#include "friction_remote.h"
#include "friction_remote_config.h"
#include "dbus_fixture.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const FrictionRemoteConfig_s config = FRICTION_REMOTE_CONFIG;
static void Frame(FrictionRemote_s *remote, uint8_t s1, uint8_t s2, uint16_t vertical, uint32_t time)
{
    uint8_t frame[18];
    DbusFixture(frame, s1, s2, vertical);
    for (unsigned i = 0U; i < 18U; ++i)
    {
        FrictionRemotePush(remote, frame[i], time);
    }
}
static FrictionRemoteRequest_s Take(FrictionRemote_s *remote, uint32_t time)
{
    FrictionRemoteRequest_s request;
    FrictionRemoteTake(remote, time, &request);
    return request;
}
static void Init(FrictionRemote_s *remote)
{
    assert(FrictionRemoteInit(remote, &config));
    FrictionRemotePush(remote, 0U, 0U);
}
int main(void)
{
    FrictionRemote_s remote;
    /* Every held running gear at startup must wait for a physical STOP. */
    for (unsigned group = 0U; group < 2U; ++group)
    {
        for (unsigned gear = 0U; gear < 3U; ++gear)
        {
            Init(&remote);
            Frame(&remote, group == 0U ? 3U : 2U, config.gear_s2[gear], 1024U, 10U);
            assert(!Take(&remote, 10U).run && remote.status.needs_off);
        }
    }
    Init(&remote);
    uint32_t time = 10U;
    /* All three S2 positions stop; no neutral-stick requirement. */
    for (unsigned gear = 0U; gear < 3U; ++gear)
    {
        Frame(&remote, 1U, config.gear_s2[gear], 1684U, time);
        assert(Take(&remote, time).qualified_off && !remote.status.needs_off);
        assert(remote.status.gear == 0U && remote.status.target_rpm == 0.0f);
        time += 10U;
    }
    const float expected[2][3] = {{1000, 2000, 3000}, {5000, 6000, 7000}};
    const uint16_t sticks[3] = {364U, 1024U, 1684U};
    for (unsigned group = 0U; group < 2U; ++group)
    {
        for (unsigned gear = 0U; gear < 3U; ++gear)
        {
            for (unsigned stick = 0U; stick < 3U; ++stick)
            {
                Frame(&remote, group == 0U ? 3U : 2U, config.gear_s2[gear], sticks[stick], time);
                FrictionRemoteRequest_s request = Take(&remote, time);
                assert(request.run && request.new_frame && request.received_ms == time);
                assert(request.target_rpm == expected[group][gear]);
                assert(remote.status.group == group + 1U && remote.status.gear == group * 3U + gear + 1U);
                assert(remote.status.selected_rpm == request.target_rpm);
                time += 10U;
            }
        }
    }
    uint32_t received = time - 10U;
    assert(!Take(&remote, received + 1U).new_frame); /* No invented keepalive. */
    assert(Take(&remote, received + 100U).run);
    assert(Take(&remote, received + 101U).stop_requested && remote.status.needs_off);
    time = received + 110U;
    Frame(&remote, 2U, 1U, 1024U, time);
    assert(!Take(&remote, time).run); /* Reconnect held high cannot recover. */
    Frame(&remote, 1U, 1U, 1684U, time + 10U);
    assert(Take(&remote, time + 10U).qualified_off);
    Frame(&remote, 2U, 1U, 1024U, time + 20U);
    assert(Take(&remote, time + 20U).run);
    Frame(&remote, 1U, 2U, 364U, time + 30U);
    Frame(&remote, 2U, 1U, 1024U, time + 40U);
    FrictionRemoteRequest_s request = Take(&remote, time + 40U);
    assert(request.stop_requested && !request.run); /* STOP survives one input batch. */
    Frame(&remote, 2U, 1U, 1024U, time + 50U);
    assert(Take(&remote, time + 50U).run);
    FrictionRemoteInvalidate(&remote);
    Frame(&remote, 2U, 1U, 1024U, time + 60U);
    assert(!Take(&remote, time + 60U).run);
    Frame(&remote, 1U, 3U, 1684U, time + 70U);
    Take(&remote, time + 70U);
    Frame(&remote, 2U, 3U, 1024U, time + 80U);
    assert(Take(&remote, time + 80U).run);
    FrictionRemoteDisarm(&remote);
    Frame(&remote, 2U, 3U, 1024U, time + 90U);
    assert(!Take(&remote, time + 90U).run);
    Frame(&remote, 1U, 2U, 1024U, time + 100U);
    assert(!Take(&remote, time + 201U).qualified_off); /* Stale STOP cannot arm. */
    Init(&remote);
    Frame(&remote, 1U, 2U, 1684U, UINT32_MAX - 20U);
    Take(&remote, UINT32_MAX - 20U);
    Frame(&remote, 2U, 3U, 1024U, UINT32_MAX - 10U);
    assert(Take(&remote, 5U).target_rpm == 6000.0f);
    Frame(&remote, 0U, 1U, 1024U, 20U);
    Frame(&remote, 2U, 1U, 1024U, 30U);
    assert(!Take(&remote, 30U).run && remote.status.needs_off);
    FrictionRemoteConfig_s bad = config;
    bad.max_rpm = NAN;
    assert(!FrictionRemoteInit(&remote, &bad));
    bad = config;
    bad.channel = 4U;
    assert(!FrictionRemoteInit(&remote, &bad));
    bad = config;
    bad.low_s1 = bad.stop_s1;
    assert(!FrictionRemoteInit(&remote, &bad));
    bad = config;
    bad.gear_s2[2] = bad.gear_s2[0];
    assert(!FrictionRemoteInit(&remote, &bad));
    bad = config;
    bad.gear_rpm[1][2] = 7001.0f;
    assert(!FrictionRemoteInit(&remote, &bad));
    bad.gear_rpm[1][2] = NAN;
    assert(!FrictionRemoteInit(&remote, &bad));
    bad.gear_rpm[1][2] = 0.0f;
    assert(!FrictionRemoteInit(&remote, &bad));
    bad = config;
    bad.direction = -1;
    assert(FrictionRemoteInit(&remote, &bad));
    FrictionRemotePush(&remote, 0U, 0U);
    Frame(&remote, 1U, 2U, 1684U, 10U);
    Take(&remote, 10U);
    Frame(&remote, 2U, 3U, 364U, 20U);
    assert(Take(&remote, 20U).target_rpm == 6000.0f);
    puts("PASS: nine switch combinations, six fixed gears independent of sticks, startup/loss/error rearm, batched STOP, stale input, wrap and config validation");
    return 0;
}
