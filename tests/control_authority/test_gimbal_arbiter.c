#include "gimbal_arbiter.h"
#include "yaw_rate.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static GimbalCommand_s Send(GimbalArbiter_s *arbiter, ControlSource_t source,
                            uint32_t time, bool arm, float rate)
{
    GimbalCommand_s input = {.received_ms = time, .sequence = time, .rate_rad_s = rate,
                             .valid = true, .arm = arm};
    assert(GimbalArbiterSubmit(arbiter, source, arbiter->authority.status.epoch, &input));
    return GimbalArbiterStep(arbiter, time);
}

static void Apply(YawRate_s *yaw, const GimbalCommand_s *command, uint32_t now)
{
    if (command->stop_requested)
    {
        YawRateDisarm(yaw);
    }
    YawRateStep(yaw, now, command->valid, true, command->arm, command->rate_rad_s);
}

int main(void)
{
    const ControlAuthorityConfig_s config = {CONTROL_SOURCE_ALL, {100, 80, 50}};
    GimbalArbiter_s arbiter;
    assert(GimbalArbiterInit(&arbiter, &config, 1.0f));
    assert(GimbalArbiterSelect(&arbiter, CONTROL_SOURCE_REMOTE));
    GimbalCommand_s output = Send(&arbiter, CONTROL_SOURCE_REMOTE, 1, true, 0);
    assert(output.valid && output.arm && output.stop_requested);
    assert(!arbiter.authority.status.authorized); /* Not a synthesized OFF. */

    output = Send(&arbiter, CONTROL_SOURCE_REMOTE, 2, false, 0);
    assert(output.valid && !output.arm);
    output = Send(&arbiter, CONTROL_SOURCE_REMOTE, 3, true, 0);
    assert(arbiter.authority.status.authorized && !output.stop_requested);
    output = Send(&arbiter, CONTROL_SOURCE_REMOTE, 4, true, 0.5f);
    assert(output.rate_rad_s == 0.5f && output.received_ms == 4U);
    for (uint32_t time = 5; time < 100; ++time)
    {
        output = GimbalArbiterStep(&arbiter, time);
        assert(output.received_ms == 4U && arbiter.authority.status.authorized);
    }
    output = GimbalArbiterStep(&arbiter, 104);
    assert(!output.valid && output.arm && output.stop_requested);
    assert(!arbiter.authority.status.authorized);
    output = Send(&arbiter, CONTROL_SOURCE_REMOTE, 105, true, 0);
    assert(output.stop_requested); /* Link recovery cannot restart. */

    uint32_t remote_epoch = arbiter.authority.status.epoch;
    assert(GimbalArbiterSelect(&arbiter, CONTROL_SOURCE_DEBUG));
    output = GimbalArbiterStep(&arbiter, 106);
    assert(!output.valid && output.rate_rad_s == 0.0f);
    GimbalCommand_s old = {.received_ms = 107, .sequence = 99, .rate_rad_s = 0.8f, .arm = true, .valid = true};
    assert(!GimbalArbiterSubmit(&arbiter, CONTROL_SOURCE_REMOTE, remote_epoch, &old));
    assert(!GimbalArbiterSubmit(&arbiter, CONTROL_SOURCE_DEBUG, remote_epoch, &old));
    output = Send(&arbiter, CONTROL_SOURCE_DEBUG, 108, true, 0);
    assert(output.stop_requested);
    (void)Send(&arbiter, CONTROL_SOURCE_DEBUG, 109, false, 0);
    output = Send(&arbiter, CONTROL_SOURCE_DEBUG, 110, true, 0);
    assert(!output.stop_requested && arbiter.authority.status.authorized);
    output = Send(&arbiter, CONTROL_SOURCE_DEBUG, 111, true, NAN);
    assert(!output.valid && !arbiter.authority.status.authorized);
    (void)Send(&arbiter, CONTROL_SOURCE_DEBUG, 112, false, 0);
    output = Send(&arbiter, CONTROL_SOURCE_DEBUG, 113, true, 0.1f);
    assert(output.arm && output.stop_requested); /* Nonneutral start requires another OFF. */
    output = Send(&arbiter, CONTROL_SOURCE_DEBUG, 114, true, 0);
    assert(output.stop_requested);

    assert(GimbalArbiterSelect(&arbiter, CONTROL_SOURCE_AUTO));
    (void)Send(&arbiter, CONTROL_SOURCE_AUTO, 115, false, 0);
    output = Send(&arbiter, CONTROL_SOURCE_AUTO, 116, true, 0);
    assert(!output.stop_requested && arbiter.authority.status.authorized);
    GimbalArbiterStop(&arbiter);
    output = GimbalArbiterStep(&arbiter, 117);
    assert(output.stop_requested && !arbiter.authority.status.authorized);
    assert(GimbalArbiterSelect(&arbiter, CONTROL_SOURCE_NONE));
    output = GimbalArbiterStep(&arbiter, 118);
    assert(!output.valid && arbiter.authority.status.selected == CONTROL_SOURCE_NONE);

    /* Feed the real yaw gate: ownership and actuator rearm must both agree. */
    YawRate_s yaw;
    const YawRateConfig_s yaw_config = {1.0f, 2.0f, 10};
    assert(YawRateInit(&yaw, &yaw_config));
    assert(GimbalArbiterSelect(&arbiter, CONTROL_SOURCE_REMOTE));
    output = Send(&arbiter, CONTROL_SOURCE_REMOTE, 120, true, 0);
    Apply(&yaw, &output, 120);
    assert(!yaw.enabled);
    output = Send(&arbiter, CONTROL_SOURCE_REMOTE, 122, false, 0);
    Apply(&yaw, &output, 122);
    output = Send(&arbiter, CONTROL_SOURCE_REMOTE, 124, true, 0);
    Apply(&yaw, &output, 124);
    assert(yaw.enabled);
    assert(GimbalArbiterSelect(&arbiter, CONTROL_SOURCE_AUTO));
    output = Send(&arbiter, CONTROL_SOURCE_AUTO, 126, true, 0);
    Apply(&yaw, &output, 126);
    assert(!yaw.enabled); /* No valid OFF manufactured at the handoff. */
    output = Send(&arbiter, CONTROL_SOURCE_AUTO, 128, false, 0);
    Apply(&yaw, &output, 128);
    output = Send(&arbiter, CONTROL_SOURCE_AUTO, 130, true, 0);
    Apply(&yaw, &output, 130);
    assert(yaw.enabled);

    GimbalCommand_s bad = {.received_ms = 132, .rate_rad_s = NAN, .valid = true, .arm = true};
    assert(GimbalArbiterSubmit(&arbiter, CONTROL_SOURCE_AUTO, arbiter.authority.status.epoch, &bad));
    bad.received_ms = 134;
    bad.rate_rad_s = 0.5f;
    assert(GimbalArbiterSubmit(&arbiter, CONTROL_SOURCE_AUTO, arbiter.authority.status.epoch, &bad));
    output = GimbalArbiterStep(&arbiter, 134);
    Apply(&yaw, &output, 134);
    assert(output.stop_requested && !arbiter.authority.status.authorized && !yaw.enabled);
    output = Send(&arbiter, CONTROL_SOURCE_AUTO, 136, true, 0);
    assert(output.stop_requested); /* Later valid input cannot hide a batched invalid one. */

    const ControlAuthorityConfig_s remote_only = {CONTROL_SOURCE_MASK(CONTROL_SOURCE_REMOTE), {100, 0, 0}};
    assert(GimbalArbiterInit(&arbiter, &remote_only, 1));
    assert(!GimbalArbiterSelect(&arbiter, CONTROL_SOURCE_AUTO));
    assert(!GimbalArbiterInit(&arbiter, &remote_only, NAN));
    assert(!GimbalArbiterStep(&arbiter, 200).valid);
    puts("PASS: gimbal ownership payload isolation, original leases, handoff, future sources and real yaw rearm");
    return 0;
}
