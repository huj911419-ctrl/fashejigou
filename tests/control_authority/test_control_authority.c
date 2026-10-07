#include "control_authority.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static const ControlAuthorityConfig_s config =
{
    .allowed_sources = CONTROL_SOURCE_ALL,
    .timeout_ms = {100U, 50U, 30U}
};

static ControlAuthority_s Create(ControlSource_t source)
{
    ControlAuthority_s authority;
    assert(ControlAuthorityInit(&authority, &config));
    assert(ControlAuthoritySelect(&authority, source));
    return authority;
}

static ControlAuthorityInput_s Input(const ControlAuthority_s *authority,
                                     uint32_t received_ms, bool arm, bool neutral)
{
    ControlAuthorityInput_s input =
    {
        .source = authority->status.selected,
        .epoch = authority->status.epoch,
        .received_ms = received_ms,
        .sequence = 7U, /* Sequence deliberately repeats; it is not a lease. */
        .valid = true,
        .arm = arm,
        .neutral = neutral
    };
    return input;
}

static void Feed(ControlAuthority_s *authority, uint32_t received_ms, bool arm, bool neutral)
{
    ControlAuthorityInput_s input = Input(authority, received_ms, arm, neutral);
    assert(ControlAuthoritySubmit(authority, &input));
    ControlAuthorityStep(authority, received_ms);
}

static void Arm(ControlAuthority_s *authority, uint32_t received_ms)
{
    Feed(authority, received_ms, false, true);
    assert(!authority->status.authorized);
    Feed(authority, received_ms + 1U, true, true);
    assert(authority->status.authorized);
    assert(authority->status.reason == 0U);
}

static void TestConfiguration(void)
{
    ControlAuthority_s authority;
    assert(!ControlAuthorityInit(NULL, &config));
    assert(!ControlAuthorityInit(&authority, NULL));
    ControlAuthorityStep(&authority, 0U);
    assert(authority.status.reason == CONTROL_AUTHORITY_CONFIG);
    assert(!ControlAuthoritySelect(&authority, CONTROL_SOURCE_REMOTE));
    ControlAuthorityConfig_s invalid = config;
    invalid.allowed_sources |= CONTROL_SOURCE_MASK(CONTROL_SOURCE_NONE);
    assert(!ControlAuthorityInit(&authority, &invalid));
    invalid = config;
    invalid.timeout_ms[0] = 0U;
    assert(!ControlAuthorityInit(&authority, &invalid));
    invalid.timeout_ms[0] = UINT32_C(0x80000000);
    assert(!ControlAuthorityInit(&authority, &invalid));

    ControlAuthorityConfig_s remote_only =
    {
        .allowed_sources = CONTROL_SOURCE_MASK(CONTROL_SOURCE_REMOTE),
        .timeout_ms = {100U, 0U, 0U}
    };
    assert(ControlAuthorityInit(&authority, &remote_only));
    assert(authority.status.reason == CONTROL_AUTHORITY_NO_OWNER);
    assert(!ControlAuthoritySelect(&authority, CONTROL_SOURCE_DEBUG));
    assert(!ControlAuthoritySelect(&authority, (ControlSource_t)-1));
    assert(!ControlAuthoritySelect(&authority, (ControlSource_t)32));
    assert(ControlAuthoritySelect(&authority, CONTROL_SOURCE_REMOTE));
    ControlAuthorityStep(&authority, 10U);
    assert(authority.status.reason == CONTROL_AUTHORITY_NO_INPUT);
    assert(!ControlAuthoritySubmit(&authority, NULL));
}

static void TestStartupAndNeutralEdge(void)
{
    ControlAuthority_s authority = Create(CONTROL_SOURCE_REMOTE);
    Feed(&authority, 0U, true, true);
    assert(authority.status.online);
    assert(!authority.status.authorized);
    assert(authority.status.reason == CONTROL_AUTHORITY_REARM_REQUIRED);
    Feed(&authority, 10U, false, false);
    assert(authority.status.reason == (CONTROL_AUTHORITY_STOP | CONTROL_AUTHORITY_SWITCH_OFF));
    Feed(&authority, 20U, true, false);
    assert(authority.status.reason == CONTROL_AUTHORITY_NOT_NEUTRAL);
    Feed(&authority, 30U, true, true);
    assert(!authority.status.authorized);
    Arm(&authority, 40U);
    Feed(&authority, 50U, true, false);
    assert(authority.status.authorized); /* Movement after neutral start is allowed. */
    Feed(&authority, 60U, false, false);
    assert(!authority.status.authorized);
}

static void TestDuplicatesAndTimeout(void)
{
    ControlAuthority_s authority = Create(CONTROL_SOURCE_REMOTE);
    Arm(&authority, 10U);
    uint32_t epoch = authority.status.epoch;
    assert(ControlAuthoritySelect(&authority, CONTROL_SOURCE_REMOTE));
    assert(authority.status.epoch == epoch);
    assert(authority.status.authorized);
    ControlAuthorityInput_s duplicate = Input(&authority, 11U, true, true);
    duplicate.sequence = UINT32_MAX;
    assert(!ControlAuthoritySubmit(&authority, &duplicate));
    assert(authority.status.input_sequence == 7U);
    ControlAuthorityStep(&authority, 110U);
    assert(authority.status.authorized);
    ControlAuthorityStep(&authority, 111U);
    assert(!authority.status.online && !authority.status.authorized);
    assert(authority.status.reason == CONTROL_AUTHORITY_TIMEOUT);
    Feed(&authority, 112U, true, true);
    assert(!authority.status.authorized); /* A new held-ON packet cannot restore. */
    Arm(&authority, 120U);

    /* A cached OFF cannot become new merely because a fault was cleared. */
    Feed(&authority, 130U, false, true);
    ControlAuthorityStop(&authority);
    ControlAuthorityStep(&authority, 131U);
    ControlAuthorityStep(&authority, 132U);
    Feed(&authority, 133U, true, true);
    assert(!authority.status.authorized);
    Arm(&authority, 140U);
}

static void TestWrongOwnerAndHandoff(void)
{
    ControlAuthority_s authority = Create(CONTROL_SOURCE_REMOTE);
    Arm(&authority, 10U);
    ControlAuthorityInput_s old = Input(&authority, 12U, false, true);
    ControlAuthorityInput_s wrong = old;
    wrong.source = CONTROL_SOURCE_AUTO;
    wrong.stop_requested = true;
    assert(!ControlAuthoritySubmit(&authority, &wrong));
    ControlAuthorityStep(&authority, 12U);
    assert(authority.status.authorized);
    uint32_t previous_epoch = authority.status.epoch;
    assert(ControlAuthoritySelect(&authority, CONTROL_SOURCE_DEBUG));
    assert(authority.status.epoch == previous_epoch + 1U);
    assert(!authority.status.authorized && !authority.status.online);
    old.source = CONTROL_SOURCE_DEBUG; /* Even relabeling source cannot cross epoch. */
    assert(!ControlAuthoritySubmit(&authority, &old));
    Feed(&authority, 20U, true, true);
    assert(!authority.status.authorized);
    Arm(&authority, 30U);
    ControlAuthorityStep(&authority, 81U);
    assert(authority.status.reason == CONTROL_AUTHORITY_TIMEOUT);
    assert(authority.status.selected == CONTROL_SOURCE_DEBUG); /* No fallback. */
    wrong = Input(&authority, 82U, false, true);
    wrong.source = CONTROL_SOURCE_REMOTE;
    assert(!ControlAuthoritySubmit(&authority, &wrong));
    assert(ControlAuthoritySelect(&authority, CONTROL_SOURCE_NONE));
    ControlAuthorityStep(&authority, 90U);
    assert(authority.status.reason == CONTROL_AUTHORITY_NO_OWNER);
}

static void TestStickyStopAndBatch(void)
{
    ControlAuthority_s authority = Create(CONTROL_SOURCE_REMOTE);
    Arm(&authority, 10U);
    ControlAuthorityInput_s off = Input(&authority, 20U, false, true);
    /* No explicit stop flag: a real accepted OFF must itself survive batching. */
    assert(ControlAuthoritySubmit(&authority, &off));
    ControlAuthorityInput_s on = Input(&authority, 21U, true, true);
    assert(ControlAuthoritySubmit(&authority, &on));
    ControlAuthorityStep(&authority, 21U);
    assert(!authority.status.authorized);
    assert((authority.status.reason & CONTROL_AUTHORITY_STOP) != 0U);
    Feed(&authority, 22U, true, true);
    assert(!authority.status.authorized);
    Arm(&authority, 30U);

    /* A duplicate STOP remains effective even though its payload is rejected. */
    on = Input(&authority, 31U, true, true);
    on.stop_requested = true;
    assert(!ControlAuthoritySubmit(&authority, &on));
    assert(!authority.status.authorized);
    Feed(&authority, 32U, true, true);
    assert(!authority.status.authorized);

    /* STOP with a newly received OFF may establish OFF for the next ON. */
    off = Input(&authority, 40U, false, true);
    off.stop_requested = true;
    assert(ControlAuthoritySubmit(&authority, &off));
    ControlAuthorityStep(&authority, 40U);
    Feed(&authority, 41U, true, true);
    assert(authority.status.authorized);

    ControlAuthorityStop(&authority);
    assert(ControlAuthoritySelect(&authority, CONTROL_SOURCE_AUTO));
    on = Input(&authority, 50U, true, true);
    assert(ControlAuthoritySubmit(&authority, &on));
    ControlAuthorityStep(&authority, 50U);
    assert((authority.status.reason & CONTROL_AUTHORITY_STOP) != 0U);
    assert(!authority.status.authorized);
}

static void TestInvalidFutureAndOrdering(void)
{
    ControlAuthority_s authority = Create(CONTROL_SOURCE_REMOTE);
    Arm(&authority, 10U);
    ControlAuthorityInput_s invalid = Input(&authority, 20U, false, true);
    invalid.valid = false;
    assert(ControlAuthoritySubmit(&authority, &invalid));
    ControlAuthorityStep(&authority, 20U);
    assert(authority.status.reason == (CONTROL_AUTHORITY_STOP | CONTROL_AUTHORITY_INVALID_INPUT));
    Feed(&authority, 21U, true, true);
    assert(!authority.status.authorized);

    ControlAuthorityInput_s future = Input(&authority, 50U, false, true);
    assert(ControlAuthoritySubmit(&authority, &future));
    ControlAuthorityStep(&authority, 49U);
    assert(authority.status.reason == (CONTROL_AUTHORITY_STOP | CONTROL_AUTHORITY_FUTURE_INPUT));
    ControlAuthorityStep(&authority, 50U);
    assert(authority.status.reason == CONTROL_AUTHORITY_NO_INPUT);
    Feed(&authority, 51U, true, true);
    assert(!authority.status.authorized); /* Future OFF is consumed, not recycled. */
    Arm(&authority, 60U);
    ControlAuthorityInput_s old = Input(&authority, 59U, false, true);
    assert(!ControlAuthoritySubmit(&authority, &old));
    old.received_ms = 61U + UINT32_C(0x80000000);
    assert(!ControlAuthoritySubmit(&authority, &old));
    ControlAuthorityStep(&authority, 62U);
    assert(authority.status.authorized);
    old = Input(&authority, 61U, false, true); /* Same time cannot mutate arm. */
    assert(!ControlAuthoritySubmit(&authority, &old));
    ControlAuthorityStep(&authority, 63U);
    assert(authority.status.authorized);

    future = Input(&authority, 10000U, true, true);
    assert(ControlAuthoritySubmit(&authority, &future));
    ControlAuthorityStep(&authority, 64U);
    assert(authority.status.reason == CONTROL_AUTHORITY_FUTURE_INPUT);
    Arm(&authority, 65U); /* Real-time OFF/ON can recover before time 10000. */
}

static void TestInvalidThenRunBatch(void)
{
    ControlAuthority_s authority = Create(CONTROL_SOURCE_REMOTE);
    Arm(&authority, 10U);
    ControlAuthorityInput_s invalid = Input(&authority, 20U, true, true);
    invalid.valid = false;
    assert(ControlAuthoritySubmit(&authority, &invalid));
    assert(!authority.status.authorized);
    ControlAuthorityInput_s run = Input(&authority, 21U, true, true);
    assert(ControlAuthoritySubmit(&authority, &run));
    ControlAuthorityStep(&authority, 21U);
    assert(authority.status.online);
    assert(!authority.status.authorized);
    assert((authority.status.reason & CONTROL_AUTHORITY_STOP) != 0U);
    Feed(&authority, 22U, true, true);
    assert(!authority.status.authorized);
    Arm(&authority, 30U);
}

static void TestWrapAndInstanceIsolation(void)
{
    ControlAuthority_s first = Create(CONTROL_SOURCE_REMOTE);
    ControlAuthority_s second = Create(CONTROL_SOURCE_AUTO);
    Arm(&first, UINT32_MAX - 1U);
    Feed(&first, 0U, true, false);
    assert(first.status.authorized);
    ControlAuthorityStep(&first, 99U);
    assert(first.status.authorized);
    ControlAuthorityStep(&first, 100U);
    assert(first.status.reason == CONTROL_AUTHORITY_TIMEOUT);
    Arm(&second, 100U);
    ControlAuthorityStop(&first);
    assert(second.status.authorized);
    assert(second.status.selected == CONTROL_SOURCE_AUTO);
    first.status.epoch = UINT32_MAX;
    assert(ControlAuthoritySelect(&first, CONTROL_SOURCE_DEBUG));
    assert(first.status.epoch == 0U);
    Arm(&first, 110U);
    assert(first.status.authorized);
    assert(second.status.authorized);
}

int main(void)
{
    TestConfiguration();
    TestStartupAndNeutralEdge();
    TestDuplicatesAndTimeout();
    TestWrongOwnerAndHandoff();
    TestStickyStopAndBatch();
    TestInvalidFutureAndOrdering();
    TestInvalidThenRunBatch();
    TestWrapAndInstanceIsolation();
    puts("control_authority: all tests passed");
    return 0;
}
