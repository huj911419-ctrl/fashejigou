#include "event_history.h"
#include "loop_stats.h"
#include "runtime_events.h"
#include "control_platform.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static bool locked;
static uint32_t now_ms;

uint32_t ControlPlatformEnter(void)
{
    assert(!locked);
    locked = true;
    return 0U;
}

void ControlPlatformExit(uint32_t previous)
{
    assert(locked && previous == 0U);
    locked = false;
}

uint32_t ControlPlatformNowMs(void)
{
    assert(locked); /* Timestamp belongs to the serialized record operation. */
    return now_ms;
}

static void TestTiming(void)
{
    LoopStats_s stats = {0};
    LoopStatsObserve(&stats, UINT32_MAX - 1U, 5U);
    assert(stats.samples == 1U && stats.max_interval_ms == 0U);
    LoopStatsObserve(&stats, 0U, 5U);
    assert(stats.last_interval_ms == 2U && stats.late_count == 0U);
    LoopStatsObserve(&stats, 5U, 5U);
    assert(stats.max_interval_ms == 5U && stats.late_count == 0U);
    LoopStatsObserve(&stats, 11U, 5U);
    LoopStatsObserve(&stats, 11U, 5U);
    assert(stats.late_count == 1U && stats.zero_interval_count == 1U);
    assert(stats.samples == 5U && stats.max_interval_ms == 6U);
    stats.samples = UINT32_MAX;
    stats.late_count = UINT32_MAX;
    LoopStatsObserve(&stats, 100U, 5U);
    assert(stats.samples == UINT32_MAX && stats.late_count == UINT32_MAX);
    LoopStatsObserve(&stats, 1000U, 0U);
    assert(stats.late_count == UINT32_MAX && stats.last_interval_ms == 900U);
}

static void TestHistory(void)
{
    EventHistory_s history = {0};
    EventHistorySnapshot_s snapshot;
    EventHistorySnapshot(&history, &snapshot);
    assert(snapshot.count == 0U && snapshot.overwritten == 0U);
    assert(!EventHistoryObserve(&history, EVENT_HISTORY_KEYS, 1U, 0U));
    assert(EventHistoryObserve(&history, 31U, 0U, UINT32_MAX));
    for (unsigned i = 0U; i < 100000U; ++i)
    {
        assert(!EventHistoryObserve(&history, 31U, 0U, i));
    }
    EventHistorySnapshot(&history, &snapshot);
    assert(snapshot.count == 1U && snapshot.entries[0].initial);
    assert(snapshot.entries[0].timestamp_ms == UINT32_MAX && snapshot.entries[0].key == 31U);
    assert(EventHistoryObserve(&history, 31U, 7U, 0U));
    assert(EventHistoryObserve(&history, 0U, 7U, 0U));
    EventHistorySnapshot(&history, &snapshot);
    assert(!snapshot.entries[1].initial && snapshot.entries[1].previous == 0U);
    assert(snapshot.entries[2].initial); /* Equal values from different keys stay distinct. */
    for (unsigned i = 0U; i < 40U; ++i)
    {
        assert(EventHistoryObserve(&history, 0U, i, i));
    }
    EventHistorySnapshot(&history, &snapshot);
    assert(snapshot.count == 32U && snapshot.overwritten == 11U);
    assert(snapshot.entries[0].value == 8U && snapshot.entries[31].value == 39U);
    assert(!EventHistoryObserve(&history, 31U, 7U, 100U)); /* Remember overwritten key state. */
    history.sequence = UINT32_MAX;
    history.overwritten = UINT32_MAX;
    assert(EventHistoryObserve(&history, 31U, 8U, 1U));
    EventHistorySnapshot(&history, &snapshot);
    assert(snapshot.entries[31].sequence == 0U && snapshot.overwritten == UINT32_MAX);
    EventHistory_s other = {0};
    EventHistorySnapshot(&other, &snapshot);
    assert(snapshot.count == 0U); /* Independent instances do not share storage. */
}

static void TestApplication(void)
{
    RuntimeEventsInit();
    EventHistorySnapshot_s snapshot;
    now_ms = 123U;
    RuntimeEventsObserve(RUNTIME_YAW_ENABLE, 0U);
    RuntimeEventsObserve(RUNTIME_YAW_ENABLE, 0U);
    now_ms = 125U;
    RuntimeEventsObserve(RUNTIME_YAW_ENABLE, 1U);
    RuntimeEventsObserve((RuntimeEventKey_e)-1, 1U);
    RuntimeEventsGet(&snapshot);
    assert(snapshot.count == 2U && snapshot.entries[1].timestamp_ms == 125U);
    assert(!locked && snapshot.entries[1].previous == 0U && snapshot.entries[1].value == 1U);
    assert(strcmp(RuntimeEventsName(RUNTIME_YAW_ENABLE), "yaw.enable") == 0);
    assert(strcmp(RuntimeEventsName(60000U), "unknown") == 0);
    RuntimeEventsGet(&snapshot);
    assert(snapshot.count == 2U); /* Query is non-consuming. */
    RuntimeEventsInit();
    RuntimeEventsGet(&snapshot);
    assert(snapshot.count == 0U);
}

int main(void)
{
    TestTiming();
    TestHistory();
    TestApplication();
    puts("PASS: runtime interval boundaries, timestamp/counter wrap, event changes, overwrite, instances and locked snapshots");
    return 0;
}
