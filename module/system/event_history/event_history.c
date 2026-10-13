#include "event_history.h"

bool EventHistoryObserve(EventHistory_s *history, uint16_t key, uint32_t value, uint32_t now_ms)
{
    if (key >= EVENT_HISTORY_KEYS)
    {
        return false;
    }
    uint32_t mask = UINT32_C(1) << key;
    bool initial = (history->seen & mask) == 0U;
    if (!initial && history->previous[key] == value)
    {
        return false;
    }
    history->entries[history->head] = (EventHistoryEntry_s)
    {
        .sequence = ++history->sequence,
        .timestamp_ms = now_ms,
        .key = key,
        .initial = initial,
        .previous = initial ? 0U : history->previous[key],
        .value = value
    };
    history->previous[key] = value;
    history->seen |= mask;
    history->head = (history->head + 1U) % EVENT_HISTORY_CAPACITY;
    if (history->count < EVENT_HISTORY_CAPACITY)
    {
        ++history->count;
    }
    else if (history->overwritten != UINT32_MAX)
    {
        ++history->overwritten;
    }
    return true;
}

void EventHistorySnapshot(const EventHistory_s *history, EventHistorySnapshot_s *snapshot)
{
    snapshot->count = history->count;
    snapshot->overwritten = history->overwritten;
    size_t start = (history->head + EVENT_HISTORY_CAPACITY - history->count) % EVENT_HISTORY_CAPACITY;
    for (size_t i = 0U; i < history->count; ++i)
    {
        snapshot->entries[i] = history->entries[(start + i) % EVENT_HISTORY_CAPACITY];
    }
}
