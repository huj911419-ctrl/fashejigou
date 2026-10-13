#ifndef EVENT_HISTORY_H
#define EVENT_HISTORY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EVENT_HISTORY_CAPACITY 32U
#define EVENT_HISTORY_KEYS 32U

typedef struct
{
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t previous;
    uint32_t value;
    uint16_t key;
    bool initial; /* First observation of this key, not a transition from zero. */
} EventHistoryEntry_s;

typedef struct
{
    EventHistoryEntry_s entries[EVENT_HISTORY_CAPACITY];
    uint32_t previous[EVENT_HISTORY_KEYS];
    uint32_t seen;
    uint32_t sequence;
    uint32_t overwritten;
    size_t head;
    size_t count;
} EventHistory_s;

typedef struct
{
    EventHistoryEntry_s entries[EVENT_HISTORY_CAPACITY]; /* Oldest -> newest. */
    uint32_t overwritten;
    size_t count;
} EventHistorySnapshot_s;

/** 只记录首次观测和状态变化；固定 RAM，满后覆盖最旧记录。
 * Zero initialization is valid. Caller owns/serializes all operations.
 * Keys 0..31, values application-defined, timestamp is a wrapping ms clock.
 * Sequence wraps at 2^32; use snapshot order, not numeric sort across wrap.
 * overwritten saturates. No allocation, printing, storage IO or clock calls. */
bool EventHistoryObserve(EventHistory_s *history, uint16_t key, uint32_t value, uint32_t now_ms);
/** Coherent copy when caller holds its own lock; does not consume/reset events. */
void EventHistorySnapshot(const EventHistory_s *history, EventHistorySnapshot_s *snapshot);

#endif
