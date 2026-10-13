#include "loop_stats.h"

static void Increment(uint32_t *value)
{
    if (*value != UINT32_MAX)
    {
        ++*value;
    }
}

void LoopStatsObserve(LoopStats_s *stats, uint32_t now_ms, uint32_t maximum_interval_ms)
{
    Increment(&stats->samples);
    if (stats->started)
    {
        uint32_t interval = now_ms - stats->last_stamp_ms;
        stats->last_interval_ms = interval;
        if (interval > stats->max_interval_ms)
        {
            stats->max_interval_ms = interval;
        }
        if (interval == 0U)
        {
            Increment(&stats->zero_interval_count);
        }
        if (maximum_interval_ms != 0U && interval > maximum_interval_ms)
        {
            Increment(&stats->late_count);
        }
    }
    stats->started = true;
    stats->last_stamp_ms = now_ms;
}
