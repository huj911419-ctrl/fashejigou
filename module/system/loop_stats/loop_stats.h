#ifndef LOOP_STATS_H
#define LOOP_STATS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t samples;
    uint32_t last_stamp_ms;
    uint32_t last_interval_ms;
    uint32_t max_interval_ms;
    uint32_t late_count;
    uint32_t zero_interval_count;
    bool started;
} LoopStats_s;

/** 单实例、单任务持有；零初始化即可使用，无硬件/RTOS 依赖。
 * Observe at one consistent point each iteration, using wrapping uint32 ms.
 * maximum_interval_ms=0 disables late counting. First sample has no interval.
 * Counters saturate at UINT32_MAX; intervals assume less than 2^32 ms apart.
 * Measures invocation spacing, NOT execution time, CPU load or stack usage. */
void LoopStatsObserve(LoopStats_s *stats, uint32_t now_ms, uint32_t maximum_interval_ms);

#endif
