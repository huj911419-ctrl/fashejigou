#include "runtime_events.h"
#include "../bench_features.h"
#include "control_platform.h"
#include <string.h>

_Static_assert(RUNTIME_EVENT_KEY_COUNT <= EVENT_HISTORY_KEYS, "Runtime event keys exceed history capacity");

static EventHistory_s history;
static const char *const names[RUNTIME_EVENT_KEY_COUNT] =
{
    "yaw.port", "rc.online", "yaw.stage", "yaw.fault", "yaw.io_history",
    "yaw.enable", "dm.on", "friction.fault", "friction.enable_mask",
    "friction.online_mask", "friction.phase", "imu.stage", "imu.fault",
    "attitude.stage", "attitude.fault", "attitude.control_valid", "yaw.inhibit"
};

void RuntimeEventsInit(void)
{
    memset(&history, 0, sizeof(history));
}

void RuntimeEventsObserve(RuntimeEventKey_e key, uint32_t value)
{
    if (!BENCH_ENABLE_EVENT_HISTORY || (unsigned)key >= RUNTIME_EVENT_KEY_COUNT)
    {
        return;
    }
    uint32_t lock = ControlPlatformEnter();
    (void)EventHistoryObserve(&history, (uint16_t)key, value, ControlPlatformNowMs());
    ControlPlatformExit(lock);
}

void RuntimeEventsGet(EventHistorySnapshot_s *snapshot)
{
    uint32_t lock = ControlPlatformEnter();
    EventHistorySnapshot(&history, snapshot);
    ControlPlatformExit(lock);
}

const char *RuntimeEventsName(uint16_t key)
{
    return key < RUNTIME_EVENT_KEY_COUNT ? names[key] : "unknown";
}
