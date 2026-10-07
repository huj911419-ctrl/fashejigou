#ifndef RUNTIME_EVENTS_H
#define RUNTIME_EVENTS_H

#include "event_history.h"

typedef enum
{
    RUNTIME_YAW_PORT,
    RUNTIME_RC_ONLINE,
    RUNTIME_YAW_STAGE,
    RUNTIME_YAW_FAULT,
    RUNTIME_YAW_IO_HISTORY,
    RUNTIME_YAW_ENABLE,
    RUNTIME_DM_ON,
    RUNTIME_FRICTION_FAULT,
    RUNTIME_FRICTION_ENABLE,
    RUNTIME_FRICTION_ONLINE,
    RUNTIME_FRICTION_PHASE,
    RUNTIME_IMU_STAGE,
    RUNTIME_IMU_FAULT,
    RUNTIME_ATTITUDE_STAGE,
    RUNTIME_ATTITUDE_FAULT,
    RUNTIME_ATTITUDE_CONTROL_VALID,
    RUNTIME_YAW_INHIBIT,
    RUNTIME_EVENT_KEY_COUNT
} RuntimeEventKey_e;

/** Application-owned history for the independent benches. Task context only.
 * Init once before producers start. Observe/Get serialize through the platform
 * short-copy lock; never print, send CAN, clear faults or enable actuators.
 * Data lives in RAM and is lost on reset. Keys and values describe software
 * observations; event order is recording order, not proof of physical causality. */
void RuntimeEventsInit(void);
void RuntimeEventsObserve(RuntimeEventKey_e key, uint32_t value);
void RuntimeEventsGet(EventHistorySnapshot_s *snapshot);
const char *RuntimeEventsName(uint16_t key);

#endif
