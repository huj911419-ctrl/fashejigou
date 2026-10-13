#ifndef FRICTION_MONITOR_H
#define FRICTION_MONITOR_H

#include "friction_control.h"
#include "speed_ready.h"

typedef enum
{
    FRICTION_PHASE_STOPPED = 0,
    FRICTION_PHASE_SPINUP = 1,
    FRICTION_PHASE_READY = 2,
    FRICTION_PHASE_RECOVERING = 3,
    FRICTION_PHASE_FAULT = 4,
    FRICTION_PHASE_BENCH = 5
} Friction_Phase_e;

/** Readiness only; this is NOT complete permission to feed or launch a projectile. */
typedef struct
{
    Friction_Phase_e phase;
    uint8_t ready_mask;
    uint8_t all_ready;
} Friction_Readiness_s;

typedef struct
{
    SpeedReady_s wheel[FRICTION_MOTOR_COUNT];
    float previous_targets[FRICTION_MOTOR_COUNT];
    Friction_Readiness_s status;
    uint8_t previously_all_ready;
    uint8_t configured;
} Friction_Monitor_s;

/** Single control-task owner. No HAL/RTOS dependency; config speed units are rpm. */
uint8_t FrictionMonitorInit(Friction_Monitor_s *monitor, const SpeedReady_Config_s *config);
/** Reset on tuning changes; status remains invalid until fresh dwell completes. */
void FrictionMonitorReset(Friction_Monitor_s *monitor);
/** Call AFTER final control/fault decisions. Only all three positive working
 * targets may produce all_ready. Partial/zero/reverse commands are bench mode.
 * Input online_mask must reflect feedback age at the supplied time. */
void FrictionMonitorUpdate(Friction_Monitor_s *monitor,
                           const Friction_Status_s *control_status,
                           uint32_t now_ms);

#endif
