#include "friction_monitor.h"

#include <math.h>
#include <string.h>

uint8_t FrictionMonitorInit(Friction_Monitor_s *monitor, const SpeedReady_Config_s *config)
{
    memset(monitor, 0, sizeof(*monitor));
    monitor->configured = 1;
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        if (!SpeedReadyInit(&monitor->wheel[i], config))
        {
            monitor->configured = 0;
        }
    }
    if (!monitor->configured)
    {
        monitor->status.phase = FRICTION_PHASE_FAULT;
    }
    return monitor->configured;
}

void FrictionMonitorReset(Friction_Monitor_s *monitor)
{
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        SpeedReadyReset(&monitor->wheel[i]);
    }
    memset(&monitor->status, 0, sizeof(monitor->status));
    monitor->previously_all_ready = 0;
}

void FrictionMonitorUpdate(Friction_Monitor_s *monitor,
                           const Friction_Status_s *control_status,
                           uint32_t now_ms)
{
    if (!monitor->configured || control_status->fault)
    {
        FrictionMonitorReset(monitor);
        monitor->status.phase = FRICTION_PHASE_FAULT;
        return;
    }
    if (!control_status->enabled_mask)
    {
        FrictionMonitorReset(monitor);
        return;
    }

    uint8_t working_targets = 1;
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        float target = control_status->requested_rpm[i];
        if (target != monitor->previous_targets[i])
        {
            /* A new three-wheel target starts a new group qualification. */
            FrictionMonitorReset(monitor);
        }
        monitor->previous_targets[i] = target;
        if (!isfinite(target) || target < monitor->wheel[i].config.minimum_target)
        {
            working_targets = 0;
        }
    }

    monitor->status.ready_mask = 0;
    for (unsigned i = 0; i < FRICTION_MOTOR_COUNT; ++i)
    {
        uint8_t bit = (uint8_t)(1U << i);
        SpeedReady_Input_s sample = {
            .target = control_status->requested_rpm[i],
            .reference = control_status->reference_rpm[i],
            .measured = control_status->feedback[i].rpm,
            .enabled = (control_status->enabled_mask & bit) != 0U,
            .feedback_valid = control_status->feedback[i].valid &&
                              (control_status->online_mask & bit) != 0U,
        };
        SpeedReady_Status_s result = SpeedReadyUpdate(&monitor->wheel[i], &sample, now_ms);
        if (result.ready)
        {
            monitor->status.ready_mask |= bit;
        }
    }

    monitor->status.all_ready = 0;
    if (control_status->enabled_mask != FRICTION_ALL_MOTORS || !working_targets)
    {
        monitor->previously_all_ready = 0;
        monitor->status.phase = FRICTION_PHASE_BENCH;
    }
    else if (monitor->status.ready_mask == FRICTION_ALL_MOTORS)
    {
        monitor->previously_all_ready = 1;
        monitor->status.all_ready = 1;
        monitor->status.phase = FRICTION_PHASE_READY;
    }
    else
    {
        monitor->status.phase = monitor->previously_all_ready ? FRICTION_PHASE_RECOVERING :
                                                             FRICTION_PHASE_SPINUP;
    }
}
