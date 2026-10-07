#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "gimbal_command.h"
#include "dm_velocity.h"
#include "yaw_rate.h"
#include "loop_stats.h"

typedef struct
{
    DmVelocityConfig_s motor;
    YawRateConfig_s yaw;
    uint32_t command_timeout_ms;
} GimbalControlConfig_s;

typedef struct
{
    uint32_t sample_ms;
    LoopStats_s timing;
    uint32_t transport_faults; /* Historical OR; distinct from current health. */
    uint32_t motor_fault;
    uint32_t inhibit;          /* YAW_INHIBIT_* at sample_ms; not a motor fault. */
    uint16_t master_id;
    DmVelocityState_e motor_state;
    DmVelocityFeedback_s feedback;
    GimbalCommand_s request;   /* Last request; check command_online before use. */
    float target_rad_s;        /* Motor-side rate after direction mapping. */
    float reference_rad_s;
    bool command_online;      /* Request valid and within its original lease. */
    bool enabled;              /* Application authorization, NOT motor ACK. */
    bool motor_online;         /* Feedback age within its configured limit. */
    bool motor_enabled;        /* Fresh feedback explicitly reports ON. */
    bool port_ready;
} GimbalControlStatus_s;

typedef struct
{
    GimbalControlConfig_s config;
    DmVelocity_s motor;
    YawRate_s yaw;
    GimbalControlStatus_s status;
    LoopStats_s timing;
    uint32_t transport_faults;
    bool configured;
    bool transport_fault_pending;
} GimbalControl_s;

/** Portable subsystem: no HAL, RTOS, global state or memory allocation.
 * Caller owns storage and serializes ALL operations in one control task.
 * Config is copied. Transport callback must copy frames, never block.
 * Init once on a stopped system; do not reinitialize an energized actuator. */
bool GimbalControlInit(GimbalControl_s *control, const GimbalControlConfig_s *config,
                       DmVelocitySend_f send, void *context, uint32_t now_ms);
/** Ordered frames from the current session. received_ms is the RX timestamp.
 * now_ms must be sampled AFTER dequeuing, from the SAME monotonic clock. */
void GimbalControlReceiveCan(GimbalControl_s *control, const DmCanFrame_s *frame,
                             uint32_t now_ms);
/** Drop queued data before reporting loss; the composition layer also invalidates
 * its input producer. Requires a new OFF -> ON sequence. Caller supplies its
 * own diagnostic fault bits. This subsystem never owns a remote decoder. */
void GimbalControlTransportFault(GimbalControl_s *control, uint32_t faults);
/** Call after draining bounded inputs; sample time AFTER draining. command is
 * copied during this call; NULL is an invalid/stop input, never a keepalive.
 * The producer's original timestamp determines age, even if passed repeatedly.
 * status is an owner-task view; adapters publish synchronized copies. */
void GimbalControlStep(GimbalControl_s *control, const GimbalCommand_s *command,
                       uint32_t now_ms, bool port_ready);

#endif
