#ifndef GIMBAL_REMOTE_H
#define GIMBAL_REMOTE_H

#include "dbus_input.h"
#include "gimbal_command.h"

typedef struct
{
    uint32_t timeout_ms;
    float max_rate_rad_s;  /* Motor-side full-stick speed, > 0. */
    float deadband;        /* Normalized stick range [0, 1). */
    uint8_t yaw_channel;
    uint8_t arm_s1_value;  /* DBUS wire value 1..3, not a universal switch label. */
    int8_t yaw_direction;  /* +1 or -1. */
} GimbalRemoteConfig_s;

typedef struct
{
    DbusInput_s input;
    float stick;          /* Raw normalized channel before direction/deadband. */
    bool online;
    bool arm_switch;
} GimbalRemoteStatus_s;

typedef struct
{
    GimbalRemoteConfig_s config;
    DbusDecoder_s decoder;
    GimbalRemoteStatus_s status;
    bool configured;
    bool stop_pending;
} GimbalRemote_s;

/** Pure C command producer; no motor, HAL, RTOS, heap or global state.
 * Caller owns storage and serializes operations in the composition task.
 * Config is copied; initialize only while the controlled subsystem is stopped.
 * Invalid config returns false and produces invalid zero-speed requests. */
bool GimbalRemoteInit(GimbalRemote_s *remote, const GimbalRemoteConfig_s *config);
/** Feed ordered bytes with their ORIGINAL RX timestamps after draining the port. */
void GimbalRemoteReceiveByte(GimbalRemote_s *remote, uint8_t byte, uint32_t received_ms);
/** Call on transport loss, AFTER the adapter has dropped queued data. */
void GimbalRemoteInvalidate(GimbalRemote_s *remote);
/** Build one request and consume the pending stop flag. Call exactly once per
 * control iteration, then deliver it to GimbalControlStep in that iteration.
 * now_ms must be sampled after draining RX. Repeated calls never renew the
 * original frame timestamp. status is an owner-task diagnostic view. */
GimbalCommand_s GimbalRemoteTakeCommand(GimbalRemote_s *remote, uint32_t now_ms);

#endif
