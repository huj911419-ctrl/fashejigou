#ifndef FRICTION_REMOTE_CONFIG_H
#define FRICTION_REMOTE_CONFIG_H

/* Six fixed MOTOR ROTOR RPM gears, shared by the enabled wheels.
 * DBUS values: 1=up, 3=middle, 2=down; verify S1/S2 in USB [remote_input].
 * S1 up=STOP, middle=low group, down=high group.
 * S2 down/middle/up selects increasing RPM. Sticks are observation only.
 * Startup/recovery: receive S1 STOP, then select a running group.
 * These are bench speeds, not calibrated projectile-speed settings. */
#define FRICTION_REMOTE_TIMEOUT_MS 100U
#define FRICTION_REMOTE_MAX_RPM 7000.0f
#define FRICTION_REMOTE_STOP_S1 1U
#define FRICTION_REMOTE_LOW_S1 3U
#define FRICTION_REMOTE_HIGH_S1 2U
#define FRICTION_REMOTE_GEAR_S2 {2U, 3U, 1U}
#define FRICTION_REMOTE_GEAR_RPM { \
    {1000.0f, 2000.0f, 3000.0f}, \
    {5000.0f, 6000.0f, 7000.0f} }
/* Retain CH1 in the legacy FRRC log for observation, never as a control input. */
#define FRICTION_REMOTE_CHANNEL 1U
#define FRICTION_REMOTE_STICK_DIRECTION 1

#define FRICTION_REMOTE_CONFIG { \
    .timeout_ms = FRICTION_REMOTE_TIMEOUT_MS, .max_rpm = FRICTION_REMOTE_MAX_RPM, \
    .gear_rpm = FRICTION_REMOTE_GEAR_RPM, .gear_s2 = FRICTION_REMOTE_GEAR_S2, \
    .stop_s1 = FRICTION_REMOTE_STOP_S1, .low_s1 = FRICTION_REMOTE_LOW_S1, \
    .high_s1 = FRICTION_REMOTE_HIGH_S1, .channel = FRICTION_REMOTE_CHANNEL, \
    .direction = FRICTION_REMOTE_STICK_DIRECTION }

#endif
