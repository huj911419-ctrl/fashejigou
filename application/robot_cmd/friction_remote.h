#ifndef FRICTION_REMOTE_H
#define FRICTION_REMOTE_H

#include "dbus_input.h"

#define FRICTION_REMOTE_GROUP_COUNT 2U
#define FRICTION_REMOTE_GEARS_PER_GROUP 3U

typedef struct
{
    uint32_t timeout_ms;
    float max_rpm;      /* Motor rotor RPM ceiling, not projectile speed. */
    float gear_rpm[FRICTION_REMOTE_GROUP_COUNT][FRICTION_REMOTE_GEARS_PER_GROUP];
    uint8_t stop_s1;    /* Distinct DBUS values 1..3: STOP, low group, high group. */
    uint8_t low_s1;
    uint8_t high_s1;
    uint8_t gear_s2[FRICTION_REMOTE_GEARS_PER_GROUP]; /* Three distinct values 1..3. */
    uint8_t channel;    /* CH0..CH3 observation only; never controls speed/rearm. */
    int8_t direction;   /* Observation sign only, +1 or -1. */
} FrictionRemoteConfig_s;

typedef struct
{
    DbusInput_s input;
    float stick;
    float target_rpm;
    float selected_rpm; /* Switch selection, even when inhibited; not motor output. */
    uint8_t group;      /* 0=STOP/invalid, 1=low, 2=high. */
    uint8_t gear;       /* 0=STOP/invalid, 1..6=configured gear. */
    bool online;
    bool switches_on;
    bool needs_off;     /* Requires a NEW S1 STOP frame; sticks are irrelevant. */
} FrictionRemoteStatus_s;

typedef struct
{
    uint32_t received_ms;
    uint32_t sequence;
    float target_rpm;
    bool new_frame;
    bool run;
    bool stop_requested; /* Sticky: OFF/invalid cannot disappear in a batch. */
    bool qualified_off;  /* Fresh physical S1 STOP, never synthetic. */
} FrictionRemoteRequest_s;

typedef struct
{
    FrictionRemoteConfig_s config;
    DbusDecoder_s decoder;
    FrictionRemoteStatus_s status;
    uint32_t taken_sequence;
    bool needs_off;
    bool stop_pending;
    bool was_online;
} FrictionRemote_s;

/** Pure C command producer, no motor/HAL/RTOS dependency. Single task owner.
 * Startup, timeout and invalid data require S1 STOP, then a running group.
 * Push timestamps are original byte receive times on the same uint32 ms clock.
 * Take never refreshes the input timestamp; only NEW frames may renew RUN. */
bool FrictionRemoteInit(FrictionRemote_s *remote, const FrictionRemoteConfig_s *config);
void FrictionRemotePush(FrictionRemote_s *remote, uint8_t byte, uint32_t received_ms);
/** Invalidate after lost bytes/UART errors. Preserve a stop until Take. */
void FrictionRemoteInvalidate(FrictionRemote_s *remote);
/** Authority loss/handoff: forget arming consent; preserve DBUS synchronization. */
void FrictionRemoteDisarm(FrictionRemote_s *remote);
void FrictionRemoteTake(FrictionRemote_s *remote, uint32_t now_ms, FrictionRemoteRequest_s *request);

#endif
