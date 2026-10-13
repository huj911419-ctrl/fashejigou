#ifndef FRICTION_H
#define FRICTION_H

#include "control/friction_control.h"
#include "control/friction_monitor.h"
#include "dji_speed_group.h"

typedef DjiSpeedTuning_s Friction_PID_Tuning_s;

typedef enum
{
    FRICTION_SOURCE_NONE,
    FRICTION_SOURCE_RTT,
    FRICTION_SOURCE_VOFA,
    FRICTION_SOURCE_USB,
    FRICTION_SOURCE_REMOTE
} Friction_Command_Source_e;

typedef enum
{
    FRICTION_SUBMIT_ACCEPTED,
    FRICTION_SUBMIT_WRONG_SOURCE,
    FRICTION_SUBMIT_UNAVAILABLE,
    FRICTION_SUBMIT_BUSY,
    FRICTION_SUBMIT_STALE,
    FRICTION_SUBMIT_REARM_REQUIRED
} Friction_Submit_Result_e;

typedef struct
{
    Friction_PID_Tuning_s motor[FRICTION_MOTOR_COUNT];
    float ramp_rpm_s;
} Friction_Tuning_s;

typedef struct
{
    Friction_Status_s status;
    Friction_Tuning_s tuning;
    Friction_Readiness_s readiness;
    float output[FRICTION_MOTOR_COUNT]; /* logical direction, raw current command */
    float current[FRICTION_MOTOR_COUNT]; /* feedback, normalized logical direction */
    float p[FRICTION_MOTOR_COUNT];
    float i[FRICTION_MOTOR_COUNT];
    float d[FRICTION_MOTOR_COUNT];
    uint32_t sample_ms;
    uint32_t sample_id;
    uint32_t tuning_revision; /* Increments for every applied transaction, including legacy tuning. */
    uint32_t command_generation; /* Mailbox generation consumed by this motor sample. */
    /* Current mailbox metadata, copied under its lock by GetTelemetry. These
     * may be newer than sample_ms; they never acknowledge physical output. */
    uint32_t source_epoch;    /* Selected-source changes, not a motor enable counter. */
    Friction_Command_Source_e command_source;
    uint8_t source_requires_off;
    uint8_t initialized;
    uint8_t command_received_now;
    uint8_t pending_mask_now;
    uint8_t tuning_pending_now;
    uint8_t profile_pending_now;
    uint16_t applied_sequence;
} Friction_Telemetry_s;

typedef enum
{
    FRICTION_TUNE_QUEUED,
    FRICTION_TUNE_RANGE,
    FRICTION_TUNE_BUSY,
    FRICTION_TUNE_UNAVAILABLE
} Friction_Tune_Result_e;

/** Task context. Copy parameters and observations atomically. readiness.all_ready
 * is only a speed qualification, not permission to feed. Consumers must check
 * sample_ms age as well as their own mechanism/referee/command conditions. */
void FrictionGetTelemetry(Friction_Telemetry_s *telemetry);
/** Queue an entire tuning transaction, applied only by the motor task.
 * base_sequence must match the snapshot used to prepare the transaction.
 * RAM only. Does not renew the motor command timeout or enable motors. */
Friction_Tune_Result_e FrictionQueueTuning(const Friction_Tuning_s *tuning,
                                         uint16_t sequence, uint16_t base_sequence);
/** Profile import admission, checked under the command mailbox lock.
 * QUEUED means ready to accept, not that any parameters have been submitted.
 * Requires outputs disabled, no pending RUN, and no pending tuning. */
Friction_Tune_Result_e FrictionProfileAdmission(void);
/** Whole profile import: checks full revision and stopped state atomically.
 * Nonzero run submissions are ignored until the motor task publishes application.
 * STOP always remains available. Never enables motors or renews their lease. */
Friction_Tune_Result_e FrictionQueueProfile(const Friction_Tuning_s *tuning,
                                          uint16_t sequence, uint32_t base_revision);
uint8_t FrictionTuningAvailable(void);
/** Immutable compiled current ceilings, C620 raw units, fixed F1/F2/F3 order. */
void FrictionGetCurrentLimits(float limits[FRICTION_MOTOR_COUNT]);

/** Call once before scheduler start. Only wheels selected by bench_features.h
 * are registered; disabled wheels need no ID. Disabled group returns success
 * without hardware registration; its task/submit calls are inert and tuning
 * returns UNAVAILABLE. Invalid selected configuration leaves all outputs off.
 * Successful init observes OFF synchronously without sending motor output.
 * Default owner is active REMOTE, otherwise USB, then RTT, then NONE.
 * Remote producer separately requires a physical S1 STOP before first RUN.
 * Without remote, the first fresh default-owner RUN needs no user OFF.
 * Repeated init is inert: it cannot clear a runtime fault or reset authority. */
uint8_t FrictionInit(void);
/** Explicit bench source selection, task context. No automatic failover.
 * A changed source discards the previous command and requests a sticky stop.
 * The new owner must submit a fresh OFF, observed by the motor task, then RUN.
 * Same-source selection is inert. A queued profile rejects changes with BUSY.
 * Sources disabled at build time return UNAVAILABLE; NONE always permits stop. */
Friction_Submit_Result_e FrictionSelectSource(Friction_Command_Source_e source);
/** Current selected mailbox owner; not proof of online input or enabled output. */
Friction_Command_Source_e FrictionGetSource(void);
/** Copy one source-labelled request under the mailbox lock. received_ms is its
 * original validated receive time on ControlPlatformNowMs' uint32 clock.
 * Rejected RUN never changes the mailbox or renews its lease. An explicit OFF
 * stops from either source, but only a fresh current-owner OFF satisfies the
 * post-switch rearm gate. A stale OFF still stops but does not satisfy that gate.
 * REMOTE RUN requires explicitly selected REMOTE; it cannot acquire NONE.
 * NULL requests a global stop. ACCEPTED is queued input, not a motor enable ACK. */
Friction_Submit_Result_e FrictionSubmitFrom(Friction_Command_Source_e source,
                                           const Friction_Command_s *command,
                                           uint32_t received_ms);
/** Task context only. Submit a complete command at <250 ms intervals by default.
 * Compatibility wrapper for RTT only, timestamped at this call. It cannot
 * submit RUN while another source owns commands. Use SubmitFrom for new code.
 * Runtime recovery requires a disabled command, then a NEW enabled command.
 * Clean startup OFF is prepared by Init before input tasks start.
 * Mask bits 0/1/2 always select F1/F2/F3; numbering never compacts.
 * Selecting a build-disabled wheel latches COMMAND fault for the whole request.
 * Zero coasts all; 7 requires all three build switches to be enabled. */
void FrictionSubmit(const Friction_Command_s *command);
/** USB friction bench command parser. Accepts off, set, pid and ramp commands.
 * Commands are queued for the motor task; a successful return is not an output ACK. */
uint8_t FrictionSubmitFromUsbCommand(const char *line, uint32_t now_ms);
/** Global sticky stop, including existing USB off semantics: releases the
 * selected source and source OFF gate; releases increment source_epoch.
 * Motor-task OFF/recovery still must be
 * consumed before RUN; a later enable cannot overwrite the pending stop. */
void FrictionDisable(void);
/** USB global STOP, task context. Retains an explicitly selected USB owner so
 * remote OFF frames cannot steal authority between USB off and set.
 * Other owners are released as with Disable, requiring fresh remote rearm. */
void FrictionStopFromUsb(void);
/** Atomic diagnostic snapshot; task context only. */
void FrictionGetStatus(Friction_Status_s *status);
/** Sole owner of motor references/PIDs; call once per control period. */
void FrictionTask(void);

#endif
