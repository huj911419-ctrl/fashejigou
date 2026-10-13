#ifndef CONTROL_AUTHORITY_H
#define CONTROL_AUTHORITY_H

#include <stdbool.h>
#include <stdint.h>

/** Explicitly selected command producer; NONE always remains selectable. */
typedef enum
{
    CONTROL_SOURCE_NONE = 0,
    CONTROL_SOURCE_REMOTE = 1,
    CONTROL_SOURCE_DEBUG = 2,
    CONTROL_SOURCE_AUTO = 3,
    CONTROL_SOURCE_COUNT = 3
} ControlSource_t;

#define CONTROL_SOURCE_MASK(source) (UINT32_C(1) << (source))
#define CONTROL_SOURCE_ALL (CONTROL_SOURCE_MASK(CONTROL_SOURCE_REMOTE) | \
                            CONTROL_SOURCE_MASK(CONTROL_SOURCE_DEBUG) | \
                            CONTROL_SOURCE_MASK(CONTROL_SOURCE_AUTO))

enum
{
    CONTROL_AUTHORITY_CONFIG = 1U,
    CONTROL_AUTHORITY_NO_OWNER = 2U,
    CONTROL_AUTHORITY_NO_INPUT = 4U,
    CONTROL_AUTHORITY_INVALID_INPUT = 8U,
    CONTROL_AUTHORITY_TIMEOUT = 16U,
    CONTROL_AUTHORITY_FUTURE_INPUT = 32U,
    CONTROL_AUTHORITY_STOP = 64U,
    CONTROL_AUTHORITY_SWITCH_OFF = 128U,
    CONTROL_AUTHORITY_REARM_REQUIRED = 256U,
    CONTROL_AUTHORITY_NOT_NEUTRAL = 512U
};

typedef struct
{
    uint32_t allowed_sources; /* CONTROL_SOURCE_MASK() bits, never NONE. */
    uint32_t timeout_ms[CONTROL_SOURCE_COUNT]; /* Indexed by source - 1. */
} ControlAuthorityConfig_s;

typedef struct
{
    ControlSource_t source;
    uint32_t epoch;       /* Captured when this owner's input path is activated. */
    uint32_t received_ms; /* ORIGINAL producer/transport timestamp, never poll time. */
    uint32_t sequence;    /* Diagnostic only; does not authorize or refresh input. */
    bool valid;
    bool arm;
    bool neutral;        /* Required only on the OFF -> ON authorization edge. */
    bool stop_requested; /* Latched across several Submit calls before Step. */
} ControlAuthorityInput_s;

typedef struct
{
    ControlSource_t selected;
    uint32_t epoch;
    uint32_t input_sequence;
    uint32_t reason;      /* Zero means authorization, not an actuator enable ACK. */
    bool authorized;
    bool online;         /* Fresh valid selected input, independent of arm state. */
} ControlAuthorityStatus_s;

typedef struct
{
    ControlAuthorityConfig_s config;
    ControlAuthorityInput_s input;
    ControlAuthorityStatus_s status;
    uint32_t off_received_ms;
    bool configured;
    bool have_input;
    bool input_pending;
    bool off_seen;
    bool stop_pending;
} ControlAuthority_s;

/** Pure C, caller-owned storage, single serialized task owner; no hardware,
 * heap, global state or automatic fallback. Config is copied. Enabled-source
 * timeouts must be 1..INT32_MAX ms; disabled-source entries are ignored.
 * Invalid config fails closed. Initial source is NONE, initially unauthorized. */
bool ControlAuthorityInit(ControlAuthority_s *authority,
                          const ControlAuthorityConfig_s *config);

/** Select an allowed owner or NONE. A change increments epoch and discards all
 * cached input/OFF authorization. Same-owner selection has no side effects.
 * Preserve a pending global STOP. Rejected selection leaves state unchanged.
 * Callers must clear their source mailboxes and activate the new epoch rather
 * than relabeling an old cached command with the new epoch. */
bool ControlAuthoritySelect(ControlAuthority_s *authority, ControlSource_t source);

/** Accept only selected source/current epoch and a strictly newer timestamp
 * (serial difference 1..INT32_MAX); sequence is diagnostic only. A duplicate or
 * older sample cannot change input or renew its lease, but its explicit STOP
 * is still latched if source/epoch match. Every accepted invalid or arm=false
 * sample also latches STOP, preserving faults/OFF across batches of commands.
 * false means payload was not accepted.
 * Submit valid=false with a new timestamp, or call Stop for immediate faults. */
bool ControlAuthoritySubmit(ControlAuthority_s *authority,
                            const ControlAuthorityInput_s *input);

/** Global sticky stop. Later submissions/selections cannot erase it before
 * Step. Revokes current authorization immediately; Step consumes the latch. */
void ControlAuthorityStop(ControlAuthority_s *authority);

/** Evaluate the cached input at now_ms; age == timeout is expired. A timestamp
 * in the future is rejected and discarded using modulo-2^32 time (all intervals
 * < 2^31 ms), so it cannot block subsequent real-time inputs.
 * Startup, handoff, STOP, invalidity and timeout require a NEW valid OFF sample
 * followed by a later NEW neutral ON sample. A current newly accepted OFF may
 * satisfy rearm in the same Step as STOP; a coalesced OFF/ON batch may not.
 * Repeated Step calls never renew freshness or reuse a consumed OFF sample.
 * Downstream actuator health/timing checks remain independently mandatory. */
void ControlAuthorityStep(ControlAuthority_s *authority, uint32_t now_ms);

#endif
